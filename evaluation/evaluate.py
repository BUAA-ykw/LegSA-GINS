
#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import json
import math
import os
from typing import List, Dict, Optional

import numpy as np
import pandas as pd

THRESHOLDS = {
    "horizontal_m": 2.0,
    "vertical_m": 3.0,
    "roll_deg": 1.0,
    "pitch_deg": 1.0,
    "yaw_deg": 2.0,
}

CONVERGENCE_RULE = {
    "horizontal_m": 2.0,
    "yaw_deg": 2.0,
    "hold_time_sec": 3.0,
}

def wrap_deg(x: np.ndarray) -> np.ndarray:
    return (x + 180.0) % 360.0 - 180.0

def rmse(x: np.ndarray) -> float:
    return float(np.sqrt(np.mean(np.square(x))))

def mae(x: np.ndarray) -> float:
    return float(np.mean(np.abs(x)))

def p95(x: np.ndarray) -> float:
    return float(np.percentile(np.abs(x), 95))

def max_abs(x: np.ndarray) -> float:
    return float(np.max(np.abs(x)))

def pass_ratio(x: np.ndarray, threshold: float) -> float:
    return float(np.mean(np.abs(x) <= threshold))

def compute_convergence_time(time_s, horizontal_err, yaw_err):
    ok = (np.abs(horizontal_err) <= CONVERGENCE_RULE["horizontal_m"]) & (np.abs(yaw_err) <= CONVERGENCE_RULE["yaw_deg"])
    start_idx = None
    for i in range(len(ok)):
        if ok[i] and start_idx is None:
            start_idx = i
        elif not ok[i]:
            start_idx = None
        if start_idx is not None and time_s[i] - time_s[start_idx] >= CONVERGENCE_RULE["hold_time_sec"]:
            return float(time_s[start_idx] - time_s[0])
    return None

def lla_to_ecef(lat_deg, lon_deg, h_m):
    a = 6378137.0
    e2 = 6.69437999014e-3
    lat = np.deg2rad(lat_deg)
    lon = np.deg2rad(lon_deg)
    sin_lat = np.sin(lat)
    cos_lat = np.cos(lat)
    sin_lon = np.sin(lon)
    cos_lon = np.cos(lon)
    N = a / np.sqrt(1.0 - e2 * sin_lat**2)
    x = (N + h_m) * cos_lat * cos_lon
    y = (N + h_m) * cos_lat * sin_lon
    z = (N * (1.0 - e2) + h_m) * sin_lat
    return x, y, z

def ecef_to_enu(x, y, z, ref_lat_deg, ref_lon_deg, ref_h_m):
    xr, yr, zr = lla_to_ecef(np.array([ref_lat_deg]), np.array([ref_lon_deg]), np.array([ref_h_m]))
    xr, yr, zr = xr[0], yr[0], zr[0]
    lat0 = math.radians(ref_lat_deg)
    lon0 = math.radians(ref_lon_deg)
    sin_lat = math.sin(lat0)
    cos_lat = math.cos(lat0)
    sin_lon = math.sin(lon0)
    cos_lon = math.cos(lon0)
    dx = x - xr
    dy = y - yr
    dz = z - zr
    e = -sin_lon * dx + cos_lon * dy
    n = -sin_lat * cos_lon * dx - sin_lat * sin_lon * dy + cos_lat * dz
    u =  cos_lat * cos_lon * dx + cos_lat * sin_lon * dy + sin_lat * dz
    return e, n, u

def parse_outage_arg(outage_str: Optional[str]) -> List[Dict]:
    if not outage_str:
        return []
    results = []
    items = [x.strip() for x in outage_str.split(";") if x.strip()]
    for item in items:
        parts = [x.strip() for x in item.split(",")]
        if len(parts) < 2:
            continue
        start = float(parts[0]); end = float(parts[1]); label = parts[2] if len(parts) >= 3 else f"outage_{start:.1f}_{end:.1f}"
        results.append({"start": start, "end": end, "label": label})
    return results

def compute_outage_metrics(err_df: pd.DataFrame, outages: List[Dict]) -> List[Dict]:
    results = []
    for item in outages:
        seg = err_df[(err_df["time"] >= item["start"]) & (err_df["time"] <= item["end"])].copy()
        if len(seg) < 2:
            continue
        dur = float(seg["time"].iloc[-1] - seg["time"].iloc[0])
        if dur <= 0:
            continue
        h_end = float(seg["horizontal_err_m"].iloc[-1])
        yaw_end = float(abs(seg["yaw_err_deg"].iloc[-1]))
        results.append({
            "label": item["label"],
            "duration_sec": dur,
            "horizontal_end_m": h_end,
            "horizontal_drift_rate_mps": h_end / dur,
            "yaw_end_deg": yaw_end,
            "yaw_drift_rate_degps": yaw_end / dur
        })
    return results

def load_nav(nav_path: str) -> pd.DataFrame:
    df = pd.read_csv(nav_path, sep=r"\s+", engine="python", header=None, comment="%")
    df = df.dropna(axis=1, how="all")
    if df.shape[1] < 11:
        raise ValueError(f"NAV 文件列数异常，至少需要 11 列，当前只有 {df.shape[1]} 列")
    nav = pd.DataFrame({
        "time": pd.to_numeric(df[1], errors="coerce"),
        "lat": pd.to_numeric(df[2], errors="coerce"),
        "lon": pd.to_numeric(df[3], errors="coerce"),
        "alt": pd.to_numeric(df[4], errors="coerce"),
        "roll": pd.to_numeric(df[8], errors="coerce"),
        "pitch": pd.to_numeric(df[9], errors="coerce"),
        "yaw": pd.to_numeric(df[10], errors="coerce"),
    }).dropna().reset_index(drop=True)
    return nav

def load_trace(trace_path: str, base_time: Optional[float]) -> pd.DataFrame:
    df = pd.read_csv(trace_path)
    def find_col(names):
        for n in names:
            for c in df.columns:
                if n.lower() == c.lower() or n.lower() in c.lower():
                    return c
        return None
    time_col = find_col(["aligned_time", "time", "stamp", "timestamp"])
    lat_col = find_col(["lat", "latitude", "pos_lat"])
    lon_col = find_col(["lon", "longitude", "pos_lon"])
    alt_col = find_col(["height", "alt", "altitude", "pos_height"])
    roll_col = find_col(["roll"])
    pitch_col = find_col(["pitch"])
    yaw_col = find_col(["yaw"])
    if not time_col:
        raise ValueError("trace 文件中找不到时间列")
    missing = [k for k, v in {"lat": lat_col, "lon": lon_col, "alt": alt_col, "roll": roll_col, "pitch": pitch_col, "yaw": yaw_col}.items() if v is None]
    if missing:
        raise ValueError(f"trace 文件缺少列: {missing}")
    gt_time = pd.to_numeric(df[time_col], errors="coerce")
    if base_time is not None and "aligned_time" not in time_col.lower():
        gt_time = gt_time - base_time
    gt = pd.DataFrame({
        "time": gt_time,
        "lat": pd.to_numeric(df[lat_col], errors="coerce"),
        "lon": pd.to_numeric(df[lon_col], errors="coerce"),
        "alt": pd.to_numeric(df[alt_col], errors="coerce"),
        "roll": pd.to_numeric(df[roll_col], errors="coerce"),
        "pitch": pd.to_numeric(df[pitch_col], errors="coerce"),
        "yaw": pd.to_numeric(df[yaw_col], errors="coerce"),
    }).dropna().sort_values("time").reset_index(drop=True)
    return gt

def load_std(std_path: str) -> pd.DataFrame:
    df = pd.read_csv(std_path, sep=r"\s+", engine="python", header=None, comment="%")
    df = df.dropna(axis=1, how="all")
    if df.shape[1] < 10:
        raise ValueError(f"STD 文件列数异常，当前只有 {df.shape[1]} 列")
    out = pd.DataFrame({
        "time": pd.to_numeric(df[0], errors="coerce"),
        "pos_n_std_m": pd.to_numeric(df[1], errors="coerce"),
        "pos_e_std_m": pd.to_numeric(df[2], errors="coerce"),
        "pos_u_std_m": pd.to_numeric(df[3], errors="coerce"),
        "vel_n_std_mps": pd.to_numeric(df[4], errors="coerce"),
        "vel_e_std_mps": pd.to_numeric(df[5], errors="coerce"),
        "vel_u_std_mps": pd.to_numeric(df[6], errors="coerce"),
        "roll_std_deg": pd.to_numeric(df[7], errors="coerce"),
        "pitch_std_deg": pd.to_numeric(df[8], errors="coerce"),
        "yaw_std_deg": pd.to_numeric(df[9], errors="coerce"),
    }).dropna().sort_values("time").reset_index(drop=True)
    out["horizontal_3sigma_m"] = 3.0 * np.sqrt(out["pos_n_std_m"]**2 + out["pos_e_std_m"]**2)
    out["roll_3sigma_deg"] = 3.0 * out["roll_std_deg"]
    out["pitch_3sigma_deg"] = 3.0 * out["pitch_std_deg"]
    out["yaw_3sigma_deg"] = 3.0 * out["yaw_std_deg"]
    return out

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--trace", required=True)
    parser.add_argument("--nav", required=True)
    parser.add_argument("--std", default=None)
    parser.add_argument("--outdir", required=True)
    parser.add_argument("--base_time", type=float, default=None)
    parser.add_argument("--yaw_truth_mode", choices=["enu", "ned"], default="enu")
    parser.add_argument("--outage", default=None)
    args = parser.parse_args()

    os.makedirs(args.outdir, exist_ok=True)
    nav = load_nav(args.nav)
    gt = load_trace(args.trace, args.base_time)

    t_start = max(nav["time"].min(), gt["time"].min())
    t_end = min(nav["time"].max(), gt["time"].max())
    if t_start >= t_end:
        raise ValueError(
            f"真值和解算结果没有时间交集。 nav:[{nav['time'].min():.3f}, {nav['time'].max():.3f}] "
            f"gt:[{gt['time'].min():.3f}, {gt['time'].max():.3f}]。如果 gt 是绝对时间，请加 --base_time。"
        )

    nav = nav[(nav["time"] >= t_start) & (nav["time"] <= t_end)].copy().reset_index(drop=True)
    t_nav = nav["time"].to_numpy()
    t_gt = gt["time"].to_numpy()

    gt_lat = np.interp(t_nav, t_gt, gt["lat"].to_numpy())
    gt_lon = np.interp(t_nav, t_gt, gt["lon"].to_numpy())
    gt_alt = np.interp(t_nav, t_gt, gt["alt"].to_numpy())
    gt_roll = np.interp(t_nav, t_gt, gt["roll"].to_numpy())
    gt_pitch = np.interp(t_nav, t_gt, gt["pitch"].to_numpy())
    gt_yaw_unwrapped = np.unwrap(np.deg2rad(gt["yaw"].to_numpy()))
    gt_yaw_interp = np.rad2deg(np.interp(t_nav, t_gt, gt_yaw_unwrapped))
    gt_yaw_for_cmp = (90.0 - gt_yaw_interp) % 360.0 if args.yaw_truth_mode == "enu" else gt_yaw_interp % 360.0

    nav_x, nav_y, nav_z = lla_to_ecef(nav["lat"].to_numpy(), nav["lon"].to_numpy(), nav["alt"].to_numpy())
    gt_x, gt_y, gt_z = lla_to_ecef(gt_lat, gt_lon, gt_alt)
    ref_lat, ref_lon, ref_alt = gt_lat[0], gt_lon[0], gt_alt[0]
    nav_e, nav_n, nav_u = ecef_to_enu(nav_x, nav_y, nav_z, ref_lat, ref_lon, ref_alt)
    gt_e, gt_n, gt_u = ecef_to_enu(gt_x, gt_y, gt_z, ref_lat, ref_lon, ref_alt)

    err_e = nav_e - gt_e
    err_n = nav_n - gt_n
    err_u = nav_u - gt_u
    horizontal_err = np.sqrt(err_e**2 + err_n**2)
    position_3d_err = np.sqrt(err_e**2 + err_n**2 + err_u**2)
    roll_err = wrap_deg(nav["roll"].to_numpy() - gt_roll)
    pitch_err = wrap_deg(nav["pitch"].to_numpy() - gt_pitch)
    yaw_err = wrap_deg(nav["yaw"].to_numpy() - gt_yaw_for_cmp)

    err_df = pd.DataFrame({
        "time": t_nav,
        "err_n_m": err_n,
        "err_e_m": err_e,
        "err_u_m": err_u,
        "horizontal_err_m": horizontal_err,
        "position_3d_err_m": position_3d_err,
        "roll_err_deg": roll_err,
        "pitch_err_deg": pitch_err,
        "yaw_err_deg": yaw_err,
    })

    summary = {
        "position": {
            "north_rmse_m": rmse(err_n),
            "east_rmse_m": rmse(err_e),
            "up_rmse_m": rmse(err_u),
            "horizontal_rmse_m": rmse(horizontal_err),
            "position_3d_rmse_m": rmse(position_3d_err),
            "horizontal_mae_m": mae(horizontal_err),
            "horizontal_p95_m": p95(horizontal_err),
            "horizontal_max_m": max_abs(horizontal_err),
            "vertical_p95_m": p95(err_u),
            "vertical_max_m": max_abs(err_u),
            "horizontal_pass_ratio": pass_ratio(horizontal_err, THRESHOLDS["horizontal_m"]),
            "vertical_pass_ratio": pass_ratio(err_u, THRESHOLDS["vertical_m"]),
        },
        "attitude": {
            "roll_rmse_deg": rmse(roll_err),
            "pitch_rmse_deg": rmse(pitch_err),
            "yaw_rmse_deg": rmse(yaw_err),
            "roll_p95_deg": p95(roll_err),
            "pitch_p95_deg": p95(pitch_err),
            "yaw_p95_deg": p95(yaw_err),
            "roll_max_deg": max_abs(roll_err),
            "pitch_max_deg": max_abs(pitch_err),
            "yaw_max_deg": max_abs(yaw_err),
            "roll_pass_ratio": pass_ratio(roll_err, THRESHOLDS["roll_deg"]),
            "pitch_pass_ratio": pass_ratio(pitch_err, THRESHOLDS["pitch_deg"]),
            "yaw_pass_ratio": pass_ratio(yaw_err, THRESHOLDS["yaw_deg"]),
        },
        "convergence": {
            "convergence_time_sec": compute_convergence_time(t_nav, horizontal_err, yaw_err)
        },
        "meta": {
            "num_samples": int(len(err_df)),
            "time_start": float(t_nav[0]),
            "time_end": float(t_nav[-1]),
            "duration_sec": float(t_nav[-1] - t_nav[0]),
            "base_time": args.base_time,
            "yaw_truth_mode": args.yaw_truth_mode,
        }
    }

    if args.outage:
        summary["outage_metrics"] = compute_outage_metrics(err_df, parse_outage_arg(args.outage))

    if args.std:
        std_df = load_std(args.std)
        t_std = std_df["time"].to_numpy()
        h3 = np.interp(t_nav, t_std, std_df["horizontal_3sigma_m"].to_numpy())
        r3 = np.interp(t_nav, t_std, std_df["roll_3sigma_deg"].to_numpy())
        p3 = np.interp(t_nav, t_std, std_df["pitch_3sigma_deg"].to_numpy())
        y3 = np.interp(t_nav, t_std, std_df["yaw_3sigma_deg"].to_numpy())
        err_df["horizontal_3sigma_m"] = h3
        err_df["roll_3sigma_deg"] = r3
        err_df["pitch_3sigma_deg"] = p3
        err_df["yaw_3sigma_deg"] = y3
        summary["consistency_3sigma"] = {
            "horizontal_in_3sigma_ratio": float(np.mean(np.abs(horizontal_err) <= h3)),
            "roll_in_3sigma_ratio": float(np.mean(np.abs(roll_err) <= r3)),
            "pitch_in_3sigma_ratio": float(np.mean(np.abs(pitch_err) <= p3)),
            "yaw_in_3sigma_ratio": float(np.mean(np.abs(yaw_err) <= y3)),
        }

    err_csv = os.path.join(args.outdir, "error_series.csv")
    json_path = os.path.join(args.outdir, "summary.json")
    err_df.to_csv(err_csv, index=False, encoding="utf-8-sig")
    with open(json_path, "w", encoding="utf-8") as f:
        json.dump(summary, f, ensure_ascii=False, indent=2)

    print("评估完成")
    print(f"误差序列: {err_csv}")
    print(f"汇总指标: {json_path}")
    print_pretty_report(summary)

    
def fmt(v, nd=4):
    if v is None:
        return "None"
    if isinstance(v, float):
        return f"{v:.{nd}f}"
    return str(v)


def print_pretty_report(summary: dict):
    print("\n" + "-" * 80)
    print(f"{'指标名称':<28} | {'双天线松耦合（评估结果）':<28}")
    print("-" * 80)

    # 位置指标
    print(f"{'北向位置误差 RMSE (m)':<28} | {fmt(summary['position']['north_rmse_m']):<28}")
    print(f"{'东向位置误差 RMSE (m)':<28} | {fmt(summary['position']['east_rmse_m']):<28}")
    print(f"{'垂向位置误差 RMSE (m)':<28} | {fmt(summary['position']['up_rmse_m']):<28}")
    print(f"{'平面位置误差 RMSE (m)':<28} | {fmt(summary['position']['horizontal_rmse_m']):<28}")
    print(f"{'三维位置误差 RMSE (m)':<28} | {fmt(summary['position']['position_3d_rmse_m']):<28}")
    print(f"{'平面位置误差 P95 (m)':<28} | {fmt(summary['position']['horizontal_p95_m']):<28}")
    print(f"{'平面位置最大误差 (m)':<28} | {fmt(summary['position']['horizontal_max_m']):<28}")
    print(f"{'垂向位置误差 P95 (m)':<28} | {fmt(summary['position']['vertical_p95_m']):<28}")
    print(f"{'垂向位置最大误差 (m)':<28} | {fmt(summary['position']['vertical_max_m']):<28}")
    print(f"{'平面误差达标占比':<28} | {fmt(summary['position']['horizontal_pass_ratio'] * 100, 2)} %")
    print(f"{'垂向误差达标占比':<28} | {fmt(summary['position']['vertical_pass_ratio'] * 100, 2)} %")

    # 姿态指标
    print(f"{'横滚角误差 RMSE (deg)':<28} | {fmt(summary['attitude']['roll_rmse_deg']):<28}")
    print(f"{'俯仰角误差 RMSE (deg)':<28} | {fmt(summary['attitude']['pitch_rmse_deg']):<28}")
    print(f"{'航向角误差 RMSE (deg)':<28} | {fmt(summary['attitude']['yaw_rmse_deg']):<28}")
    print(f"{'横滚角误差 P95 (deg)':<28} | {fmt(summary['attitude']['roll_p95_deg']):<28}")
    print(f"{'俯仰角误差 P95 (deg)':<28} | {fmt(summary['attitude']['pitch_p95_deg']):<28}")
    print(f"{'航向角误差 P95 (deg)':<28} | {fmt(summary['attitude']['yaw_p95_deg']):<28}")
    print(f"{'横滚角最大误差 (deg)':<28} | {fmt(summary['attitude']['roll_max_deg']):<28}")
    print(f"{'俯仰角最大误差 (deg)':<28} | {fmt(summary['attitude']['pitch_max_deg']):<28}")
    print(f"{'航向角最大误差 (deg)':<28} | {fmt(summary['attitude']['yaw_max_deg']):<28}")
    print(f"{'横滚角达标占比':<28} | {fmt(summary['attitude']['roll_pass_ratio'] * 100, 2)} %")
    print(f"{'俯仰角达标占比':<28} | {fmt(summary['attitude']['pitch_pass_ratio'] * 100, 2)} %")
    print(f"{'航向角达标占比':<28} | {fmt(summary['attitude']['yaw_pass_ratio'] * 100, 2)} %")

    # 收敛信息
    conv = summary.get("convergence", {}).get("convergence_time_sec", None)
    print(f"{'收敛时间 (s)':<28} | {fmt(conv):<28}")

    print("-" * 80)

    # 断信漂移率
    if "outage_metrics" in summary and summary["outage_metrics"]:
        print("\n断信区间漂移率统计")
        print("-" * 80)
        print(f"{'断信标签':<16} | {'时长(s)':<10} | {'平面末端误差(m)':<16} | {'平面漂移率(m/s)':<16}")
        print("-" * 80)
        for item in summary["outage_metrics"]:
            print(
                f"{item['label']:<16} | "
                f"{fmt(item['duration_sec']):<10} | "
                f"{fmt(item['horizontal_end_m']):<16} | "
                f"{fmt(item['horizontal_drift_rate_mps']):<16}"
            )
        print("-" * 80)

        print(f"{'断信标签':<16} | {'航向末端误差(deg)':<18} | {'航向漂移率(deg/s)':<18}")
        print("-" * 80)
        for item in summary["outage_metrics"]:
            print(
                f"{item['label']:<16} | "
                f"{fmt(item['yaw_end_deg']):<18} | "
                f"{fmt(item['yaw_drift_rate_degps']):<18}"
            )
        print("-" * 80)

    # 3σ 一致性
    if "consistency_3sigma" in summary:
        c3 = summary["consistency_3sigma"]
        print("\n3σ 一致性统计")
        print("-" * 80)
        print(f"{'指标名称':<28} | {'3σ覆盖率':<28}")
        print("-" * 80)
        print(f"{'平面位置误差 3σ覆盖率':<28} | {fmt(c3['horizontal_in_3sigma_ratio'] * 100, 2)} %")
        print(f"{'横滚角误差 3σ覆盖率':<28} | {fmt(c3['roll_in_3sigma_ratio'] * 100, 2)} %")
        print(f"{'俯仰角误差 3σ覆盖率':<28} | {fmt(c3['pitch_in_3sigma_ratio'] * 100, 2)} %")
        print(f"{'航向角误差 3σ覆盖率':<28} | {fmt(c3['yaw_in_3sigma_ratio'] * 100, 2)} %")
        print("-" * 80)


if __name__ == "__main__":
    main()
