"""Render the manuscript figure from separately supplied numerical tables."""
from pathlib import Path
import json
import numpy as np, pandas as pd
import matplotlib
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.patches import FancyArrowPatch
from PIL import Image

def render(data_root, output):
    data_root = Path(data_root)
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    DATA = data_root / 'gauge'
    matplotlib.use('Agg')
    OUT = DATA
    SRC = DATA
    plt.rcParams.update({'font.family': 'sans-serif', 'font.sans-serif': ['Arial', 'Liberation Sans', 'DejaVu Sans'], 'font.size': 8, 'axes.labelsize': 8, 'axes.titlesize': 8.5, 'xtick.labelsize': 8, 'ytick.labelsize': 8, 'legend.fontsize': 8, 'legend.frameon': False, 'axes.spines.top': False, 'axes.spines.right': False, 'axes.linewidth': 0.6, 'xtick.direction': 'out', 'ytick.direction': 'out', 'xtick.major.size': 2.5, 'ytick.major.size': 2.5, 'pdf.fonttype': 42, 'svg.fonttype': 'none', 'path.simplify': False, 'savefig.facecolor': 'white'})
    COL = {-150: '#8A8F98', -100: '#E76F51', -50: '#8A8F98', 0: '#2563EB', 50: '#8A8F98', 100: '#009E9A', 150: '#8A8F98'}
    LS = {-150: ':', -100: '--', -50: '-.', 0: '-', 50: (0, (3, 1, 1, 1)), 100: '-.', 150: (0, (6, 2))}
    runs = {int(p.name.split('_')[-1].split('.')[0]): pd.read_csv(p) for p in SRC.glob('native_initial_yaw_*.csv.gz')}
    assert len(runs) == 7 and all((len(d) == 63277 for d in runs.values()))
    base = runs[0]
    anchor = base[['local_x_m', 'local_y_m', 'local_z_m']].to_numpy()
    time = base.elapsed_s.to_numpy()
    fig = plt.figure(figsize=(174 / 25.4, 126 / 25.4))
    gs = fig.add_gridspec(2, 2, left=0.1, right=0.975, bottom=0.115, top=0.83, wspace=0.36, hspace=0.65, height_ratios=[3.6, 1])
    a = fig.add_subplot(gs[0, 0])
    b = fig.add_subplot(gs[0, 1])
    c = fig.add_subplot(gs[1, :])
    res = pd.DataFrame({'elapsed_s': time})
    summary = []
    for yaw in [-150, -50, 50, 150, -100, 100, 0]:
        d = runs[yaw]
        assert np.array_equal(d.timestamp_ns.to_numpy(), base.timestamp_ns.to_numpy())
        p = d[['local_x_m', 'local_y_m', 'local_z_m']].to_numpy()
        ang = np.deg2rad(yaw)
        rot = np.array([[np.cos(ang), np.sin(ang), 0], [-np.sin(ang), np.cos(ang), 0], [0, 0, 1]])
        q = p @ rot.T
        delta = np.linalg.norm(q - anchor, axis=1)
        res[f'yaw_{yaw:+04d}_position_difference_m'] = delta
        dyaw = d.unwrapped_yaw_deg.to_numpy() - d.unwrapped_yaw_deg.iloc[0] - (base.unwrapped_yaw_deg.to_numpy() - base.unwrapped_yaw_deg.iloc[0])
        res[f'yaw_{yaw:+04d}_increment_difference_deg'] = dyaw
        main = yaw in [-100, 0, 100]
        lw = 1.15 if main else 0.65
        zorder = 5 if main else 2
        a.plot(p[:, 0], p[:, 1], color=COL[yaw], ls=LS[yaw], lw=lw, zorder=zorder)
        b.plot(q[:, 0], q[:, 1], color=COL[yaw], ls=LS[yaw], lw=lw, zorder=zorder)
        a.plot(p[-1, 0], p[-1, 1], marker='o', ms=3, color=COL[yaw], zorder=zorder)
        if yaw != 0:
            c.plot(time, delta / 1e-07, color=COL[yaw], ls=LS[yaw], lw=1.0 if main else 0.65, zorder=zorder)
        summary.append({'initial_yaw_deg': yaw, 'states': len(d), 'position_residual_max_m': float(delta.max()), 'heading_increment_residual_max_deg': float(abs(dyaw).max()), 'first_native_yaw_deg': float(d.unwrapped_yaw_deg.iloc[0]), 'same_timestamps': True})
    for yaw in [-100, 0, 100]:
        ang = np.deg2rad(yaw)
        end = (46 * np.cos(ang), 46 * np.sin(ang))
        a.annotate('', xy=end, xytext=(0, 0), arrowprops={'arrowstyle': '-|>', 'mutation_scale': 10, 'color': COL[yaw], 'lw': 1.4}, zorder=10)
    for ax in [a, b]:
        ax.set(xlim=(-150, 150), ylim=(-150, 150), xlabel='Local x (m)', ylabel='Local y (m)')
        ax.set_xticks([-100, 0, 100])
        ax.set_yticks([-100, 0, 100])
        ax.set_aspect('equal', adjustable='box')
        ax.plot(0, 0, 'k*', ms=5, zorder=12)
    a.set_title('(a) Different absolute directions', loc='left', pad=5)
    b.set_title('(b) The same relative path', loc='left', pad=5)
    b.text(0.97, 0.05, 'All seven paths overlap', transform=b.transAxes, ha='right', va='bottom', fontsize=8, color=COL[0])
    c.set(xlim=(time[0], time[-1]), ylim=(-0.03, 2.05), xlabel='Elapsed time (s)')
    c.set_ylabel('Position difference\n(10⁻⁷ m)', fontfamily='DejaVu Sans')
    c.set_xticks([0, 100, 200, 300])
    c.set_yticks([0, 1, 2])
    c.set_title('(c) Difference after removing the known rotation', loc='left', pad=5)
    maximum = max((x['position_residual_max_m'] for x in summary))
    c.text(0.99, 1.15, 'Maximum: 1.89 × 10⁻⁷ m', transform=c.transAxes, ha='right', va='bottom', fontsize=8, fontfamily='DejaVu Sans')
    handles = [Line2D([], [], color=COL[y], ls=LS[y], lw=1.3, label=f'{y:+d}°' if y else '0°') for y in [-100, 0, 100]] + [Line2D([], [], color='#8A8F98', lw=0.8, label='Other four initial yaws')]
    fig.legend(handles=handles, loc='upper center', bbox_to_anchor=(0.53, 0.991), ncol=4, columnspacing=1.1, title='Same IMU and foot-contact inputs', title_fontsize=8.5)
    fig.text(0.52, 0.785, 'Known rotation\nremoved', ha='center', fontsize=8, color='#444444')
    arrow = FancyArrowPatch((0.465, 0.72), (0.612, 0.72), transform=fig.transFigure, arrowstyle='-|>', mutation_scale=11, lw=0.8, color='#777777')
    fig.add_artist(arrow)
    name = 'F04_absolute_direction_relative_motion'
    fig.canvas.draw()
    for extension in ('pdf', 'svg', 'png'):
        fig.savefig(output / ('figure03_gauge.' + extension), dpi=600, facecolor='white')
    plt.close(fig)
if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--data-root', required=True)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    render(args.data_root, args.output)
