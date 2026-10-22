"""Render the manuscript figure from separately supplied numerical tables."""
from pathlib import Path
import json
import numpy as np
import pandas as pd
import matplotlib
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D

def render(data_root, output):
    data_root = Path(data_root)
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    DATA = data_root / 'heading'
    matplotlib.use('Agg')
    O = DATA
    OLD = DATA / 'external'
    OWN = DATA / 'receiver_heading'
    NEW = pd.read_csv(O / 'ACCEPTED_ERROR_SERIES.csv')
    SUM = pd.read_csv(O / 'ACCEPTANCE_SUMMARY.csv')
    SEQ = ['BY2', 'BY2H', 'BY2O']
    NAMES = ['Nominal walking', 'Large body motion', 'Antenna obstruction']
    START = {'BY2': 66, 'BY2H': 413, 'BY2O': 3186}
    COL = {'own': '#2563EB', 'input': '#64748B', 'rtk': '#E76F51', 'EXT01': '#009E9A', 'EXT02': '#8B5CF6', 'dd': '#4B5563'}

    def read(p):
        d = pd.read_csv(p)
        return d
    OWN_DATA = {s: read(OWN / f'{s}_input_heading_figure_match.csv.gz') for s in SEQ}
    RTK = {s: {v: read(OLD / f'{s}_{v}_full_saved_heading.csv.gz') for v in ['V0', 'V0E', 'V1', 'V2']} for s in SEQ}
    RAW = {s: read(OLD / f'{s}_all_methods_all_window_epochs.csv.gz') for s in SEQ}
    plt.rcParams.update({'font.family': 'sans-serif', 'font.sans-serif': ['DejaVu Sans'], 'font.size': 8, 'axes.titlesize': 8.5, 'axes.labelsize': 8, 'xtick.labelsize': 8, 'ytick.labelsize': 8, 'legend.fontsize': 8, 'axes.spines.top': False, 'axes.spines.right': False, 'axes.linewidth': 0.6, 'pdf.fonttype': 42, 'svg.fonttype': 'none', 'path.simplify': False, 'legend.frameon': False})

    def ecdf(ax, e, color, lw=1, ls='-', alpha=1, zorder=3, limit=15):
        e = np.sort(np.abs(np.asarray(e, dtype=float)))
        e = e[np.isfinite(e)]
        if len(e):
            n = np.searchsorted(e, limit, side='right')
            ax.step(np.r_[0, e[:n], limit], np.r_[0, np.arange(1, n + 1) / len(e), n / len(e)], where='post', color=color, lw=lw, ls=ls, alpha=alpha, zorder=zorder)
    fig = plt.figure(figsize=(174 / 25.4, 192 / 25.4))
    gs = fig.add_gridspec(3, 3, left=0.102, right=0.976, bottom=0.215, top=0.875, hspace=0.72, wspace=0.38, height_ratios=[1.0, 1.0, 1.08])
    a = fig.add_subplot(gs[0, :2])
    ins = fig.add_subplot(gs[0, 2])
    bg = gs[1, :].subgridspec(1, 2, wspace=0.36)
    bs = [fig.add_subplot(bg[0, j]) for j in range(2)]
    cs = [fig.add_subplot(gs[2, j]) for j in range(3)]
    own = OWN_DATA['BY2H']
    rtk = RTK['BY2H']['V0']
    a.plot(own.elapsed_s, own.input_error_deg.where(own.input_heading_valid), color=COL['input'], lw=0.8, ls='--', alpha=1, zorder=1)
    a.plot(own.elapsed_s, own.input_error_deg.where(own.accepted), color=COL['own'], lw=1.55, zorder=5)
    a.scatter(rtk.loc[rtk.valid.eq(1), 't_rel_s'] - 413, rtk.loc[rtk.valid.eq(1), 'error_valid_deg'], marker='s', s=6, facecolors='none', edgecolors=COL['rtk'], linewidths=0.6, zorder=3)
    a.axvspan(80, 100, color='#CED9DE', alpha=0.45, zorder=0)
    a.set_title('(a) Large body motion: full recording', loc='left', pad=6)
    a.set(xlim=(0, 270), ylim=(-185, 185), yticks=[-180, 0, 180], ylabel='Heading error (deg)', xlabel='Time (s)')
    a.axhline(0, color='#CCCCCC', lw=0.4, zorder=0)
    ins.set_facecolor('white')
    zz = own.elapsed_s.between(80, 100)
    zr = (rtk.t_rel_s - 413).between(80, 100) & rtk.valid.eq(1)
    ins.plot(own.loc[zz, 'elapsed_s'], own.loc[zz, 'input_error_deg'].where(own.loc[zz, 'accepted']), color=COL['own'], lw=1.4)
    ins.scatter(rtk.loc[zr, 't_rel_s'] - 413, rtk.loc[zr, 'error_valid_deg'], marker='s', s=5, facecolors='none', edgecolors=COL['rtk'], linewidths=0.6)
    values = np.r_[own.loc[zz & own.accepted, 'input_error_deg'].dropna(), rtk.loc[zr, 'error_valid_deg'].dropna()]
    lo = min(-2, float(np.min(values)) - 1)
    hi = max(2, float(np.max(values)) + 1)
    ins.set(xlim=(80, 100), ylim=(lo, hi), xticks=[80, 90, 100])
    ins.tick_params(labelsize=8, length=2, pad=1)
    ins.set_yticks([-5, 0, 5])
    ins.set_title('Turning interval', loc='left', fontsize=8.5, pad=6)
    ins.set_xlabel('Time (s)')
    ins.set_ylabel('Heading error (deg)', labelpad=1)
    for sp in ins.spines.values():
        sp.set_visible(True)
        sp.set_color('#8B969C')
        sp.set_linewidth(0.5)
    b_source = []
    b_summary = []
    for (method_index, (b, m, label)) in enumerate(zip(bs, ['EXT01', 'EXT02'], ['C-LAMBDA', 'C-WLS'])):
        d = NEW[NEW.sequence.eq('BY2H') & NEW.method_id.eq(m)]
        categories = [('Original', d.native_valid, 'error_valid_deg'), ('Added ratio screen', d.accepted, 'error_accepted_deg')]
        vectors = []
        ticks = []
        for (j, (support, mask, error_column)) in enumerate(categories):
            q = d.loc[mask & d[error_column].notna(), ['epoch_index', error_column]].copy()
            v = np.abs(q[error_column].to_numpy())
            vectors.append(v)
            jitter = np.random.default_rng(731 + method_index * 2 + j).uniform(-0.13, 0.13, len(v))
            b.scatter(np.full(len(v), j) + jitter, v, s=3.5, color=COL[m], alpha=0.45, linewidths=0, zorder=2, rasterized=False)
            ticks.append(f'{support}\n{int(mask.sum())} / {len(d)} outputs')
            b_source.extend((dict(method_id=m, support=support, epoch_index=int(k), absolute_error_deg=float(e)) for (k, e) in zip(q.epoch_index, v)))
            b_summary.append(dict(method_id=m, support=support, denominator=len(d), outputs=int(mask.sum()), scored=len(v), q25_deg=float(np.quantile(v, 0.25)), median_deg=float(np.median(v)), q75_deg=float(np.quantile(v, 0.75))))
        bp = b.boxplot(vectors, positions=[0, 1], widths=0.43, showfliers=False, patch_artist=True, medianprops={'color': '#111827', 'linewidth': 1.15}, whiskerprops={'color': COL[m], 'linewidth': 0.85}, capprops={'color': COL[m], 'linewidth': 0.85}, boxprops={'edgecolor': COL[m], 'linewidth': 1.1, 'facecolor': 'none'}, zorder=4)
        b.set_xticks([0, 1], ticks)
        b.tick_params(axis='x', length=0, pad=4)
        b.set(xlim=(-0.45, 1.45), ylim=(-5, 185), yticks=[0, 90, 180], ylabel='Absolute heading error (deg)')
        b.set_title(('(b) ' if method_index == 0 else '') + label + ' — large body motion', loc='left', pad=6)
        b.grid(axis='y', color='#E5E7EB', lw=0.45, zorder=0)
    metrics = []

    def metric(seq, method, support, d, valid, errorcol):
        e = d.loc[valid, errorcol].dropna().to_numpy()
        metrics.append(dict(sequence=seq, method=method, support=support, denominator=len(d), outputs=int(valid.sum()), scored=len(e), coverage_percent=100 * valid.mean(), rmse_deg=float(np.sqrt(np.mean(e ** 2))) if len(e) else np.nan, p95_abs_deg=float(np.percentile(np.abs(e), 95)) if len(e) else np.nan))
        return e
    for (j, (seq, name, ax)) in enumerate(zip(SEQ, NAMES, cs)):
        own = OWN_DATA[seq]
        oe = metric(seq, 'Proposed direction', 'native used observations', own, own.accepted, 'input_error_deg')
        metric(seq, 'Proposed direction', 'input before native selection', own, own.input_heading_valid, 'input_error_deg')
        rtk = RTK[seq]['V0']
        re = metric(seq, 'RTKLIB V0', 'native fixed', rtk, rtk.valid.eq(1), 'error_valid_deg')
        dd = RAW[seq][RAW[seq].method_id.eq('EXT03')].copy()
        de = metric(seq, 'DD-KF', 'native ratio fixed', dd, dd.error_ratio_fixed_deg.notna(), 'error_ratio_fixed_deg')
        curves = [(oe, COL['own'], 1.9, '-', 1, 8), (re, COL['rtk'], 1.2, '-', 1, 6), (de, COL['dd'], 1.05, '-.', 1, 2)]
        counts = [f'Ours: {int(own.accepted.sum())}/{len(own)}', f'RTKLIB: {int(rtk.valid.eq(1).sum())}/{len(rtk)}']
        for (m, label) in [('EXT01', 'C-L'), ('EXT02', 'C-WLS')]:
            d = NEW[NEW.sequence.eq(seq) & NEW.method_id.eq(m)]
            e = metric(seq, label, 'native original', d, d.native_valid, 'error_valid_deg')
            es = metric(seq, label, 'project-added ratio screen >=3', d, d.accepted, 'error_accepted_deg')
            curves += [(e, COL[m], 1.0, '--', 0.9, 1), (es, COL[m], 1.3, '-', 1, 5)]
            counts.append(f'{label}: {int(d.native_valid.sum())}→{int(d.accepted.sum())}/{len(d)}')
        counts.append(f'DD-KF: {int(dd.error_ratio_fixed_deg.notna().sum())}/{len(dd)}')
        for (e, c, lw, ls, alpha, z) in curves:
            ecdf(ax, e, c, lw, ls, alpha, z)
        ax.set(xlim=(0, 15), ylim=(0, 1.025), xticks=[0, 5, 10, 15], yticks=[0, 0.5, 1], xlabel='Absolute heading error (deg)')
        ax.set_title(f'({chr(99 + j)}) {name}', loc='left', pad=7)
        if j == 0:
            ax.set_ylabel('Cumulative fraction')
        ax.grid(axis='y', color='#E5E5E5', lw=0.4)
        inset = ax.inset_axes([0.49, 0.14, 0.48, 0.3])
        inset.set_facecolor('white')
        for (e, c, lw, ls, alpha, z) in curves:
            ecdf(inset, e, c, 0.8 if lw < 1.5 else 1.15, ls, alpha, z, limit=180)
        inset.set(xlim=(0, 180), ylim=(0, 1.025), xticks=[0, 180], yticks=[])
        inset.tick_params(labelsize=8, length=2, pad=1)
        inset.get_xticklabels()[0].set_ha('left')
        inset.get_xticklabels()[-1].set_ha('right')
        for sp in inset.spines.values():
            sp.set_visible(True)
            sp.set_linewidth(0.45)
            sp.set_color('#777777')
        ax.text(0, -0.35, 'Outputs / full window', transform=ax.transAxes, fontsize=8, weight='bold', va='top')
        for (k, txt) in enumerate(counts):
            ax.text(0, -0.47 - 0.112 * k, txt, transform=ax.transAxes, fontsize=8, va='top', color='#444444')
        for v in ['V0E', 'V1', 'V2']:
            d = RTK[seq][v]
            metric(seq, 'RTKLIB ' + v, 'native fixed', d, d.valid.eq(1), 'error_valid_deg')
    ours_handles = [Line2D([], [], color=COL['own'], lw=1.9, label='Ours: selected direction'), Line2D([], [], color=COL['input'], lw=0.85, ls='--', label='Ours: input before selection')]
    baseline_handles = [Line2D([], [], marker='s', mfc='none', mec=COL['rtk'], color=COL['rtk'], lw=1.2, ms=3.5, label='RTKLIB (default)'), Line2D([], [], color=COL['EXT01'], lw=1.3, label='C-LAMBDA'), Line2D([], [], color=COL['EXT02'], lw=1.3, label='C-WLS'), Line2D([], [], color=COL['dd'], ls='-.', lw=0.9, label='DD-KF (native ratio fixed)')]
    fig.legend(handles=ours_handles, loc='upper left', bbox_to_anchor=(0.09, 0.995), ncol=1, handlelength=2.5, labelspacing=0.6, title='Our direction observations', title_fontsize=8)
    fig.legend(handles=baseline_handles, loc='upper right', bbox_to_anchor=(0.99, 0.995), ncol=2, columnspacing=1.0, handlelength=2.2, labelspacing=0.6, title='Baselines', title_fontsize=8)
    fig.text(0.54, 0.019, 'C-LAMBDA / C-WLS in (c–e): dashed = original; solid = added ratio screen (≥3).', ha='center', fontsize=8)
    fig.canvas.draw()
    for extension in ('pdf', 'svg', 'png'):
        fig.savefig(output / ('figure04_heading.' + extension), dpi=600, facecolor='white')
    plt.close(fig)
if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--data-root', required=True)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    render(args.data_root, args.output)
