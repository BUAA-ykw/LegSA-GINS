"""Render the manuscript figure from separately supplied numerical tables."""
from pathlib import Path
import json
import numpy as np, pandas as pd
import matplotlib
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.text import Text
from PIL import Image

def render(data_root, output):
    data_root = Path(data_root)
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    DATA = data_root / 'weighting'
    matplotlib.use('Agg')
    P = DATA
    S = DATA
    cases = json.loads((S / 'SELECTED_CASES.json').read_text())
    bins = pd.read_csv(S / 'tables/ONE_SECOND_DISPLAY_SUMMARIES.csv')
    assert len(bins) == 1440
    plt.rcParams.update({'font.family': 'sans-serif', 'font.sans-serif': ['Arial', 'Liberation Sans', 'DejaVu Sans'], 'font.size': 8, 'axes.labelsize': 8, 'axes.titlesize': 8.5, 'xtick.labelsize': 8, 'ytick.labelsize': 8, 'legend.fontsize': 8, 'legend.frameon': False, 'axes.spines.top': False, 'axes.spines.right': False, 'axes.linewidth': 0.65, 'xtick.direction': 'out', 'ytick.direction': 'out', 'xtick.major.size': 2.5, 'ytick.major.size': 2.5, 'pdf.fonttype': 42, 'svg.fonttype': 'none', 'path.simplify': False})
    BLUE = '#175C9E'
    OFF = '#C66B3D'
    GRAY = '#4F565E'
    TEAL = '#238B8A'

    def trace(ax, case, source, color, ls='-', z=3):
        d = bins[bins.case.eq(case) & bins.source.eq(source)].sort_values('interval_start_s')
        assert len(d) == 120
        edge = np.r_[d.interval_start_s.to_numpy(), d.interval_end_s.iloc[-1]]
        lo = np.r_[d.minimum.to_numpy(), d.minimum.iloc[-1]]
        hi = np.r_[d.maximum.to_numpy(), d.maximum.iloc[-1]]
        ax.fill_between(edge, lo, hi, step='post', color=color, alpha=0.1, linewidth=0, zorder=1, rasterized=True)
        ax.plot((d.interval_start_s + d.interval_end_s) / 2, d['median'], color=color, ls=ls, lw=1.05 if source == 'weighting_off' else 0.98, zorder=z)
        return d
    fig = plt.figure(figsize=(174 / 25.4, 198 / 25.4))
    widths = [1.15 / 3] * 3 + [0.4] + [0.8 / 2] * 2 + [0.4] + [1.45 / 4] * 4
    gs = fig.add_gridspec(3, 11, width_ratios=widths, left=0.1, right=0.985, bottom=0.11, top=0.835, hspace=0.56, wspace=0)
    axs = [[fig.add_subplot(gs[r, 0:3]), fig.add_subplot(gs[r, 4:6]), fig.add_subplot(gs[r, 7:11])] for r in range(3)]
    fig.legend(handles=[Line2D([], [], color=GRAY, lw=1, label='Added input change'), Line2D([], [], color=TEAL, ls='--', lw=1, label='Reported uncertainty')], loc='upper left', bbox_to_anchor=(0.035, 0.999), ncol=1, fontsize=8, handlelength=2.5)
    fig.legend(handles=[Line2D([], [], color=OFF, ls='--', lw=1.1, label='Without source weighting'), Line2D([], [], color=BLUE, lw=1.1, label='LegSA-GINS')], loc='upper right', bbox_to_anchor=(0.99, 0.999), ncol=1, fontsize=8, handlelength=2.5)
    headers = ['Input degradation', 'Source covariance', 'Heading response']
    subheads = ['Change from clean input\n1-s median / range', 'Actual updates\nMedian · IQR · range', '1-s median / range\n274-s RMSE: yaw (°) / H (m)']
    for j in range(3):
        pos = axs[0][j].get_position()
        xc = pos.x0 + pos.width / 2
        fig.text(xc, 0.928, headers[j], ha='center', fontweight='bold', fontsize=8.5)
        fig.text(xc, 0.889, subheads[j], ha='center', va='center', fontsize=8, linespacing=1.2)
    labels = ['Position noise', 'Heading quality', 'Body-velocity noise']
    params = ['σH/V = 3/5 m', 'σ = 10°; low std', 'σ = 1 m/s']
    for (r, case) in enumerate(cases):
        typ = case['type']
        (a, b, c) = axs[r]
        d = trace(a, typ, 'input_change', GRAY)
        trace(a, typ, 'reported_uncertainty', TEAL, '--', 4)
        ymax = float(d.maximum.max())
        a.set(xlim=(0, 120), ylim=(0, ymax * 1.28), xlabel='Time (s)', ylabel=f"Input change ({case['input_unit']})")
        a.set_xticks([0, 60, 120])
        a.yaxis.set_major_locator(plt.MaxNLocator(4))
        a.set_title(f'({chr(97 + r * 3)}) {labels[r]}', loc='left', fontweight='bold', pad=7)
        a.text(0.97, 0.98, params[r], transform=a.transAxes, ha='right', va='top', fontsize=8)
        q = pd.read_csv(S / 'tables' / f'{typ}_actual_covariance_summary.csv')
        for (i, z) in q.iterrows():
            b.plot([z.minimum, z.maximum], [i, i], color='#7A838D', lw=0.7)
            b.plot([z.q25, z.q75], [i, i], color='#7A838D', lw=4, solid_capstyle='butt')
            b.scatter(z['median'], i, s=22, color=BLUE, zorder=4)
        b.axvline(1, color='#C2C8CE', ls=':', lw=0.6, zorder=0)
        b.set(xlim=(0, 11.2), ylim=(2.45, -0.45), xlabel='Multiplier (×)')
        b.set_yticks([0, 1, 2], ['Position', 'Body v', 'Heading'])
        b.set_xticks([1, 5, 10])
        b.set_title(f'({chr(98 + r * 3)})', loc='left', fontweight='bold', pad=7)
        trace(c, typ, 'weighting_off', OFF, '--', 5)
        trace(c, typ, 'full', BLUE, '-', 6)
        c.axhline(0, color='#A8AFB7', ls=':', lw=0.6, zorder=0)
        c.set(xlim=(0, 120), xlabel='Time (s)', ylabel='Heading error (deg)')
        c.set_xticks([0, 60, 120])
        c.yaxis.set_major_locator(plt.MaxNLocator(4))
        metric = f"{case['off_yaw_rmse_deg']:.4g}→{case['full_yaw_rmse_deg']:.4g}° / {case['off_H_rmse_m']:.4g}→{case['full_H_rmse_m']:.4g} m"
        c.set_title(f'({chr(99 + r * 3)}) {metric}', loc='left', fontsize=8, fontweight='normal', pad=7)
        if typ == 'D53':
            c.text(0.5, -0.31, 'Near-neutral yaw: ΔRMSE = −0.0008°', transform=c.transAxes, ha='center', va='top', fontsize=8)
    name = 'F08_heading_quality_scaling'
    fig.canvas.draw()
    for extension in ('pdf', 'svg', 'png'):
        fig.savefig(output / ('figure09_weighting.' + extension), dpi=600, facecolor='white')
    plt.close(fig)
if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--data-root', required=True)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    render(args.data_root, args.output)
