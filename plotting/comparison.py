"""Render the manuscript figure from separately supplied numerical tables."""
from pathlib import Path
import json
import numpy as np, pandas as pd
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.ticker import MaxNLocator
from PIL import Image

def render(data_root, output):
    data_root = Path(data_root)
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    DATA = data_root / 'comparison'
    plt.rcParams.update({'font.family': 'sans-serif', 'font.sans-serif': ['Arial', 'Liberation Sans', 'DejaVu Sans'], 'font.size': 8, 'axes.labelsize': 8, 'axes.titlesize': 8.5, 'xtick.labelsize': 8, 'ytick.labelsize': 8, 'legend.fontsize': 8, 'legend.frameon': False, 'axes.spines.top': False, 'axes.spines.right': False, 'axes.linewidth': 0.6, 'xtick.direction': 'out', 'ytick.direction': 'out', 'xtick.major.size': 2.5, 'ytick.major.size': 2.5, 'pdf.fonttype': 42, 'svg.fonttype': 'none', 'path.simplify': False, 'savefig.facecolor': 'white'})
    OUT = DATA
    NAV = DATA
    SEQ = ['BY2', 'BY2H', 'BY2O']
    DUR = {'BY2': 274, 'BY2H': 270, 'BY2O': 377}
    SCENE = {'BY2': 'Nominal walking', 'BY2H': 'Large body motion', 'BY2O': 'Antenna obstruction'}
    COL = {'F04': '#2563EB', 'LC01': '#E76F51', 'LC01-S': '#E76F51', 'EXT05C': '#8B5CF6', 'EXT05C-S': '#8B5CF6', 'OISAM': '#009E9A', 'LC01-BR': '#64748B', 'WEN_TC': '#197A69', 'GNC': '#8B5CF6'}
    LS = {'F04': '-', 'LC01': '--', 'LC01-S': ':', 'EXT05C': '--', 'EXT05C-S': ':', 'OISAM': '-.', 'LC01-BR': '-.'}
    NAME = {'F04': 'LegSA-GINS', 'LC01': 'Dual-pos. IEKF (LIT)', 'LC01-S': 'Dual-pos. IEKF (cal.)', 'EXT05C': 'Single-pos. IEKF (LIT)', 'EXT05C-S': 'Single-pos. IEKF (cal.)', 'OISAM': 'OiSAM (reinitialized)', 'LC01-BR': 'Baseline-retaining IEKF'}
    METHODS = ['F04', 'LC01', 'LC01-S', 'EXT05C', 'EXT05C-S', 'OISAM']

    def read(p):
        p = Path(p)
        d = pd.read_csv(p)
        return d

    def line(ax, t, y, gap=0.101, **kw):
        t = np.asarray(t)
        y = np.asarray(y)
        for ids in np.split(np.arange(len(t)), np.flatnonzero(np.diff(t) > gap) + 1):
            if len(ids):
                ax.plot(t[ids], y[ids], **kw)

    def panel(ax, k, title):
        ax.set_title(f'({k}) {title}', loc='left', pad=5)

    def tidy(ax, n=4):
        ax.xaxis.set_major_locator(MaxNLocator(n))
        ax.yaxis.set_major_locator(MaxNLocator(n))
        ax.tick_params(pad=2)

    def export(fig, name, *metadata):
        for extension in ('pdf', 'svg', 'png'):
            fig.savefig(output / (name + '.' + extension), dpi=600, facecolor='white')
        plt.close(fig)

    def natural_common():
        return ({s: read(NAV / (s + '_COMMON_HEADING.csv.gz')) for s in SEQ}, pd.DataFrame())

    def fig9():
        (tables, tab) = natural_common()
        fig = plt.figure(figsize=(174 / 25.4, 212 / 25.4))
        grid = fig.add_gridspec(4, 2, left=0.1, right=0.982, bottom=0.11, top=0.89, width_ratios=[1.5, 1], height_ratios=[1, 1, 1, 1.15], wspace=0.34, hspace=0.86)
        axs = np.array([[fig.add_subplot(grid[r, c]) for c in range(2)] for r in range(4)])
        for (row, s) in enumerate(SEQ):
            q = tables[s]
            ax = axs[row, 0]
            ec = axs[row, 1]
            for m in METHODS[1:] + ['F04']:
                line(ax, q.elapsed_s, q[m], gap=1.001, color=COL[m], ls=LS[m], lw=1.15 if m == 'F04' else 0.8, zorder=6 if m == 'F04' else 3)
                v = np.sort(abs(q[m].to_numpy()))
                ec.step(np.r_[0, v], np.r_[0, np.arange(1, len(v) + 1) / len(v)], where='post', color=COL[m], ls=LS[m], lw=1.2 if m == 'F04' else 1, zorder=6 if m == 'F04' else 3)
            ax.axhline(0, color='#B8B8B8', lw=0.5)
            ax.set(xlim=(0, DUR[s]), xlabel='Elapsed time (s)', ylabel='Heading error (°)')
            tidy(ax, 4)
            if s == 'BY2O':
                for (lo, hi) in [(183.94, 225.95), (309.94, 322.94)]:
                    ax.axvspan(lo, hi, color='#E7E7E7', zorder=0)
            ec.set(xlim=(0, max((float(np.max(abs(q[m]))) for m in METHODS)) * 1.03), ylim=(0, 1.02), xlabel='Absolute heading error (°)', ylabel='Cumulative probability')
            ec.set_yticks([0, 0.5, 1])
            ec.xaxis.set_major_locator(MaxNLocator(4))
            panel(ax, chr(97 + row * 2), SCENE[s])
            panel(ec, chr(98 + row * 2), f'Common epochs: n = {len(q)}')
            if ec.get_xlim()[1] > 20:
                ins = ec.inset_axes([0.46, 0.17, 0.49, 0.42])
                for m in METHODS[1:] + ['F04']:
                    v = np.sort(abs(q[m].to_numpy()))
                    ins.step(np.r_[0, v], np.r_[0, np.arange(1, len(v) + 1) / len(v)], where='post', color=COL[m], ls=LS[m], lw=0.9)
                ins.set(xlim=(0, 10), ylim=(0, 1))
                ins.set_xticks([0, 5, 10])
                ins.set_yticks([1])
                ins.tick_params(labelsize=8, pad=1)
                ins.set_title('0–10°', fontsize=8, pad=1)
        ax = axs[3, 0]
        pairs = read(NAV / 'F09_ALL18_INTERRUPTION_CASES.csv')
        duration_origin = 196.2
        for m in ['LC01', 'EXT05C', 'LC01-BR', 'F04']:
            d = read(NAV / (m + '_A2_20s_seed00.csv.gz'))
            d = d[d.time.between(duration_origin - 5, duration_origin + 35)]
            line(ax, d.time - duration_origin, d.yaw_err_deg, color=COL[m], ls=LS[m], lw=1.15 if m == 'F04' else 0.8, zorder=5 if m == 'F04' else 3)
        ax.axvspan(0, 20, color='#E7E7E7', zorder=0)
        ax.axvline(20, color='#888888', lw=0.6)
        ax.axhline(0, color='#B8B8B8', lw=0.5)
        ax.set(xlim=(-5, 35), xlabel='Time from position interruption (s)', ylabel='Heading error (°)')
        tidy(ax, 4)
        panel(ax, 'g', 'Position interruption: 20 s')
        ax.annotate('Positions return', xy=(20, 0.99), xytext=(0.98, 1.1), xycoords=('data', 'axes fraction'), textcoords='axes fraction', ha='right', va='bottom', fontsize=8, arrowprops={'arrowstyle': '-', 'lw': 0.6, 'color': '#666666'})
        ec = axs[3, 1]
        mm = ['F04', 'LC01', 'EXT05C', 'LC01-BR']
        full_names = ['LegSA-GINS', 'Dual-position IEKF', 'Single-position IEKF', 'IEKF with retained baseline']
        for (i, m) in enumerate(mm):
            vals = pairs[m + '_yaw_rmse_deg'].to_numpy()
            assert len(vals) == 18
            ys = np.linspace(-0.09, 0.09, len(vals)) + i
            ec.scatter(vals, ys, s=14, color=COL[m], edgecolors='white', linewidth=0.25, zorder=3)
            ec.plot([np.median(vals)] * 2, [i - 0.14, i + 0.14], color='#111827', lw=1.5, zorder=4)
            ec.text(0.015, i - 0.24, full_names[i], transform=ec.get_yaxis_transform(), ha='left', va='bottom', fontsize=8)
        ec.set_yticks([])
        ec.set_ylim(3.48, -0.75)
        ec.set_xlabel('Whole-run heading RMSE (°)')
        ec.set_xlim(0, None)
        ec.xaxis.set_major_locator(MaxNLocator(4))
        panel(ec, 'h', '18 position-interruption cases')
        ec.text(0.5, -0.43, '18 cases per method\nDots: cases; black mark: median', transform=ec.transAxes, ha='center', va='top', fontsize=8)
        handles = [Line2D([], [], color=COL[m], ls=LS[m], lw=1.2, label=NAME[m]) for m in METHODS] + [Line2D([], [], color=COL['LC01-BR'], ls=LS['LC01-BR'], lw=1.2, label='Baseline-retaining IEKF (g,h)')]
        fig.legend(handles=handles, loc='upper center', bbox_to_anchor=(0.54, 0.993), ncol=2, columnspacing=1.3, handlelength=2.8, labelspacing=0.35)
        export(fig, 'figure10_comparison')
    fig9()
if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--data-root', required=True)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    render(args.data_root, args.output)
