"""Render the manuscript figure from separately supplied numerical tables."""
from pathlib import Path
import json
import numpy as np, pandas as pd
import matplotlib
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.patches import Rectangle
from PIL import Image

def render(data_root, output):
    data_root = Path(data_root)
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    DATA = data_root / 'routes'
    matplotlib.use('Agg')
    O = DATA
    D = DATA
    R = data_root
    R3 = DATA
    R4 = DATA
    YAW = data_root / 'comparison'
    SEQ = ['BY2', 'BY2H', 'BY2O']
    SCENE = {'BY2': 'Nominal walking', 'BY2H': 'Large body motion', 'BY2O': 'Antenna obstruction'}
    COL = {'REFERENCE_POI': '#151515', 'F04': '#154C8A', 'LC01': '#B65A38', 'LC01-S': '#B65A38', 'EXT05C': '#7B5C8E', 'EXT05C-S': '#7B5C8E', 'OISAM_RESTART_POI': '#378177', 'WEN_TC': '#66786D', 'GNC': '#877B66'}
    LS = {'REFERENCE_POI': (0, (4, 2)), 'F04': '-', 'LC01': '--', 'LC01-S': ':', 'EXT05C': '--', 'EXT05C-S': ':', 'OISAM_RESTART_POI': '-.', 'WEN_TC': (0, (6, 2)), 'GNC': '-'}
    NAME = {'REFERENCE_POI': 'Truth', 'F04': 'LegSA-GINS', 'LC01': 'Dual-pos. IEKF (LIT)', 'LC01-S': 'Dual-pos. IEKF (cal.)', 'EXT05C': 'Single-pos. IEKF (LIT)', 'EXT05C-S': 'Single-pos. IEKF (cal.)', 'OISAM_RESTART_POI': 'OiSAM (reinitialized)', 'WEN_TC': 'TC-FGO', 'GNC': 'GNC-FGO'}
    POI = ['LC01', 'LC01-S', 'EXT05C', 'EXT05C-S', 'OISAM_RESTART_POI', 'F04']
    ALL = ['WEN_TC', 'GNC'] + POI
    plt.rcParams.update({'font.family': 'sans-serif', 'font.sans-serif': ['Arial', 'Liberation Sans', 'DejaVu Sans'], 'font.size': 8, 'axes.labelsize': 8, 'axes.titlesize': 8.5, 'xtick.labelsize': 8, 'ytick.labelsize': 8, 'legend.fontsize': 8, 'legend.frameon': False, 'axes.spines.top': False, 'axes.spines.right': False, 'axes.linewidth': 0.6, 'xtick.direction': 'out', 'ytick.direction': 'out', 'xtick.major.size': 2.5, 'ytick.major.size': 2.5, 'pdf.fonttype': 42, 'svg.fonttype': 'none', 'path.simplify': False, 'savefig.facecolor': 'white'})
    cache = {}
    uses = []

    def data(s, m):
        key = (s, m)
        if key not in cache:
            path = (R4 if m == 'OISAM_RESTART_POI' else R3) / f'{s}__{m}.csv.gz'
            cache[key] = pd.read_csv(path)
            assert np.all(np.diff(cache[key].time_s.to_numpy()) > 0)
        return cache[key].copy()

    def route(ax, d, m, origin=(0, 0), scale=1, detail=False):
        v = d.valid.to_numpy(bool)
        x = (d.east_m.to_numpy() - origin[0]) * scale
        y = (d.north_m.to_numpy() - origin[1]) * scale
        x[~v] = np.nan
        y[~v] = np.nan
        gap = 1.001 if m.startswith('OISAM') or m in ['WEN_TC', 'GNC'] else 0.101
        br = np.flatnonzero(np.diff(d.time_s) > gap) + 1
        if 'block_id' in d:
            br = np.unique(np.r_[br, np.flatnonzero(np.diff(d.block_id.fillna(-1)) != 0) + 1])
        ax.plot(np.insert(x, br, np.nan), np.insert(y, br, np.nan), color=COL[m], ls=LS[m], lw=1.25 if m == 'F04' else 0.95 if m == 'REFERENCE_POI' else 0.82, rasterized=detail, zorder=6 if m == 'F04' else 5 if m == 'REFERENCE_POI' else 3)
        if m == 'GNC' and len(d):
            marker = np.arange(0, len(d), 20)
            marker = marker[v[marker]]
            ax.scatter(x[marker], y[marker], color=COL[m], marker='D', s=6, linewidths=0, zorder=4)
    (fig, axs) = plt.subplots(3, 3, figsize=(174 / 25.4, 174 / 25.4), gridspec_kw={'width_ratios': [1, 1, 1.15]})
    fig.subplots_adjust(left=0.08, right=0.987, bottom=0.085, top=0.815, wspace=0.5, hspace=0.56)
    windows = []
    for (i, s) in enumerate(SEQ):
        (lo, hi) = (40.0, 140.0) if s != 'BY2O' else (154.945, 254.945)
        (a, b) = (88.0, 92.0) if s != 'BY2O' else (228.0, 233.0)
        center = (a + b) / 2
        ref = data(s, 'REFERENCE_POI')
        t = ref.elapsed_s.to_numpy()
        assert np.all(np.diff(t) > 0)
        centre = np.array([np.interp(center, t, ref[k]) for k in ['east_m', 'north_m']])
        ax = axs[i, 0]
        for m in ALL + ['REFERENCE_POI']:
            q = data(s, m)
            route(ax, q, m)
        ax.set_aspect('equal', adjustable='datalim')
        ax.set(xlabel='East (m)' if i == 2 else '', ylabel='North (m)')
        ax.locator_params(nbins=3)
        ax.set_title(f'({chr(97 + i * 3)}) ' + SCENE[s], loc='left', pad=5, fontsize=8)
        ax = axs[i, 1]
        for m in POI + ['REFERENCE_POI']:
            q = data(s, m)
            q = q[q.elapsed_s.between(a, b)]
            route(ax, q, m, origin=centre, scale=100, detail=True)
        ax.set(xlim=(-40, 40), ylim=(-35, 35), xlabel='East offset (cm)' if i == 2 else '', ylabel='North offset (cm)')
        ax.set_aspect('equal', adjustable='datalim')
        ax.set_xticks([-30, 0, 30])
        ax.set_yticks([-30, 0, 30])
        ax.set_title(f'({chr(98 + i * 3)}) ' + f'POI detail · {a:g}–{b:g} s', loc='left', pad=5, fontsize=8)
        ax = axs[i, 2]
        q = pd.read_csv(YAW / (s + '_COMMON_HEADING.csv.gz'))
        z = q[q.elapsed_s.between(lo, hi)]
        tt = z.elapsed_s.to_numpy()
        br = np.flatnonzero(np.diff(tt) > 1.001) + 1
        for m in ['LC01', 'LC01-S', 'EXT05C', 'EXT05C-S', 'OISAM_RESTART_POI', 'F04']:
            field = 'OISAM' if m == 'OISAM_RESTART_POI' else m
            ax.plot(np.insert(tt, br, np.nan), np.insert(z[field].to_numpy(), br, np.nan), color=COL[m], ls=LS[m], lw=1.1 if m == 'F04' else 0.75, zorder=6 if m == 'F04' else 3)
        ax.axhline(0, color='#B0B0B0', lw=0.5)
        ax.axvspan(a, b, color='#DDE7F2', zorder=0)
        if s == 'BY2O':
            ax.axvspan(183.94, 225.95, color='#E5E7EB', zorder=0)
        ax.set(xlim=(lo, hi), xlabel='Elapsed time (s)' if i == 2 else '', ylabel='Heading error (°)')
        ax.locator_params(nbins=3)
        ax.set_title(f'({chr(99 + i * 3)}) Heading error', loc='left', pad=5, fontsize=8)
        windows.append({'sequence': s, 'heading_start_s': lo, 'heading_end_s': hi, 'heading_duration_s': hi - lo, 'detail_start_s': a, 'detail_end_s': b, 'detail_centre_s': center, 'detail_east_reference_m': centre[0], 'detail_north_reference_m': centre[1], 'selection': 'BY2/H: prior fixed88–92s reference-defined turning neighbourhood. BY2O: first5s recovery interval after registered obstruction with reference displacement>=2m and median1s chord speed>=0.3m/s; no estimate error inspected.', 'local_methods_point': 'POI_MIDPOINT', 'full_route_point_note': 'TC/GNC at native registered GNSS1; all other main trajectories at POI; overview only, not same-point precision ranking.'})
    legend = ['REFERENCE_POI', 'F04', 'LC01', 'LC01-S', 'EXT05C', 'EXT05C-S', 'OISAM_RESTART_POI', 'WEN_TC', 'GNC']
    handles = [Line2D([], [], color=COL[m], ls=LS[m], marker='D' if m == 'GNC' else None, markersize=3, lw=1, label=NAME[m]) for m in legend]
    fig.legend(handles=handles, loc='upper center', bbox_to_anchor=(0.53, 0.998), ncol=3, columnspacing=1.0, handlelength=2.5, labelspacing=0.6)
    fig.canvas.draw()
    for (i, w) in enumerate(windows):
        w['rendered_detail_xlim_cm'] = list(axs[i, 1].get_xlim())
        w['rendered_detail_ylim_cm'] = list(axs[i, 1].get_ylim())
        (x0, x1) = w['rendered_detail_xlim_cm']
        (y0, y1) = w['rendered_detail_ylim_cm']
        centre = np.array([w['detail_east_reference_m'], w['detail_north_reference_m']])
        axs[i, 0].add_patch(Rectangle(centre + [x0, y0] / np.array([100, 100]), (x1 - x0) / 100, (y1 - y0) / 100, fill=False, edgecolor='#333333', lw=0.8, zorder=10))
        axs[i, 0].annotate('Zoom', xy=centre, xytext=(0.55, 0.4), textcoords='axes fraction', fontsize=8, ha='center', arrowprops={'arrowstyle': '->', 'lw': 0.6, 'color': '#444444'}, zorder=11)
    fig.canvas.draw()
    name = 'F03_integrated_routes_and_heading'
    ids = list('abcdefghi')
    for extension in ('pdf', 'svg', 'png'):
        fig.savefig(output / ('figure05_routes.' + extension), dpi=600, facecolor='white')
    plt.close(fig)
if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--data-root', required=True)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    render(args.data_root, args.output)
