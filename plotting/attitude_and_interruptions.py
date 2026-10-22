"""Render the manuscript figure from separately supplied numerical tables."""
from pathlib import Path
import json
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.text import Text
from PIL import Image

def render(data_root, output):
    data_root = Path(data_root)
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    DATA = data_root / 'aiding'
    plt.rcParams.update({'font.family': 'sans-serif', 'font.sans-serif': ['Arial', 'Liberation Sans', 'DejaVu Sans'], 'font.size': 8, 'axes.labelsize': 8, 'axes.titlesize': 8.5, 'xtick.labelsize': 8, 'ytick.labelsize': 8, 'legend.fontsize': 8, 'legend.frameon': False, 'axes.spines.top': False, 'axes.spines.right': False, 'axes.linewidth': 0.65, 'xtick.direction': 'out', 'ytick.direction': 'out', 'xtick.major.size': 2.5, 'ytick.major.size': 2.5, 'pdf.fonttype': 42, 'svg.fonttype': 'none', 'path.simplify': False})
    OUT = DATA
    BASE = DATA / 'tables'
    PREV = DATA / 'tables'
    BLUE = '#2563EB'
    ORANGE = '#E76F51'
    PURPLE = '#8B5CF6'
    GRAY = '#5F6B7A'
    ST = {'F01': (GRAY, ':', 'Basic GNSS/INS'), 'F03': (ORANGE, '--', 'Basic + heading'), 'F04': (BLUE, '-', 'LegSA-GINS'), 'A05': (PURPLE, '--', 'Without tilt aiding'), 'A06': (PURPLE, '--', 'Without body velocity'), 'A04': (PURPLE, '--', 'Without source weighting'), 'INPUT': (PURPLE, ':', 'Direction observation')}
    SEQ = ['BY2', 'BY2H', 'BY2O']
    START = {'BY2': 66, 'BY2H': 413, 'BY2O': 3186}
    SCENES = ['Nominal walking', 'Large body motion', 'Antenna obstruction']
    AV = pd.read_csv(BASE / 'series_index.csv')
    CORE = pd.read_csv(PREV / 'case_metrics.csv')
    ADD = pd.read_csv(PREV / 'interruption_metrics.csv')
    PAIR = pd.read_csv(PREV / 'paired_metrics.csv')
    NAT = pd.read_csv(PREV / 'natural_metrics.csv')
    DEFS = pd.read_csv(PREV / 'disturbance_definitions.csv')
    CACHE = {}

    def series(run):
        if run not in CACHE:
            CACHE[run] = pd.read_csv(DATA / 'series' / f'{run}.csv.gz')
        return CACHE[run]

    def native(seq, m):
        r = AV[(AV.sequence_id == seq) & (AV.method_id == m)]
        r = r[r.case_id == 'C00_clean_normal'] if seq == 'BY2' else r[r.domain == 'SEQUENCE']
        assert len(r) == 1
        return series(r.run_id.iloc[0])

    def runrow(case, m):
        r = CORE[(CORE.case_id == case) & (CORE.internal_method_id == m)]
        assert len(r) == 1
        return r.iloc[0]

    def curve(ax, t, y, m, lw=0.8):
        t = np.asarray(t)
        y = np.asarray(y)
        (c, ls, _) = ST[m]
        for inds in np.split(np.arange(len(t)), np.flatnonzero(np.diff(t) > 0.1) + 1):
            if len(inds):
                ax.plot(t[inds], y[inds], color=c, ls=ls, lw=lw, rasterized=True, zorder=6 if m == 'F04' else 5 if m in ['F03', 'A04', 'A05', 'A06'] else 4)

    def zero(ax):
        ax.axhline(0, color='#AAAAAA', lw=0.5, ls=':', zorder=-3)

    def title(ax, label, text):
        ax.set_title(f'({label}) {text}', loc='left', fontweight='bold', pad=7)

    def legend(fig, methods, y=0.997, ncol=None):
        fig.legend(handles=[Line2D([], [], color=ST[m][0], ls=ST[m][1], lw=1.2, label=ST[m][2]) for m in methods], loc='upper center', bbox_to_anchor=(0.53, y), ncol=ncol or len(methods), handlelength=2.6, columnspacing=1.4)

    def save(fig, name, *metadata):
        for extension in ('pdf', 'svg', 'png'):
            fig.savefig(output / (name + '.' + extension), dpi=600, facecolor='white')
        plt.close(fig)

    def fig6():
        events_path = OUT / 'tables/F06_F03_SHARED_EVENTS.csv'
        if events_path.exists():
            ev = pd.read_csv(events_path)
            windows = {r.sequence: (float(r.elapsed_start_s), float(r.elapsed_end_s), r.label) for (_, r) in ev.iterrows()}
        else:
            windows = {'BY2': (80, 100, 'Reference-defined turning interval'), 'BY2H': (80, 100, 'Reference-defined turning interval'), 'BY2O': (173.94, 235.95, 'Registered obstruction with 10-s context')}
        (fig, axs) = plt.subplots(3, 3, figsize=(174 / 25.4, 174 / 25.4), gridspec_kw={'height_ratios': [1.3, 1.2, 0.85]})
        fig.subplots_adjust(left=0.1, right=0.985, bottom=0.07, top=0.915, wspace=0.34, hspace=0.58)
        legend(fig, ['F01', 'F03', 'F04'], y=1.0)
        rows = []
        for (j, (seq, scene)) in enumerate(zip(SEQ, SCENES)):
            (lo, hi, label) = windows[seq]
            ds = {m: native(seq, m) for m in ['F01', 'F03', 'F04']}
            ax = axs[0, j]
            for (m, d) in ds.items():
                curve(ax, d.time - START[seq], d.yaw_err_deg, m, 0.65 if m == 'F04' else 0.85)
            zero(ax)
            ax.axvspan(lo, hi, color='#EEE9DF', zorder=-4)
            ax.set(xlim=(0, 274 if seq != 'BY2O' else 377), ylim=(-20, 23), xlabel='Time (s)')
            ax.set_xticks([0, 100, 200] if seq != 'BY2O' else [0, 100, 200, 300])
            ax.set_yticks([-15, 0, 15])
            title(ax, chr(97 + j), scene)
            if j == 0:
                ax.set_ylabel('Heading error (deg)')
            else:
                ax.tick_params(labelleft=False)
            ax = axs[1, j]
            for (m, d) in ds.items():
                t = d.time - START[seq]
                q = t.between(lo, hi)
                curve(ax, t[q], d.loc[q, 'yaw_err_deg'], m, 1)
            zero(ax)
            ax.set(xlim=(lo, hi), xlabel='Time (s)')
            title(ax, chr(100 + j), 'Turn: 80–100 s' if seq != 'BY2O' else 'Obstruction and recovery')
            if seq == 'BY2O':
                ax.axvspan(183.94, 225.95, color='#EEE9DF', zorder=-4)
                ax.set_xticks([180, 200, 220])
            else:
                ax.set_xticks([80, 85, 90, 95, 100])
            if j == 0:
                ax.set_ylabel('Heading error (deg)')
            ax = axs[2, j]
            for (i, (metric, lab)) in enumerate([('roll_rmse_deg', 'Roll'), ('pitch_rmse_deg', 'Pitch')]):
                vals = []
                for m in ['A05', 'F04']:
                    v = float(NAT[(NAT.sequence_id == seq) & (NAT.internal_method_id == m)][metric].iloc[0])
                    vals.append(v)
                    rows.append({'sequence_id': seq, 'method_id': m, 'metric': metric, 'rmse_deg': v})
                ax.plot(vals, [i, i], color='#BBBBBB', lw=1.3)
                ax.scatter(vals, [i, i], s=26, c=[PURPLE, BLUE], marker='o', zorder=3)
            ax.set(ylim=(-0.6, 1.6), xlim=(1.3, 3.6), xlabel='Whole-window RMSE (deg)')
            ax.set_yticks([0, 1], ['Roll', 'Pitch'])
            ax.invert_yaxis()
            ax.set_xticks([1.5, 2.5, 3.5])
            title(ax, chr(103 + j), 'Tilt-aiding contribution')
        axs[2, 1].legend(handles=[Line2D([], [], color=PURPLE, marker='o', lw=0, label='Tilt off'), Line2D([], [], color=BLUE, marker='o', lw=0, label='Full')], loc='lower right', fontsize=8, handlelength=0.8)
        save(fig, 'figure06_attitude')

    def status(ax, fam):
        ax.set(xlim=(-4, 32), ylim=(-0.5, 2.7))
        ax.axis('off')
        for (i, label) in enumerate(['Position', 'Heading', 'Body update']):
            y = 2 - i
            active = i > 0 and fam == 'A2'
            ax.plot([-4, 32], [y, y], color='#53636C', lw=4, solid_capstyle='butt')
            if not active:
                ax.plot([0, 20], [y, y], color='#DDD7CD', lw=4, solid_capstyle='butt')
            ax.text(-5, y, label, ha='right', va='center', fontsize=8)
        ax.text(10, 2.55, 'interruption', ha='center', fontsize=8, color='#555555')
        ax.axvline(0, color='#999', lw=0.6)
        ax.axvline(20, color='#999', lw=0.6)

    def fig7():
        fig = plt.figure(figsize=(174 / 25.4, 204 / 25.4))
        gs = fig.add_gridspec(4, 2, height_ratios=[0.34, 0.75, 1.1, 1.0], left=0.17, right=0.98, bottom=0.085, top=0.915, hspace=0.63, wspace=0.48)
        legend(fig, ['A06', 'F04'], y=1.0)
        stats = []
        end = []
        for (col, fam) in enumerate(['A1', 'A2']):
            sa = fig.add_subplot(gs[0, col])
            status(sa, fam)
            sa.set_title('All GNSS interrupted' if fam == 'A1' else 'Heading retained', loc='left', fontweight='bold', pad=17)
            rows = ADD[(ADD.case_family == fam) & (ADD.duration_s == 20) & (ADD.seed_index == 'seed_00')]
            t0 = float(rows.outage_start_s.iloc[0])
            ds = {}
            for m in ['A06', 'F04']:
                rr = rows[rows.internal_method_id == m].iloc[0]
                d = series(rr.run_id)
                ds[m] = d
                b = d[(d.time >= t0) & (d.time < t0 + 20)].iloc[-1]
                end.append({'family': fam, 'method_id': m, 'run_id': rr.run_id, 'last_before_restoration_time_s': float(b.time), 'horizontal_error_m': float(b.horizontal_err_m), 'yaw_error_deg': float(b.yaw_err_deg), 'whole_window_horizontal_rmse_m': rr.horizontal_rmse_m})
            for (row, field, yl) in [(1, 'yaw_err_deg', 'Heading error (deg)'), (2, 'horizontal_err_m', 'Horizontal error (m)')]:
                ax = fig.add_subplot(gs[row, col])
                for (m, d) in ds.items():
                    t = d.time - t0
                    k = t.between(-4, 32)
                    curve(ax, t[k], d.loc[k, field], m, 0.9)
                ax.axvspan(0, 20, color='#F3F0EA', zorder=-4)
                ax.axvline(20, color='#777', ls=':', lw=0.7)
                ax.set(xlim=(-4, 32), ylabel=yl)
                title(ax, chr(97 + (row - 1) * 2 + col), 'Heading error' if row == 1 else 'Position error and recovery')
                if row == 1:
                    ax.set_xticklabels([])
                else:
                    ax.set_xlabel('Time from interruption start (s)')
                    ax.set_xticks([0, 10, 20, 30])
                    ax.text(20.3, 0.95, 'restored', transform=ax.get_xaxis_transform(), fontsize=8, va='top')
                    if col == 1:
                        ins = ax.inset_axes([0.11, 0.52, 0.4, 0.32])
                        d = ds['F04']
                        t = d.time - t0
                        k = t.between(0, 20)
                        curve(ins, t[k], d.loc[k, field], 'F04', 0.9)
                        ins.set(xlim=(0, 20), ylim=(0, 2.5))
                        ins.set_xticks([0, 20])
                        ins.set_yticks([0, 2])
                        ins.tick_params(labelsize=8, pad=1)
                        ins.set_title('Full: same interval', fontsize=8, pad=2)
            ax = fig.add_subplot(gs[3, col])
            dur = [10, 20, 30] if fam == 'A1' else [10, 20]
            for (i, n) in enumerate(dur):
                dd = ADD[(ADD.case_family == fam) & (ADD.duration_s == n)]
                a = dd[dd.internal_method_id == 'F04'].set_index('seed_index').sort_index()
                b = dd[dd.internal_method_id == 'A06'].set_index('seed_index').sort_index()
                assert len(a) == len(b) == 9
                for (j, k) in enumerate(a.index):
                    xx = [i - 0.13 + (j - 4) * 0.013, i + 0.13 + (j - 4) * 0.013]
                    yy = [b.loc[k, 'horizontal_rmse_m'], a.loc[k, 'horizontal_rmse_m']]
                    ax.plot(xx, yy, color='#C3C3C3', lw=0.7)
                    ax.scatter(xx, yy, c=[PURPLE, BLUE], s=17, zorder=3)
                stats.append({'condition': fam, 'duration_s': n, 'pairs': 9, 'without_HV_whole_window_median_m': b.horizontal_rmse_m.median(), 'full_whole_window_median_m': a.horizontal_rmse_m.median()})
            ax.set_yscale('log')
            ax.set_ylim(0.075, 65)
            ax.set_yticks([0.1, 1, 10], ['0.1', '1', '10'])
            ax.set_xticks(range(len(dur)), [str(v) for v in dur])
            ax.set_xlim(-0.45, len(dur) - 0.55)
            ax.set(xlabel='Interruption duration (s)', ylabel='Whole-window H RMSE (m)')
            title(ax, chr(101 + col), 'Nine paired placements; log scale')
        fig.text(0.54, 0.018, 'Status strips for full method: dark = available / updated; light = paused.', ha='center', fontsize=8)
        save(fig, 'figure07_interruptions')

    def corrected_tables():
        d = DEFS.copy()
        inactive = {'D29', 'D40', 'D55', 'D56'}

        def group(r):
            k = r.degradation_type_id
            if k == 'CLEAN':
                return 'Clean reference'
            if k in inactive:
                return 'Unchanged estimator inputs'
            if k in ['D51', 'D52']:
                return 'Onboard tilt observations'
            if k in ['D53', 'D54']:
                return 'Body velocity observations'
            return {'dual_yaw': 'Short-baseline heading', 'gnss_outage': 'GNSS observation availability', 'gnss_sampling': 'GNSS sampling', 'position_value': 'Position values', 'position_std_status': 'Position uncertainty', 'velocity_raw_doppler': 'Velocity observations', 'multi_source_mixed': 'Multiple observation sources'}[r.case_family]
        d['physical_input_group'] = d.apply(group, axis=1)
        d['registered_active_measurement_path'] = ~d.degradation_type_id.isin(inactive | {'CLEAN'})
        d['physical_duplicate_of'] = np.where(d.degradation_type_id == 'D28', 'D25', '')
        d['exposure_note'] = ''
        notes = {'D29': 'Metadata-only; formal solver values unchanged.', 'D40': 'Baseline magnitude metadata has no active scalar-measurement path.', 'D55': 'Contact/motion audit metadata only.', 'D56': 'Foot-speed/contact audit metadata only.', 'D25': 'Same realized physical parameters as D28; identities retained.', 'D28': 'Same realized std x4 as D25; not independent mechanism.', 'D51': 'RP dropout 50%; remaining roll/pitch sigma 3 deg; HV rotation source unchanged.', 'D52': 'RP bias vector norm 2 deg, not 2 deg on each axis.', 'D53': 'HV N/E noise sigma 1 m/s.', 'D54': 'Seeds 00–03 scale x1.5; 04–07 dropout 50%; 08 both; retain branch.'}
        for (k, v) in notes.items():
            d.loc[d.degradation_type_id == k, 'exposure_note'] = v
        for k in ['D32', 'D33', 'D34', 'D35', 'D38']:
            d.loc[d.degradation_type_id == k, 'exposure_note'] = 'Realized angular disturbance extended over original 1-s cells; not independent 5-Hz noise.'
        merged = CORE.merge(d[['degradation_type_id', 'physical_input_group', 'registered_active_measurement_path', 'physical_duplicate_of', 'exposure_note']], on='degradation_type_id', how='left')
        assert len(merged) == 5951
        summaries = []
        for ((k, m), x) in merged.groupby(['degradation_type_id', 'internal_method_id']):
            finite = x.yaw_rmse_deg.dropna()
            summaries.append({'degradation_type_id': k, 'method_id': m, 'registered': len(x), 'completed': int((x.evaluation_status == 'COMPLETED').sum()), 'diverged': int((x.failure_classification == 'ALGORITHM_FAILURE_DIVERGED').sum()), 'no_heading': int(x.failure_classification.str.contains('HEADING').sum()), 'conditional_yaw_rmse_median_deg': finite.median() if len(finite) else np.nan})
        return d

    def fig10_r2():
        corrected_tables()
        spike_event = json.loads((OUT / 'F10_SPIKE_EVENT_SELECTION.json').read_text())
        (fig, axs) = plt.subplots(3, 3, figsize=(174 / 25.4, 210 / 25.4), gridspec_kw={'height_ratios': [0.85, 1.05, 1.05]})
        fig.subplots_adjust(left=0.105, right=0.985, bottom=0.075, top=0.885, wspace=0.36, hspace=0.53)
        legend(fig, ['F01', 'F03', 'F04', 'INPUT'], y=1.0)
        fig.text(0.55, 0.949, 'Nine registered variants per nonzero level; one recorded route', ha='center', fontsize=8)
        configs = [(['CLEAN', 'D30', 'D31'], [0, 5, 20], 'Heading interruption (s)', 'Availability'), (['CLEAN', 'D32', 'D33'], [0, 1, 5], 'Heading-noise σ (deg)', 'Noise level'), (['CLEAN', 'D34', 'D35'], [0, 5, 10], '10° spike probability (%)', 'Outlier frequency')]
        rows = []
        process = []
        for (j, (types, levels, xlab, ttl)) in enumerate(configs):
            ax = axs[0, j]
            for m in ['F01', 'F03', 'F04']:
                med = []
                for (typ, x) in zip(types, levels):
                    d = CORE[(CORE.degradation_type_id == typ) & CORE.internal_method_id.eq(m)]
                    assert d.evaluation_status.eq('COMPLETED').all()
                    y = d.yaw_rmse_deg.to_numpy()
                    med.append(np.median(y))
                    off = np.linspace(-0.025, 0.025, len(y)) * max(levels)
                    ax.scatter(x + off, y, color=ST[m][0], s=13, alpha=0.75, marker={'F01': 's', 'F03': '^', 'F04': 'o'}[m], zorder=3)
                    for (_, r) in d.iterrows():
                        rows.append({'panel': j, 'physical_level': x, 'type': typ, 'method': m, 'case_id': r.case_id, 'yaw_rmse_deg': r.yaw_rmse_deg})
                ax.plot(levels, med, color=ST[m][0], ls=ST[m][1], lw=1.1, marker={'F01': 's', 'F03': '^', 'F04': 'o'}[m], ms=3.4)
            ax.set(xlabel=xlab, ylim=(0, 9.4))
            ax.set_xticks(levels)
            ax.set_yticks([0, 2, 4, 6, 8])
            title(ax, chr(97 + j), ttl)
            if j == 0:
                ax.set_ylabel('Whole-window yaw RMSE (deg)')
            else:
                ax.tick_params(labelleft=False)
            typ = types[-1]
            cid = typ + '_seed_00'
            d = series(runrow(cid, 'F04').run_id)
            obs = pd.read_csv(OUT / 'data' / f'{cid}_OBSERVATION_ERROR.csv.gz')
            ax = axs[1, j]
            ax.plot(obs.elapsed_s, obs.input_error_deg, color=PURPLE, ls=':', lw=0.9, rasterized=True)
            curve(ax, d.time - 66, d.yaw_err_deg, 'F04', 0.8)
            zero(ax)
            ax.set(xlim=(0, 274), xlabel='Time (s)')
            ax.set_xticks([0, 100, 200])
            title(ax, chr(100 + j), ['20-s heading interruption', '5° heading noise', '10% heading spikes'][j])
            if j == 0:
                ax.set_ylabel('Angular error (deg)')
            if j == 0:
                ax.axvspan(130.2, 150.2, color='#E5E7EB', zorder=-4)
                (lo, hi) = (125.2, 160.2)
            elif j == 1:
                (lo, hi) = (80, 100)
            else:
                (lo, hi) = (spike_event['detail_start_elapsed_s'], spike_event['detail_end_elapsed_s'])
            ax.axvspan(lo, hi, color='#F3EEE6', zorder=-5)
            ax = axs[2, j]
            q = obs.elapsed_s.between(lo, hi)
            ox = obs.loc[q, 'elapsed_s'] - 130.2 if j == 0 else obs.loc[q, 'elapsed_s']
            ax.plot(ox, obs.loc[q, 'input_error_deg'], color=PURPLE, ls=':', lw=1.0, marker='.', ms=2.0, rasterized=True)
            t = d.time - 66
            k = t.between(lo, hi)
            xx = t[k] - 130.2 if j == 0 else t[k]
            curve(ax, xx, d.loc[k, 'yaw_err_deg'], 'F04', 1.0)
            zero(ax)
            if j == 0:
                ax.axvspan(0, 20, color='#E5E7EB', zorder=-4)
                ax.axvline(20, color='#667085', ls=':', lw=0.7)
                ax.set(xlim=(-5, 30), xlabel='Time from heading loss (s)')
                ax.set_xticks([0, 10, 20, 30])
                title(ax, 'g', 'Loss and restoration')
                ax.text(10, 0.95, 'Direction\nunavailable', transform=ax.get_xaxis_transform(), ha='center', va='top', fontsize=8)
            elif j == 1:
                ax.set(xlim=(lo, hi), xlabel='Time (s)')
                ax.set_xticks([80, 90, 100])
                title(ax, 'h', 'Reference turn: 80–100 s')
            else:
                t0 = spike_event['first_selected_event_elapsed_s']
                t1 = spike_event['event_display_end_s']
                ax.axvspan(t0, t1, color='#E5E7EB', zorder=-4)
                ax.set(xlim=(lo, hi), xlabel='Time (s)')
                ax.set_xticks([lo, t0, hi])
                title(ax, 'i', f'Recorded spike at {t0:g} s')
            if j == 0:
                ax.set_ylabel('Angular error (deg)')
            selection = ['first registered variant; actual registered loss interval with fixed5s before /10s after', 'same preselected reference turn as natural Fig6; no error-based selection', spike_event['selection_rule']][j]
            process.append({'case_id': cid, 'detail_start_elapsed_s': lo, 'detail_end_elapsed_s': hi, 'selection': selection})
        w = pd.read_csv(OUT / 'tables/EXISTING_HEADING_WINDOW_SUMMARY.csv')
        s = w[w.evaluator_version.eq('v3') & w.case_id.isin(['D30_seed_00', 'D31_seed_00']) & w.method_id.isin(['F01', 'F03', 'F04']) & w.window_id.str.startswith('registered_fault_01')]
        save(fig, 'figure08_heading_disturbances')
    fig6()
    fig7()
    fig10_r2()
if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--data-root', required=True)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    render(args.data_root, args.output)
