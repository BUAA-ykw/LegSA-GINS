# Reproducing the manuscript calculations

## Navigation

Build the C++ executable, prepare the five observation files per sequence, and run the profiles listed in the root README. The three sequence configurations retain the paper's parameters and initial conditions. The Python launcher replaces only paths and explicit method switches; it preserves the decimal parameter tokens used by the native configuration reader.

Natural-sequence results use the entire declared time window. The public launcher preserves unsuccessful outcomes. A run is classified as divergent if any native row is nonfinite, has invalid latitude/longitude, exceeds a 10 km displacement from its first state, exceeds 50 m/s, or changes geodetic height by more than 1 km. These are the manuscript's whole-run bounds. An enabled heading channel with no accepted observation is reported separately. The criteria do not remove individual epochs from a completed run.

## Controlled observations

`configs/experiments/disturbances.json` defines the sixty disturbance types. `cases.json` contains the 541 fixed cases, including the undisturbed control. `interruptions.json` contains 45 combinations: nine placements for each of the three complete-loss durations and each of the two translation-only-loss durations.

`experiments.perturbations.apply_degradation` applies a case to a `ProviderBundle` of normalized source tables. `experiments.seeds` preserves the fixed component-specific random substreams. The returned source tables retain independent observation-validity fields. `preprocessing.case_heading.lift_heading` transfers the heading disturbance to the exact receiver epoch grid, and `preprocessing.body_motion.correct_hv` constructs the corresponding heading-dependent horizontal velocity. Direct horizontal-velocity perturbations are applied after the frame transform.

The distinction between these processing steps matters: changing only the online scalar yaw while leaving its dependent horizontal-velocity input unchanged is not the paper's heading-disturbance experiment. Likewise, complete GNSS loss removes both heading preparation support and the GNSS events that schedule auxiliary corrections. Translation-only interruption preserves heading.

The normalized `ProviderBundle` tables are named `gnss_position`, `receiver_velocity`, `dual_yaw`, `raw_doppler`, `go2_rp`, `go2_hv` and `source_quality_metadata`. Their parsers and field access are in `experiments/perturbations.py`; prepared native fields are documented in `data_format.md`. The case definitions are configuration, not stored experiment results.

## Position reference point and statistics

`python -m evaluation.run` first applies the declared IMU-to-midpoint position transform, then calls the manuscript evaluator. The position transform uses the estimated attitude and the fixed lever arm `[0.03, 0.03 - baseline_median/2, -0.30]` m in FRD. The reference and native outputs are retained independently; time, velocity and attitude columns are unchanged.

The original evaluation arithmetic is in `evaluation/evaluate.py`. It matches the commercial reference to navigation times, uses WGS84 position differences and wrapped yaw differences, and calculates sample-based RMSE. Standard-deviation output accompanies the same native run and is not a transported midpoint covariance.

For interruption summaries, calculate one RMSE over the complete 274 s BY2 window for each placement, then take the median of the nine values. Do not replace this with RMSE over only the interruption. Endpoint errors are separate quantities.

`python -m evaluation.summarize FULL.csv CONTROL.csv summary.json` summarizes a pair of case-level tables. Required identity fields are `case_id` and `status`; supported error fields are `horizontal_rmse_m`, `vertical_rmse_m`, `roll_rmse_deg`, `pitch_rmse_deg` and `yaw_rmse_deg`. Outcome counts use all supplied rows. Paired differences use cases completed by both configurations and report full minus control. Failed or unavailable rows must remain present in the input tables.

## Numerical figures

The numerical rendering routines are extracted from the final manuscript figure sources. They read separately supplied figure tables; no estimator or parameter fitting is executed by a rendering command.

| Manuscript figure | Command module | Input subdirectory |
|---|---|---|
| 3: known initial yaw rotations | `plotting.gauge` | `gauge` |
| 4: direction error and availability | `plotting.heading` | `heading` |
| 5: routes and heading | `plotting.routes` | `routes` and `comparison` |
| 6–8: attitude, interruptions and heading disturbances | `plotting.attitude_and_interruptions` | `aiding` |
| 9: covariance weighting | `plotting.weighting` | `weighting` |
| 10: navigation-method heading comparison | `plotting.comparison` | `comparison` |

Example:

```bash
python -m plotting.routes --data-root /path/to/figure_tables --output outputs/figures
```

The `aiding/tables` inputs are `series_index.csv`, `case_metrics.csv`, `interruption_metrics.csv`, `paired_metrics.csv`, `natural_metrics.csv` and `disturbance_definitions.csv`, together with the event and heading-observation tables referenced by the rendering source. Individual error series are under `aiding/series/<run_id>.csv.gz`.

The comparison inputs include the three `BY2*_COMMON_HEADING.csv.gz` tables, `F09_ALL18_INTERRUPTION_CASES.csv`, and the four representative interruption series. Route tables use `<sequence>__<method>.csv.gz`. The heading figure consumes its acceptance summary, admitted observation errors, receiver-heading series and external-method series. The weighting figure uses selected-case definitions, one-second display summaries and actual covariance summaries. Figure 3 uses seven `native_initial_yaw_<angle>.csv.gz` tables containing timestamps, elapsed time, local position and unwrapped yaw.

The code exports PNG, PDF and SVG. Original photographs, diagrams, document assembly, intermediate figure revisions, and private numerical data are outside this source release.
