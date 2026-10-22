# Comparison methods

The manuscript compares observation-processing and navigation routes with different information sets. Use each method's native output point and validity support when forming a comparison. C-LAMBDA/C-WLS and moving-base RTKLIB produce direction observations. The IEKFs and OiSAM estimate attitude; TC-FGO and GNC-FGO do not provide a self-estimated yaw in this implementation.

## Heading processing

`baselines.clambda.solve_clambda` consumes the stacked observations, ambiguity design, baseline design and observation covariance. The baseline length is supplied explicitly. Production-dimensional searches use the pinned RTKLIB LAMBDA library through `lambda_bridge_path`; the small exact-search branch is intended for small mathematical instances.

`baselines.cwls.adapt_metric_double_differences` builds a C-WLS model from code/phase double differences, geometry, full covariance, wavelength and baseline length. Pass that model to `solve_single_baseline_cwls`. The input has already been synchronized and screened for the appropriate carrier/satellite conditions.

The cost-ratio threshold of three shown in the manuscript is an additional screening step. It must be reported separately from the unscreened candidate output and must not be described as the original methods' complete acceptance policy.

## RTKLIB

Use the pinned upstream source revision:

```bash
git clone https://github.com/tomojitakasu/RTKLIB.git third_party/RTKLIB
git -C third_party/RTKLIB checkout 180043ee24b6d2b168f98b64be15f69d50046b1a
make -C preprocessing/rtklib RTKLIB_ROOT="$PWD/third_party/RTKLIB"
```

The bridge build produces `liblegsa_rtklib_bridge.so`, `librtklib_legsa.so` and `libgnss_corrections.so`. Supply these paths explicitly to the raw satellite-state, C-LAMBDA and corrected-pseudorange interfaces. For moving-base processing, build RTKLIB's `rnx2rtkp` application and use `configs/baselines/rtklib_moving_base.conf` with the two observation files and broadcast navigation files. This is the configuration used by the default moving-base route in the manuscript.

The Doppler preparation functions are:

```python
from preprocessing.doppler_build import build_rtklib_doppler_helper
from preprocessing.doppler import run_rtklib_doppler_velocity_provider
```

Both take explicit source/input/output paths. The helper exposes Doppler-derived velocity; its position solution is not substituted for the navigation estimator output.

## Position-aided invariant EKF

`baselines.iekf.PavlasekIEKF` contains the process and position-update mathematics. The constructor takes an `ExtendedPose`, covariance, antenna lever arms, gyro/accelerometer process PSDs, gravity and a `two_receiver` selector. `propagate` takes body angular rate, specific force and the measured interval; `update` takes the receiver positions and their covariance matrices.

The literature settings are in `configs/baselines/iekf_literature.yaml`. The fixed calibrated sensor values are in `configs/baselines/imu_calibrated.yaml`. `baselines.parameters.load_h02_parameters` prepares the literature or calibrated parameter object; the recorded-gap policy drops intervals exceeding 0.1 s and continues with the stated sequence convention. `baselines.relative_baseline` contains the retained-relative-baseline update used during absolute-position interruption.

## Factor graphs

```bash
python -m pip install -e '.[comparisons]'
python -m baselines.run_fgo --method oisam \
  --input /path/to/oisam_inputs.npz --sequence BY2 \
  --config configs/baselines/oisam_2025.json --output outputs/oisam
```

OiSAM uses `gnss`, `imu`, `imu_intervals` and `end_s` arrays. `gnss` is the prepared observation matrix; `imu` contains the seven increment columns. `imu_intervals` supplies the unrounded beginning and ending time of each increment. `preprocessing` and `baselines.fgo.inputs` provide the interval and geodetic helpers. The manuscript route retains explicit missing nodes and reinitializes at recorded IMU gaps. It does not use an extra online heading factor after each initialization.

TC-FGO and GNC-FGO use the arrays accepted by `baselines.fgo.wen_tc.solve` and `baselines.fgo.gnc.solve`. These include corrected pseudorange observations, satellite positions, epoch/constellation indices and measurement uncertainties. GNC also consumes raw-Doppler velocity and covariance. TC additionally takes the separate AHRS/preintegration arrays through `--ahrs`. `preprocessing.gnss_factors` supplies the raw observation arithmetic. The `*_paper_contract_20261004.json` configurations specify the corresponding paper-matched variants.

The factor-graph implementations are adaptations used in this manuscript. OiSAM uses GTSAM factors and Ceres relinearization. TC/GNC retain their GNSS1 antenna positions and explicit unavailable states; do not fill their missing attitude fields from another estimator.

## Contact-inertial direction experiment

The contact-inertial estimator used for Figure 3 is provided under `baselines/contact` and requires Eigen 3.3 or newer:

```bash
cmake -S baselines/contact -B build/contact -DCMAKE_BUILD_TYPE=Release
cmake --build build/contact --parallel
python -m preprocessing.prepare_contact /path/to/robot_log.txt contact.csv
build/contact/contact_inekf contact.csv 0 contact_yaw_0.csv
```

The executable's second argument is the conventional initial yaw in degrees. Run the same input at −150, −100, −50, 0, 50, 100 and 150 degrees. The initial world states and covariance are rotated consistently. Contact fields remain the recorded robot-provided kinematic proxies. Alignment for Figure 3 removes only the known initial yaw, with no fitted rotation, translation, scale or reference trajectory.
