# Data formats

## Prepared estimator inputs

The run entry point expects the following layout. Each sequence has its own prepared observations; the source repository does not contain recordings or generated observation tables.

```text
prepared_data/
  BY2/
    IMU.imu
    GNSS.gnss
    DOPPLER.csv
    INCLINATION.csv
    HORIZONTAL_VELOCITY.csv
  BY2H/
    ...
  BY2O/
    ...
```

Times are seconds relative to the sequence epoch in `configs/evaluation.json`. State propagation uses a forward-right-down body frame and a north-east-down navigation frame. Geodetic positions use WGS84.

### IMU.imu

Whitespace-separated columns, without a header:

```text
time dtheta_x dtheta_y dtheta_z dvel_x dvel_y dvel_z
```

Angular increments are in radians and velocity increments are in m/s. These are already calibrated, installed body-frame increments. Do not multiply them by a nominal sample period or apply the installation transform again. The preparation code retains the recorded current-sample interval and rejects intervals longer than the configured 0.1 s input limit. The first aligned increment initializes the preceding IMU sample; the first navigation output is at the following sample.

### GNSS.gnss

Eighteen whitespace-separated columns:

```text
time lat_deg lon_deg height_m
std_n_m std_e_m std_d_m
vn_mps ve_mps vd_mps std_vn_mps std_ve_mps std_vd_mps
body_heading_deg heading_std_deg
position_valid velocity_valid heading_valid
```

All columns occur on the same physical line. The last three fields are independent 0/1 flags. A missing heading must not remove an otherwise valid position or velocity observation. Body heading is clockwise from north and already incorporates the lateral-baseline installation convention. GNSS1 is the right antenna; GNSS2 is the left antenna.

### DOPPLER.csv

The Doppler reader consumes the following CSV header:

```text
time,source_time,vn,ve,vd,std_vn,std_ve,std_vd,sat_count,rtklib_solution_sat_count,doppler_sat_count,gdop_like,provider_status,valid,quality,quality_flag,raw_doppler_backend_id,obs_source_hash,nav_source_hash,conversion_config_hash,covariance_policy
```

Velocity and standard deviation units are m/s. A usable observation has `provider_status=available`, a true validity field, and matching input identity fields. The prepared Doppler observations are derived from raw GNSS measurements; they are distinct from receiver NAV-PVT velocity. The fixed configurations include the corresponding observation identities and use a five-satellite minimum, a 0.05 s match tolerance and the declared conservative covariance policy.

### INCLINATION.csv

```text
time,roll_rad,pitch_rad,std_roll_rad,std_pitch_rad,source_status,mode,gait_type,quality_flag,go2_roll_pitch_truth_claim
```

Angles and their standard deviations are in radians. Roll and pitch are prepared in the estimator's body convention. They are weak observations, not reference truth. The paper configuration assigns 1.6 degrees to each inclination component and a 0.02 s match tolerance.

### HORIZONTAL_VELOCITY.csv

```text
time,vn,ve,vd,std_vn,std_ve,std_vd,confidence,confidence_level,update_flag,reason_codes,source_status,quality_flag,contact_model,contact_label,frame_candidate,prior_policy,diagnostic_only,go2_velocity_truth_claim,valid,go2_source_valid,a1_heading_valid
```

Velocity and standard deviations are in m/s. Only the two horizontal components update the paper estimator. The table has already been rotated using robot inclination and the separately prepared heading series. Heading is unwrapped and interpolated without extrapolation, with samples inside gaps longer than 1.2 s excluded. The validity fields must reflect this support. The runtime uses a 0.08 s matching tolerance.

## Raw-data preparation code

The following modules expose the actual parsing and numerical transformations:

| Module | Role |
|---|---|
| `preprocessing.robot_log`, `preprocessing.imu` | Read robot messages, convert FLU to FRD, apply the installation rotation and form inertial increments |
| `preprocessing.calibrated_imu` | Apply the fixed acceleration scale to unrounded current-sample force before integration |
| `preprocessing.receiver`, `preprocessing.ubx` | Decode receiver messages and preserve receiver epoch identity |
| `preprocessing.heading`, `preprocessing.heading_variants` | Form exact-epoch dual-receiver heading and fixed-solution eligibility |
| `preprocessing.status_heading`, `preprocessing.body_motion` | Prepare the separate status-based heading series, inclination and horizontal velocity |
| `preprocessing.doppler_build`, `preprocessing.doppler` | Build and run the RTKLIB velocity-export helper on observation/navigation files |
| `preprocessing.raw_gnss`, `preprocessing.gnss_factors` | Satellite states, double differences, pseudorange corrections and Doppler least squares for comparisons |
| `preprocessing.case_heading` | Preserve the controlled heading perturbations on the exact receiver epoch grid |

These are Python APIs for the recorded formats, rather than a universal converter for arbitrary robot logs. The native reader headers define the final interface. The comparison methods use their own stated input interfaces; passing the LegSA-GINS output back as a sensor observation is not part of the processing chain.

## Evaluation reference

The evaluator accepts a CSV with a time column and latitude, longitude, height, roll, pitch and yaw. Latitude/longitude and attitude are in degrees; height is in metres. A column named `aligned_time` is already relative. Otherwise `--base_time` converts the absolute timestamp to the sequence epoch. The manuscript reference yaw uses the ENU convention, which the evaluator converts to NED heading.

The reference is read only by evaluation. It is not an input to the native estimator or to the preparation of its motion observations.
