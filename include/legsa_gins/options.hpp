// LegSA-GINS. Inertial backbone derived from KF-GINS; see THIRD_PARTY_NOTICES.md.
#pragma once

#include "legsa_gins/factors/raw_doppler_types.hpp"
#include "legsa_gins/factors/go2_weak_prior_types.hpp"
#include "legsa_gins/source_aware/measurement_source.hpp"
#include "legsa_gins/types.hpp"

#include <string>

namespace legsa_gins {

struct PortOptions {
  std::string imu_path;
  std::string gnss_path;
  Vec3 antlever_m = makeVec3(0.0, 0.0, 0.0);
  Vec3 init_pos_blh_rad_m = makeVec3(0.0, 0.0, 0.0);
  Vec3 init_vel_ned_mps = makeVec3(0.0, 0.0, 0.0);
  Vec3 init_att_rad = makeVec3(0.0, 0.0, 0.0);
  Vec3 init_pos_std_m = makeVec3(1.0, 1.0, 1.0);
  Vec3 init_vel_std_mps = makeVec3(0.1, 0.1, 0.1);
  Vec3 init_att_std_rad = makeVec3(1.0 * D2R, 1.0 * D2R, 1.0 * D2R);
  ImuError init_imu_error;
  ImuError init_imu_error_std;
  ImuNoise imunoise;
  double starttime = 0.0;
  double endtime = 0.0;
  bool enable_receiver_velocity_update = true;
  bool enable_basic_dual_yaw_baseline = false;
  bool enable_dual_yaw_update = true;
  double basic_dual_yaw_fixed_std_deg = 1.5;
  std::string basic_dual_yaw_residual_sign = "pred_minus_obs";
  RawDopplerFactorConfig raw_doppler_config;
  Go2AttitudeWeakPriorConfig go2_attitude_prior_config;
  Go2VelocityDiagnosticPriorConfig go2_velocity_prior_diagnostic_config;
  source_aware::SourceAwarePolicyConfig source_aware_policy_config;
  bool yaw_scheme_C_enabled = true;
  double yaw_std_min_deg = 0.5;
  double yaw_std_soft_deg = 3.0;
  double yaw_std_hard_deg = 6.0;
  double yaw_res_soft_deg = 6.0;
  double yaw_res_hard_deg = 15.0;
  double yaw_downweight_scale = 2.5;
};

}
