// LegSA-GINS. Inertial backbone derived from KF-GINS; see THIRD_PARTY_NOTICES.md.
#pragma once

#include "legsa_gins/types.hpp"

#include <cstddef>
#include <string>

namespace legsa_gins {

struct RawDopplerVelocityMeasurement {
  double time = 0.0;
  double source_time = 0.0;
  Vec3 velocity_ned_mps = makeVec3(0.0, 0.0, 0.0);
  Vec3 std_ned_mps = makeVec3(1.0, 1.0, 1.0);
  std::size_t sat_count = 0;
  double gdop_like = 0.0;
  std::string provider_status = "provider_missing";
  std::string quality = "invalid";
  bool valid = true;
  bool lineage_valid = true;
  std::string raw_doppler_backend_id;
  std::string obs_source_hash;
  std::string nav_source_hash;
  std::string conversion_config_hash;
  std::string covariance_policy;
};

struct RawDopplerFactorConfig {
  bool enable_raw_doppler = false;
  bool raw_doppler_solver_enabled = false;
  std::string raw_doppler_factor_path;
  std::string raw_doppler_factor_source = "none";
  double raw_doppler_time_tolerance_sec = 0.05;
  std::size_t raw_doppler_min_sat = 5;
  double raw_doppler_residual_gate_mps = 3.0;
  double raw_doppler_R_scale = 1.0;
  std::string raw_doppler_mode = "doppler_ls_velocity";

  bool formal_lineage_required = false;
  std::string raw_doppler_backend_id;
  std::string raw_doppler_backend_source_files;
  std::string raw_doppler_backend_source_hashes;
  std::string helper_executable_hash;
  std::string obs_source_hash;
  std::string nav_source_hash;
  std::string conversion_config_hash;
  std::string covariance_policy;
  bool rtklib_position_solution_used_as_solver_input = false;
  bool nav_pvt_velocity_used_as_raw_doppler = false;
  bool gnss_velocity_used_as_raw_doppler = false;
  bool status_fallback_used = false;
  bool legacy_provider_used = false;
};

struct RawDopplerFactorStatus {
  bool code_present = true;
  bool solver_enabled = false;
  bool toy_factor_applied = false;
  std::size_t epoch_count = 0;
  std::size_t valid_epoch_count = 0;
  std::size_t invalid_epoch_count = 0;
  std::size_t update_count = 0;
  std::size_t reject_count = 0;
  std::size_t sat_count_min = 0;
  std::size_t sat_count_median = 0;
  std::size_t sat_count_max = 0;
  double residual_p95_mps = 0.0;
  std::string provider_status = "provider_missing";
  std::string factor_source = "none";
  bool velocity_not_nav_pvt = true;
  bool velocity_not_gnss_15col = true;
  bool status_fallback_used = false;
  bool legacy_provider_used = false;
  bool lineage_proven = false;
  std::string backend_id;
  std::string obs_source_hash;
  std::string nav_source_hash;
  std::string conversion_config_hash;
  std::string covariance_policy;
};

}
