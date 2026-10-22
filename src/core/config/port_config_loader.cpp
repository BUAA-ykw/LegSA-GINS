// LegSA-GINS. Inertial backbone derived from KF-GINS; see THIRD_PARTY_NOTICES.md.
#include "legsa_gins/config/port_config_loader.hpp"

#include "legsa_gins/common/earth.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <unordered_map>
#include <utility>

namespace legsa_gins {
namespace {
std::string trim(std::string value) {
  auto not_space = [](unsigned char ch) { return !std::isspace(ch); };
  value.erase(value.begin(), std::find_if(value.begin(), value.end(), not_space));
  value.erase(std::find_if(value.rbegin(), value.rend(), not_space).base(), value.end());
  return value;
}

std::string normalizeLine(std::string line) {
  const auto hash = line.find('#');
  if (hash != std::string::npos) {
    line = line.substr(0, hash);
  }
  std::replace(line.begin(), line.end(), '[', ' ');
  std::replace(line.begin(), line.end(), ']', ' ');
  std::replace(line.begin(), line.end(), ',', ' ');
  return trim(line);
}

std::vector<double> parseVector(std::string value) {
  value = normalizeLine(value);
  std::istringstream stream(value);
  std::vector<double> out;
  double x = 0.0;
  while (stream >> x) {
    out.push_back(x);
  }
  return out;
}
Vec3 vecOrDefault(const std::unordered_map<std::string, std::string>& kv, const std::string& key, Vec3 fallback) {
  auto it = kv.find(key);
  if (it == kv.end()) {
    return fallback;
  }
  const std::vector<double> values = parseVector(it->second);
  if (values.size() < 3) {
    return fallback;
  }
  return makeVec3(values[0], values[1], values[2]);
}

Vec3 vecOrDefault(const std::unordered_map<std::string, std::string>& kv,
                  const std::string& key,
                  const std::string& alias,
                  Vec3 fallback) {
  auto it = kv.find(key);
  if (it == kv.end()) {
    it = kv.find(alias);
  }
  if (it == kv.end()) {
    return fallback;
  }
  const std::vector<double> values = parseVector(it->second);
  if (values.size() < 3) {
    return fallback;
  }
  return makeVec3(values[0], values[1], values[2]);
}

double scalarOrDefault(const std::unordered_map<std::string, std::string>& kv, const std::string& key, double fallback) {
  auto it = kv.find(key);
  if (it == kv.end()) {
    return fallback;
  }
  std::istringstream stream(it->second);
  double value = fallback;
  stream >> value;
  return value;
}

bool boolOrDefault(const std::unordered_map<std::string, std::string>& kv, const std::string& key, bool fallback) {
  auto it = kv.find(key);
  if (it == kv.end()) {
    return fallback;
  }
  std::string value = trim(it->second);
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char ch) { return std::tolower(ch); });
  if (value == "true" || value == "1" || value == "yes" || value == "on") {
    return true;
  }
  if (value == "false" || value == "0" || value == "no" || value == "off") {
    return false;
  }
  return fallback;
}

std::string stringOrDefault(const std::unordered_map<std::string, std::string>& kv,
                            const std::string& key,
                            const std::string& fallback) {
  auto it = kv.find(key);
  if (it == kv.end()) {
    return fallback;
  }
  std::string value = trim(it->second);

  if (value.size() >= 2 && ((value.front() == '"' && value.back() == '"') ||
                            (value.front() == '\'' && value.back() == '\''))) {
    value = value.substr(1, value.size() - 2);
  }
  return value;
}

std::unordered_map<std::string, std::string> readKeyValues(const std::string& path) {
  std::ifstream input(path);
  if (!input) {
    throw std::runtime_error("failed to open config: " + path);
  }
  std::unordered_map<std::string, std::string> kv;
  std::string line;
  while (std::getline(input, line)) {
    line = normalizeLine(line);
    if (line.empty()) {
      continue;
    }
    auto pos = line.find('=');
    if (pos == std::string::npos) {
      pos = line.find(':');
    }
    if (pos == std::string::npos) {
      continue;
    }
    kv[trim(line.substr(0, pos))] = trim(line.substr(pos + 1));
  }
  return kv;
}
}
PortOptions PortConfigLoader::loadKeyValue(const std::string& path) { return loadYamlLike(path); }
PortOptions PortConfigLoader::loadYamlLike(const std::string& path) {
  const auto kv = readKeyValues(path);
  PortOptions options;
  Vec3 initpos = vecOrDefault(kv, "initpos", options.init_pos_blh_rad_m);
  initpos[0] *= D2R;
  initpos[1] *= D2R;
  options.imu_path = stringOrDefault(kv, "imupath", stringOrDefault(kv, "imu_path", ""));
  options.gnss_path = stringOrDefault(kv, "gnsspath", stringOrDefault(kv, "gnss_path", ""));
  options.init_pos_blh_rad_m = initpos;
  options.init_vel_ned_mps = vecOrDefault(kv, "initvel", options.init_vel_ned_mps);
  options.init_att_rad = scale(vecOrDefault(kv, "initatt", options.init_att_rad), D2R);
  options.antlever_m = vecOrDefault(kv, "antlever", options.antlever_m);
  options.init_pos_std_m = vecOrDefault(kv, "initposstd", options.init_pos_std_m);
  options.init_vel_std_mps = vecOrDefault(kv, "initvelstd", options.init_vel_std_mps);
  options.init_att_std_rad = scale(vecOrDefault(kv, "initattstd", scale(options.init_att_std_rad, R2D)), D2R);
  options.init_imu_error.gyrbias = scale(vecOrDefault(kv, "initgyrbias", options.init_imu_error.gyrbias), D2R / 3600.0);
  options.init_imu_error.accbias = scale(vecOrDefault(kv, "initaccbias", options.init_imu_error.accbias), 1.0e-5);
  options.init_imu_error.gyrscale = scale(vecOrDefault(kv, "initgyrscale", options.init_imu_error.gyrscale), 1.0e-6);
  options.init_imu_error.accscale = scale(vecOrDefault(kv, "initaccscale", options.init_imu_error.accscale), 1.0e-6);
  options.imunoise.gyr_arw = scale(vecOrDefault(kv, "arw", scale(options.imunoise.gyr_arw, 60.0 / D2R)), D2R / 60.0);
  options.imunoise.acc_vrw = scale(vecOrDefault(kv, "vrw", scale(options.imunoise.acc_vrw, 60.0)), 1.0 / 60.0);
  options.imunoise.gyrbias_std = scale(vecOrDefault(kv, "gbstd", scale(options.imunoise.gyrbias_std, 3600.0 / D2R)), D2R / 3600.0);
  options.imunoise.accbias_std = scale(vecOrDefault(kv, "abstd", scale(options.imunoise.accbias_std, 1.0e5)), 1.0e-5);
  options.imunoise.gyrscale_std = scale(vecOrDefault(kv, "gsstd", scale(options.imunoise.gyrscale_std, 1.0e6)), 1.0e-6);
  options.imunoise.accscale_std = scale(vecOrDefault(kv, "asstd", scale(options.imunoise.accscale_std, 1.0e6)), 1.0e-6);
  options.imunoise.corr_time = scalarOrDefault(kv, "corrtime", options.imunoise.corr_time / 3600.0) * 3600.0;
  options.init_imu_error_std.gyrbias =
      scale(vecOrDefault(kv, "initgyrbiasstd", "initbgstd", scale(options.imunoise.gyrbias_std, 3600.0 / D2R)),
            D2R / 3600.0);
  options.init_imu_error_std.accbias =
      scale(vecOrDefault(kv, "initaccbiasstd", "initbastd", scale(options.imunoise.accbias_std, 1.0e5)), 1.0e-5);
  options.init_imu_error_std.gyrscale =
      scale(vecOrDefault(kv, "initgyrscalestd", "initsgstd", scale(options.imunoise.gyrscale_std, 1.0e6)),
            1.0e-6);
  options.init_imu_error_std.accscale =
      scale(vecOrDefault(kv, "initaccscalestd", "initsastd", scale(options.imunoise.accscale_std, 1.0e6)),
            1.0e-6);
  options.starttime = scalarOrDefault(kv, "starttime", options.starttime);
  options.endtime = scalarOrDefault(kv, "endtime", options.endtime);
  options.enable_receiver_velocity_update =
      boolOrDefault(kv, "enable_receiver_velocity_update", options.enable_receiver_velocity_update);
  options.enable_receiver_velocity_update =
      boolOrDefault(kv, "enable_receiver_velocity", options.enable_receiver_velocity_update);
  options.enable_basic_dual_yaw_baseline =
      boolOrDefault(kv, "enable_basic_dual_yaw_baseline", options.enable_basic_dual_yaw_baseline);
  options.enable_dual_yaw_update = boolOrDefault(kv, "enable_dual_yaw_update", options.enable_dual_yaw_update);
  options.enable_dual_yaw_update = boolOrDefault(kv, "enable_dual_yaw", options.enable_dual_yaw_update);
  options.basic_dual_yaw_fixed_std_deg =
      scalarOrDefault(kv, "basic_dual_yaw_fixed_std_deg", options.basic_dual_yaw_fixed_std_deg);
  options.yaw_std_min_deg = scalarOrDefault(kv, "yaw_std_min_deg", options.yaw_std_min_deg);
  options.yaw_std_soft_deg = scalarOrDefault(kv, "yaw_std_soft_deg", options.yaw_std_soft_deg);
  options.yaw_std_hard_deg = scalarOrDefault(kv, "yaw_std_hard_deg", options.yaw_std_hard_deg);
  options.yaw_res_soft_deg = scalarOrDefault(kv, "yaw_res_soft_deg", options.yaw_res_soft_deg);
  options.yaw_res_hard_deg = scalarOrDefault(kv, "yaw_res_hard_deg", options.yaw_res_hard_deg);
  options.yaw_downweight_scale =
      scalarOrDefault(kv, "yaw_downweight_scale", options.yaw_downweight_scale);
  options.basic_dual_yaw_residual_sign =
      stringOrDefault(kv, "basic_dual_yaw_residual_sign", options.basic_dual_yaw_residual_sign);
  options.raw_doppler_config.enable_raw_doppler =
      boolOrDefault(kv, "enable_raw_doppler", options.raw_doppler_config.enable_raw_doppler);
  options.raw_doppler_config.raw_doppler_factor_path =
      stringOrDefault(kv, "raw_doppler_factor_path", options.raw_doppler_config.raw_doppler_factor_path);
  options.raw_doppler_config.raw_doppler_factor_source =
      stringOrDefault(kv, "raw_doppler_factor_source", options.raw_doppler_config.raw_doppler_factor_source);
  options.raw_doppler_config.raw_doppler_time_tolerance_sec =
      scalarOrDefault(kv, "raw_doppler_time_tolerance_sec",
                      options.raw_doppler_config.raw_doppler_time_tolerance_sec);
  options.raw_doppler_config.raw_doppler_min_sat =
      static_cast<std::size_t>(std::max(0.0, scalarOrDefault(kv, "raw_doppler_min_sat",
                                                             static_cast<double>(options.raw_doppler_config.raw_doppler_min_sat))));
  options.raw_doppler_config.raw_doppler_residual_gate_mps =
      scalarOrDefault(kv, "raw_doppler_residual_gate_mps",
                      options.raw_doppler_config.raw_doppler_residual_gate_mps);
  options.raw_doppler_config.raw_doppler_R_scale =
      scalarOrDefault(kv, "raw_doppler_R_scale", options.raw_doppler_config.raw_doppler_R_scale);
  options.raw_doppler_config.raw_doppler_mode =
      stringOrDefault(kv, "raw_doppler_mode", options.raw_doppler_config.raw_doppler_mode);
  options.raw_doppler_config.raw_doppler_backend_id =
      stringOrDefault(kv, "raw_doppler_backend_id", options.raw_doppler_config.raw_doppler_backend_id);
  options.raw_doppler_config.raw_doppler_backend_source_files = stringOrDefault(
      kv, "raw_doppler_backend_source_files", options.raw_doppler_config.raw_doppler_backend_source_files);
  options.raw_doppler_config.raw_doppler_backend_source_hashes = stringOrDefault(
      kv, "raw_doppler_backend_source_hashes", options.raw_doppler_config.raw_doppler_backend_source_hashes);
  options.raw_doppler_config.helper_executable_hash =
      stringOrDefault(kv, "helper_executable_hash", options.raw_doppler_config.helper_executable_hash);
  options.raw_doppler_config.obs_source_hash =
      stringOrDefault(kv, "obs_source_hash", options.raw_doppler_config.obs_source_hash);
  options.raw_doppler_config.nav_source_hash =
      stringOrDefault(kv, "nav_source_hash", options.raw_doppler_config.nav_source_hash);
  options.raw_doppler_config.conversion_config_hash =
      stringOrDefault(kv, "conversion_config_hash", options.raw_doppler_config.conversion_config_hash);
  options.raw_doppler_config.covariance_policy =
      stringOrDefault(kv, "covariance_policy", options.raw_doppler_config.covariance_policy);
  options.raw_doppler_config.rtklib_position_solution_used_as_solver_input =
      boolOrDefault(kv,
                    "rtklib_position_solution_used_as_solver_input",
                    options.raw_doppler_config.rtklib_position_solution_used_as_solver_input);
  options.raw_doppler_config.nav_pvt_velocity_used_as_raw_doppler =
      boolOrDefault(kv,
                    "nav_pvt_velocity_used_as_raw_doppler",
                    options.raw_doppler_config.nav_pvt_velocity_used_as_raw_doppler);
  options.raw_doppler_config.gnss_velocity_used_as_raw_doppler =
      boolOrDefault(kv,
                    "gnss_velocity_used_as_raw_doppler",
                    options.raw_doppler_config.gnss_velocity_used_as_raw_doppler);
  options.raw_doppler_config.status_fallback_used =
      boolOrDefault(kv, "status_fallback_used", options.raw_doppler_config.status_fallback_used);
  options.raw_doppler_config.legacy_provider_used =
      boolOrDefault(kv, "legacy_provider_used", options.raw_doppler_config.legacy_provider_used);
  options.go2_attitude_prior_config.enable_go2_attitude_weak_prior =
      boolOrDefault(kv,
                    "enable_go2_attitude_weak_prior",
                    options.go2_attitude_prior_config.enable_go2_attitude_weak_prior);
  options.go2_attitude_prior_config.enable_go2_attitude_weak_prior =
      boolOrDefault(kv,
                    "enable_go2_roll_pitch_prior",
                    options.go2_attitude_prior_config.enable_go2_attitude_weak_prior);
  options.go2_attitude_prior_config.go2_attitude_prior_path =
      stringOrDefault(kv,
                      "go2_attitude_prior_path",
                      options.go2_attitude_prior_config.go2_attitude_prior_path);
  options.go2_attitude_prior_config.go2_attitude_prior_time_tolerance_sec =
      scalarOrDefault(kv,
                      "go2_attitude_prior_time_tolerance_sec",
                      options.go2_attitude_prior_config.go2_attitude_prior_time_tolerance_sec);
  options.go2_attitude_prior_config.go2_attitude_prior_std_roll_deg =
      scalarOrDefault(kv,
                      "go2_attitude_prior_std_roll_deg",
                      options.go2_attitude_prior_config.go2_attitude_prior_std_roll_deg);
  options.go2_attitude_prior_config.go2_attitude_prior_std_pitch_deg =
      scalarOrDefault(kv,
                      "go2_attitude_prior_std_pitch_deg",
                      options.go2_attitude_prior_config.go2_attitude_prior_std_pitch_deg);
  options.go2_attitude_prior_config.go2_attitude_prior_sourceaware =
      boolOrDefault(kv,
                    "go2_attitude_prior_sourceaware",
                    options.go2_attitude_prior_config.go2_attitude_prior_sourceaware);
  options.go2_attitude_prior_config.go2_attitude_prior_diagnostic_only =
      boolOrDefault(kv,
                    "go2_attitude_prior_diagnostic_only",
                    options.go2_attitude_prior_config.go2_attitude_prior_diagnostic_only);
  options.go2_attitude_prior_config.go2_position_prior_enabled =
      boolOrDefault(kv,
                    "go2_position_prior_enabled",
                    options.go2_attitude_prior_config.go2_position_prior_enabled);
  options.go2_attitude_prior_config.go2_velocity_prior_enabled =
      boolOrDefault(kv,
                    "go2_velocity_prior_enabled",
                    options.go2_attitude_prior_config.go2_velocity_prior_enabled);
  options.go2_attitude_prior_config.go2_yaw_prior_enabled =
      boolOrDefault(kv,
                    "go2_yaw_prior_enabled",
                    options.go2_attitude_prior_config.go2_yaw_prior_enabled);
  options.go2_velocity_prior_diagnostic_config.enable_go2_velocity_prior_diagnostic =
      boolOrDefault(kv,
                    "enable_go2_velocity_prior_diagnostic",
                    options.go2_velocity_prior_diagnostic_config.enable_go2_velocity_prior_diagnostic);
  options.go2_velocity_prior_diagnostic_config.enable_go2_horizontal_velocity_prior =
      boolOrDefault(kv,
                    "enable_go2_horizontal_velocity_prior",
                    options.go2_velocity_prior_diagnostic_config.enable_go2_horizontal_velocity_prior);
  options.go2_velocity_prior_diagnostic_config.go2_velocity_prior_diagnostic_path =
      stringOrDefault(kv,
                      "go2_velocity_prior_diagnostic_path",
                      options.go2_velocity_prior_diagnostic_config.go2_velocity_prior_diagnostic_path);
  options.go2_velocity_prior_diagnostic_config.go2_horizontal_velocity_prior_path =
      stringOrDefault(kv,
                      "go2_horizontal_velocity_prior_path",
                      options.go2_velocity_prior_diagnostic_config.go2_horizontal_velocity_prior_path);
  options.go2_velocity_prior_diagnostic_config.go2_velocity_prior_time_tolerance_sec =
      scalarOrDefault(kv,
                      "go2_velocity_prior_time_tolerance_sec",
                      options.go2_velocity_prior_diagnostic_config.go2_velocity_prior_time_tolerance_sec);
  options.go2_velocity_prior_diagnostic_config.go2_velocity_prior_std_scale =
      scalarOrDefault(kv,
                      "go2_velocity_prior_std_scale",
                      options.go2_velocity_prior_diagnostic_config.go2_velocity_prior_std_scale);
  options.go2_velocity_prior_diagnostic_config.go2_velocity_prior_std_scale =
      scalarOrDefault(kv,
                      "go2_horizontal_velocity_prior_std_scale",
                      options.go2_velocity_prior_diagnostic_config.go2_velocity_prior_std_scale);
  options.go2_velocity_prior_diagnostic_config.go2_diagnostic_prior_only =
      boolOrDefault(kv,
                    "go2_diagnostic_prior_only",
                    options.go2_velocity_prior_diagnostic_config.go2_diagnostic_prior_only);
  options.go2_velocity_prior_diagnostic_config.go2_horizontal_velocity_prior_vertical_disabled =
      boolOrDefault(kv,
                    "go2_horizontal_velocity_prior_vertical_disabled",
                    options.go2_velocity_prior_diagnostic_config.go2_horizontal_velocity_prior_vertical_disabled);
  options.go2_velocity_prior_diagnostic_config.go2_horizontal_velocity_prior_source_aware_enabled =
      boolOrDefault(kv,
                    "go2_horizontal_velocity_prior_source_aware_enabled",
                    options.go2_velocity_prior_diagnostic_config.go2_horizontal_velocity_prior_source_aware_enabled);
  options.go2_velocity_prior_diagnostic_config.go2_horizontal_velocity_prior_mode =
      stringOrDefault(kv,
                      "go2_horizontal_velocity_prior_mode",
                      options.go2_velocity_prior_diagnostic_config.go2_horizontal_velocity_prior_mode);
  options.go2_velocity_prior_diagnostic_config.go2_horizontal_velocity_adaptive_std_enabled =
      boolOrDefault(kv,
                    "go2_horizontal_velocity_adaptive_std_enabled",
                    options.go2_velocity_prior_diagnostic_config.go2_horizontal_velocity_adaptive_std_enabled);
  options.go2_velocity_prior_diagnostic_config.go2_horizontal_velocity_bounded_std_policy =
      stringOrDefault(kv,
                      "go2_horizontal_velocity_bounded_std_policy",
                      options.go2_velocity_prior_diagnostic_config.go2_horizontal_velocity_bounded_std_policy);
  options.go2_velocity_prior_diagnostic_config.go2_horizontal_velocity_strength_policy =
      stringOrDefault(kv,
                      "go2_horizontal_velocity_strength_policy",
                      options.go2_velocity_prior_diagnostic_config.go2_horizontal_velocity_strength_policy);
  options.source_aware_policy_config.enable_source_aware_weighting =
      boolOrDefault(kv,
                    "enable_source_aware_weighting",
                    options.source_aware_policy_config.enable_source_aware_weighting);
  options.source_aware_policy_config.enable_source_aware_weighting =
      boolOrDefault(kv,
                    "enable_source_aware",
                    options.source_aware_policy_config.enable_source_aware_weighting);
  options.source_aware_policy_config.source_aware_policy_version =
      stringOrDefault(kv,
                      "source_aware_policy_version",
                      options.source_aware_policy_config.source_aware_policy_version);
  options.source_aware_policy_config.source_aware_mode =
      stringOrDefault(kv, "source_aware_mode", options.source_aware_policy_config.source_aware_mode);
  options.source_aware_policy_config.source_aware_max_R_scale =
      scalarOrDefault(kv,
                      "source_aware_max_R_scale",
                      options.source_aware_policy_config.source_aware_max_R_scale);
  options.source_aware_policy_config.source_aware_global_cap =
      scalarOrDefault(kv,
                      "source_aware_global_cap",
                      options.source_aware_policy_config.source_aware_max_R_scale);
  options.source_aware_policy_config.source_aware_use_innovation_covariance =
      boolOrDefault(kv,
                    "source_aware_use_innovation_covariance",
                    options.source_aware_policy_config.source_aware_use_innovation_covariance);
  options.source_aware_policy_config.source_aware_deadband_normalized =
      scalarOrDefault(kv,
                      "source_aware_deadband_normalized",
                      options.source_aware_policy_config.source_aware_deadband_normalized);
  options.source_aware_policy_config.source_aware_moderate_normalized =
      scalarOrDefault(kv,
                      "source_aware_moderate_normalized",
                      options.source_aware_policy_config.source_aware_moderate_normalized);
  options.source_aware_policy_config.source_aware_strong_normalized =
      scalarOrDefault(kv,
                      "source_aware_strong_normalized",
                      options.source_aware_policy_config.source_aware_strong_normalized);
  options.source_aware_policy_config.source_aware_receiver_position_cap =
      scalarOrDefault(kv,
                      "source_aware_receiver_position_cap",
                      options.source_aware_policy_config.source_aware_receiver_position_cap);
  options.source_aware_policy_config.source_aware_receiver_velocity_cap =
      scalarOrDefault(kv,
                      "source_aware_receiver_velocity_cap",
                      options.source_aware_policy_config.source_aware_receiver_velocity_cap);
  options.source_aware_policy_config.source_aware_dual_yaw_cap =
      scalarOrDefault(kv,
                      "source_aware_dual_yaw_cap",
                      options.source_aware_policy_config.source_aware_dual_yaw_cap);
  options.source_aware_policy_config.source_aware_raw_doppler_cap =
      scalarOrDefault(kv,
                      "source_aware_raw_doppler_cap",
                      options.source_aware_policy_config.source_aware_raw_doppler_cap);
  options.source_aware_policy_config.source_aware_go2_attitude_cap =
      scalarOrDefault(kv,
                      "source_aware_go2_attitude_cap",
                      options.source_aware_policy_config.source_aware_go2_attitude_cap);
  options.source_aware_policy_config.source_aware_go2_horizontal_velocity_cap =
      scalarOrDefault(kv,
                      "source_aware_go2_horizontal_velocity_cap",
                      options.source_aware_policy_config.source_aware_go2_horizontal_velocity_cap);
  options.source_aware_policy_config.source_aware_go2_readiness_lsim_enabled =
      boolOrDefault(kv,
                    "source_aware_go2_readiness_lsim_enabled",
                    options.source_aware_policy_config.source_aware_go2_readiness_lsim_enabled);
  options.source_aware_policy_config.source_aware_go2_readiness_low_scale =
      scalarOrDefault(kv,
                      "source_aware_go2_readiness_low_scale",
                      options.source_aware_policy_config.source_aware_go2_readiness_low_scale);
  options.source_aware_policy_config.source_aware_go2_impact_or_rough_scale =
      scalarOrDefault(kv,
                      "source_aware_go2_impact_or_rough_scale",
                      options.source_aware_policy_config.source_aware_go2_impact_or_rough_scale);
  options.source_aware_policy_config.source_aware_go2_motion_unknown_scale =
      scalarOrDefault(kv,
                      "source_aware_go2_motion_unknown_scale",
                      options.source_aware_policy_config.source_aware_go2_motion_unknown_scale);
  options.source_aware_policy_config.source_aware_go2_in_place_turn_scale =
      scalarOrDefault(kv,
                      "source_aware_go2_in_place_turn_scale",
                      options.source_aware_policy_config.source_aware_go2_in_place_turn_scale);
  options.source_aware_policy_config.source_aware_reject_extreme =
      boolOrDefault(kv,
                    "source_aware_reject_extreme",
                    options.source_aware_policy_config.source_aware_reject_extreme);
  options.source_aware_policy_config.source_aware_no_R_shrink =
      boolOrDefault(kv,
                    "source_aware_no_R_shrink",
                    options.source_aware_policy_config.source_aware_no_R_shrink);
  options.source_aware_policy_config.source_aware_trace_enabled =
      boolOrDefault(kv,
                    "source_aware_trace_enabled",
                    options.source_aware_policy_config.source_aware_trace_enabled);
  options.source_aware_policy_config.source_aware_enable_rolling_innovation_baseline =
      boolOrDefault(kv,
                    "source_aware_enable_rolling_innovation_baseline",
                    options.source_aware_policy_config.source_aware_enable_rolling_innovation_baseline);
  options.source_aware_policy_config.source_aware_rolling_window_size =
      static_cast<std::size_t>(std::max(1.0, scalarOrDefault(kv,
                                                             "source_aware_rolling_window_size",
                                                             static_cast<double>(options.source_aware_policy_config.source_aware_rolling_window_size))));
  options.source_aware_policy_config.source_aware_rolling_mad_floor =
      scalarOrDefault(kv,
                      "source_aware_rolling_mad_floor",
                      options.source_aware_policy_config.source_aware_rolling_mad_floor);
  options.source_aware_policy_config.source_aware_method_family =
      stringOrDefault(kv,
                      "source_aware_method_family",
                      options.source_aware_policy_config.source_aware_method_family);
  options.source_aware_policy_config.source_aware_method_k0 =
      scalarOrDefault(kv,
                      "source_aware_method_k0",
                      options.source_aware_policy_config.source_aware_method_k0);
  options.source_aware_policy_config.source_aware_method_k1 =
      scalarOrDefault(kv,
                      "source_aware_method_k1",
                      options.source_aware_policy_config.source_aware_method_k1);
  options.source_aware_policy_config.source_aware_method_c =
      scalarOrDefault(kv,
                      "source_aware_method_c",
                      options.source_aware_policy_config.source_aware_method_c);
  options.source_aware_policy_config.source_aware_method_alpha =
      scalarOrDefault(kv,
                      "source_aware_method_alpha",
                      options.source_aware_policy_config.source_aware_method_alpha);
  options.source_aware_policy_config.source_aware_method_phi =
      scalarOrDefault(kv,
                      "source_aware_method_phi",
                      options.source_aware_policy_config.source_aware_method_phi);
  options.source_aware_policy_config.source_aware_method_base_gain =
      scalarOrDefault(kv,
                      "source_aware_method_base_gain",
                      options.source_aware_policy_config.source_aware_method_base_gain);
  const std::array<std::pair<source_aware::MeasurementSource, const char*>, source_aware::kMeasurementSourceCount>
      source_keys{{
          {source_aware::MeasurementSource::kReceiverPosition, "receiver_position"},
          {source_aware::MeasurementSource::kReceiverVelocity, "receiver_velocity"},
          {source_aware::MeasurementSource::kDualAntennaYaw, "dual_antenna_yaw"},
          {source_aware::MeasurementSource::kRawDopplerVelocity, "raw_doppler_velocity"},
          {source_aware::MeasurementSource::kGo2AttitudeRollPitch, "go2_attitude_roll_pitch"},
          {source_aware::MeasurementSource::kGo2HorizontalVelocity, "go2_horizontal_velocity"},
      }};
  for (const auto& item : source_keys) {
    auto& source_config = options.source_aware_policy_config.sources[source_aware::sourceIndex(item.first)];
    const std::string prefix = std::string("source_aware_") + item.second + "_";
    source_config.enabled = boolOrDefault(kv, prefix + "enabled", source_config.enabled);
    source_config.lsim_enabled = boolOrDefault(kv, prefix + "lsim_enabled", source_config.lsim_enabled);
    source_config.oim_enabled = boolOrDefault(kv, prefix + "oim_enabled", source_config.oim_enabled);
  }

  auto& hv = options.go2_velocity_prior_diagnostic_config;
  if (hv.enable_go2_horizontal_velocity_prior) hv.enable_go2_velocity_prior_diagnostic = true;
  hv.go2_diagnostic_prior_only = false;
  options.go2_attitude_prior_config.go2_attitude_prior_diagnostic_only = false;
  options.raw_doppler_config.formal_lineage_required = options.raw_doppler_config.enable_raw_doppler;
  if (!hv.go2_horizontal_velocity_prior_path.empty()) hv.go2_velocity_prior_diagnostic_path = hv.go2_horizontal_velocity_prior_path;
  if (options.enable_basic_dual_yaw_baseline) options.yaw_scheme_C_enabled = false;
  return options;
}
}
