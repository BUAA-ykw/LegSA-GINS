// LegSA-GINS. Inertial backbone derived from KF-GINS; see THIRD_PARTY_NOTICES.md.
#include "legsa_gins/factors/go2_weak_prior_factor.hpp"

#include <algorithm>
#include <cmath>

namespace legsa_gins {

bool Go2WeakPriorFactor::isActive(const Go2AttitudeWeakPriorMeasurement& measurement) {
  return measurement.source_status == "active" && measurement.std_roll_rad > 0.0 && measurement.std_pitch_rad > 0.0;
}

std::vector<double> Go2WeakPriorFactor::residual(const NavState& state,
                                                 const Go2AttitudeWeakPriorMeasurement& measurement) {

  return {wrapRad(state.euler_rad[0] - measurement.roll_rad), wrapRad(state.euler_rad[1] - measurement.pitch_rad)};
}

Matrix Go2WeakPriorFactor::designMatrix(const NavState& state) {
  Matrix H(2, RANK, 0.0);
  const double pitch = state.euler_rad[1];
  const double yaw = state.euler_rad[2];
  const double cos_yaw = std::cos(yaw);
  const double sin_yaw = std::sin(yaw);
  const double cos_pitch = std::cos(pitch);
  double sec_pitch = 1.0 / cos_pitch;
  if (std::fabs(pitch) > 60.0 * D2R && std::fabs(sec_pitch) > 2.0) {
    sec_pitch = std::copysign(2.0, sec_pitch);
  }

  H(0, PHI_ID) = -cos_yaw * sec_pitch;
  H(0, PHI_ID + 1) = -sin_yaw * sec_pitch;
  H(1, PHI_ID) = sin_yaw;
  H(1, PHI_ID + 1) = -cos_yaw;
  return H;
}

Matrix Go2WeakPriorFactor::covariance(const Go2AttitudeWeakPriorMeasurement& measurement,
                                      const Go2AttitudeWeakPriorConfig& config) {
  (void)measurement;

  const double roll_std = std::max(1.0e-6, config.go2_attitude_prior_std_roll_deg * D2R);
  const double pitch_std = std::max(1.0e-6, config.go2_attitude_prior_std_pitch_deg * D2R);
  Matrix R(2, 2, 0.0);
  R(0, 0) = roll_std * roll_std;
  R(1, 1) = pitch_std * pitch_std;
  return R;
}

double Go2WeakPriorFactor::wrapRad(double value) {
  while (value > kPi) {
    value -= 2.0 * kPi;
  }
  while (value <= -kPi) {
    value += 2.0 * kPi;
  }
  return value;
}

}
