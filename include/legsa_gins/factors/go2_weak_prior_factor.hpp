// LegSA-GINS. Inertial backbone derived from KF-GINS; see THIRD_PARTY_NOTICES.md.
#pragma once

#include "legsa_gins/factors/go2_weak_prior_types.hpp"
#include "legsa_gins/nav_state.hpp"
#include "legsa_gins/types.hpp"

#include <vector>

namespace legsa_gins {

class Go2WeakPriorFactor {
 public:
  static bool isActive(const Go2AttitudeWeakPriorMeasurement& measurement);
  static std::vector<double> residual(const NavState& state,
                                      const Go2AttitudeWeakPriorMeasurement& measurement);
  static Matrix designMatrix(const NavState& state);
  static Matrix covariance(const Go2AttitudeWeakPriorMeasurement& measurement,
                           const Go2AttitudeWeakPriorConfig& config);
  static double wrapRad(double value);
};

}
