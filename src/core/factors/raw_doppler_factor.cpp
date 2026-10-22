// LegSA-GINS. Inertial backbone derived from KF-GINS; see THIRD_PARTY_NOTICES.md.
#include "legsa_gins/factors/raw_doppler_factor.hpp"

#include <algorithm>
#include <cmath>

namespace legsa_gins {

bool RawDopplerFactor::isProviderBacked(const RawDopplerVelocityMeasurement& measurement) {
  return measurement.valid && measurement.lineage_valid && measurement.provider_status == "available";
}

Vec3 RawDopplerFactor::positiveStd(const RawDopplerVelocityMeasurement& measurement) {
  return makeVec3(std::max(std::fabs(measurement.std_ned_mps[0]), 1.0e-3),
                  std::max(std::fabs(measurement.std_ned_mps[1]), 1.0e-3),
                  std::max(std::fabs(measurement.std_ned_mps[2]), 1.0e-3));
}

double RawDopplerFactor::residualNorm(const Vec3& nav_velocity_ned_mps,
                                      const RawDopplerVelocityMeasurement& measurement) {
  const Vec3 residual = subtract(nav_velocity_ned_mps, measurement.velocity_ned_mps);
  return norm(residual);
}

}
