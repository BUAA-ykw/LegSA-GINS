// LegSA-GINS. Inertial backbone derived from KF-GINS; see THIRD_PARTY_NOTICES.md.
#pragma once

#include "legsa_gins/factors/raw_doppler_types.hpp"

namespace legsa_gins {

class RawDopplerFactor {
 public:
  static bool isProviderBacked(const RawDopplerVelocityMeasurement& measurement);
  static Vec3 positiveStd(const RawDopplerVelocityMeasurement& measurement);
  static double residualNorm(const Vec3& nav_velocity_ned_mps,
                             const RawDopplerVelocityMeasurement& measurement);
};

}
