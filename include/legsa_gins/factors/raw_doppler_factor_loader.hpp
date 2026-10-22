// LegSA-GINS. Inertial backbone derived from KF-GINS; see THIRD_PARTY_NOTICES.md.
#pragma once

#include "legsa_gins/factors/raw_doppler_types.hpp"

#include <string>
#include <vector>

namespace legsa_gins {

struct RawDopplerFactorLoadResult {
  std::vector<RawDopplerVelocityMeasurement> measurements;
  RawDopplerFactorStatus status;
};

class RawDopplerFactorLoader {
 public:
  static RawDopplerFactorLoadResult loadCsv(const std::string& path, const RawDopplerFactorConfig& config);
};

}
