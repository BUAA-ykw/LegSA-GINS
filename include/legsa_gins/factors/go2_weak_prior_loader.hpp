// LegSA-GINS. Inertial backbone derived from KF-GINS; see THIRD_PARTY_NOTICES.md.
#pragma once

#include "legsa_gins/factors/go2_weak_prior_types.hpp"

#include <string>
#include <vector>

namespace legsa_gins {

struct Go2AttitudeWeakPriorLoadResult {
  std::vector<Go2AttitudeWeakPriorMeasurement> measurements;
  Go2AttitudeWeakPriorStatus status;
};

struct Go2VelocityDiagnosticPriorLoadResult {
  std::vector<Go2VelocityDiagnosticPriorMeasurement> measurements;
  Go2VelocityDiagnosticPriorStatus status;
};

struct Go2ReadinessLsimMetadataLoadResult {
  std::vector<Go2ReadinessLsimMetadataMeasurement> measurements;
  Go2ReadinessLsimMetadataStatus status;
};

class Go2WeakPriorLoader {
 public:
  static Go2AttitudeWeakPriorLoadResult loadCsv(const std::string& path,
                                                const Go2AttitudeWeakPriorConfig& config);
  static Go2VelocityDiagnosticPriorLoadResult loadVelocityDiagnosticCsv(
      const std::string& path,
      const Go2VelocityDiagnosticPriorConfig& config);
  static Go2ReadinessLsimMetadataLoadResult loadReadinessLsimMetadataCsv(
      const std::string& path,
      const Go2ReadinessLsimMetadataConfig& config);
};

}
