// LegSA-GINS. Inertial backbone derived from KF-GINS; see THIRD_PARTY_NOTICES.md.
#pragma once

#include "legsa_gins/source_aware/measurement_source.hpp"

#include <array>
#include <deque>

namespace legsa_gins::source_aware {

class SourceAwarePolicy {
 public:
  explicit SourceAwarePolicy(SourceAwarePolicyConfig config);

  const SourceAwarePolicyConfig& config() const;
  std::string branchId() const;
  bool enabledFor(MeasurementSource source) const;
  SourceWeightResult evaluate(const SourceMetadata& metadata, const ObservationInnovation& innovation);

 private:
  double sourceCap(MeasurementSource source) const;
  double capScale(double value, MeasurementSource source) const;
  bool n6bPolicyEnabled() const;
  void applyRollingBaseline(MeasurementSource source, double normalized, SourceWeightResult& result);
  double lsimScale(const SourceMetadata& metadata, SourceWeightResult& result) const;
  double oimScale(const SourceMetadata& metadata,
                  const ObservationInnovation& innovation,
                  SourceWeightResult& result) const;

  SourceAwarePolicyConfig config_;
  std::array<std::deque<double>, kMeasurementSourceCount> rolling_normalized_by_source_;
};

}
