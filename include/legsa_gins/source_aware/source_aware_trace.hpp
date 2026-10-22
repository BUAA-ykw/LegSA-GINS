// LegSA-GINS. Inertial backbone derived from KF-GINS; see THIRD_PARTY_NOTICES.md.
#pragma once

#include "legsa_gins/source_aware/measurement_source.hpp"

#include <string>
#include <vector>

namespace legsa_gins::source_aware {

struct SourceAwareTraceRow {
  double time = 0.0;
  std::size_t update_index = 0;
  SourceWeightResult result;
};

class SourceAwareTrace {
 public:
  void add(double time, std::size_t update_index, const SourceWeightResult& result);
  bool empty() const;
  const std::vector<SourceAwareTraceRow>& rows() const;
  SourceAwareRuntimeStats stats() const;
  void writeCsv(const std::string& output_dir) const;

 private:
  std::vector<SourceAwareTraceRow> rows_;
};

}
