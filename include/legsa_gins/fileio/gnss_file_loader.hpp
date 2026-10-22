// LegSA-GINS. Inertial backbone derived from KF-GINS; see THIRD_PARTY_NOTICES.md.
#pragma once

#include "legsa_gins/gnss.hpp"

#include <string>
#include <vector>

namespace legsa_gins {

class GnssFileLoader {
 public:
  GnssFileLoader() = default;
  explicit GnssFileLoader(const std::string& path);
  static std::vector<GnssData> loadFifteenColumn(const std::string& path);
  bool next(GnssData& gnss);
  bool isEof() const;
  bool isOpen() const;
  bool allValidityExplicit() const;

 private:
  std::vector<GnssData> rows_;
  std::size_t index_ = 0;
};

}
