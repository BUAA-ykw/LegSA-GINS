// LegSA-GINS. Inertial backbone derived from KF-GINS; see THIRD_PARTY_NOTICES.md.
#pragma once

#include "legsa_gins/nav_state.hpp"
#include "legsa_gins/options.hpp"

#include <string>
#include <vector>

namespace legsa_gins {

class FileSaver {
 public:
  static void writeNav(const std::string& output_dir, const std::vector<NavState>& states);
  static void writeStd(const std::string& output_dir, const std::vector<std::vector<double>>& covariances);
  static void writeEvalNav(const std::string& output_dir, const std::vector<NavState>& states);
  static void writeExactCompatible(const std::string& output_dir,
                                   const std::vector<NavState>& states,
                                   const std::vector<std::vector<double>>& covariances);
};

}
