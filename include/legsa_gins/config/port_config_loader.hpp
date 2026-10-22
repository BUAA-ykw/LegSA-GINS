// LegSA-GINS. Inertial backbone derived from KF-GINS; see THIRD_PARTY_NOTICES.md.
#pragma once

#include "legsa_gins/options.hpp"

#include <string>

namespace legsa_gins {

class PortConfigLoader {
 public:
  static PortOptions loadKeyValue(const std::string& path);
  static PortOptions loadYamlLike(const std::string& path);
};

}
