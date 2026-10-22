// LegSA-GINS. Inertial backbone derived from KF-GINS; see THIRD_PARTY_NOTICES.md.
#pragma once

#include "legsa_gins/types.hpp"

namespace legsa_gins {

struct ImuData {
  double time = 0.0;
  double dt = 0.0;
  Vec3 dtheta = makeVec3(0.0, 0.0, 0.0);
  Vec3 dvel = makeVec3(0.0, 0.0, 0.0);
  double odovel = 0.0;
  bool compensated = false;
};

}
