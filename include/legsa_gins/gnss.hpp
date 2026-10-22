// LegSA-GINS. Inertial backbone derived from KF-GINS; see THIRD_PARTY_NOTICES.md.
#pragma once

#include "legsa_gins/types.hpp"

namespace legsa_gins {

struct GnssData {
  double time = 0.0;
  Vec3 blh_rad_m = makeVec3(0.0, 0.0, 0.0);
  Vec3 std_ned_m = makeVec3(1.0, 1.0, 1.0);
  Vec3 vel_ned_mps = makeVec3(0.0, 0.0, 0.0);
  Vec3 vel_std_mps = makeVec3(1.0, 1.0, 1.0);
  double yaw_deg = 0.0;
  double yaw_std_deg = 1.0;
  double yaw_rad = 0.0;
  double yaw_std_rad = D2R;
  bool has_position = true;
  bool has_velocity = true;
  bool has_yaw = true;

  bool validity_explicit = false;
  bool isvalid = false;
};

}
