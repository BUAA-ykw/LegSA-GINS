// LegSA-GINS. Inertial backbone derived from KF-GINS; see THIRD_PARTY_NOTICES.md.
#pragma once

#include "legsa_gins/types.hpp"

namespace legsa_gins {

struct NavState {
  double time = 0.0;
  Vec3 pos_blh_rad_m = makeVec3(0.0, 0.0, 0.0);
  Vec3 vel_ned_mps = makeVec3(0.0, 0.0, 0.0);
  Vec3 euler_rad = makeVec3(0.0, 0.0, 0.0);
  Matrix3 cbn = identityMatrix3();
  Quaternion qbn{};
  ImuError imu_error;
};

}
