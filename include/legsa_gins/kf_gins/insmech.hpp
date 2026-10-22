// LegSA-GINS. Inertial backbone derived from KF-GINS; see THIRD_PARTY_NOTICES.md.
#pragma once

#include "legsa_gins/imu.hpp"
#include "legsa_gins/nav_state.hpp"

namespace legsa_gins {

class INSMech {
 public:
  static void insMech(const NavState& pvapre, NavState& pvacur, const ImuData& imupre, const ImuData& imucur);
  static void velUpdate(const NavState& pvapre, NavState& pvacur, const ImuData& imupre, const ImuData& imucur);
  static void posUpdate(const NavState& pvapre, NavState& pvacur, const ImuData& imupre, const ImuData& imucur);
  static void attUpdate(const NavState& pvapre, NavState& pvacur, const ImuData& imupre, const ImuData& imucur);
  static NavState propagateOneStep(const NavState& previous, const ImuData& imu);
};

}
