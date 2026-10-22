// LegSA-GINS. Inertial backbone derived from KF-GINS; see THIRD_PARTY_NOTICES.md.
#pragma once

#include "legsa_gins/types.hpp"

#include <utility>

namespace legsa_gins {

class Earth {
 public:
  static constexpr double kWgs84Wie = 7.2921151467E-5;
  static constexpr double kWgs84F = 0.0033528106647474805;
  static constexpr double kWgs84A = 6378137.0;
  static constexpr double kWgs84B = 6356752.3142451793;
  static constexpr double kWgs84E1 = 0.0066943799901413156;
  static constexpr double kWgs84E2 = 0.0067394967422764341;

  static double degToRad(double deg);
  static double radToDeg(double rad);
  static double gravity(const Vec3& blh_rad_m);
  static std::pair<double, double> meridianPrimeVerticalRadius(double lat_rad);
  static double RN(double lat_rad);
  static Matrix3 cne(const Vec3& blh_rad_m);
  static Quaternion qne(const Vec3& blh_rad_m);
  static Vec3 blh(const Quaternion& qne_value, double height_m);
  static Vec3 blh2ecef(const Vec3& blh_rad_m);
  static Vec3 ecef2blh(const Vec3& ecef_m);
  static Matrix3 DR(const Vec3& blh_rad_m);
  static Matrix3 DRi(const Vec3& blh_rad_m);
  static Vec3 iewe();
  static Vec3 iewn(const Vec3& blh_rad_m);
  static Vec3 enwn(const Vec3& blh_rad_m, const Vec3& vel_ned_mps);
};

}
