// LegSA-GINS. Inertial backbone derived from KF-GINS; see THIRD_PARTY_NOTICES.md.
#pragma once

#include "legsa_gins/types.hpp"

#include <array>

namespace legsa_gins {

class Rotation {
 public:
  static Quaternion normalize(const Quaternion& quaternion);
  static Quaternion multiply(const Quaternion& lhs, const Quaternion& rhs);
  static Quaternion inverse(const Quaternion& quaternion);
  static Quaternion matrix2quaternion(const Matrix3& matrix);
  static Matrix3 quaternion2matrix(const Quaternion& quaternion);
  static Vec3 matrix2euler(const Matrix3& matrix);
  static Vec3 quaternion2euler(const Quaternion& quaternion);
  static Quaternion rotvec2quaternion(const Vec3& rotvec);
  static Vec3 quaternion2vector(const Quaternion& quaternion);
  static Matrix3 euler2matrix(const Vec3& euler_rad);
  static Quaternion euler2quaternion(const Vec3& euler_rad);
  static Matrix3 skewSymmetric(const Vec3& vector);
  static std::array<std::array<double, 4>, 4> quaternionleft(const Quaternion& quaternion);
  static std::array<std::array<double, 4>, 4> quaternionright(const Quaternion& quaternion);
  static Matrix3 eulerToMatrix(const Vec3& euler_rad);
  static Vec3 matrixToEuler(const Matrix3& matrix);
  static double wrapRad(double angle_rad);
  static double wrap2Pi(double angle_rad);
};

}
