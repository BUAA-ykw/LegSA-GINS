// LegSA-GINS. Inertial backbone derived from KF-GINS; see THIRD_PARTY_NOTICES.md.
#pragma once

#include "legsa_gins/imu.hpp"

#include <string>
#include <vector>

namespace legsa_gins {

class ImuFileLoader {
 public:
  ImuFileLoader() = default;
  explicit ImuFileLoader(const std::string& path);
  static std::vector<ImuData> loadSevenColumn(const std::string& path);
  bool next(ImuData& imu);
  bool isEof() const;
  bool isOpen() const;
  double starttime() const;
  double endtime() const;

 private:
  std::vector<ImuData> rows_;
  std::size_t index_ = 0;
};

}
