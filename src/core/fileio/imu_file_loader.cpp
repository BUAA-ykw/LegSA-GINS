// LegSA-GINS. Inertial backbone derived from KF-GINS; see THIRD_PARTY_NOTICES.md.
#include "legsa_gins/fileio/imu_file_loader.hpp"

#include <fstream>
#include <sstream>
#include <stdexcept>

namespace legsa_gins {

ImuFileLoader::ImuFileLoader(const std::string& path) : rows_(loadSevenColumn(path)) {}

std::vector<ImuData> ImuFileLoader::loadSevenColumn(const std::string& path) {
  std::ifstream input(path);
  if (!input) {
    throw std::runtime_error("failed to open IMU file: " + path);
  }
  std::vector<ImuData> rows;
  std::string line;
  while (std::getline(input, line)) {
    if (line.empty() || line[0] == '#') {
      continue;
    }
    std::istringstream stream(line);
    ImuData imu;
    stream >> imu.time >> imu.dtheta[0] >> imu.dtheta[1] >> imu.dtheta[2] >> imu.dvel[0] >>
        imu.dvel[1] >> imu.dvel[2];
    if (stream) {
      if (!rows.empty()) {
        imu.dt = imu.time - rows.back().time;
      }

      rows.push_back(imu);
    }
  }
  return rows;
}

bool ImuFileLoader::next(ImuData& imu) {
  if (index_ >= rows_.size()) {
    return false;
  }
  imu = rows_[index_++];
  return true;
}

bool ImuFileLoader::isEof() const {
  return index_ >= rows_.size();
}

bool ImuFileLoader::isOpen() const {
  return !rows_.empty();
}

double ImuFileLoader::starttime() const {
  return rows_.empty() ? 0.0 : rows_.front().time;
}

double ImuFileLoader::endtime() const {
  return rows_.empty() ? 0.0 : rows_.back().time;
}

}
