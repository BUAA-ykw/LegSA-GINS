// LegSA-GINS. Inertial backbone derived from KF-GINS; see THIRD_PARTY_NOTICES.md.
#include "legsa_gins/fileio/file_saver.hpp"

#include "legsa_gins/common/earth.hpp"
#include "legsa_gins/source_aware/source_aware_policy.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <stdexcept>
#include <string>

namespace legsa_gins {
namespace {
void ensureOutputDir(const std::string& output_dir) {
  std::filesystem::create_directories(output_dir);
}

double stdOutputScale(std::size_t index) {
  if (index >= PHI_ID && index < PHI_ID + 3) {
    return R2D;
  }
  if (index >= BG_ID && index < BG_ID + 3) {
    return R2D * 3600.0;
  }
  if (index >= BA_ID && index < BA_ID + 3) {
    return 1.0e5;
  }
  if (index >= SG_ID && index < SG_ID + 3) {
    return 1.0e6;
  }
  if (index >= SA_ID && index < SA_ID + 3) {
    return 1.0e6;
  }
  return 1.0;
}

const char* stdOutputName(std::size_t index) {
  static constexpr const char* kNames[kErrorStateSize] = {
      "std_pos_n_m",        "std_pos_e_m",        "std_pos_d_m",
      "std_vel_n_mps",      "std_vel_e_mps",      "std_vel_d_mps",
      "std_roll_deg",       "std_pitch_deg",      "std_yaw_deg",
      "std_gyrbias_x_dph",  "std_gyrbias_y_dph",  "std_gyrbias_z_dph",
      "std_accbias_x_mgal", "std_accbias_y_mgal", "std_accbias_z_mgal",
      "std_gyrscale_x_ppm", "std_gyrscale_y_ppm", "std_gyrscale_z_ppm",
      "std_accscale_x_ppm", "std_accscale_y_ppm", "std_accscale_z_ppm",
  };
  return kNames[index];
}
}
void FileSaver::writeNav(const std::string& output_dir, const std::vector<NavState>& states) {
  ensureOutputDir(output_dir);
  std::ofstream out(std::filesystem::path(output_dir) / "LegSA_PORT_NAV.nav");
  out << std::fixed << std::setprecision(10);
  out << "# time lat_deg lon_deg height_m vn ve vd roll_deg pitch_deg yaw_deg\n";
  for (const auto& state : states) {
    out << state.time << ' ' << Earth::radToDeg(state.pos_blh_rad_m[0]) << ' '
        << Earth::radToDeg(state.pos_blh_rad_m[1]) << ' ' << state.pos_blh_rad_m[2] << ' '
        << state.vel_ned_mps[0] << ' ' << state.vel_ned_mps[1] << ' ' << state.vel_ned_mps[2]
        << ' ' << Earth::radToDeg(state.euler_rad[0]) << ' ' << Earth::radToDeg(state.euler_rad[1])
        << ' ' << Earth::radToDeg(state.euler_rad[2]) << '\n';
  }
}

void FileSaver::writeStd(const std::string& output_dir, const std::vector<std::vector<double>>& covariances) {
  ensureOutputDir(output_dir);
  std::ofstream out(std::filesystem::path(output_dir) / "LegSA_PORT_STD.csv");
  out << "row";
  for (std::size_t i = 0; i < kErrorStateSize; ++i) {
    out << ',' << stdOutputName(i);
  }
  out << '\n';
  for (std::size_t row = 0; row < covariances.size(); ++row) {
    out << row;
    for (std::size_t i = 0; i < kErrorStateSize; ++i) {
      const std::size_t diagonal_index = covariances[row].size() == kErrorStateSize
                                             ? i
                                             : i * kErrorStateSize + i;
      const double value = diagonal_index < covariances[row].size() ? covariances[row][diagonal_index] : 0.0;

      out << ',' << std::sqrt(std::max(0.0, value)) * stdOutputScale(i);
    }
    out << '\n';
  }
}

void FileSaver::writeEvalNav(const std::string& output_dir, const std::vector<NavState>& states) {
  ensureOutputDir(output_dir);
  std::ofstream out(std::filesystem::path(output_dir) / "EVAL_NAV.csv");
  out << "time,lat_deg,lon_deg,height_m,vn,ve,vd,roll_deg,pitch_deg,yaw_deg\n";
  out << std::fixed << std::setprecision(10);
  for (const auto& state : states) {
    out << state.time << ',' << Earth::radToDeg(state.pos_blh_rad_m[0]) << ','
        << Earth::radToDeg(state.pos_blh_rad_m[1]) << ',' << state.pos_blh_rad_m[2] << ','
        << state.vel_ned_mps[0] << ',' << state.vel_ned_mps[1] << ',' << state.vel_ned_mps[2]
        << ',' << Earth::radToDeg(state.euler_rad[0]) << ',' << Earth::radToDeg(state.euler_rad[1])
        << ',' << Earth::radToDeg(state.euler_rad[2]) << '\n';
  }
}

void FileSaver::writeExactCompatible(
    const std::string& output_dir,
    const std::vector<NavState>& states,
    const std::vector<std::vector<double>>& covariances) {
  ensureOutputDir(output_dir);
  if (states.size() != covariances.size()) {
    throw std::runtime_error("exact-compatible writer state/covariance row mismatch");
  }
  std::ofstream nav(std::filesystem::path(output_dir) / "KF_GINS_Navresult.nav");
  std::ofstream stdfile(std::filesystem::path(output_dir) / "KF_GINS_STD.txt");
  std::ofstream imuerr(std::filesystem::path(output_dir) / "KF_GINS_IMU_ERR.txt");
  if (!nav || !stdfile || !imuerr) {
    throw std::runtime_error("failed to open exact-compatible writer outputs");
  }
  auto write_value = [](std::ostream& out, double value) {
    out << std::left << std::setw(15) << std::fixed << std::setprecision(9) << value << ' ';
  };
  for (std::size_t row = 0; row < states.size(); ++row) {
    const auto& state = states[row];
    for (double value : std::array<double, 11>{
             0.0,
             state.time,
             Earth::radToDeg(state.pos_blh_rad_m[0]),
             Earth::radToDeg(state.pos_blh_rad_m[1]),
             state.pos_blh_rad_m[2],
             state.vel_ned_mps[0],
             state.vel_ned_mps[1],
             state.vel_ned_mps[2],
             Earth::radToDeg(state.euler_rad[0]),
             Earth::radToDeg(state.euler_rad[1]),
             Earth::radToDeg(state.euler_rad[2]),
         }) {
      write_value(nav, value);
    }
    nav << '\n';

    write_value(stdfile, state.time);
    for (std::size_t index = 0; index < kErrorStateSize; ++index) {
      const auto& covariance = covariances[row];
      const std::size_t diagonal_index = covariance.size() == kErrorStateSize
                                             ? index
                                             : index * kErrorStateSize + index;
      const double variance = diagonal_index < covariance.size() ? covariance[diagonal_index] : 0.0;
      write_value(stdfile, std::sqrt(std::max(0.0, variance)) * stdOutputScale(index));
    }
    stdfile << '\n';

    write_value(imuerr, state.time);
    for (double value : state.imu_error.gyrbias) {
      write_value(imuerr, value * R2D * 3600.0);
    }
    for (double value : state.imu_error.accbias) {
      write_value(imuerr, value * 1.0e5);
    }
    for (double value : state.imu_error.gyrscale) {
      write_value(imuerr, value * 1.0e6);
    }
    for (double value : state.imu_error.accscale) {
      write_value(imuerr, value * 1.0e6);
    }
    imuerr << '\n';
  }
}
}
