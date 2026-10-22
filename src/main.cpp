// LegSA-GINS navigation executable.
#include "legsa_gins/config/port_config_loader.hpp"
#include "legsa_gins/common/rotation.hpp"
#include "legsa_gins/factors/go2_weak_prior_loader.hpp"
#include "legsa_gins/factors/raw_doppler_factor_loader.hpp"
#include "legsa_gins/fileio/file_saver.hpp"
#include "legsa_gins/fileio/imu_file_loader.hpp"
#include "legsa_gins/fileio/gnss_file_loader.hpp"
#include "legsa_gins/kf_gins/gi_engine.hpp"

#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

int main(int argc, char** argv) {
  using namespace legsa_gins;
  try {
    std::string config, output;
    for (int i = 1; i < argc; ++i) {
      const std::string arg = argv[i];
      if (arg == "--help") {
        std::cout << "Usage: legsa_gins --config CONFIG.yaml --output-dir DIRECTORY\n";
        return 0;
      }
      if (i + 1 == argc) throw std::runtime_error("missing value for " + arg);
      if (arg == "--config") config = argv[++i];
      else if (arg == "--output-dir") output = argv[++i];
      else throw std::runtime_error("unknown argument: " + arg);
    }
    if (config.empty() || output.empty()) throw std::runtime_error("use --config and --output-dir (or --help)");
    auto options = PortConfigLoader::loadYamlLike(config);
    GIEngine engine(options);
    if (options.raw_doppler_config.enable_raw_doppler) {
      auto input = RawDopplerFactorLoader::loadCsv(options.raw_doppler_config.raw_doppler_factor_path,
                                                options.raw_doppler_config);
      if (!input.status.solver_enabled) throw std::runtime_error("Doppler velocity input unavailable");
      engine.setRawDopplerVelocityMeasurements(input.measurements, input.status);
    }
    if (options.go2_attitude_prior_config.enable_go2_attitude_weak_prior) {
      auto input = Go2WeakPriorLoader::loadCsv(options.go2_attitude_prior_config.go2_attitude_prior_path,
                                             options.go2_attitude_prior_config);
      if (!input.status.solver_enabled) throw std::runtime_error("robot inclination input unavailable");
      engine.setGo2AttitudeWeakPriors(input.measurements, input.status);
    }
    if (options.go2_velocity_prior_diagnostic_config.enable_go2_horizontal_velocity_prior) {
      auto input = Go2WeakPriorLoader::loadVelocityDiagnosticCsv(
          options.go2_velocity_prior_diagnostic_config.go2_velocity_prior_diagnostic_path,
          options.go2_velocity_prior_diagnostic_config);
      if (!input.status.solver_enabled) throw std::runtime_error("robot horizontal velocity input unavailable");
      engine.setGo2VelocityDiagnosticPriors(input.measurements, input.status);
    }
    NavState initial;
    initial.time = options.starttime;
    initial.pos_blh_rad_m = options.init_pos_blh_rad_m;
    initial.vel_ned_mps = options.init_vel_ned_mps;
    initial.euler_rad = options.init_att_rad;
    initial.qbn = Rotation::euler2quaternion(initial.euler_rad);
    initial.cbn = Rotation::quaternion2matrix(initial.qbn);
    initial.imu_error = options.init_imu_error;
    engine.initialize(initial);
    ImuFileLoader imu_loader(options.imu_path);
    GnssFileLoader gnss_loader(options.gnss_path);
    ImuData imu;
    bool has_imu = false;
    while (imu_loader.next(imu)) {
      if (imu.time >= options.starttime) { has_imu = true; break; }
    }
    if (!has_imu) throw std::runtime_error("no IMU samples in the evaluation window");
    // The first aligned IMU initializes the preceding increment; output starts at the next sample.
    engine.addImuData(imu, true);
    double current_imu_time = imu.time;
    GnssData gnss;
    bool has_gnss = false;
    while (gnss_loader.next(gnss)) {
      if (gnss.time > options.starttime) { has_gnss = true; break; }
    }
    if (has_gnss) engine.addGnssData(gnss);
    std::vector<NavState> states;
    std::vector<std::vector<double>> covariances;
    while (imu_loader.next(imu)) {
      if (options.endtime > 0.0 && imu.time > options.endtime) break;
      // Advance one pending GNSS event, preserving the paper's update schedule.
      if (has_gnss && gnss.time < current_imu_time && !gnss_loader.isEof()) {
        has_gnss = gnss_loader.next(gnss);
        if (has_gnss) engine.addGnssData(gnss);
      }
      engine.addImuData(imu);
      engine.newImuProcess();
      if (engine.covHealthFailCount()) throw std::runtime_error("navigation covariance diverged");
      states.push_back(engine.navState());
      covariances.push_back(engine.getCovariance());
      current_imu_time = imu.time;
    }
    FileSaver::writeNav(output, states);
    FileSaver::writeStd(output, covariances);
    FileSaver::writeEvalNav(output, states);
    FileSaver::writeExactCompatible(output, states, covariances);
    engine.writeSourceAwareTrace(output);
    std::ofstream summary(std::filesystem::path(output) / "summary.json");
    const bool no_heading = options.enable_dual_yaw_update &&
        engine.yawUpdateCount() == engine.yawRejectCount();
    summary << "{\n  \"status\": \"" << (no_heading ? "NO_VALID_HEADING" : "COMPLETED")
      << "\",\n  \"states\": " << states.size()
      << ",\n  \"position_updates\": " << engine.positionUpdateCount()
      << ",\n  \"receiver_velocity_updates\": " << engine.velocityUpdateCount()
      << ",\n  \"heading_accepted\": " << engine.yawUpdateCount()-engine.yawRejectCount()
      << ",\n  \"heading_rejected\": " << engine.yawRejectCount()
      << ",\n  \"doppler_updates\": " << engine.rawDopplerUpdateCount()
      << ",\n  \"inclination_updates\": " << engine.go2AttitudeWeakPriorUpdateCount()
      << ",\n  \"horizontal_velocity_updates\": " << engine.go2VelocityDiagnosticPriorUpdateCount()
      << "\n}\n";
    std::cout << "Wrote " << states.size() << " navigation states to " << output << '\n';
    return no_heading ? 3 : 0;
  } catch (const std::exception& e) {
    std::cerr << "legsa_gins: " << e.what() << '\n';
    return std::string(e.what()) == "navigation covariance diverged" ? 4 : 1;
  }
}
