// LegSA-GINS. Inertial backbone derived from KF-GINS; see THIRD_PARTY_NOTICES.md.
#pragma once

#include "legsa_gins/gnss.hpp"
#include "legsa_gins/imu.hpp"
#include "legsa_gins/nav_state.hpp"
#include "legsa_gins/options.hpp"
#include "legsa_gins/factors/go2_weak_prior_types.hpp"
#include "legsa_gins/factors/raw_doppler_types.hpp"
#include "legsa_gins/source_aware/source_aware_policy.hpp"
#include "legsa_gins/source_aware/source_aware_trace.hpp"

#include <cstddef>
#include <string>
#include <vector>

namespace legsa_gins {

class GIEngine {
 public:
  explicit GIEngine(PortOptions options);

  void initialize(const NavState& initial_state);
  void setRawDopplerVelocityMeasurements(const std::vector<RawDopplerVelocityMeasurement>& measurements,
                                         const RawDopplerFactorStatus& status);
  void setGo2AttitudeWeakPriors(const std::vector<Go2AttitudeWeakPriorMeasurement>& measurements,
                                const Go2AttitudeWeakPriorStatus& status);
  void setGo2VelocityDiagnosticPriors(const std::vector<Go2VelocityDiagnosticPriorMeasurement>& measurements,
                                      const Go2VelocityDiagnosticPriorStatus& status);

  void addImuData(const ImuData& imu, bool compensate = false);
  void addGnssData(const GnssData& gnss);
  int isToUpdate() const;
  int isToUpdate(double imutime1, double imutime2, double updatetime) const;
  ImuData imuInterpolate(const ImuData& previous, ImuData& current, double time) const;
  ImuData imuCompensate(const ImuData& imu) const;
  void imuCompensateInPlace(ImuData& imu) const;
  void insPropagation();
  void insPropagation(ImuData& imupre, ImuData& imucur);
  void gnssUpdate();
  void gnssUpdate(GnssData& gnss);
  void EKFPredict();
  void EKFPredict(const Matrix& Phi, const Matrix& Qd);
  void EKFUpdate(const std::vector<double>& dz, const Matrix& H, const Matrix& R);
  void stateFeedback();
  void newImuProcess();
  bool checkCov() const;

  const NavState& navState() const;
  NavState getNavState() const;
  const std::vector<double>& getCovariance() const;
  double timestamp() const;
  std::size_t propagationCount() const;
  std::size_t covHealthFailCount() const;
  double covHealthFirstFailureTime() const;
  std::size_t updateCount() const;
  std::size_t positionUpdateCount() const;
  std::size_t velocityUpdateCount() const;
  std::size_t yawUpdateCount() const;
  std::size_t yawNormalCount() const;
  std::size_t yawDownweightCount() const;
  std::size_t yawRejectCount() const;
  std::size_t sourceAwareEvaluationCount() const;
  std::size_t sourceAwareWeightChangedCount() const;
  std::size_t rawDopplerUpdateCount() const;
  std::size_t rawDopplerRejectCount() const;
  std::size_t rawDopplerEpochCount() const;
  std::size_t rawDopplerSatCountMin() const;
  std::size_t rawDopplerSatCountMedian() const;
  std::size_t rawDopplerSatCountMax() const;
  double rawDopplerResidualP95() const;
  RawDopplerFactorStatus rawDopplerStatus() const;
  std::size_t go2AttitudeWeakPriorUpdateCount() const;
  std::size_t go2AttitudeWeakPriorRejectCount() const;
  Go2AttitudeWeakPriorStatus go2AttitudeWeakPriorStatus() const;
  std::size_t go2VelocityDiagnosticPriorUpdateCount() const;
  std::size_t go2VelocityDiagnosticPriorRejectCount() const;
  Go2VelocityDiagnosticPriorStatus go2VelocityDiagnosticPriorStatus() const;

  source_aware::SourceAwareRuntimeStats sourceAwareStats() const;

  void writeSourceAwareTrace(const std::string& output_dir) const;

 private:
  void initializeCovariance();
  void initializeQc();
  void buildErrorStateMatrices(const ImuData& imu, Matrix& F, Matrix& G, Matrix& Phi, Matrix& Qd) const;
  void applyPositionUpdate(GnssData& gnss);

  void applyVelocityUpdate(GnssData& gnss);
  void applyYawUpdate(GnssData& gnss);
  void applyBasicDualYawUpdate(GnssData& gnss);

  void applyRawDopplerUpdateForTime(double update_time);
  void applyGo2AttitudeWeakPriorForTime(double update_time);
  void applyGo2VelocityDiagnosticPriorForTime(double update_time);

  source_aware::SourceWeightResult applySourceAwareWeighting(
      source_aware::MeasurementSource source,
      const source_aware::SourceMetadata& metadata,
      const std::vector<double>& dz,
      const Matrix& H,
      const Matrix& R,
      Matrix& scaled_R);
  double wrapYawResidual(double residual_rad) const;
  Matrix covarianceMatrix() const;
  void setCovarianceMatrix(const Matrix& matrix);

  PortOptions options_;
  NavState pvapre_;
  NavState pvacur_;
  ImuError imuerror_;
  ImuData imupre_;
  ImuData imucur_;
  GnssData gnssdata_;
  Matrix Cov_;
  Matrix Qc_;
  std::vector<double> dx_;
  double timestamp_ = 0.0;
  std::size_t propagation_count_ = 0;
  std::size_t cov_health_fail_count_ = 0;
  double cov_health_first_failure_time_ = -1.0;
  std::size_t update_count_ = 0;
  std::size_t position_update_count_ = 0;
  std::size_t velocity_update_count_ = 0;
  std::size_t yaw_update_count_ = 0;
  std::size_t yaw_normal_count_ = 0;
  std::size_t yaw_downweight_count_ = 0;
  std::size_t yaw_reject_count_ = 0;
  std::size_t source_aware_evaluation_count_ = 0;
  std::size_t source_aware_weight_changed_count_ = 0;
  std::vector<RawDopplerVelocityMeasurement> raw_doppler_measurements_;
  RawDopplerFactorStatus raw_doppler_status_;
  std::vector<double> raw_doppler_residual_norms_;
  std::vector<Go2AttitudeWeakPriorMeasurement> go2_attitude_priors_;
  Go2AttitudeWeakPriorStatus go2_attitude_prior_status_;
  std::vector<double> go2_roll_residuals_;
  std::vector<double> go2_pitch_residuals_;
  std::vector<Go2VelocityDiagnosticPriorMeasurement> go2_velocity_diagnostic_priors_;
  Go2VelocityDiagnosticPriorStatus go2_velocity_diagnostic_prior_status_;
  source_aware::SourceAwarePolicy source_aware_policy_;
  source_aware::SourceAwareTrace source_aware_trace_;
  bool initialized_ = false;
};

}
