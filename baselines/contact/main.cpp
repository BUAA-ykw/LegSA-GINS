// Contact-inertial heading-rotation experiment.
#include "hartley_inekf/backend.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <vector>
using namespace hartley;
struct Sample { long long ns; Vector3 gyro, accel; std::array<Vector3,4> feet; unsigned contact, added, removed; };
double median(std::vector<double> a) {
  std::sort(a.begin(),a.end()); auto n=a.size();
  return n%2 ? a[n/2] : .5*(a[n/2-1]+a[n/2]);
}
std::vector<ContactMeasurement> measurements(const Sample& s,unsigned mask) {
  std::vector<ContactMeasurement> out;
  for(int i=0;i<4;++i) if(mask&(1u<<i)) out.push_back({i,s.feet[i],.0001*Matrix3::Identity()});
  return out;
}
int main(int argc,char** argv) {
  try {
    if(argc!=4) { std::cerr<<"Usage: contact_inekf INPUT.csv INITIAL_YAW_DEG OUTPUT.csv\n"; return 2; }
    std::ifstream input(argv[1]); std::string line; std::getline(input,line); std::vector<Sample> rows;
    while(std::getline(input,line)) {
      if(line.empty())continue;
      std::replace(line.begin(),line.end(),',',' ');std::istringstream in(line);Sample s;
      in>>s.ns;
      for(int j=0;j<3;++j)in>>s.gyro[j];for(int j=0;j<3;++j)in>>s.accel[j];
      for(auto& foot:s.feet)for(int j=0;j<3;++j)in>>foot[j];in>>s.contact>>s.added>>s.removed;
      if(!in)throw std::runtime_error("invalid contact input row");rows.push_back(s);
    }
    if(rows.size()<2)throw std::runtime_error("contact input has fewer than two rows");
    Vector3 gyro_sum=Vector3::Zero(),accel_sum=Vector3::Zero();std::vector<double> gn,an;std::size_t count=0;
    for(const auto& s:rows) {
      if(s.ns>=rows.front().ns+5000000000LL)break;
      gyro_sum+=s.gyro;accel_sum+=s.accel;gn.push_back(s.gyro.norm());an.push_back(s.accel.norm());++count;
    }
    if(median(gn)>=.05||std::abs(median(an)-9.81)>=.5)throw std::runtime_error("static initialization unavailable");
    const auto accel=accel_sum/static_cast<double>(count);StateMean mean;
    double roll=std::atan2(accel.y(),accel.z()),pitch=std::atan2(-accel.x(),std::hypot(accel.y(),accel.z()));
    mean.rotation=Eigen::AngleAxisd(pitch,Vector3::UnitY()).toRotationMatrix()*Eigen::AngleAxisd(roll,Vector3::UnitX()).toRotationMatrix();
    mean.gyro_bias=gyro_sum/static_cast<double>(count);mean.accelerometer_bias=accel+mean.rotation.transpose()*Vector3(0,0,-9.81);
    Matrix cov=Matrix::Zero(15,15);double angle=30*M_PI/180;
    cov.diagonal()<<angle*angle,angle*angle,angle*angle,1,1,1,.01,.01,.01,.000025,.000025,.000025,.0025,.0025,.0025;
    ContinuousNoiseDensity noise;
    noise.gyro_measurement_rad_per_s_per_sqrt_hz=2.865130e-4;
    noise.accelerometer_measurement_m_per_s2_per_sqrt_hz=1.285395e-3;
    noise.gyro_bias_rw_rad_per_s2_per_sqrt_hz=2.996871e-5;
    noise.accelerometer_bias_rw_m_per_s3_per_sqrt_hz=1.594412e-4;
    HartleyInEkf filter(mean,cov,H5Eq61NoisePolicy::go2ImuPaperContact(noise,.05));
    filter.initializeContactsEq32WithIndependentPrior(measurements(rows.front(),rows.front().contact));
    const double yaw=std::stod(argv[2]);
    if(yaw!=0)filter.applyInitialGaugeTransform(expSO3(Vector3(0,0,yaw*M_PI/180)));
    std::ofstream output(argv[3]);if(!output)throw std::runtime_error("cannot open output");
    output<<"timestamp_ns,time_s,x_m,y_m,z_m,yaw_deg\n"<<std::setprecision(17);
    for(std::size_t i=0;i<rows.size();++i) {
      if(i) {
        double dt=(rows[i].ns-rows[i-1].ns)*1e-9;if(dt<=0)throw std::runtime_error("nonincreasing time");
        filter.propagate(rows[i-1].gyro,rows[i-1].accel,dt);
        std::vector<int> ended;for(int j=0;j<4;++j)if(rows[i].removed&(1u<<j))ended.push_back(j);
        filter.processContactLifecycle(measurements(rows[i],rows[i].contact&~rows[i].added),ended,measurements(rows[i],rows[i].added));
      }
      const auto& s=filter.stateMean();
      output<<rows[i].ns<<','<<(rows[i].ns-rows.front().ns)*1e-9<<','<<s.position.x()<<','<<s.position.y()<<','<<s.position.z()<<','<<std::atan2(s.rotation(1,0),s.rotation(0,0))*180/M_PI<<'\n';
    }
    return 0;
  } catch(const std::exception& e) {std::cerr<<e.what()<<'\n';return 1;}
}
