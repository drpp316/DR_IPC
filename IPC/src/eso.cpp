#include "../include/eso.h"
#include <Eigen/Geometry>
#include <ros/ros.h>

ESO::ESO(double dt) : dt_(dt), last_tstamp_(0.0) {
  d_hat_.setZero();
  z1_.setZero();
  z2_.setZero();
  L1_.setIdentity() * 50.0;
  L2_.setIdentity() * 200.0; // tunable gains for ESO convergence
}

void ESO::update(double T, const Eigen::Matrix3d &R, const Eigen::Vector3d &omega_body, 
                  const Eigen::Quaterniond &q, const Eigen::Vector3d &v_odom, double m) {
  // Model: dv/dt = -g*e_z + R*[0,0,T]/m + d
  // 
  // Observer structure:
  //   dz1/dt = -g*e_z + R*[0,0,T]/m + z2 + L1*(v_odom - z1)
  //   dz2/dt = L2*(v_odom - z1)
  //
  // Compute thrust-induced acceleration in world frame: a_thrust = R*[0,0,T]/m
  Eigen::Vector3d thrust_body(0.0, 0.0, T);
  Eigen::Vector3d a_thrust = (R * thrust_body) / m;
  
  // Observation error: e = v_odom - z1 (velocity measurement minus estimate)
  Eigen::Vector3d e = v_odom - z1_;
  
  // Gravity term in world frame (downward acceleration)
  Eigen::Vector3d g_term = -g_ * e_z_;
  
  // ESO discrete integrator:
  // Update z1 (velocity estimate): dz1/dt = -g*e_z + a_thrust + z2 + L1*e
  z1_ += dt_ * (g_term + a_thrust + z2_ + L1_ * e);
  
  // Update z2 (disturbance acceleration estimate): dz2/dt = L2*e
  z2_ += dt_ * (L2_ * e);
  
  // Store and timestamp the disturbance estimate
  {
    std::lock_guard<std::mutex> lk(mtx_);
    d_hat_ = z2_;  // estimated disturbance acceleration in world frame (m/s^2)
    last_tstamp_ = ros::Time::now().toSec();
  }
}

Eigen::Vector3d ESO::get_d_hat() {
  std::lock_guard<std::mutex> lk(mtx_);
  return d_hat_;
}

Eigen::Vector3d ESO::get_d_hat_with_timestamp(double &tstamp) {
  std::lock_guard<std::mutex> lk(mtx_);
  tstamp = last_tstamp_;
  return d_hat_;
}
