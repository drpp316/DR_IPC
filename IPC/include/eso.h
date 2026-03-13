#pragma once
#include <Eigen/Dense>
#include <mutex>

// Extended State Observer based on accurate dynamics: dv/dt = -g*e_z + R*T*e_z/m + d
// State: z1 ~ velocity (world frame), z2 ~ disturbance acceleration d (world frame)
// Model: dv/dt = -g*e_z + R*[0,0,T]/m + d
// where:
//   v: velocity (estimated by z1)
//   T: thrust command from NMPC (scalar, N)
//   R: rotation matrix from body to world
//   d: disturbance acceleration (estimated by z2)
//   g, e_z: gravity magnitude and upward unit vector
//   m: vehicle mass (kg)
// 
// Observer equations (with feedback e = v_odom - z1):
//   dz1/dt = -g*e_z + R*[0,0,T]/m + z2 + L1*e
//   dz2/dt = L2*e
//
// The observer:
//   - Uses NMPC's true thrust output T (not post-processed)
//   - Computes the 3D acceleration R*[0,0,T]/m from the thrust in body frame
//   - Uses velocity feedback from odometry to drive convergence
//   - Leaves all deviations (wind, model errors, etc.) to be estimated in z2
//
class ESO {
public:
  ESO(double dt = 0.005);
  // update with NMPC thrust output and measured odometry velocity
  // Inputs:
  //   T: thrust command from NMPC (scalar, in Newtons)
  //   R: rotation matrix from body to world frame (to transform thrust vector [0,0,T] to world)
  //   omega_body: body angular velocity (rad/s)
  //   q: orientation quaternion (for future use)
  //   v_odom: odometry velocity estimate (world frame)
  //   m: vehicle mass (kg)
  // Model: dv/dt = -g*e_z + R*[0,0,T]/m + d
  // Outputs: disturbance acceleration estimate d_hat in world frame (m/s^2)
  void update(double T, const Eigen::Matrix3d &R, const Eigen::Vector3d &omega_body, 
              const Eigen::Quaterniond &q, const Eigen::Vector3d &v_odom, double m);
  Eigen::Vector3d get_d_hat(); // disturbance acceleration in world frame [m/s^2]
  Eigen::Vector3d get_d_hat_with_timestamp(double &tstamp);
private:
  double dt_;
  double last_tstamp_;
  Eigen::Vector3d d_hat_; // estimated disturbance acceleration (m/s^2, world frame)
  std::mutex mtx_;
  // ESO internal states
  Eigen::Vector3d z1_, z2_; // z1: velocity estimate, z2: disturbance acceleration estimate
  Eigen::Matrix3d L1_, L2_; // observer gains (diagonal matrices)
  double g_ = 9.81;             // gravity magnitude (m/s^2)
  Eigen::Vector3d e_z_ = Eigen::Vector3d(0, 0, 1); // gravity direction in world frame
};
