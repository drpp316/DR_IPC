#include <math.h>
#include <iostream>
#include "disturbance_observer.h"

DisturbanceObserver::DisturbanceObserver(double dt)
        : dt_(dt),
          l_(Eigen::VectorXd::Zero(3)),
          z_(Eigen::VectorXd::Zero(10)),
          d_(Eigen::VectorXd::Zero(3))
{
    omega = 2.0;
    l_.block<3, 1>(0, 0) << 3 * omega, 3 * omega * omega, omega * omega * omega;
}

void DisturbanceObserver::kalman_geso_update(const Eigen::VectorXd& y, const Eigen::VectorXd& u, double m, double g_z)
{
    // KF state: [p(0:2), q_wxyz(3:6), v(7:9)]
    Eigen::MatrixXd A(kf_.state_size_, kf_.state_size_);
    A.setZero();
    A.block<3, 3>(0, 7) = Eigen::Matrix3d::Identity();

    const double qw = y(3), qx = y(4), qy = y(5), qz = y(6);

    Eigen::MatrixXd B_u(kf_.state_size_, kf_.control_size_);
    Eigen::MatrixXd B_d(kf_.state_size_, 3);
    B_u.setZero();
    B_d.setZero();

    // Quaternion kinematics w.r.t body rates
    B_u(3, 0) = -0.5 * qx;
    B_u(3, 1) = -0.5 * qy;
    B_u(3, 2) = -0.5 * qz;

    B_u(4, 0) =  0.5 * qw;
    B_u(4, 1) = -0.5 * qz;
    B_u(4, 2) =  0.5 * qy;

    B_u(5, 0) =  0.5 * qz;
    B_u(5, 1) =  0.5 * qw;
    B_u(5, 2) = -0.5 * qx;

    B_u(6, 0) = -0.5 * qy;
    B_u(6, 1) =  0.5 * qx;
    B_u(6, 2) =  0.5 * qw;

    // Thrust direction in world frame
    Eigen::Vector3d z_b;
    z_b << 2.0 * (qw * qy + qx * qz),
           2.0 * (qy * qz - qw * qx),
           1.0 - 2.0 * (qx * qx + qy * qy);
    B_u.block<3, 1>(7, 3) = z_b / m;

    B_d.block<3, 3>(7, 0) = Eigen::Matrix3d::Identity();

    const Eigen::MatrixXd A2 = A * A;
    const Eigen::MatrixXd A3 = A2 * A;
    const Eigen::MatrixXd A4 = A3 * A;

    A = Eigen::MatrixXd::Identity(kf_.state_size_, kf_.state_size_)
      + dt_ * A
      + 0.5 * dt_ * dt_ * A2
      + (1.0 / 6.0) * dt_ * dt_ * dt_ * A3
      + (1.0 / 24.0) * dt_ * dt_ * dt_ * dt_ * A4;

    B_u = dt_ * B_u;
    B_d = dt_ * B_d;

    Eigen::VectorXd tail = B_d * d_.block<3, 1>(0, 0)
                         - g_z * dt_ * Eigen::VectorXd::Unit(kf_.state_size_, 9);

    kf_.predict(A, B_u, tail, u);
    // Use full odometry measurement [p, q, v] for update to reduce model drift
    // and improve response (especially when maneuvering with disturbances).
    kf_.update(y.block<10, 1>(0, 0));

    const Eigen::VectorXd x_kf = kf_.getState();

    const double f_x = z_b(0) * u(3) / m;
    const double f_y = z_b(1) * u(3) / m;
    const double f_z = z_b(2) * u(3) / m - g_z;

    Eigen::Vector3d e;
    if (interconnected_mode_) {
        // Interconnected mode: KF filtered position drives ESO error
        e(0) = x_kf(0) - z_(0);
        e(1) = x_kf(1) - z_(1);
        e(2) = x_kf(2) - z_(2);
    } else {
        // Legacy mode: raw measurement position drives ESO error
        e = y.block<3, 1>(0, 0) - z_.block<3, 1>(0, 0);
    }

    // ESO state update
    z_(0) += (z_(3) + l_(0) * e(0)) * dt_;
    z_(1) += (z_(4) + l_(0) * e(1)) * dt_;
    z_(2) += (z_(5) + l_(0) * e(2)) * dt_;

    z_(3) += (f_x + d_(0) + l_(1) * e(0)) * dt_;
    z_(4) += (f_y + d_(1) + l_(1) * e(1)) * dt_;
    z_(5) += (f_z + d_(2) + l_(1) * e(2)) * dt_;

    // Disturbance estimate update
    d_(0) += l_(2) * e(0) * dt_;
    d_(1) += l_(2) * e(1) * dt_;
    d_(2) += l_(2) * e(2) * dt_;
}
