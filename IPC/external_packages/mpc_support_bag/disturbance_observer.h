#pragma once
#include <ros/ros.h>
#include <Eigen/Dense>
#include <deque>

class DiscreteKalmanFilter {
public:
    DiscreteKalmanFilter(int state_size, int measurement_size, int control_size) {
        // 初始化状态向量、协方差矩阵等
        x_ = Eigen::VectorXd(state_size);  // 状态向量
        P_ = Eigen::MatrixXd(state_size, state_size);  // 状态协方差矩阵
        A_ = Eigen::MatrixXd(state_size, state_size);  // 系统矩阵A
        B_ = Eigen::MatrixXd(state_size, control_size);  // 控制矩阵B
        H_ = Eigen::MatrixXd(measurement_size, state_size);  // 测量矩阵H

        R_ = Eigen::MatrixXd(measurement_size, measurement_size);  // 测量噪声协方差矩阵R
        Q_ = Eigen::MatrixXd(state_size, state_size);  // 过程噪声协方差矩阵Q
        Qa_ = Eigen::MatrixXd(state_size, state_size);  // 自适应噪声协方差矩阵Qa
        Ck_ = Eigen::MatrixXd(measurement_size, measurement_size);  // 自适应噪声协方差矩阵Ck_
        I_ = Eigen::MatrixXd::Identity(state_size, state_size);  // 单位矩阵
        K_ = Eigen::MatrixXd(state_size, measurement_size); // 卡尔曼增益
        innovation_ = Eigen::VectorXd(measurement_size);  // 新息（测量残差）
        state_size_ = state_size;
        measurement_size_ = measurement_size;
        control_size_ = control_size;
    }

    void init(const Eigen::VectorXd& initial_state, const Eigen::MatrixXd& initial_covariance,
              const Eigen::MatrixXd& system_matrix, const Eigen::MatrixXd& control_matrix, 
              const Eigen::MatrixXd& measurement_matrix, const Eigen::MatrixXd& measurement_noise, 
              const Eigen::MatrixXd& process_noise) {
        x_ = initial_state;  // 初始状态
        P_ = initial_covariance;  // 初始协方差矩阵
        A_ = system_matrix;  // 系统矩阵A
        B_ = control_matrix;  // 控制矩阵B
        H_ = measurement_matrix;  // 测量矩阵H
        R_ = measurement_noise;  // 测量噪声协方差矩阵R
        Q_ = process_noise;  // 过程噪声协方差矩阵Q
    }

    void init(const Eigen::VectorXd& initial_state)
    {
        innovation_ = Eigen::VectorXd::Zero(measurement_size_);
        m_ = 50;
        x_ = initial_state;
        P_ = 0.01 * Eigen::MatrixXd::Identity(state_size_, state_size_);
        P_.block<3,3>(7,7) = 0.1*Eigen::MatrixXd::Identity(3, 3);

        Q_ = (0.001 * process_noise_scale_) * Eigen::MatrixXd::Identity(state_size_, state_size_);
        Q_.block<3,3>(7,7) = (0.0016 * process_noise_scale_) * Eigen::MatrixXd::Identity(3, 3);
        Qa_ = Q_;
        
        R_ = (0.0008 * measurement_noise_scale_) * Eigen::MatrixXd::Identity(measurement_size_, measurement_size_);
        // R_(0,0) = 0.01;
        // R_(1,1) = 0.01;
        H_ = Eigen::MatrixXd::Zero(measurement_size_, state_size_);
        H_.block(0, 0, measurement_size_, measurement_size_) = Eigen::MatrixXd::Identity(measurement_size_, measurement_size_);
        A_ = Eigen::MatrixXd::Identity(state_size_, state_size_);
        B_ = Eigen::MatrixXd::Zero(state_size_, control_size_);
        Ck_ = Eigen::MatrixXd::Zero(measurement_size_, measurement_size_);
    }
    void predict(const Eigen::MatrixXd A, const Eigen::MatrixXd Bu, const Eigen::VectorXd tail, const Eigen::VectorXd& control_input) {
        // 预测步骤：x = A * x + B * u
        B_ = Bu;
        A_ = A;
        x_ = A_ * x_ + B_ * control_input + tail;
        
        // 更新协方差矩阵
        // P_ = A_ * P_ * A_.transpose() + Q_;
        P_ = A_ * P_ * A_.transpose() + Qa_;
    }

    void update(const Eigen::VectorXd& measurement) {
        // 更新步骤
        // 计算创新（测量残差）
        innovation_ = measurement - H_ * x_;

        samples_.push_back(innovation_);
        Ck_ += innovation_ * innovation_.transpose();
        if (samples_.size() > m_) {
            Eigen::VectorXd old_q = samples_.front();
            Ck_ -= old_q * old_q.transpose();
            samples_.pop_front();
        }

        // R_ = Ck_/static_cast<double>(samples_.size()) - H_ * P_ * H_.transpose();
        // std::cout << "R_: " << R_ << std::endl;

        // 计算卡尔曼增益
        Eigen::MatrixXd S = H_ * P_ * H_.transpose() + R_;
        K_ = P_ * H_.transpose() * S.inverse();

        // 更新状态估计
        x_ = x_ + K_ * innovation_;

        // 更新协方差矩阵
        P_ = (I_ - K_ * H_) * P_;
        // std::cout << "P" << P_ << std::endl;

        
        Qa_ = K_ * Ck_/static_cast<double>(samples_.size()) * K_.transpose();
        // std::cout << "Qa_: " << Qa_ << std::endl;
    }

    Eigen::VectorXd getState() const {
        return x_;
    }

    void setNoiseScale(double process_scale, double measurement_scale) {
        if (process_scale > 0.0) {
            process_noise_scale_ = process_scale;
        }
        if (measurement_scale > 0.0) {
            measurement_noise_scale_ = measurement_scale;
        }
    }

    int state_size_;
    int measurement_size_;
    int control_size_;
    Eigen::MatrixXd Qa_; // 自适应噪声协方差矩阵

private:
    Eigen::VectorXd x_;  // 状态向量
    Eigen::VectorXd innovation_;  // 新息（测量残差）
    Eigen::MatrixXd P_;  // 状态协方差矩阵
    Eigen::MatrixXd A_;  // 系统矩阵A
    Eigen::MatrixXd B_;  // 控制矩阵B
    Eigen::MatrixXd H_;  // 测量矩阵H
    Eigen::MatrixXd R_;  // 测量噪声协方差矩阵R
    Eigen::MatrixXd Q_;  // 过程噪声协方差矩阵Q
    Eigen::MatrixXd I_;  // 单位矩阵
    Eigen::MatrixXd K_; // 卡尔曼增益
    Eigen::MatrixXd Ck_; // 新息协方差矩阵
    std::deque<Eigen::VectorXd> samples_;
    int m_;
    double process_noise_scale_{1.0};
    double measurement_noise_scale_{1.0};
};

class DisturbanceObserver
{
public:
    DisturbanceObserver(double dt);
    DisturbanceObserver() = default;
    ~DisturbanceObserver() = default;

    // KF state/measurement:
    // state x = [p(3), q(4), v(3)] -> 10D
    // measurement y = [p(3), q(4), v(3)] -> 10D
    DiscreteKalmanFilter kf_{10, 10, 4};

    // 主流程使用的 KF-ESO 更新
    void kalman_geso_update(const Eigen::VectorXd& y, const Eigen::VectorXd& u, double m, double g_z);
    
    // 获取估计状态
    Eigen::VectorXd getState() const {
        return z_;
    }

    // 获取kf估计状态
    Eigen::VectorXd getKfState() const {
        return kf_.getState();
    }

    // 获取估计扰动
    Eigen::VectorXd getDisturbance() const {
        return d_;
    }

    void setFirstZ(const Eigen::VectorXd& z) {
        z_ = z;
        kf_.init(z_);
    }

    void setDt(double dt) {
        dt_ = dt;
    }

    double getDt() const {
        return dt_;
    }

    void setOmega(double o) {
        if (o <= 0.0) {
            return;
        }
        omega = o;
        if (l_.size() >= 3) {
            l_(0) = 3.0 * omega;
            l_(1) = 3.0 * omega * omega;
            l_(2) = omega * omega * omega;
        }
    }

    // 互联观测器开关：
    // true  -> KF 给 DOB 状态滤波，DOB 给 KF 扰动估计（双向互联）
    // false -> 退回旧逻辑（DOB 直接用测量位置误差）
    void setInterconnectedMode(bool enabled) {
        interconnected_mode_ = enabled;
    }

    bool getInterconnectedMode() const {
        return interconnected_mode_;
    }

    void setKalmanNoiseScale(double process_scale, double measurement_scale) {
        kf_.setNoiseScale(process_scale, measurement_scale);
    }

private:
    double dt_;               // 采样时间
    Eigen::VectorXd l_;       // ESO增益

    Eigen::VectorXd z_;       // 扩展状态 [p, q, v] 的估计容器
    Eigen::VectorXd d_;       // 扰动估计 [d_x, d_y, d_z]

    double omega = 2.0;
    bool interconnected_mode_ = true;

};
