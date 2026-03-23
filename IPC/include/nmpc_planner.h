#ifndef NMPC_PLANNER_H_
#define NMPC_PLANNER_H_

#include <algorithm>
#include <cmath>
#include <vector>

#include <Eigen/Dense>
#include <ros/ros.h>

#include "mpc_wrapper.h"

class MPCPlannerClass {
public:
    MPCPlannerClass() = default;

    explicit MPCPlannerClass(ros::NodeHandle& nh) {
        nh.param("/ipc2_node/mpc/horizon", MPC_HORIZON, ACADO_N);
        nh.param("/ipc2_node/mpc/step", MPC_STEP, 0.1);
        nh.param("/ipc2_node/mpc/model_mass", model_mass_, 1.735);

        if (MPC_HORIZON < 1 || MPC_HORIZON > ACADO_N) {
            const int horizon_old = MPC_HORIZON;
            MPC_HORIZON = std::max(1, std::min(ACADO_N, MPC_HORIZON));
            ROS_WARN("[NMPC] horizon(%d) out of range [1, %d], clamped to %d.",
                     horizon_old, ACADO_N, MPC_HORIZON);
        }

        double r_p = 200.0;
        double r_v = 10.0;
        double r_q = 50.0;
        double r_yaw = 50.0;
        double r_p_n = r_p;
        double r_v_n = r_v;
        double r_q_n = r_q;
        double r_yaw_n = r_yaw;
        double r_w = 2.0;
        double r_t = 1.0;
        double r_sfc_eps = 3000.0;
        nh.param("/ipc2_node/mpc/R_p", r_p, r_p);
        nh.param("/ipc2_node/mpc/R_v", r_v, r_v);
        nh.param("/ipc2_node/mpc/R_phi", r_q, r_q);
        nh.param("/ipc2_node/mpc/R_yaw", r_yaw, r_q);
        nh.param("/ipc2_node/mpc/R_pN", r_p_n, r_p_n);
        nh.param("/ipc2_node/mpc/R_vN", r_v_n, r_v_n);
        nh.param("/ipc2_node/mpc/R_phiN", r_q_n, r_q_n);
        nh.param("/ipc2_node/mpc/R_yawN", r_yaw_n, r_yaw);
        nh.param("/ipc2_node/mpc/R_w", r_w, r_w);
        nh.param("/ipc2_node/mpc/R_T", r_t, r_t);
        nh.param("/ipc2_node/mpc/R_sfc_eps", r_sfc_eps, r_sfc_eps);

        q_.setZero();
        q_(0, 0) = r_p;
        q_(1, 1) = r_p;
        q_(2, 2) = r_p;
        q_(3, 3) = r_yaw;
        q_(4, 4) = r_q;
        q_(5, 5) = r_q;
        q_(6, 6) = r_yaw;
        q_(7, 7) = r_v;
        q_(8, 8) = r_v;
        q_(9, 9) = r_v;

        qn_.setZero();
        qn_(0, 0) = r_p_n;
        qn_(1, 1) = r_p_n;
        qn_(2, 2) = r_p_n;
        qn_(3, 3) = r_yaw_n;
        qn_(4, 4) = r_q_n;
        qn_(5, 5) = r_q_n;
        qn_(6, 6) = r_yaw_n;
        qn_(7, 7) = r_v_n;
        qn_(8, 8) = r_v_n;
        qn_(9, 9) = r_v_n;

        r_.setZero();
        r_(0, 0) = r_w;
        r_(1, 1) = r_w;
        r_(2, 2) = r_w;
        r_(3, 3) = r_t;
        if (ACADO_NU > 4) {
            r_(4, 4) = r_sfc_eps;
        }

        double wx_lim = 1.5;
        double wy_lim = 0.7;
        double wz_lim = 2.0;
        double t_min = 0.0;
        double t_max = 2.0 * kGravity * model_mass_;
        double sfc_eps_max = 2.0;
        nh.param("/ipc2_node/mpc/wx_max", wx_lim, wx_lim);
        nh.param("/ipc2_node/mpc/wy_max", wy_lim, wy_lim);
        nh.param("/ipc2_node/mpc/wz_max", wz_lim, wz_lim);
        nh.param("/ipc2_node/mpc/T_min", t_min, t_min);
        nh.param("/ipc2_node/mpc/T_max", t_max, t_max);
        nh.param("/ipc2_node/mpc/sfc_eps_max", sfc_eps_max, sfc_eps_max);

        control_lower_bound_.setZero();
        control_upper_bound_.setZero();
        if (ACADO_NU > 0) {
            control_lower_bound_(0) = -wx_lim;
            control_upper_bound_(0) = wx_lim;
        }
        if (ACADO_NU > 1) {
            control_lower_bound_(1) = -wy_lim;
            control_upper_bound_(1) = wy_lim;
        }
        if (ACADO_NU > 2) {
            control_lower_bound_(2) = -wz_lim;
            control_upper_bound_(2) = wz_lim;
        }
        if (ACADO_NU > 3) {
            control_lower_bound_(3) = t_min;
            control_upper_bound_(3) = t_max;
        }
        if (ACADO_NU > 4) {
            control_lower_bound_(4) = 0.0;
            control_upper_bound_(4) = std::max(0.0, sfc_eps_max);
        }

        nh.param("/ipc2_node/mpc/disturbance_alpha", disturbance_alpha_, 0.1);
        nh.param("/ipc2_node/mpc/disturbance_alpha_z", disturbance_alpha_z_, 0.1);

        nh.param("/ipc2_node/mpc/disturbance_limit", disturbance_limit_, 100.0);
        nh.param("/ipc2_node/mpc/disturbance_limit_z", disturbance_limit_z_, 100.0);

        disturbance_alpha_ = std::max(0.0, std::min(1.0, disturbance_alpha_));
        disturbance_alpha_z_ = std::max(0.0, std::min(1.0, disturbance_alpha_z_));

        nh.param("/ipc2_node/mpc/sfc_active_planes", sfc_active_planes_, kSfcMaxPlanes);
        sfc_active_planes_ = std::max(1, std::min(kSfcMaxPlanes, sfc_active_planes_));

        resetBuffers();
        mpc_wrapper_.initMpc(current_states_, predicted_states_, reference_states_,
                             reference_controls_, q_, r_);
        mpc_wrapper_.setMpcWeight(ACADO_N, qn_, r_);
        writeInputBounds();
    }

    ~MPCPlannerClass() = default;

    void SetStatus(Eigen::Vector3d p0, Eigen::Vector3d v0, Eigen::Vector3d /*a0*/) {
        current_states_(0) = static_cast<real_t>(p0.x());
        current_states_(1) = static_cast<real_t>(p0.y());
        current_states_(2) = static_cast<real_t>(p0.z());
        current_states_(7) = static_cast<real_t>(v0.x());
        current_states_(8) = static_cast<real_t>(v0.y());
        current_states_(9) = static_cast<real_t>(v0.z());
        if (ACADO_NX > 10) {
            current_states_.segment(10, ACADO_NX - 10).setZero();
        }

        X_0_.head<3>() = p0;
        X_0_.segment<3>(3) = v0;
        X_0_.tail<3>().setZero();
    }

    void SetAttitude(const Eigen::Quaterniond& q) {
        const Eigen::Quaterniond qn = q.normalized();
        current_states_(3) = static_cast<real_t>(qn.w());
        current_states_(4) = static_cast<real_t>(qn.x());
        current_states_(5) = static_cast<real_t>(qn.y());
        current_states_(6) = static_cast<real_t>(qn.z());
    }

    void SetGoal(Eigen::Vector3d pr, Eigen::Vector3d vr,
                 Eigen::Vector3d /*ar*/, int step, double yaw_ref = 0.0) {
        if (step < 0 || step > ACADO_N) {
            return;
        }

        // Keep references inside per-step SFC via projection.
        // This improves feasibility and avoids asking solver to chase
        // references that are outside the corridor.
        Eigen::Vector3d p_ref = pr;
        if (step < ACADO_N && step < static_cast<int>(planes_.size()) &&
            planes_[step].rows() > 0) {
            p_ref = ProjectIntoFSC(pr, planes_[step]);
        }

        const double half_yaw = 0.5 * yaw_ref;
        const double q_w_ref = std::cos(half_yaw);
        const double q_z_ref = std::sin(half_yaw);

        reference_states_(0, step) = static_cast<real_t>(p_ref.x());
        reference_states_(1, step) = static_cast<real_t>(p_ref.y());
        reference_states_(2, step) = static_cast<real_t>(p_ref.z());
        reference_states_(3, step) = static_cast<real_t>(q_w_ref);
        reference_states_(4, step) = static_cast<real_t>(0.0);
        reference_states_(5, step) = static_cast<real_t>(0.0);
        reference_states_(6, step) = static_cast<real_t>(q_z_ref);
        reference_states_(7, step) = static_cast<real_t>(vr.x());
        reference_states_(8, step) = static_cast<real_t>(vr.y());
        reference_states_(9, step) = static_cast<real_t>(vr.z());
    }

    void SetDisturbance(const Eigen::Vector3d& d) {
        Eigen::Vector3d filtered = disturbance_filtered_;
        filtered.head<2>() = disturbance_alpha_ * disturbance_filtered_.head<2>()
                           + (1.0 - disturbance_alpha_) * d.head<2>();
        filtered(2) = disturbance_alpha_z_ * disturbance_filtered_(2)
                    + (1.0 - disturbance_alpha_z_) * d(2);

        for (int i = 0; i < 2; ++i) {
            filtered(i) = std::max(-disturbance_limit_,
                                   std::min(disturbance_limit_, filtered(i)));
        }
        filtered(2) = std::max(-disturbance_limit_z_,
                               std::min(disturbance_limit_z_, filtered(2)));
        disturbance_filtered_ = filtered;
        RefreshOnlineData();
    }

    bool Run() {
        mpc_wrapper_.setMpcReference(reference_states_, reference_controls_);
        bool ok = SolveWithOnlineData(online_data_);

        if (!ok) {
            const Eigen::Matrix<real_t, ACADO_NOD, ACADO_N + 1> relaxed_online =
                BuildRelaxedOnlineData(online_data_);
            ok = SolveWithOnlineData(relaxed_online);
            if (ok) {
                ROS_WARN_THROTTLE(0.5, "[NMPC] solve failed with SFC, fallback solve without SFC succeeded.");
            }
        }

        // If both attempts fail, ACADO may be stuck in a bad numeric state.
        // Reinitialize solver memory once and retry.
        if (!ok) {
            ResetSolver();
            ok = SolveWithOnlineData(online_data_);
            if (!ok) {
                const Eigen::Matrix<real_t, ACADO_NOD, ACADO_N + 1> relaxed_online =
                    BuildRelaxedOnlineData(online_data_);
                ok = SolveWithOnlineData(relaxed_online);
            }
            if (ok) {
                ROS_WARN_THROTTLE(0.5, "[NMPC] recovered after solver reset.");
            }
        }

        if (ok) {
            last_control_ = current_control_;
        } else {
            current_control_ = BuildSafeHoverControl();
        }
        return ok;
    }

    void GetBodyrateCmd(Eigen::Vector3d& w_cmd, double& T_cmd) const {
        w_cmd << current_control_(0), current_control_(1), current_control_(2);
        T_cmd = current_control_(3);
    }

    double GetSfcSlackCmd() const {
        if (ACADO_NU > 4) {
            return static_cast<double>(current_control_(4));
        }
        return 0.0;
    }

    void GetPredictedPath(std::vector<Eigen::Vector3d>& path) const {
        path.clear();
        path.reserve(MPC_HORIZON + 1);

        // Read ACADO state buffer with explicit stage-major indexing:
        // x[k * ACADO_NX + j], k in [0, N], j in [0, NX-1].
        // This avoids any ambiguity from matrix mapping/storage order.
        for (int i = 0; i <= MPC_HORIZON; ++i) {
            const int base = i * ACADO_NX;
            path.emplace_back(acadoVariables.x[base + 0],
                              acadoVariables.x[base + 1],
                              acadoVariables.x[base + 2]);
        }
    }

    void GetReferencePath(std::vector<Eigen::Vector3d>& path) const {
        path.clear();
        path.reserve(MPC_HORIZON + 1);
        for (int i = 0; i <= MPC_HORIZON; ++i) {
            path.emplace_back(reference_states_(0, i), reference_states_(1, i), reference_states_(2, i));
        }
    }

    void GetOptimCmd(Eigen::Vector3d& u, int segment) {
        if (segment < 0) {
            segment = 0;
        }
        if (segment >= ACADO_N) {
            segment = ACADO_N - 1;
        }
        Eigen::Matrix<real_t, ACADO_NU, 1> u_full;
        mpc_wrapper_.getMpcControl(u_full, segment);
        u << u_full(0), u_full(1), u_full(2);
    }

    void SystemModel(Eigen::MatrixXd& A, Eigen::MatrixXd& B, double dt) {
        A = Eigen::MatrixXd::Identity(9, 9);
        A(0, 3) = dt;
        A(1, 4) = dt;
        A(2, 5) = dt;
        A(3, 6) = dt;
        A(4, 7) = dt;
        A(5, 8) = dt;

        B = Eigen::MatrixXd::Zero(9, 3);
        B(6, 0) = dt;
        B(7, 1) = dt;
        B(8, 2) = dt;
    }

    void SetFSC(Eigen::Matrix<double, Eigen::Dynamic, 4>& planes, int step) {
        if (step < 0 || step >= ACADO_N || step >= static_cast<int>(planes_.size())) {
            return;
        }
        // Keep only the first sfc_active_planes_ constraints for this step.
        // Extra planes are discarded here so all downstream consumers
        // (online data + reference projection) see the same active set.
        const int keep = std::max(0, std::min(static_cast<int>(planes.rows()), sfc_active_planes_));
        if (keep == 0) {
            planes_[step].resize(0, 4);
        } else {
            planes_[step] = planes.topRows(keep);
        }
        RefreshOnlineData();
    }

    bool IsInFSC(Eigen::Vector3d pos, Eigen::Matrix<double, Eigen::Dynamic, 4>& planes) {
        if (planes.rows() == 0) {
            return false;
        }
        for (int i = 0; i < planes.rows(); ++i) {
            if (pos.x() * planes(i, 0) + pos.y() * planes(i, 1) +
                pos.z() * planes(i, 2) + planes(i, 3) > 0.0) {
                return false;
            }
        }
        return true;
    }

    Eigen::VectorXd X_0_;
    Eigen::VectorXd X_r_;
    int MPC_HORIZON{ACADO_N};
    double MPC_STEP{0.1};
    int ctrl_delay_{0};
    std::vector<Eigen::Vector3d> u_last_;
    std::vector<Eigen::Matrix<double, Eigen::Dynamic, 4>> planes_;

private:
    void resetBuffers() {
        current_states_.setZero();
        current_states_(3) = static_cast<real_t>(1.0);
        predicted_states_ = current_states_.replicate(1, ACADO_N + 1);

        reference_states_.setZero();
        for (int i = 0; i <= ACADO_N; ++i) {
            reference_states_(2, i) = static_cast<real_t>(1.0);
            reference_states_(3, i) = static_cast<real_t>(1.0);
        }

        reference_controls_.setZero();
        if (ACADO_NU > 3) {
            reference_controls_.row(3).setConstant(static_cast<real_t>(model_mass_ * kGravity));
        }
        current_control_ = BuildSafeHoverControl();
        last_control_ = current_control_;
        online_data_.setZero();
        disturbance_filtered_.setZero();

        X_0_ = Eigen::VectorXd::Zero(9);
        X_r_ = Eigen::VectorXd::Zero(9 * ACADO_N);
        planes_.resize(ACADO_N);
        for (auto& p : planes_) {
            p.resize(0, 4);
        }
        RefreshOnlineData();
    }

    Eigen::Vector3d ProjectIntoFSC(
        const Eigen::Vector3d& p,
        const Eigen::Matrix<double, Eigen::Dynamic, 4>& planes) const {
        if (planes.rows() == 0) {
            return p;
        }

        Eigen::Vector3d proj = p;
        for (int iter = 0; iter < 3; ++iter) {
            bool inside = true;
            for (int i = 0; i < planes.rows(); ++i) {
                const Eigen::Vector3d n(planes(i, 0), planes(i, 1), planes(i, 2));
                const double n2 = n.squaredNorm();
                if (n2 < 1e-9) {
                    continue;
                }
                const double violation = n.dot(proj) + planes(i, 3);
                if (violation > 0.0) {
                    proj -= (violation / n2) * n;
                    inside = false;
                }
            }
            if (inside) {
                break;
            }
        }
        return proj;
    }

    void RefreshOnlineData() {
        for (int col = 0; col <= ACADO_N; ++col) {
            const int step = (col >= ACADO_N) ? (ACADO_N - 1) : col;
            const Eigen::Matrix<double, Eigen::Dynamic, 4>* planes = nullptr;
            if (step >= 0 && step < static_cast<int>(planes_.size())) {
                planes = &planes_[step];
            }
            FillOnlineDataColumn(col, planes);
        }
    }

    void FillOnlineDataColumn(
        int col, const Eigen::Matrix<double, Eigen::Dynamic, 4>* planes) {
        online_data_.col(col).setZero();

        online_data_(0, col) = static_cast<real_t>(disturbance_filtered_.x());
        online_data_(1, col) = static_cast<real_t>(disturbance_filtered_.y());
        online_data_(2, col) = static_cast<real_t>(disturbance_filtered_.z());

        const int rows = (planes == nullptr) ? 0 : planes->rows();
        const int n = std::min(rows, sfc_active_planes_);
        for (int i = 0; i < n; ++i) {
            const int base = kDisturbanceDim + i * kSfcCoeffDim;
            const double nx = (*planes)(i, 0);
            const double ny = (*planes)(i, 1);
            const double nz = (*planes)(i, 2);
            const double nd = (*planes)(i, 3);

            online_data_(base + 0, col) = static_cast<real_t>(nx);
            online_data_(base + 1, col) = static_cast<real_t>(ny);
            online_data_(base + 2, col) = static_cast<real_t>(nz);
            online_data_(base + 3, col) = static_cast<real_t>(nd);
        }
    }

    void writeInputBounds() {
        Eigen::Map<Eigen::Matrix<real_t, ACADO_NU, ACADO_N, Eigen::ColMajor>> lb(acadoVariables.lbValues);
        Eigen::Map<Eigen::Matrix<real_t, ACADO_NU, ACADO_N, Eigen::ColMajor>> ub(acadoVariables.ubValues);
        lb = control_lower_bound_.replicate(1, ACADO_N);
        ub = control_upper_bound_.replicate(1, ACADO_N);
    }

    bool SolveWithOnlineData(
        const Eigen::Matrix<real_t, ACADO_NOD, ACADO_N + 1>& online_data) {
        mpc_wrapper_.setMpcOnlineData(online_data);
        if (!mpc_wrapper_.solveMpc(current_states_)) {
            return false;
        }
        mpc_wrapper_.getMpcControl(current_control_, 0);
        if (!current_control_.allFinite()) {
            return false;
        }
        return true;
    }

    Eigen::Matrix<real_t, ACADO_NOD, ACADO_N + 1> BuildRelaxedOnlineData(
        const Eigen::Matrix<real_t, ACADO_NOD, ACADO_N + 1>& online_data) const {
        Eigen::Matrix<real_t, ACADO_NOD, ACADO_N + 1> relaxed = online_data;
        if (ACADO_NOD > kDisturbanceDim) {
            relaxed.block(kDisturbanceDim, 0, ACADO_NOD - kDisturbanceDim, ACADO_N + 1).setZero();
        }
        return relaxed;
    }

    void ResetSolver() {
        predicted_states_ = current_states_.replicate(1, ACADO_N + 1);
        mpc_wrapper_.initMpc(current_states_, predicted_states_, reference_states_,
                             reference_controls_, q_, r_);
        mpc_wrapper_.setMpcWeight(ACADO_N, qn_, r_);
        writeInputBounds();
    }

    Eigen::Matrix<real_t, ACADO_NU, 1> BuildSafeHoverControl() const {
        Eigen::Matrix<real_t, ACADO_NU, 1> u;
        u.setZero();
        if (ACADO_NU > 3) {
            const double t_hover = model_mass_ * kGravity;
            const double t_safe = std::max(
                static_cast<double>(control_lower_bound_(3)),
                std::min(static_cast<double>(control_upper_bound_(3)), t_hover));
            u(3) = static_cast<real_t>(t_safe);
        }
        return u;
    }

    static constexpr double kGravity = 9.8066;
    static constexpr int kDisturbanceDim = 3;
    static constexpr int kSfcCoeffDim = 4;
    static constexpr int kSfcMaxPlanes = ACADO_NPAC;

    MpcWrapper mpc_wrapper_;
    double model_mass_{1.735};
    double disturbance_alpha_{0.8};
    double disturbance_limit_{5.0};
    double disturbance_alpha_z_{0.95};
    double disturbance_limit_z_{2.0};
    int sfc_active_planes_{kSfcMaxPlanes};
    Eigen::Vector3d disturbance_filtered_{Eigen::Vector3d::Zero()};

    Eigen::Matrix<real_t, ACADO_NX, 1> current_states_;
    Eigen::Matrix<real_t, ACADO_NX, ACADO_N + 1> predicted_states_;
    Eigen::Matrix<real_t, kCostStateSize, ACADO_N + 1> reference_states_;
    Eigen::Matrix<real_t, ACADO_NU, ACADO_N> reference_controls_;
    Eigen::Matrix<real_t, ACADO_NOD, ACADO_N + 1> online_data_;

    Eigen::Matrix<real_t, kCostStateSize, kCostStateSize> q_;
    Eigen::Matrix<real_t, kCostStateSize, kCostStateSize> qn_;
    Eigen::Matrix<real_t, ACADO_NU, ACADO_NU> r_;
    Eigen::Matrix<real_t, ACADO_NU, 1> control_lower_bound_;
    Eigen::Matrix<real_t, ACADO_NU, 1> control_upper_bound_;
    Eigen::Matrix<real_t, ACADO_NU, 1> current_control_;
    Eigen::Matrix<real_t, ACADO_NU, 1> last_control_;
};

#endif  // NMPC_PLANNER_H_
