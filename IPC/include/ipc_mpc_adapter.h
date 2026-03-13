#ifndef IPC_MPC_ADAPTER_H_
#define IPC_MPC_ADAPTER_H_

/**
 * ipc_mpc_adapter.h
 * ─────────────────────────────────────────────────────────────────────────────
 * Drop-in replacement for mpc.h.
 *
 * Architecture
 * ────────────
 *   planner.cpp  (同步调用，主线程)
 *       │  SetStatus / SetAttitude / SetGoal / Run / GetOptimCmd / SystemModel
 *       ▼
 *   MPCPlannerClass   ← 本文件，桥接层
 *       │  直接调用 MpcWrapper 的同步接口
 *       ▼
 *   MpcWrapper  →  ACADO/qpOASES
 *
 * 注意：完全绕过 MpcRosApplication，不启动任何后台线程。
 * planner.cpp 的 TimerCallback 每次调用 Run()，MpcWrapper 立刻求解并返回。
 *
 * planner.cpp 只需两处改动：
 *   1. #include "mpc.h"  →  #include "ipc_mpc_adapter.h"
 *   2. OdomCallback末尾加：  mpc_->SetAttitude(odom_q_);
 *
 * ACADO 模型维度 (acado_common.h):
 *   ACADO_NX  = 10  [px py pz  qw qx qy qz  vx vy vz]
 *   ACADO_NU  =  4  [wx wy wz  T]
 *   ACADO_N   = 20
 *   ACADO_NOD =  3  [dx dy dz]
 */

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <cmath>
#include <vector>

// ACADO 生成代码 (external_packages/mpc_controller/quadrotor_mpc_export/)
#include "acado_common.h"
#include "acado_auxiliary_functions.h"

// MpcWrapper (external_packages/mpc_support_bag/)
#include "mpc_wrapper.h"

// ── 物理常数 ──────────────────────────────────────────────────────────────── //
static constexpr double kAdpG    = 9.8066;
static constexpr double kAdpMass = 1.735;    // iris + Mid360 + pendulum [kg]
static constexpr double kAdpHov  = 0.758;    // hover_perc，归一化推力用

// ─────────────────────────────────────────────────────────────────────────── //
class MPCPlannerClass {
public:

    MPCPlannerClass() {}

    // ── 构造：读 ROS 参数，初始化 MpcWrapper ─────────────────────────────── //
    MPCPlannerClass(ros::NodeHandle& nh)
    {
        // planner.cpp 直接读取的成员
        nh.param("/ipc2_node/mpc/horizon",    MPC_HORIZON, ACADO_N);
        nh.param("/ipc2_node/mpc/step",       MPC_STEP,    0.1);
        nh.param("/ipc2_node/mpc/ctrl_delay", ctrl_delay_, 0);

        // ── 权重 ──────────────────────────────────────────────────────── //
       
        nh.param("/ipc2_node/mpc/R_p",   Rp, 200.0);
        nh.param("/ipc2_node/mpc/R_v",   Rv, 10.0);
        nh.param("/ipc2_node/mpc/R_phi", Rq, 50.0);
        nh.param("/ipc2_node/mpc/R_w",   Rw, 1.0);
        nh.param("/ipc2_node/mpc/R_T",   RT, 1.0);

        // Q: 状态代价  [px py pz  qw qx qy qz  vx vy vz]
        Q_.setZero();
        Q_(0,0)=Rp; Q_(1,1)=Rp; Q_(2,2)=Rp;
        Q_(3,3)=Rq; Q_(4,4)=Rq; Q_(5,5)=Rq; Q_(6,6)=Rq;
        Q_(7,7)=Rv; Q_(8,8)=Rv; Q_(9,9)=Rv;

        // R: 输入代价  [wx wy wz  T]
        R_.setZero();
        R_(0,0)=Rw; R_(1,1)=Rw; R_(2,2)=Rw;
        R_(3,3)=RT;

        // ── 输入约束 ──────────────────────────────────────────────────── //
        nh.param("/ipc2_node/mpc/wx_max", wx_lim, 1.5);
        nh.param("/ipc2_node/mpc/wy_max", wy_lim, 1.5);
        nh.param("/ipc2_node/mpc/wz_max", wz_lim, 2.0);
        nh.param("/ipc2_node/mpc/T_min",  T_min,  0.);
        nh.param("/ipc2_node/mpc/T_max",  T_max,  25.0);
        u_lb_ << (real_t)-wx_lim, (real_t)-wy_lim, (real_t)-wz_lim, (real_t)T_min;
        u_ub_ << (real_t) wx_lim, (real_t) wy_lim, (real_t) wz_lim, (real_t)T_max;

        // ── 初始化内部矩阵 ────────────────────────────────────────────── //
        x0_.setZero();     x0_(3) = 1.0f;   // qw=1，悬停姿态
        xpred_.setZero();  for(int i=0;i<=ACADO_N;++i) xpred_(3,i)=1.0f;

        yref_.setZero();
        for(int i=0;i<=ACADO_N;++i) yref_(3,i) = 1.0f;  // qw=1

        uref_.setZero();
        for(int i=0;i<ACADO_N;++i)
            uref_(3,i) = (real_t)(kAdpMass * kAdpG);  // 悬停推力

        od_.setZero();

        // 预测缓存
        pred_p_.assign(ACADO_N+1, Eigen::Vector3d::Zero());
        pred_v_.assign(ACADO_N+1, Eigen::Vector3d::Zero());
        pred_a_.assign(ACADO_N+1, Eigen::Vector3d(0,0,0));
        pred_u_.assign(ACADO_N,
            Eigen::Vector4d(0,0,0, kAdpMass*kAdpG));

        // planner.cpp 直接访问的缓冲区
        X_0_.resize(9);   X_0_.setZero();
        X_r_.resize(9*ACADO_N); X_r_.setZero();
        planes_.resize(MPC_HORIZON);

        // ── 初始化 MpcWrapper（对应 MpcRosApplication 的 init() 逻辑）── //
        mpc_wrapper_.initMpc(x0_, xpred_, yref_, uref_, Q_, R_);
        writeBounds();

        ROS_INFO("[IpcMpcAdapter] ready. N=%d dt=%.2fs mass=%.3fkg",
                 ACADO_N, MPC_STEP, kAdpMass);
    }

    ~MPCPlannerClass() {}

    // ════════════════════════════════════════════════════════════════════════ //
    //   planner.cpp 调用的接口（与原 MPCPlannerClass 完全一致）
    // ════════════════════════════════════════════════════════════════════════ //

    /**
     * SetStatus — 由 planner.cpp 在每次 TimerCallback 中调用。
     * a0 不需要（NMPC 用四元数动力学，不需要加速度状态）。
     */
    void SetStatus(Eigen::Vector3d p0, Eigen::Vector3d v0,
                   Eigen::Vector3d /*a0*/)
    {
        x0_(0)=(real_t)p0.x(); x0_(1)=(real_t)p0.y(); x0_(2)=(real_t)p0.z();
        x0_(7)=(real_t)v0.x(); x0_(8)=(real_t)v0.y(); x0_(9)=(real_t)v0.z();
        // 四元数由 SetAttitude 单独更新，不在这里清零

        // 同步到 X_0_（planner 用于 rollout）
        X_0_.head<3>() = p0;
        X_0_.segment<3>(3) = v0;
        X_0_.tail<3>()  = pred_a_[0];   // 用上一帧的预测加速度
    }

    /**
     * SetAttitude — 【新增，仅一行】在 planner.cpp OdomCallback 末尾调用：
     *     mpc_->SetAttitude(odom_q_);
     */
    void SetAttitude(const Eigen::Quaterniond& q)
    {
        x0_(3)=(real_t)q.w(); x0_(4)=(real_t)q.x();
        x0_(5)=(real_t)q.y(); x0_(6)=(real_t)q.z();
    }

    /**
     * SetGoal — 设置 horizon 中第 step 节点的参考状态。
     * ar 不需要（NMPC 不用加速度参考）。
     */
    void SetGoal(Eigen::Vector3d pr, Eigen::Vector3d vr,
                 Eigen::Vector3d /*ar*/, int step)
    {
        if (step < 0 || step > ACADO_N) return;
        yref_(0,step)=(real_t)pr.x(); yref_(1,step)=(real_t)pr.y();
        yref_(2,step)=(real_t)pr.z();
        // qw=1 在初始化时已设好，保持不变
        yref_(7,step)=(real_t)vr.x(); yref_(8,step)=(real_t)vr.y();
        yref_(9,step)=(real_t)vr.z();
    }

    /** SetFSC — Phase B 加入 ACADO 约束，当前先留空。 */
    void SetFSC(Eigen::Matrix<double, Eigen::Dynamic, 4>& /*planes*/,
                int /*step*/) {}

    bool IsInFSC(Eigen::Vector3d pos,
                 Eigen::Matrix<double, Eigen::Dynamic, 4>& planes)
    {
        if (planes.rows() == 0) return false;
        for (int i = 0; i < planes.rows(); ++i)
            if (pos.x()*planes(i,0) + pos.y()*planes(i,1) +
                pos.z()*planes(i,2) + planes(i,3) > 0)
                return false;
        return true;
    }

    /**
     * Run — 同步求解 NMPC，在 planner.cpp 的 TimerCallback 主线程中执行。
     * 对应 MpcRosApplication::mpcProcess() 里的核心三步，但不含线程和 sleep。
     */
    bool Run()
    {
        // Step 1: 推送参考和在线数据（对应 setReference / setOnlineData）
        mpc_wrapper_.setMpcReference(yref_, uref_);
        mpc_wrapper_.setMpcOnlineData(od_);

        // Step 2: 求解（对应 mpc_wrapper_.solveMpc）
        bool ok = mpc_wrapper_.solveMpc(x0_);

        // Step 3: 缓存结果，建立预测轨迹（供 GetOptimCmd / rollout 使用）
        if (ok) {
            buildPrediction();
            X_0_.tail<3>() = pred_a_[0];
        }
        return ok;
    }

    /**
     * GetOptimCmd — 返回第 segment 段的虚拟 jerk。
     *
     * planner.cpp 把它传给 ConvertCommand(acc, jerk)，
     * jerk 在微分平坦里只影响角速度前馈项（通常很小），
     * 用相邻预测加速度差分近似已经足够。
     */
    void GetOptimCmd(Eigen::Vector3d& u, int segment)
    {
        segment = std::max(0, std::min(segment, ACADO_N-1));
        int next = std::min(segment+1, ACADO_N);
        u = (pred_a_[next] - pred_a_[segment]) / MPC_STEP;
    }

    /**
     * SystemModel — planner.cpp 用于轨迹 rollout 可视化。
     * 保持与原始 [p,v,a] 9 维线性模型相同的 A(9×9) B(9×3) 结构。
     */
    void SystemModel(Eigen::MatrixXd& A, Eigen::MatrixXd& B, double dt)
    {
        A = Eigen::MatrixXd::Identity(9,9);
        A(0,3)=dt; A(1,4)=dt; A(2,5)=dt;   // p += v*dt
        A(3,6)=dt; A(4,7)=dt; A(5,8)=dt;   // v += a*dt
        B = Eigen::MatrixXd::Zero(9,3);
        B(6,0)=dt; B(7,1)=dt; B(8,2)=dt;   // a += jerk*dt
    }

    // ════════════════════════════════════════════════════════════════════════ //
    //   可选扩展接口
    // ════════════════════════════════════════════════════════════════════════ //

    /**
     * SetDisturbance — 将 DOB 估计的干扰写入 ACADO online data。
     * 若 planner.cpp 后续接入干扰观测器，调用此函数即可。
     */
    void SetDisturbance(const Eigen::Vector3d& d)
    {
        for (int i = 0; i <= ACADO_N; ++i) {
            od_(0,i)=(real_t)d.x();
            od_(1,i)=(real_t)d.y();
            od_(2,i)=(real_t)d.z();
        }
    }

    /** 直接获取 NMPC bodyrate + 推力输出（节点0）。 */
    void GetBodyrateCmd(Eigen::Vector3d& w_cmd, double& T_cmd) const
    {
        w_cmd = pred_u_[0].head<3>();
        T_cmd = pred_u_[0](3);
    }

    /** 归一化推力 [0,1]，与 BodyrateCtrlPub 兼容。 */
    double GetNormalizedThrust() const
    {
        double thr2acc = kAdpG / kAdpHov;
        return std::max(0.1, std::min(0.9,
               pred_u_[0](3) / (kAdpMass * thr2acc)));
    }

    // ════════════════════════════════════════════════════════════════════════ //
    //   planner.cpp 直接读取的公有成员（保持与原类相同）
    // ════════════════════════════════════════════════════════════════════════ //
    Eigen::VectorXd  X_0_;          // [p(3) v(3) a(3)]  当前状态，用于 rollout
    Eigen::VectorXd  X_r_;          // 参考序列（兼容，未直接使用）
    int    MPC_HORIZON = ACADO_N;
    double MPC_STEP    = 0.1;
    int    ctrl_delay_ = 0;
    std::vector<Eigen::Vector3d> u_last_;   // 兼容，未使用
    std::vector<Eigen::Matrix<double, Eigen::Dynamic, 4>> planes_;

private:

    // ── 写输入上下界到 ACADO 内存（通过 MpcWrapper 暴露的 Eigen::Map）─── //
    void writeBounds()
    {
        // acadoVariables.lbValues / ubValues 由 mpc_wrapper.h 中的
        // Eigen::Map acado_lower_bounds_ / acado_upper_bounds_ 映射。
        // 这里直接写入原始内存，与 MpcWrapper 内部保持一致。
        Eigen::Map<Eigen::Matrix<real_t, ACADO_NU, ACADO_N, Eigen::ColMajor>>
            lb(acadoVariables.lbValues),
            ub(acadoVariables.ubValues);
        lb = u_lb_.replicate(1, ACADO_N);
        ub = u_ub_.replicate(1, ACADO_N);
    }

    /**
     * buildPrediction — 用最新求解的控制序列前向积分，
     * 建立 pred_p_ / pred_v_ / pred_a_ / pred_u_ 缓存。
     *
     * 对应 MpcRosApplication 里 predicted_states_ 的作用，
     * 但这里用连续动力学重新积分，数值更准确。
     */
    void buildPrediction()
    {
        Eigen::Matrix<real_t, ACADO_NU, ACADO_N> ctrl;
        mpc_wrapper_.getMpcControls(ctrl);

        Eigen::Vector3d p = x0_.head<3>().cast<double>();
        Eigen::Quaterniond q(
            (double)x0_(3),(double)x0_(4),
            (double)x0_(5),(double)x0_(6));
        Eigen::Vector3d v(
            (double)x0_(7),(double)x0_(8),(double)x0_(9));

        pred_p_[0] = p;
        pred_v_[0] = v;

        for (int i = 0; i < ACADO_N; ++i) {
            double wx=(double)ctrl(0,i), wy=(double)ctrl(1,i),
                   wz=(double)ctrl(2,i), T =(double)ctrl(3,i);

            // 世界系加速度 = R * e3 * (T/m) - g*e3
            Eigen::Vector3d zB = q * Eigen::Vector3d(0,0,1);
            Eigen::Vector3d a  = zB * (T/kAdpMass)
                               - Eigen::Vector3d(0,0,kAdpG);

            pred_a_[i]  = a;
            pred_u_[i]  = Eigen::Vector4d(wx, wy, wz, T);

            // 欧拉积分位置/速度
            p += v * MPC_STEP + 0.5 * a * MPC_STEP * MPC_STEP;
            v += a * MPC_STEP;

            // 四元数积分
            double norm_w = std::sqrt(wx*wx + wy*wy + wz*wz);
            Eigen::Quaterniond dq;
            if (norm_w > 1e-6) {
                dq = Eigen::Quaterniond(
                    Eigen::AngleAxisd(norm_w * MPC_STEP,
                        Eigen::Vector3d(wx,wy,wz)/norm_w));
            } else {
                dq = Eigen::Quaterniond::Identity();
            }
            q = (q * dq).normalized();

            pred_p_[i+1] = p;
            pred_v_[i+1] = v;
        }
        pred_a_[ACADO_N] = pred_a_[ACADO_N-1];
    }

    // ── MpcWrapper（唯一的 NMPC 求解器实例）─────────────────────────────── //
    MpcWrapper mpc_wrapper_;

    // ── ACADO 输入矩阵 ───────────────────────────────────────────────────── //
    Eigen::Matrix<real_t, ACADO_NX, 1>                  x0_;     // 当前状态
    Eigen::Matrix<real_t, ACADO_NX, ACADO_N+1>          xpred_;  // initMpc 用
    Eigen::Matrix<real_t, kCostStateSize, ACADO_N+1>    yref_;   // 状态参考
    Eigen::Matrix<real_t, ACADO_NU, ACADO_N>            uref_;   // 输入参考
    Eigen::Matrix<real_t, ACADO_NOD, ACADO_N+1>         od_;     // 在线数据（干扰）

    // ── 权重 & 约束 ──────────────────────────────────────────────────────── //
    Eigen::Matrix<real_t, kCostStateSize, kCostStateSize> Q_;
    Eigen::Matrix<real_t, ACADO_NU, ACADO_NU>             R_;
    Eigen::Matrix<real_t, ACADO_NU, 1>                    u_lb_, u_ub_;

    // ── 预测轨迹缓存 ─────────────────────────────────────────────────────── //
    std::vector<Eigen::Vector3d> pred_p_;   // 预测位置 [0..N]
    std::vector<Eigen::Vector3d> pred_v_;   // 预测速度 [0..N]
    std::vector<Eigen::Vector3d> pred_a_;   // 预测加速度 [0..N]
    std::vector<Eigen::Vector4d> pred_u_;   // 控制输出 [wx,wy,wz,T] [0..N-1]
};

#endif  // IPC_MPC_ADAPTER_H_
