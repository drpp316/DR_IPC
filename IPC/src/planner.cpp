#include "planner.h"
#include "../include/polytope/emvp.hpp"
#include <algorithm>
#include <limits>

#define BACKWARD_HAS_DW 1
#include "backward.hpp"
namespace backward{
    backward::SignalHandling sh;
}

namespace {
struct PlaneViolationInfo {
    double max_violation{0.0};  // max(n*p + d), <=0 means inside all half-spaces
    int worst_index{-1};
};

PlaneViolationInfo EvaluatePlaneViolation(
    const Eigen::Matrix<double, Eigen::Dynamic, 4>& planes,
    const Eigen::Vector3d& p)
{
    PlaneViolationInfo info;
    if (planes.rows() <= 0) {
        info.max_violation = std::numeric_limits<double>::quiet_NaN();
        info.worst_index = -1;
        return info;
    }

    info.max_violation = -std::numeric_limits<double>::infinity();
    for (int i = 0; i < planes.rows(); ++i) {
        const double val = planes(i, 0) * p.x() +
                           planes(i, 1) * p.y() +
                           planes(i, 2) * p.z() +
                           planes(i, 3);
        if (val > info.max_violation) {
            info.max_violation = val;
            info.worst_index = i;
        }
    }
    return info;
}

Eigen::Matrix<double, Eigen::Dynamic, 4> FilterAndSortPlanes(
    const Eigen::Matrix<double, Eigen::Dynamic, 4>& planes,
    const Eigen::Vector3d& p,
    double tol = 1e-6)
{
    struct PlaneItem {
        double margin;
        Eigen::RowVector4d row;
    };
    std::vector<PlaneItem> keep;
    keep.reserve(planes.rows());

    for (int i = 0; i < planes.rows(); ++i) {
        const Eigen::Vector3d n(planes(i, 0), planes(i, 1), planes(i, 2));
        const double n2 = n.squaredNorm();
        if (n2 < 1e-12) {
            continue;
        }
        const double val = n.dot(p) + planes(i, 3);
        if (val <= tol) {
            const double margin = -val / std::sqrt(n2);
            keep.push_back(PlaneItem{margin, planes.row(i)});
        }
    }

    std::sort(keep.begin(), keep.end(),
              [](const PlaneItem& a, const PlaneItem& b) {
                  // Prefer the planes closest to the current point so the
                  // active set keeps the most relevant SFC faces first.
                  return a.margin < b.margin;
              });

    Eigen::Matrix<double, Eigen::Dynamic, 4> filtered;
    filtered.resize(static_cast<int>(keep.size()), 4);
    for (int i = 0; i < static_cast<int>(keep.size()); ++i) {
        filtered.row(i) = keep[i].row;
    }
    return filtered;
}
}  // namespace

void PlannerClass::WriteSfcPlaneCount(const std::string& tag,
                                      const Eigen::Matrix<double, Eigen::Dynamic, 4>& planes,
                                      int sfc_steps,
                                      int path_size,
                                      const Eigen::Vector3d* ref_p)
{
    if (!write_sfc_.is_open()) {
        return;
    }
    const PlaneViolationInfo odom_info = EvaluatePlaneViolation(planes, odom_p_);
    const bool has_ref = (ref_p != nullptr);
    const PlaneViolationInfo ref_info =
        has_ref ? EvaluatePlaneViolation(planes, *ref_p) : PlaneViolationInfo{};
    const double nan_v = std::numeric_limits<double>::quiet_NaN();

    write_sfc_ << std::fixed << std::setprecision(6)
               << ros::Time::now().toSec() << ","
               << tag << ","
               << planes.rows() << ","
               << sfc_steps << ","
               << astar_index_ << ","
               << path_size << ","
               << odom_p_.x() << ","
               << odom_p_.y() << ","
               << odom_p_.z() << ","
               << odom_info.max_violation << ","
               << odom_info.worst_index << ","
               << (has_ref ? (*ref_p).x() : nan_v) << ","
               << (has_ref ? (*ref_p).y() : nan_v) << ","
               << (has_ref ? (*ref_p).z() : nan_v) << ","
               << (has_ref ? ref_info.max_violation : nan_v) << ","
               << (has_ref ? ref_info.worst_index : -1)
               << std::endl;

    WriteSfcPlaneDetails(tag, planes, sfc_steps, path_size, ref_p);
}

void PlannerClass::WriteSfcPlaneDetails(const std::string& tag,
                                        const Eigen::Matrix<double, Eigen::Dynamic, 4>& planes,
                                        int sfc_steps,
                                        int path_size,
                                        const Eigen::Vector3d* ref_p)
{
    if (!log_sfc_constraints_ || !write_sfc_detail_.is_open()) {
        return;
    }

    const bool has_ref = (ref_p != nullptr);
    const double nan_v = std::numeric_limits<double>::quiet_NaN();
    const double t_now = ros::Time::now().toSec();

    if (planes.rows() <= 0) {
        write_sfc_detail_ << std::fixed << std::setprecision(6)
                          << t_now << ","
                          << tag << ","
                          << -1 << ","
                          << 0 << ","
                          << sfc_steps << ","
                          << astar_index_ << ","
                          << path_size << ","
                          << odom_p_.x() << "," << odom_p_.y() << "," << odom_p_.z() << ","
                          << (has_ref ? (*ref_p).x() : nan_v) << ","
                          << (has_ref ? (*ref_p).y() : nan_v) << ","
                          << (has_ref ? (*ref_p).z() : nan_v) << ","
                          << nan_v << "," << nan_v << "," << nan_v << "," << nan_v << ","
                          << nan_v << "," << nan_v
                          << std::endl;
        return;
    }

    for (int i = 0; i < planes.rows(); ++i) {
        const double a = planes(i, 0);
        const double b = planes(i, 1);
        const double c = planes(i, 2);
        const double d = planes(i, 3);
        const double val_odom = a * odom_p_.x() + b * odom_p_.y() + c * odom_p_.z() + d;
        const double val_ref = has_ref
                                   ? a * (*ref_p).x() + b * (*ref_p).y() + c * (*ref_p).z() + d
                                   : nan_v;
        write_sfc_detail_ << std::fixed << std::setprecision(6)
                          << t_now << ","
                          << tag << ","
                          << i << ","
                          << planes.rows() << ","
                          << sfc_steps << ","
                          << astar_index_ << ","
                          << path_size << ","
                          << odom_p_.x() << "," << odom_p_.y() << "," << odom_p_.z() << ","
                          << (has_ref ? (*ref_p).x() : nan_v) << ","
                          << (has_ref ? (*ref_p).y() : nan_v) << ","
                          << (has_ref ? (*ref_p).z() : nan_v) << ","
                          << a << "," << b << "," << c << "," << d << ","
                          << val_odom << "," << val_ref
                          << std::endl;
    }
}



void PlannerClass::TimerCallback(const ros::TimerEvent &)
{

    timer_mutex_.lock();
    //mode_ = Command;

    // Keep publishing minimal setpoints before odom is ready,
    if (!has_odom_flag_) {
        const Eigen::Vector3d zero_rate = Eigen::Vector3d::Zero();
        const double min_thrust = 0.08;
        BodyrateCtrlPub(zero_rate, min_thrust, ros::Time::now());
        WriteNmpcDebug(ros::Time::now().toSec(), zero_rate, 0.0, min_thrust, 0.0,
                       Eigen::Vector3d::Zero(), false);
        timer_mutex_.unlock();
        return;
    }

    //SFC 约束步长设置
    ros::Time t_start = ros::Time::now();
    int sfc_steps = sfc_apply_steps_;
    if (sfc_steps < 0) {
        sfc_steps = 0;
    }
    if (sfc_steps > mpc_->MPC_HORIZON) {
        sfc_steps = mpc_->MPC_HORIZON;
    }

    if (mode_ == Command) {
        if (new_goal_flag_) {
            new_goal_flag_ = false;
            replan_flag_ = false;
            PathReplan(true);
        }
        if (new_goal_flag_ == false && replan_flag_) {
            replan_flag_ = false;
            PathReplan(false);
        }
        log_times_[1] = (ros::Time::now() - t_start).toSec() * 1000.0;

        // Feed NMPC with A* references and SFC corridor-constrained horizon goals.
        ros::Time sfc_start = ros::Time::now();
        if (follow_path_.size() > 0) { // follow A* path
            double min_dis = 10000.0;
            for (int i = 0; i < follow_path_.size(); i++) { // find the nearest point
                double dis = (odom_p_ - follow_path_[i]).norm();
                if (dis < min_dis) {
                    min_dis = dis;
                    astar_index_ = i;
                }
            }

            int goal_in_sfc = astar_index_;
            Update_SfC(sfc_steps, goal_in_sfc);//sfc

            // MPC set goal p/v/yaw
            mpc_goals_.clear();
            Eigen::Vector3d last_p_ref;
            double last_yaw_ref = yaw_;
            for (int i = 0; i < mpc_->MPC_HORIZON; i++) {
                int index = astar_index_ + i * ref_dis_;
                if (index >= goal_in_sfc) index = goal_in_sfc;
                if (index >= follow_path_.size()) index = follow_path_.size() - 1;
                const Eigen::Vector3d p_target = follow_path_[index];
                const Eigen::Vector3d p_ref = p_target;

                Eigen::Vector3d v_r(0, 0, 0);
                if (i == 0) v_r = (p_ref - odom_p_) / mpc_->MPC_STEP;
                else v_r = (p_ref - last_p_ref) / mpc_->MPC_STEP;
                if (ref_v_max_ > 1e-3) {
                    const double v_norm = v_r.norm();
                    if (v_norm > ref_v_max_) {
                        v_r = v_r / v_norm * ref_v_max_;
                    }
                }
                last_p_ref = p_ref;

                double yaw_ref_step = last_yaw_ref;
                if (yaw_ctrl_flag_) {
                    int next_index = index + std::max(1, ref_dis_);
                    if (next_index >= follow_path_.size()) next_index = follow_path_.size() - 1;
                    const Eigen::Vector3d dir = follow_path_[next_index] - follow_path_[index];
                    auto wrap_pi = [](double a) {
                        while (a > M_PI) a -= 2.0 * M_PI;
                        while (a < -M_PI) a += 2.0 * M_PI;
                        return a;
                    };
                    if (dir.head<2>().norm() > 1e-3) {
                        const double yaw_target = std::atan2(dir.y(), dir.x());
                        const double yaw_delta = wrap_pi(yaw_target - yaw_);
                        yaw_ref_step = yaw_ + yaw_delta;
                    } else {
                        yaw_ref_step = yaw_r_;//“全局方向”
                    }
                    const double max_step_dyaw = std::max(0.05, yaw_rate_limit_) * mpc_->MPC_STEP;
                    double dyaw_step = wrap_pi(yaw_ref_step - last_yaw_ref);
                    dyaw_step = std::max(-max_step_dyaw, std::min(max_step_dyaw, dyaw_step));
                    yaw_ref_step = last_yaw_ref + dyaw_step;
                } else {
                    yaw_ref_step = 0.0;
                }
                last_yaw_ref = yaw_ref_step;

                mpc_->SetGoal(p_ref, v_r, Eigen::Vector3d::Zero(), i, yaw_ref_step);
                // std::cout << "mpc goal " << i << ": " << follow_path_[index].transpose() << " v: " << v_r.transpose() << std::endl;
                mpc_goals_.push_back(p_ref);
            }

            // Always update terminal reference at step N to avoid stale tail behavior.
            int terminal_index = astar_index_ + mpc_->MPC_HORIZON * ref_dis_;
            if (terminal_index >= goal_in_sfc) terminal_index = goal_in_sfc;
            if (terminal_index >= follow_path_.size()) terminal_index = follow_path_.size() - 1;
            const Eigen::Vector3d p_terminal_target = follow_path_[terminal_index];
            const Eigen::Vector3d p_terminal = p_terminal_target;
            Eigen::Vector3d v_terminal =
                (p_terminal - last_p_ref) / std::max(1e-3, mpc_->MPC_STEP);
            if (ref_v_max_ > 1e-3) {
                const double v_norm = v_terminal.norm();
                if (v_norm > ref_v_max_) {
                    v_terminal = v_terminal / v_norm * ref_v_max_;
                }
            }
            mpc_->SetGoal(p_terminal, v_terminal, Eigen::Vector3d::Zero(),
                          mpc_->MPC_HORIZON, last_yaw_ref);
            AstarPublish(mpc_goals_, 3, 0.1);

            // yaw control
            if (astar_index_ < follow_path_.size() - 0.3 / path_dis_) {
                yaw_r_ = std::atan2(follow_path_.back().y()-odom_p_.y(), follow_path_.back().x()-odom_p_.x());
                // yaw_r_ = std::atan2(follow_path_[astar_index_+ref_dis_].y()-odom_p_.y(), follow_path_[astar_index_+ref_dis_].x()-odom_p_.x());
            }
        } else { // stay at initial position
            Clear_SfC(0);
            yaw_r_ = 0.0;
            mpc_goals_.clear();
            for (int i = 0; i < mpc_->MPC_HORIZON; i++) {
                mpc_->SetGoal(goal_p_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), i, yaw_r_);
                mpc_goals_.push_back(goal_p_);
            }
            mpc_->SetGoal(goal_p_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
                          mpc_->MPC_HORIZON, yaw_r_);
            AstarPublish(mpc_goals_, 3, 0.1);
        }
        log_times_[2] = (ros::Time::now() - sfc_start).toSec() * 1000.0;
    }

    if (mode_ == Hover) {
        Clear_SfC(0);
        mpc_goals_.clear();
        for (int i = 0; i < mpc_->MPC_HORIZON; i++) {
            mpc_->SetGoal(goal_p_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), i, yaw_r_);
            mpc_goals_.push_back(goal_p_);
        }
        mpc_->SetGoal(goal_p_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
                      mpc_->MPC_HORIZON, yaw_r_);
        AstarPublish(mpc_goals_, 3, 0.1);
    }

    //空平面
    if (mode_ == Command) {
        Clear_SfC(sfc_steps);//清空MPC，后面步长约束
    } else {
        Clear_SfC(0);
    }
    

    // calculate NMPC
    ros::Time mpc_start = ros::Time::now();

    mpc_->SetStatus(odom_p_, odom_v_, odom_a_);

    double T_cmd_last;
    Eigen::Vector3d w_cmd_last;
    mpc_->GetBodyrateCmd(w_cmd_last, T_cmd_last);

    Eigen::VectorXd y = Eigen::VectorXd::Zero(ACADO_NX);
    y << odom_p_.x(), odom_p_.y(), odom_p_.z(),
         odom_q_.w(), odom_q_.x(), odom_q_.y(), odom_q_.z(),
         odom_v_.x(), odom_v_.y(), odom_v_.z();

    Eigen::VectorXd u = Eigen::VectorXd::Zero(ACADO_NU);
    if (ACADO_NU > 0) u(0) = w_cmd_last.x();
    if (ACADO_NU > 1) u(1) = w_cmd_last.y();
    if (ACADO_NU > 2) u(2) = w_cmd_last.z();
    if (ACADO_NU > 3) u(3) = T_cmd_last;

    if (!disturbance_inited_) {
        disturbance_observer_.setFirstZ(y);
        disturbance_inited_ = true;
    }
    disturbance_observer_.kalman_geso_update(y, u, uav_mass_, 9.8066);

    const Eigen::VectorXd d_hat_full = disturbance_observer_.getDisturbance();
    Eigen::Vector3d d_hat = Eigen::Vector3d::Zero();
    if (d_hat_full.size() >= 3) {
        d_hat << d_hat_full(0), d_hat_full(1), d_hat_full(2);
    }
    mpc_->SetDisturbance(d_hat);


    bool success_flag = mpc_->Run();
    log_times_[3] = (ros::Time::now() - mpc_start).toSec() * 1000.0;

    //rviz 显示NMPC预测的轨迹
    if (success_flag) {
        std::vector<Eigen::Vector3d> path;
        std::vector<Eigen::Vector3d> path_vis;
        std::vector<Eigen::Vector3d> ref_path;
        mpc_->GetPredictedPath(path);
        mpc_->GetReferencePath(ref_path);

        const size_t keep_n =
            std::min(path.size(), static_cast<size_t>(pred_vis_steps_ + 1));
        if (keep_n > 0) {
            path_vis.assign(path.begin(), path.begin() + keep_n);
        }
        MPCPathPublish(path_vis);
        WriteNmpcPredDebug(ros::Time::now().toSec(), path, ref_path, success_flag);
    }

   

    Eigen::Vector3d w_cmd;
    double T_cmd;
    mpc_->GetBodyrateCmd(w_cmd, T_cmd);
    const double sfc_eps_cmd = mpc_->GetSfcSlackCmd();

    /*
    if (yaw_ctrl_flag_) {
        double yaw_error = yaw_r_ - yaw_;
        if (yaw_error > M_PI) yaw_error -= 2.0 * M_PI;
        if (yaw_error < -M_PI) yaw_error += 2.0 * M_PI;
        const double wz_yaw = yaw_gain_ * yaw_error;
        w_cmd.z() = std::max(-yaw_rate_limit_, std::min(yaw_rate_limit_, wz_yaw));
    }*/

    // 用planner自己估计的thr2acc_，而不是hardcode
    double thrust_norm = T_cmd / (uav_mass_ * thr2acc_);
    thrust_norm = std::max(0.1, std::min(0.9, thrust_norm));
    
    ROS_INFO_THROTTLE(0.3, "[NMPC] w=[%.3f %.3f %.3f] T=%.2fN thr2acc=%.3f norm=%.3f",
    w_cmd.x(), w_cmd.y(), w_cmd.z(), T_cmd, thr2acc_, thrust_norm);
    BodyrateCtrlPub(w_cmd, thrust_norm, ros::Time::now());

    WriteNmpcDebug(ros::Time::now().toSec(), w_cmd, T_cmd, thrust_norm,
                   sfc_eps_cmd, d_hat, success_flag);

    timed_thrust_.push(std::pair<ros::Time, double>(ros::Time::now(), thrust_));
    while (timed_thrust_.size() > 100) {
        timed_thrust_.pop();
    }
    //log_times_[4] = (ros::Time::now() - df_start).toSec() * 1000.0;

    // log_times_[0] = (ros::Time::now() - t_start).toSec() * 1000.0;
    WriteLogTime();
    timer_mutex_.unlock();
}


//把 NMPC 某一段预测步上的 SFC 约束清空。
void PlannerClass::Clear_SfC(int begin_step) 
{
    if (begin_step < 0) {
        begin_step = 0;
    }
    if (begin_step >= mpc_->MPC_HORIZON) {
        return;
    }
    Eigen::Matrix<double, Eigen::Dynamic, 4> empty_planes;
    for (int i = begin_step; i < mpc_->MPC_HORIZON; ++i) {
        mpc_->SetFSC(empty_planes, i);
    }
}

/*
作用：在 Command 模式下，按照当前 A* 路径和多面体生成结果，给 NMPC 的前 sfc_steps 步写入 SFC 约束，并更新 goal_in_sfc。
它内部会做这些事：
先清空旧约束（防止残留）
根据当前位置和路径生成局部/长段多面体
把约束写到前 sfc_steps 步
给后续“参考点截断到安全段”提供 goal_in_sfc*/
void PlannerClass::Update_SfC(int sfc_steps, int& goal_in_sfc)
{
    Clear_SfC(0);//sfc清空
    /*if (sfc_steps <= 0) {
        return;
    }*/

    const int path_size = static_cast<int>(follow_path_.size());
    goal_in_sfc = astar_index_;
    const int ref_index = std::min(std::max(astar_index_, 0), std::max(path_size - 1, 0));
    const Eigen::Vector3d ref_eval = (path_size > 0) ? follow_path_[ref_index] : odom_p_;

    if (path_size - astar_index_ <= ref_dis_) { // near the end, find the sfc at current point
        goal_in_sfc = path_size;
        Eigen::Matrix<double, Eigen::Dynamic, 4> planes;
        GenerateAPolytope(odom_p_, odom_p_, planes, 0);
        Eigen::Matrix<double, Eigen::Dynamic, 4> planes_used =
            FilterAndSortPlanes(planes, odom_p_);
        WriteSfcPlaneCount("near_end", planes_used, sfc_steps, path_size, &ref_eval);
        if (planes_used.rows() == 0) {
            ROS_WARN_THROTTLE(0.5, "[SFC] near-end planes all violated, skip constraints this cycle.");
            return;
        }
        for (int i = 0; i < sfc_steps; i++) {
            mpc_->SetFSC(planes_used, i);
        }
        return;
    }

    // find the biggest and the longest sfc
    Eigen::Matrix<double, Eigen::Dynamic, 4> planes, last_planes;
    GenerateAPolytope(follow_path_[astar_index_], follow_path_[astar_index_], planes, 0);
    Eigen::Matrix<double, Eigen::Dynamic, 4> planes_used =
        FilterAndSortPlanes(planes, odom_p_);
    WriteSfcPlaneCount("initial", planes_used, sfc_steps, path_size, &ref_eval);
    if (planes_used.rows() == 0) {
        ROS_WARN_THROTTLE(0.5, "[SFC] initial planes all violated, skip constraints this cycle.");
        return;
    }
    last_planes = planes_used;
    int init_num = 0;
    if (mpc_->IsInFSC(odom_p_, planes) == false) { // if odom is not in initial sfc
        init_num = (odom_p_ - follow_path_[astar_index_]).norm() / path_dis_ / ref_dis_;
        ROS_INFO("\033[35m UAV is out sfc! init num is %d \033[0m", init_num);
    }
    int first_id = astar_index_, mpc_goal_index = 0;
    for (int i = first_id + 1; i < path_size; i++) { // find last pos in first sfc
        if (mpc_->IsInFSC(follow_path_[i], planes)) {
            first_id = i;
            goal_in_sfc = i;
        } else {
            first_id -= 0.2 / path_dis_;
            if (first_id < astar_index_) first_id = astar_index_;
            break;
        }
    }
    mpc_goal_index = (first_id - astar_index_) / ref_dis_ + init_num + 1;
    assert(first_id >= astar_index_);
    for (int i = init_num; i <= mpc_goal_index && i < sfc_steps; i++) {
        mpc_->SetFSC(planes_used, i);
    }

    int end_index = first_id + (mpc_->MPC_HORIZON - mpc_goal_index) * ref_dis_;
    if (end_index >= path_size) end_index = path_size - 1;
    for (int i = end_index; i >= first_id && mpc_goal_index < mpc_->MPC_HORIZON - 1; i--) {
        if (local_astar_->CheckPoint(follow_path_[i]) == false) continue;
        if (local_astar_->CheckLineObstacleFree(follow_path_[first_id], follow_path_[i]) == false) continue;
        i = i - 0.2 / path_dis_;
        if (i <= first_id) break;
        Eigen::Matrix<double, Eigen::Dynamic, 4> long_planes;
        GenerateAPolytope(follow_path_[first_id], follow_path_[i], long_planes, 1);
        Eigen::Matrix<double, Eigen::Dynamic, 4> long_used =
            FilterAndSortPlanes(long_planes, odom_p_);
        WriteSfcPlaneCount("long", long_used, sfc_steps, path_size, &ref_eval);
        if (long_used.rows() > 0) {
            for (int j = mpc_goal_index + 1; j < sfc_steps; j++) {
                mpc_->SetFSC(long_used, j);
            }
            mpc_goal_index = mpc_->MPC_HORIZON;
            for (int k = first_id; k < path_size; k++) {
                if (mpc_->IsInFSC(follow_path_[k], long_used)) goal_in_sfc = k;
                else break;
            }
            // ROS_INFO("\033[43;37m long sfc : %d \033[0m", goal_in_sfc);
            break;
        }
    }
    for (int i = mpc_goal_index + 1; i < sfc_steps; i++) {
        if (last_planes.rows() > 0) {
            mpc_->SetFSC(last_planes, i);
        }
    }
    if (last_planes.rows() > 0) {
        WriteSfcPlaneCount("last", last_planes, sfc_steps, path_size, &ref_eval);
    }
}


void PlannerClass::GenerateAPolytope(Eigen::Vector3d p1, Eigen::Vector3d p2, Eigen::Matrix<double, Eigen::Dynamic, 4>& planes, uint8_t index)
{
    planes.resize(0, 4);
    ros::Time start_t = ros::Time::now();
    Eigen::Vector3d box_max(10, 10, 3), box_min(-10, -10, -0.5);

    Eigen::Matrix<double, 6, 4> bd;
    bd.setZero();
    bd(0, 0) = 1.0;
    bd(1, 0) = -1.0;
    bd(2, 1) = 1.0;
    bd(3, 1) = -1.0;
    bd(4, 2) = 1.0;
    bd(5, 2) = -1.0;
    bd(0, 3) = -p1.x()-box_max.x();
    bd(1, 3) =  p1.x()+box_min.x();
    bd(2, 3) = -p1.y()-box_max.y();
    bd(3, 3) =  p1.y()+box_min.y();
    bd(4, 3) = -box_max.z();
    bd(5, 3) = +box_min.z();

    Polytope p;
    if (local_pc_.empty()) { // 障碍物点云为空，直接返回一个方块
        planes.resize(6, 4); // Ax + By + Cz + D = 0
        planes.row(0) <<  1,  0,  0, -p1.x()-box_max.x();
        planes.row(1) <<  0,  1,  0, -p1.y()-box_max.y();
        planes.row(2) <<  0,  0,  1, -box_max.z();
        planes.row(3) << -1,  0,  0,  p1.x()+box_min.x();
        planes.row(4) <<  0, -1,  0,  p1.y()+box_min.y();
        planes.row(5) <<  0,  0, -1,  box_min.z();
        return ; 
    }
    Eigen::Map<const Eigen::Matrix<double, 3, -1, Eigen::ColMajor>> pp(local_pc_[0].data(), 3, local_pc_.size());

    bool success = emvp::emvp(bd, pp, p1, p2, p, sfc_dis_, false, 1);
    if (success) {
        if (index == 0) p.Visualize(sfc_pub_, "emvp", true);
        p.Visualize(sfc_pub_, "emvp", false);
        planes = p.GetPlanes();
    } else {
        p.Reset();
    }
}

void PlannerClass::PathReplan(bool extend)
{
    astar_path_.clear();
    waypoints_.clear();
    follow_path_.clear();

    Eigen::Vector3d start_p, end_p;
    start_p = odom_p_;
    // start_p = odom_p_ + odom_v_ * 0.1;
    if (extend) {
        local_astar_->SetCenter(Eigen::Vector3d(odom_p_.x(), odom_p_.y(), 0.0));
    } else {
        // ROS_WARN("[MPC FSM]: Replan!");
    }
    local_astar_->setObsVector(local_pc_, expand_dyn_);

    bool add_goal_flag = false;
    end_p = goal_p_;
    double delta_x = goal_p_.x() - odom_p_.x();
    double delta_y = goal_p_.y() - odom_p_.y();
    if (std::fabs(delta_x) > map_upp_.x() || std::fabs(delta_y) > map_upp_.y()) {
        add_goal_flag = true;
        if (std::fabs(delta_x) > std::fabs(delta_y)) {
            end_p.x() = odom_p_.x() + (delta_x/std::fabs(delta_x)) * (map_upp_.x() - resolution_);
            end_p.y() = odom_p_.y() + ((map_upp_.x() - resolution_)/std::fabs(delta_x)) * delta_y;
        } else {
            end_p.x() = odom_p_.x() + ((map_upp_.y() - resolution_)/std::fabs(delta_y)) * delta_x;
            end_p.y() = odom_p_.y() + (delta_y/std::fabs(delta_y)) * (map_upp_.y() - resolution_);
        }
    }
    // start_p.z() = end_p.z(); // only for 2d path searching
    // std::cout << "odom: " << odom_p_.transpose() << " end_p: " << end_p.transpose() << std::endl;

    int point_num = 8;
    double r = expand_fix_ / 2;
    if (local_astar_->CheckStartEnd(start_p) == false) {
        ROS_INFO("\033[41;37m start point in obstacle \033[0m");
        while (true) {
            bool flag = false;
            for(int i = 0; i < point_num; i++) {
                Eigen::Vector3d pt;
                pt << start_p.x() + r*sin(M_PI*2*i/point_num), start_p.y() + r*cos(M_PI*2*i/point_num), start_p.z();
                double dis_min = 10000.0;
                double dis = (odom_p_ - pt).norm();
                if(local_astar_->CheckStartEnd(pt) == true && dis < dis_min) {
                    dis_min = dis;
                    start_p = pt;
                    flag = true;
                }
            }
            if (flag) {
                ROS_INFO("\033[1;32m Change start goal! %f %f %f\033[0m", start_p.x(), start_p.y(), start_p.z());
                break;
            }
            r += expand_fix_ / 2;
        }
    }
    r = expand_fix_ / 2;
    if (local_astar_->CheckStartEnd(end_p) == false) {
        ROS_INFO("\033[41;37m end point in obstacle \033[0m");
        while (true) {
            bool flag = false;
            for(int i = 0; i < point_num; i++) {
                Eigen::Vector3d pt;
                pt << end_p.x() + r*sin(M_PI*2*i/point_num), end_p.y() + r*cos(M_PI*2*i/point_num), end_p.z();
                double dis_min = 10000.0;
                double dis = (odom_p_ - pt).norm();
                if(local_astar_->CheckStartEnd(pt) == true && dis < dis_min) {
                    dis_min = dis;
                    end_p = pt;
                    flag = true;
                }
            }
            if (flag) {
                ROS_INFO("\033[1;32m Change end goal! %f %f %f\033[0m", end_p.x(), end_p.y(), end_p.z());
                break;
            }
            r += expand_fix_ / 2;
        }
    }

    bool search_flag = local_astar_->SearchPath(start_p, end_p);
    if (search_flag) {
        local_astar_->GetPath(astar_path_);
        AstarPublish(astar_path_, 0, 0.1);

        local_astar_->FloydHandle(astar_path_, waypoints_);
        // waypoints_.insert(waypoints_.begin(), odom_p_);
        if (add_goal_flag && local_astar_->CheckPoint(goal_p_)) waypoints_.push_back(goal_p_);
        AstarPublish(waypoints_, 1, 0.1);

        for (int i = 0; i < waypoints_.size()-1; i++) {
            Eigen::Vector3d vector = waypoints_[i+1] - waypoints_[i];
            int num = vector.norm() / path_dis_; // Insert a point every 0.1m (adjust)
            for (int j = 0; j < num; j++) {
                Eigen::Vector3d pt = waypoints_[i] + vector * j / num;
                follow_path_.push_back(pt);
            }
        }
        follow_path_.push_back(waypoints_.back());
        AstarPublish(follow_path_, 2, path_dis_);
    } else {
        ROS_INFO("\033[41;37m No path! Stay at current point! \033[0m");
        follow_path_.push_back(odom_p_);
    }

    geometry_msgs::PoseStamped msg;
    msg.header.frame_id = "world";
    msg.header.stamp = ros::Time::now();
    msg.pose.position.x = follow_path_.back().x();
    msg.pose.position.y = follow_path_.back().y();
    msg.pose.position.z = follow_path_.back().z();    
    goal_pub_.publish(msg);

    ros::Time now = ros::Time::now();
    local_astar_->Reset(); // reset astar data at the end
    // std::cout << "astar reset time is: " << (ros::Time::now() - now).toSec()*1000 << " ms. " << std::endl;
}

void PlannerClass::GoalCallback(const geometry_msgs::PoseStampedConstPtr& msg)
{
    goal_mutex_.lock();

    static Eigen::Vector3d last_goal;
    Eigen::Vector3d new_goal(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);

    if (last_goal != new_goal && mode_ == Command) {
        // goal_p_ << msg->pose.position.x, msg->pose.position.y, goal_p_.z(); // 2d path searching
        goal_p_ << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
        if (goal_p_.z() > map_upp_.z() - 0.5) goal_p_.z() = map_upp_.z() - 0.5;
        if (goal_p_.z() < 0.5) goal_p_.z() = 0.5;
        new_goal_flag_ = true;
    }
    last_goal = new_goal;

    goal_mutex_.unlock();
}

void PlannerClass::OdomCallback(const nav_msgs::OdometryConstPtr& msg)
{
    odom_mutex_.lock();
    odom_time_ = msg->header.stamp;
    odom_p_ << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z;
    odom_v_ << msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z; 
    Eigen::Quaterniond qn_ = Eigen::Quaterniond(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, 
                                 msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
    odom_q_=qn_.normalized();
    /*odom_a_ = odom_q_ * Eigen::Vector3d(0, 0, 1) * (thrust_ * thr2acc_) - Gravity_;
    if (perfect_simu_flag_) {
        odom_v_.setZero();
        // odom_a_.setZero();
    }
    */
    yaw_ = tf::getYaw(msg->pose.pose.orientation);
    mpc_->SetAttitude(odom_q_);
    has_odom_flag_ = true;
    odom_mutex_.unlock();
}

void PlannerClass::IMUCallback(const sensor_msgs::ImuConstPtr& msg)
{
    imu_mutex_.lock();
    imu_a_ << msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z; // body frame
    Eigen::Matrix3d Rotate = odom_q_.toRotationMatrix().inverse();
    imu_a_ = Rotate * imu_a_;
    imu_mutex_.unlock();
}

void PlannerClass::LocalPcCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    local_pc_mutex_.lock();

    ros::Time now = ros::Time::now();
    local_pc_.clear();
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*msg, cloud);

    vec_cloud_.push_back(cloud);
    if (vec_cloud_.size() > 10) vec_cloud_.pop_front();
    static_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>());
    for (int i = 0; i < vec_cloud_.size(); i++) *static_cloud_ += vec_cloud_[i];
    pcl::CropBox<pcl::PointXYZ> cb; // CropBox filter (delete unuseful points)
    cb.setMin(Eigen::Vector4f(odom_p_.x() - (map_upp_.x()-0.5), odom_p_.y() - (map_upp_.y()-0.5), 0.2, 1.0));
    cb.setMax(Eigen::Vector4f(odom_p_.x() + (map_upp_.x()-0.5), odom_p_.y() + (map_upp_.y()-0.5), map_upp_.z(), 1.0));
    cb.setInputCloud(static_cloud_);
    cb.filter(*static_cloud_);
    pcl::VoxelGrid<pcl::PointXYZ> vf;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cur_cloud_ds(new pcl::PointCloud<pcl::PointXYZ>());
    Eigen::Vector3f pos = odom_p_.cast<float>();
    vf.setLeafSize(0.2, 0.2, 0.2);
    vf.setInputCloud(static_cloud_);
    vf.filter(static_map_);
    for(auto &point: static_map_.points) {
        local_pc_.push_back(Eigen::Vector3d(point.x, point.y, point.z));
    }

    // update astar map
    static int obs_count = 0;
    local_astar_->SetCenter(Eigen::Vector3d(odom_p_.x(), odom_p_.y(), 0.0));
    local_astar_->setObsVector(local_pc_, expand_fix_);
    std::vector<Eigen::Vector3d> remain_path;
    remain_path.insert(remain_path.begin(), follow_path_.begin()+astar_index_, follow_path_.end());
    if (local_astar_->CheckPathFree(remain_path) == false) replan_flag_ = true;
    // bool flag = local_astar_->CheckPathFree(follow_path_);     // check path is free or not
    // if (flag == false) obs_count++;
    // else obs_count = 0;
    // if (obs_count >= 2) {
    //     obs_count = 0;
    //     replan_flag_ = true;
    // }
    
    // local_astar_->GetOccupyPcl(cloud);
    // cloud.width = cloud.points.size();
    // cloud.height = 1;
    // cloud.is_dense = true;
    // sensor_msgs::PointCloud2 map_msg;
    // pcl::toROSMsg(cloud, map_msg);
    // map_msg.header.frame_id = "world";
    // gird_map_pub_.publish(map_msg);

    static int pub_count = 0;
    if (++pub_count % 10 == 0) {  // 每10帧发一次，5Hz
        local_astar_->GetOccupyPcl(cloud);
        cloud.width = cloud.points.size();
        cloud.height = 1;
        cloud.is_dense = true;
        sensor_msgs::PointCloud2 map_msg;
        pcl::toROSMsg(cloud, map_msg);
        map_msg.header.frame_id = "world";
        gird_map_pub_.publish(map_msg);
    }

    log_times_[0] = (ros::Time::now() - now).toSec() * 1000.0;

    local_pc_mutex_.unlock();
}

void PlannerClass::RCInCallback(const mavros_msgs::RCInConstPtr& msg)
{
    rc_mutex_.lock();

    if (simu_flag_) {
        mode_ = Command;
    } else {
        static UAVMode_e mode_last = Manual;
        static uint16_t takeoff_ch_last = 0;
        if (msg->channels[4] > 1650 && msg->channels[4] < 1800 && mode_last != Command) {
            mode_ = Command;
            ROS_INFO("\033[1;32m[MPC FSM]: Manual or Hover --> Command.\033[0m");
        }
        if (mode_ == Command && msg->channels[4] > 1850) {
            mode_ = Hover;
            goal_p_ = odom_p_;
            ROS_INFO("\033[32m[MPC FSM]: Command --> Hover. Pos is: %f %f %f\033[32m", goal_p_.x(), goal_p_.y(), goal_p_.z());
        }
        if (mode_ == Hover && takeoff_ch_last < 1500 && msg->channels[10] > 1500) {
            goal_p_.z() = 0.0;
            ROS_WARN("[MPC FSM]: Land! Pos is: %f %f %f", goal_p_.x(), goal_p_.y(), goal_p_.z());
        }
        
        takeoff_ch_last = msg->channels[10];
        mode_last = mode_;
    }

    rc_mutex_.unlock();
}
