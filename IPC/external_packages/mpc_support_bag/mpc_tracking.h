#pragma once

#include <deque>
#include <nav_msgs/Odometry.h>
#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/ContactsState.h>
#include <geometry_msgs/Wrench.h>
#include <mutex>
#include "mpc_ros_application.h"
#include "disturbance_observer.h"
#include "plot.h"
#include <std_msgs/Float64MultiArray.h>

#include <cmath>
#include <iostream>
#include <fstream>
#include <iomanip>

#include <ctime>   // [新增] 用于时间戳
#include <sstream> // [新增] 用于字符串拼接

#include <std_msgs/Bool.h>

//#include "px4_interface.h"
 
// 轨迹点结构
struct Point2D {
    double x;
    double y;
};

// --- [新增] 数据结构定义 ---
struct FullLogData {
    double time;
    // 位置信息
    double px, py, pz;
    // 控制信息 (核心对比)
    double wx, wy, wz;  // 角速度

    double F_mpc;      // MPC 算出来的基准推力
    double dF_dob;     // DOB 算出来的补偿推力 (delta_u)
    double F_total;    // 两者相加
    double F_sat;      // 模拟物理限幅后的值 (用于证明 Case 2 越界)

    // 干扰信息
    double d_x,d_y,d_z; // Z轴干扰估计值
    // 姿态四元数  
    double qw, qx, qy, qz; 
};


//extern bool observer_enabled_;// 全局观测器使能标志位（extern声明，仅在此处声明，不定义）
bool observer_enabled_=true;// 全局观测器使能标志位（extern声明，仅在此处声明，不定义）

class TrackingMpc: public MpcRosApplication 
{
private:
    ros::Subscriber odom_subscriber_;
    ros::Subscriber gdt_subscriber_;
    ros::Subscriber contact_subscriber_;
    ros::Publisher payload_force_publisher_;

    ros::Publisher disturbance_pub_;  // 干扰估计发布者
    std_msgs::Float64MultiArray disturbance_msg_;  // 干扰消息

    ros::Publisher T_pub_;  // 推力发布者
    std_msgs::Float64MultiArray T_msg_;  // 推力消息

    ros::Publisher pd_pub_;  // 参考位置发布者
    std_msgs::Float64MultiArray pd_msg_;  // 参考位置消息
 

    float delta_T;//推力补偿
    float delta_T_sat;//推力补偿  饱和后
    float T_sat;//推力  饱和后
    int  hmpc_flag;  //是否启动多速率MPC方案。
    int  DOBhz;  
    int  MPChz; 
    int  DOBus;
    int  MPCus;  
    int  trajectory_set; 
    double omega;
    
      


    Eigen::MatrixXd qrs_;
    double gamma_;
    Eigen::MatrixXd Gamma_ = Eigen::MatrixXd::Zero(ACADO_NX, ACADO_NX);
    Eigen::MatrixXd ESKF_A_ = Eigen::MatrixXd::Zero(ACADO_NX + ACADO_NOD -1, ACADO_NX + ACADO_NOD -1);

    Eigen::Matrix<real_t, ACADO_NOD, ACADO_N + 1> online_data_ = 
        Eigen::Matrix<real_t, ACADO_NOD, ACADO_N + 1>::Zero();

    double mpc_time_step_ = 0.1;
    int mpc_steps_ = 20;
    Eigen::MatrixXd mpc_Bd_ = Eigen::MatrixXd::Zero(ACADO_NX, 3);
    Eigen::MatrixXd mpc_A_ = Eigen::MatrixXd::Zero(ACADO_NX, ACADO_NX);
    Eigen::MatrixXd mpc_P_ = Eigen::MatrixXd::Zero(ACADO_NX, ACADO_NX);

    std::thread trajectory_creation_thread_;

    std::thread disturbance_estimation_thread_;  // 干扰估计线程
    std::atomic<bool> is_estimation_running_;    // 线程运行标志（原子变量，线程安全）
    std::mutex disturbance_mutex_;               // 保护d_hat_的互斥锁
 

    std::mutex mpc_mutex_;

    Eigen::Vector3d attitude_;

    enum TrajectoryType
    {
        TRAJECTORY_NONE = 0,
        TRAJECTORY_CIRCLE,
        TRAJECTORY_CIRCLE_2,
        TRAJECTORY_LEMNISCATE, 
        TRAJECTORY_point,
        TRAJECTORY_LINE,
    };

    enum ForcePubType
    {
        FORCE_PUBLISH_NONE = 0,
        FORCE_PUBLISH_ZERO,
        FORCE_PUBLISH_PAYLOAD,
        FORCE_PUBLISH_MINUS_PAYLOAD,
    };

    float payload_force_ = 0.2 * kG; // 负载力，N

    double lemniscate_a_ = 3.0; // 半长轴

    double speed_;

    struct CurrentTargetStates
    {
        real_t target_x;
        real_t target_y;
        real_t target_z;
        Eigen::Quaternion<real_t> target_q;
        Eigen::Vector3d target_euler;
        real_t target_vx;
        real_t target_vy;
        real_t target_vz;
        real_t target_thrust;
        real_t target_wx;
        real_t target_wy;
        real_t target_wz;

        static constexpr real_t kStartPosX = 0.0;
        static constexpr real_t kStartPosY = 0.0;
        static constexpr real_t kStartPosZ = 2;    //初始高度
        CurrentTargetStates(): target_x(kStartPosX), target_y(kStartPosY), 
                              target_z(kStartPosZ), target_vx(0.0), 
                              target_vy(0.0), target_vz(0.0), target_thrust(kG*kMass),
                              target_wx(0.0), target_wy(0.0), target_wz(0.0)
        {
            target_q.w() = 1.0;
            target_q.x() = 0.0;
            target_q.y() = 0.0;
            target_q.z() = 0.0;
        }
    };
    bool is_first_estimation_ = true;
    bool is_contact_ = false;
    bool is_at_startposition_ = false;
    
    CurrentTargetStates current_target_states_;
    TrajectoryType trajectory_type_;
    ForcePubType force_pub_type_ = FORCE_PUBLISH_NONE;
    DataLogger pos_logger_{"~/data/quadrotor_mpc/pos.csv"};
    std::string file_path = "/home/go2/ws_my/src/quadrotor_mpc/data/";

    Eigen::Matrix<real_t, Eigen::Dynamic, 1> d_hat_ = Eigen::Matrix<real_t, Eigen::Dynamic, 1>::Zero(3);
    Eigen::Matrix<real_t, Eigen::Dynamic, 1> d0_hat_ = Eigen::Matrix<real_t, Eigen::Dynamic, 1>::Zero(3);
    Eigen::Matrix<real_t, Eigen::Dynamic, 1> dd_hat_ = Eigen::Matrix<real_t, Eigen::Dynamic, 1>::Zero(3);
    Eigen::Matrix<real_t, Eigen::Dynamic, 1> pre_d_hat_ = Eigen::Matrix<real_t, Eigen::Dynamic, 1>::Zero(3);

    std::vector<Point2D> raw_traj_;
    std::vector<double> arc_len_;

    DisturbanceObserver disturbance_observer_{0.01};
    void disturbanceEstimationLoop();  // 关键：必须在类内声明
    
  


public:
    TrackingMpc();
    ~TrackingMpc();

    bool let_mpc_run_;
    bool line_flag=false;

    //   成员变量：记录直线轨迹开始的时刻
    std::chrono::steady_clock::time_point line_start_time_;
    
    //   标记是否正在飞直线
    bool is_flying_line_ = false;

    void init() override;
    void odomCallback(const nav_msgs::OdometryConstPtr& odom_msg);
    void gdtCallback(const gazebo_msgs::ModelStatesConstPtr& gdt_msg);
    void setReference() override;
    void setWeight() override;
    void setOnlineData() override;
    void mpcProcess() override;
    void createTrajectory();
    Eigen::Vector3f getTargetPosition();
    Eigen::Quaternion<real_t> euler2Quaternion(double roll, double pitch, double yaw);
    void dataLog(double time);
    Eigen::Vector3d Quaternion2Euler(Eigen::Quaterniond q);
    std::vector<Point2D> generateLemniscateTrajectory(double a, double t_step);
    std::vector<double> computeArcLength(const std::vector<Point2D>& traj);

    // 获取当前状态的接口（返回10维状态向量）
    Eigen::Matrix<real_t, ACADO_NX, 1> getCurrentStates() const {
        return current_states_;
    }

    //获取干扰估计的接口（返回3维干扰 d_hat_）
    Eigen::Matrix<real_t, Eigen::Dynamic, 1> getDisturbanceEstimate() const {
        return d_hat_;
    }

    //获取传入NMPC干扰d0的接口（返回3维干扰 d_hat_）
    Eigen::Matrix<real_t, Eigen::Dynamic, 1> getD0hat() const {
        return d0_hat_;
    }

    // 新增：获取无人机质量的接口
    real_t getDroneMass() const {
        return kMass;  // 假设 kMass 是 TrackingMpc 中定义的无人机质量（如1.0kg）
    }

    void startDisturbanceEstimation();  // 启动干扰估计线程
    void stopDisturbanceEstimation();   // 停止干扰估计线程

    float get_delta_T(){
        return delta_T;
    }
    void set_delta_T(float x){
        delta_T=x;
    }

    float get_delta_T_sat(){
        return delta_T;
    }
    void set_delta_T_sat(float x){
        delta_T_sat=x;
    }

    void set_T_sat(float x){
         T_sat=x;
    }

    float get_T_sat(){
        return T_sat;
    }
    
    int get_hmpc_flag(){
        return hmpc_flag;
    }
    int get_DOBUS(){
        return DOBus;
    }

        // [新增 2] 成员变量
    std::vector<FullLogData> history_log_; 
    bool is_logging_ = false;             // 记录开关
    double log_start_time_ = 0.0;         // 记录开始的时间戳
    const double LOG_MAX_THRUST = 25.0;   // 物理最大油门

    // [新增 3] 函数声明
    void saveToCSV(); 
};




