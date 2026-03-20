#include "mpc_tracking.h"


 
//构造函数
TrackingMpc::TrackingMpc(): 
trajectory_type_(TRAJECTORY_point),  // 初始化成员变量   TRAJECTORY_LEMNISCATE TRAJECTORY_CIRCLE
speed_(3.0),
let_mpc_run_(false),
gamma_(0),
is_estimation_running_(false) 
{
    init();
    startDisturbanceEstimation(); // 初始化完成后启动干扰估计线程
}

TrackingMpc::~TrackingMpc()
{
    stopDisturbanceEstimation();  // 停止干扰估计线程
}

void TrackingMpc::init() 
{

    /*---------状态变量初始化（无人机初始状态设置）-----*/
    // 1. 定义并初始化无人机初始状态（ACADO_NX 是状态维度，如10维：3位置+4姿态+3速度）
    Eigen::Matrix<real_t, ACADO_NX, 1> initial_states;
    initial_states << 0.0, 0.0, 0.0, 
                      1.0, 0.0, 0.0, 0.0,
                      0.0, 0.0, 0.0;

    // 2. 初始化当前状态、预测状态、参考状态和控制量
    current_states_ = initial_states;
    predicted_states_ = initial_states.replicate(1, ACADO_N + 1);
    reference_states_.setZero();// 参考轨迹矩阵初始化为全0
    reference_controls_.setZero();
    current_control_.setZero();

    /*-------MPC 相关矩阵初始化（控制参数配置）-------------*/
    //（具体作用需结合MPC求解逻辑，可能是松弛因子）
    Gamma_ = Eigen::Matrix<double, ACADO_NX, ACADO_NX>::Identity() * gamma_;
    

    // 2. 初始化状态权重矩阵 q_（kCostStateSize 是成本函数中的状态维度，通常与ACADO_NX一致）
    q_ = (Eigen::Matrix<real_t, kCostStateSize, 1>() <<
        10.0 * Eigen::Matrix<real_t, 2, 1>::Ones(),//10
        9,
        10 * Eigen::Matrix<real_t, 4, 1>::Ones(),
        0.1 * Eigen::Matrix<real_t, 3, 1>::Ones()).finished().asDiagonal();

    // 3. 初始化控制量权重矩阵 r_（ACADO_NU 是控制量维度，如4维：3角速度+1推力）
    r_ = (Eigen::Matrix<real_t, ACADO_NU, 1>() <<
        0.1 * Eigen::Matrix<real_t, ACADO_NU, 1>::Ones()).finished().asDiagonal();

    // 初始化MPC包装器（mpc_wrapper_），传入初始状态、预测状态、参考轨迹、权重矩阵等参数
    mpc_wrapper_.initMpc(current_states_, predicted_states_, reference_states_, 
        reference_controls_, q_, r_);

    // 获取MPC控制量的约束边界（如推力范围、角速度上限）    
    mpc_wrapper_.getMpcBound(control_lower_bound_, control_upper_bound_);

    //传感器数据订阅（获取无人机状态）
    // odom_subscriber_ = nh_.subscribe("/mavros/local_position/odom", 
    //             1, &TrackingMpc::odomCallback, this);

    gdt_subscriber_ = nh_.subscribe("/gazebo/model_states",
                1, &TrackingMpc::gdtCallback, this);

    // 初始化干扰发布者
    disturbance_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("/disturbance_estimate", 10);
    // 初始化消息数据大小（3个元素：d_x, d_y, d_z）
    disturbance_msg_.data.resize(3);

    // 初始化推力发布者
    T_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("/T_compensation", 1);
    // 初始化消息数据大小（3个元素：T0, dt, T0+dt）
    T_msg_.data.resize(3);

    // 初始化参考位置发布者
    pd_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("/position_ref", 1);
    // 初始化消息数据大小（3个元素：xr,yr,zr）
    pd_msg_.data.resize(3);


    //轨迹生成线程启动（异步准备轨迹数据）创建一个新的线程,并列（并发）运行
    trajectory_creation_thread_ = std::thread(&TrackingMpc::createTrajectory, this);

    // ★★★★★★ 读取参数 ★★★★★★ 
    nh_.param<int>("hmpc_flag", hmpc_flag, 1);  // 读取 yaml hmpc_flag
    nh_.param<int>("DOBhz", DOBhz, 100);  // 读取读取 yaml DOBhz
    nh_.param<int>("MPChz", MPChz, 100);  // 读取读取 yaml MPChz
    nh_.param<double>("omega", omega, 2.0);  // 读取读取 yaml MPChz
    DOBus=1000000/DOBhz; //us
    MPCus=1000/MPChz; //ms

    disturbance_observer_.setOmega(omega);
    nh_.param<int>("trajectory_set", trajectory_set, 0);  // 读取读取 yaml 
    trajectory_type_=TRAJECTORY_CIRCLE;


    nh_.param<double>("speed_", speed_, 3);  // 读取读取 yaml

    printf("hmpc_flag:  %d ; traj_set:%d ; speed_:%f ;\r\n ",hmpc_flag,trajectory_set,speed_);
    printf(" MPChz:  %d; DOBhz:  %d  DOBus:%d  omega:%f  \r\n ",MPChz,DOBhz,DOBus,omega);
}





void TrackingMpc::odomCallback(const nav_msgs::OdometryConstPtr& odom_msg) 
{
    int i = 0;
    double current_time = odom_msg->header.stamp.toSec();
    current_states_[i++] = static_cast<real_t>(odom_msg->pose.pose.position.x);
    current_states_[i++] = static_cast<real_t>(odom_msg->pose.pose.position.y);
    current_states_[i++] = static_cast<real_t>(odom_msg->pose.pose.position.z);
    current_states_[i++] = static_cast<real_t>(odom_msg->pose.pose.orientation.w);
    current_states_[i++] = static_cast<real_t>(odom_msg->pose.pose.orientation.x);
    current_states_[i++] = static_cast<real_t>(odom_msg->pose.pose.orientation.y);
    current_states_[i++] = static_cast<real_t>(odom_msg->pose.pose.orientation.z);
    current_states_[i++] = static_cast<real_t>(odom_msg->twist.twist.linear.x);
    current_states_[i++] = static_cast<real_t>(odom_msg->twist.twist.linear.y);
    current_states_[i++] = static_cast<real_t>(odom_msg->twist.twist.linear.z);
    current_states_.segment(3,4).normalize();

    if(current_states_.segment(3,4).dot(reference_states_.col(0).segment(3,4)) < 0)
    {
        current_states_.segment(3,4) = -current_states_.segment(3,4);
        Eigen::Matrix<real_t, 4, 1> q = current_states_.segment(3,4);
    }

    is_new_data_ = true;
    mpc_count_ = 0;
}


void TrackingMpc::gdtCallback(const gazebo_msgs::ModelStatesConstPtr& gdt_msg)
{
    int i = 0; 
    for(int j = 0; j < gdt_msg->name.size(); j++)
    {
        if(gdt_msg->name[j] == "iris")
        {
            current_states_[i++] = static_cast<real_t>(gdt_msg->pose[j].position.x);
            current_states_[i++] = static_cast<real_t>(gdt_msg->pose[j].position.y);
            current_states_[i++] = static_cast<real_t>(gdt_msg->pose[j].position.z);
            current_states_[i++] = static_cast<real_t>(gdt_msg->pose[j].orientation.w);
            current_states_[i++] = static_cast<real_t>(gdt_msg->pose[j].orientation.x);
            current_states_[i++] = static_cast<real_t>(gdt_msg->pose[j].orientation.y);
            current_states_[i++] = static_cast<real_t>(gdt_msg->pose[j].orientation.z);
            current_states_[i++] = static_cast<real_t>(gdt_msg->twist[j].linear.x);
            current_states_[i++] = static_cast<real_t>(gdt_msg->twist[j].linear.y);
            current_states_[i++] = static_cast<real_t>(gdt_msg->twist[j].linear.z);
            current_states_.segment(3,4).normalize();// 四元数标准化（确保模长为1，避免数值误差导致的姿态计算错误）

            // 四元数方向修正（避免姿态表示的二义性）
            if(current_states_.segment(3,4).dot(reference_states_.col(0).segment(3,4)) < 0)
            {
                current_states_.segment(3,4) = -current_states_.segment(3,4);
                Eigen::Matrix<real_t, 4, 1> q = current_states_.segment(3,4);
            }

            //姿态转换（四元数→欧拉角）
            Eigen::Quaterniond q(current_states_(3), current_states_(4), 
                current_states_(5), current_states_(6));
            
            attitude_ = Quaternion2Euler(q);

            if(trajectory_type_!=TRAJECTORY_NONE && is_at_startposition_ && !is_contact_)
            {
                static auto start_time = std::chrono::steady_clock::now();
                auto current_time = std::chrono::steady_clock::now();
                dataLog(std::chrono::duration<double>(current_time - start_time).count());
            }
            is_new_data_ = true;
            mpc_count_ = 0;

            break;
        }
    }
}









// 干扰估计线程的主函数（独立循环更新d_hat_）
void TrackingMpc::disturbanceEstimationLoop() 
{
    // [新增] 预留空间，防止 vector 频繁扩容卡顿
    history_log_.reserve(600000); // 预留约 60秒的数据量

    while(ros::ok())
    {
        static auto current_time = std::chrono::steady_clock::now();

        auto pre_time = current_time;
        current_time = std::chrono::steady_clock::now();

        // 1. 提取当前状态（位置、姿态、速度）到向量 y
        Eigen::VectorXd y = Eigen::VectorXd::Zero(ACADO_NX);
        y << current_states_(0), current_states_(1), current_states_(2),
             current_states_(3), current_states_(4), current_states_(5),
             current_states_(6), current_states_(7), current_states_(8),
             current_states_(9);

        // 2. 初始化干扰观测器（首次运行时）
        if(is_first_estimation_|| !observer_enabled_)
        {
            is_first_estimation_ = false;
            Eigen::VectorXd z = Eigen::VectorXd::Zero(ACADO_NX);
            z << y; 
            disturbance_observer_.setFirstZ(z);
        }
        else
        {
            // 非首次运行时，更新时间间隔 dt（当前时刻 - 上一时刻）
            disturbance_observer_.setDt(
                std::chrono::duration<double>(current_time - pre_time).count());
        }
        
        // 3. 提取当前控制量到向量 u
        Eigen::VectorXd u = Eigen::VectorXd::Zero(ACADO_NU);
        float tmp_u3=get_T_sat();  //饱和控制推力

        float tmp_delta_u3=0;
        if(hmpc_flag>=2) tmp_delta_u3=1*get_delta_T();
        //u << current_control_(0), current_control_(1), current_control_(2), current_control_(3)+tmp_delta_u3;
        u << current_control_(0), current_control_(1), current_control_(2), tmp_u3;

        //printf("DOB: u0- %f u- %f usat- %f\r\n ",current_control_(3),current_control_(3)+tmp_delta_u3,tmp_u3);

        // 4. 调用干扰观测器估计外部干扰（如阵风、负载变化）
        if (observer_enabled_) {
            // disturbance_observer_.update(y, u, current_states_(5), current_states_(6), current_states_(7), kMass, kG, visual_jacob);
            // disturbance_observer_.finite_update(y, u, current_states_(5), current_states_(6), current_states_(7), kMass, kG, visual_jacob);
            // disturbance_observer_.nonlinear_update(y, u, current_states_(5), current_states_(6), current_states_(7), kMass, kG, visual_jacob);
            disturbance_observer_.kalman_geso_update(y, u, kMass, kG);  // 干扰估计
            d_hat_.block<3,1>(0,0) = disturbance_observer_.getDisturbance().block<3, 1>(0, 0).cast<real_t>();// 保存估计的干扰（3维力干扰）
            //d_hat_.block<3,1>(0,0) = Eigen::Matrix<real_t, 3, 1>::Zero();  // 覆盖为零向量
        } else {
                // 标志位为false：观测器输出强制为0
                d_hat_.setZero();
        }
 
        disturbance_msg_.data[0] = d_hat_(0, 0);  // 赋值 d_x 估计值
        disturbance_msg_.data[1] = d_hat_(1, 0);  // 赋值 d_y 估计值
        disturbance_msg_.data[2] = d_hat_(2, 0);  // 赋值 d_z 估计值
        disturbance_pub_.publish(disturbance_msg_);  // 发布干扰消息

        T_msg_.data[0] = current_control_(3);  // mpc
        T_msg_.data[1] = current_control_(3)+tmp_delta_u3;  // 补偿T
        T_msg_.data[2] = tmp_u3;  //饱和T
        T_pub_ .publish(T_msg_);  // 

        // -----------------------------------------------------------
        // [新增] 1. 准备要记录的数据
        // -----------------------------------------------------------
        
        // A. 获取 MPC 计算的控制量
        double u_wx = current_control_(0);
        double u_wy = current_control_(1);
        double u_wz = current_control_(2);
        double u_mpc_thrust = current_control_(3); 
        
        // B. 获取 DOB 补偿量 
        // (get_delta_T 是你在 px4_interface 计算并 set 进来的，或者在这里估算的)
    

        // C. 计算总推力 (Commanded)
        double u_total_thrust = u_mpc_thrust+tmp_delta_u3;

        // D. 模拟物理限幅 (Saturated) -> 论文画图关键
        double u_sat_thrust = u_total_thrust;
        //if(u_sat_thrust > LOG_MAX_THRUST) u_sat_thrust = LOG_MAX_THRUST;
        if(u_sat_thrust < 0.0) u_sat_thrust = 0.0; 

        // -----------------------------------------------------------
        // [新增] 2. 如果开关打开，则记录数据
        // -----------------------------------------------------------
        if (is_logging_) {
            // 计算相对时间
            double current_log_time = std::chrono::duration<double>(current_time - std::chrono::steady_clock::now()).count() + ros::Time::now().toSec(); 
            // 或者简单点，用累加时间：
            static double log_duration = 0;
            log_duration += std::chrono::duration<double>(current_time - pre_time).count();

            if (history_log_.size() < 600000) { // 内存保护
                FullLogData data;
                data.time = log_duration;
                
                // 位置
                data.px = current_states_(0);
                data.py = current_states_(1);
                data.pz = current_states_(2);
                
                // 角速度
                data.wx = u_wx;
                data.wy = u_wy;
                data.wz = u_wz;

                // 推力对比
                data.F_mpc   = u_mpc_thrust;
                data.dF_dob  = tmp_delta_u3;
                data.F_total = u_total_thrust; // Case 2 越界
                data.F_sat   = tmp_u3;   // Case 2 削顶

                // 干扰估计
                data.d_x = d_hat_(0, 0);
                data.d_y = d_hat_(1, 0);
                data.d_z = d_hat_(2, 0);

                //记录姿态四元数
                data.qw = current_states_(3);
                data.qx = current_states_(4);
                data.qy = current_states_(5);
                data.qz = current_states_(6);

                history_log_.push_back(data);
            }
        }

        std::this_thread::sleep_for(std::chrono::microseconds(DOBus));   
        //std::this_thread::sleep_for(std::chrono::milliseconds(1));//200hz
    } 

}

// 启动干扰估计线程
void TrackingMpc::startDisturbanceEstimation() {
    is_estimation_running_ = true;
    // 启动线程，绑定到干扰估计循环函数
    disturbance_estimation_thread_ = std::thread(&TrackingMpc::disturbanceEstimationLoop, this);
}

// 停止干扰估计线程
void TrackingMpc::stopDisturbanceEstimation() {
    is_estimation_running_ = false;
    if (disturbance_estimation_thread_.joinable()) {
        disturbance_estimation_thread_.join();  // 等待线程退出
    }
}







// MPC 控制器的 核心控制循环
void TrackingMpc::mpcProcess()
{
    while (ros::ok())
    {
        static auto current_time = std::chrono::steady_clock::now();
        auto pre_time = current_time;
        current_time = std::chrono::steady_clock::now();

        if(is_contact_)
        {
            mpc_error_ = CONTACT_DETECTED;
            return;
        }

        //disturbanceEstimationLoop();

        // 1. 构造在线数据（将估计的干扰 d_hat_ 复制到 MPC 所有预测步）
        //（d_hat_由disturbance_estimation_thread_更新）
        if(hmpc_flag!=2) online_data_ = (Eigen::Matrix<real_t, ACADO_NOD, 1>() << d_hat_.block<3,1>(0,0)).finished().replicate(1, ACADO_N + 1);
        else online_data_.setZero(); 

        // printf("onlinedata: %f\t %f\t %f\n", online_data_(0,0) ,online_data_(1,0), online_data_(2,0));
        
        setReference();// 2. 更新参考轨迹（如 8字形、圆形轨迹的当前目标点）
        setWeight(); // 3. 更新 MPC 权重矩阵（状态跟踪权重 q_、控制量权重 r_ 等）
        setOnlineData();// 4. 设置 MPC 在线数据（将 online_data_ 传递给求解器）

        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now(); 
        std::chrono::milliseconds duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - current_time);
        double period = duration.count();

        if(is_new_data_)
        {
            is_new_data_ = false;
            Eigen::Matrix<float, 10, 1> state_float;
            if (true){
                state_float= current_states_; //测量状态
            }
            else{
                state_float = disturbance_observer_.getKfState().cast<float>();//KF滤波状态                   
            }


            if(mpc_wrapper_.solveMpc(state_float)) //mpc 求解
            {
                // std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now(); 
                // std::chrono::milliseconds duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
                // double period = duration.count();
                // std::cout << "mpcsolve time: " << period << " ms" << std::endl;         
                mpc_wrapper_.getMpcControl(current_control_, 0); // 获取第 0 步的控制量（当前应执行的控制）
                // std::cout << "mpc_control:" << current_control_.transpose() << std::endl;
            }
            else
            {
                ROS_WARN("MPC solve failed!");
            }

            if(hmpc_flag==3) d0_hat_=d_hat_;
        }
        else  // 若无新数据（状态数据丢失或延迟）
        {
            mpc_count_++;
            if(mpc_count_ >= ACADO_N)
            {
                mpc_error_ = DATA_LOST;
            }
            else
            {
                mpc_wrapper_.getMpcControl(current_control_, mpc_count_);// 未超过阈值时，使用之前求解的未来步控制量（滚动优化的容错机制）
            }
        }
        
        
        std::this_thread::sleep_for(std::chrono::milliseconds(MPCus));//100hz
    } 
}

void TrackingMpc::setReference() 
{
    reference_controls_ = (Eigen::Matrix<real_t, ACADO_NU, 1>() << 
        current_target_states_.target_wx, current_target_states_.target_wy,
        current_target_states_.target_wz,
        current_target_states_.target_thrust ).finished().replicate(1, ACADO_N);
    // 调用MpcWrapper函数将reference传入到mpc_wrapper_中
    mpc_wrapper_.setMpcReference(reference_states_, reference_controls_);
}

void TrackingMpc::setWeight()
{
 
}

void TrackingMpc::setOnlineData()
{
    mpc_wrapper_.setMpcOnlineData(online_data_);
}


Eigen::Vector3d TrackingMpc::Quaternion2Euler(Eigen::Quaterniond q)
{
    Eigen::Vector3d euler(0, 0, 0);
    euler(0) = atan2(2 * (q.y() * q.z() + q.w() * q.x()),
                    q.w() * q.w() - q.x() * q.x() - q.y() * q.y() + q.z() * q.z());
    euler(1) = asin(-2 * (q.x() * q.z() - q.w() * q.y()));
    euler(2) = atan2(2 * (q.x() * q.y() + q.w() * q.z()),
                   q.w() * q.w() + q.x() * q.x() - q.y() * q.y() - q.z() * q.z());
    return euler;
}

Eigen::Quaternion<real_t> TrackingMpc::euler2Quaternion(double roll, double pitch, double yaw)
{
    Eigen::Quaternion<real_t> q;
    real_t r, p, y;
    r = roll/2.0;
    p = pitch/2.0;
    y = yaw/2.0;
    q.w() = cos(r) * cos(p) * cos(y) + sin(r) * sin(p) * sin(y);
    q.x() = sin(r) * cos(p) * cos(y) - cos(r) * sin(p) * sin(y);
    q.y() = cos(r) * sin(p) * cos(y) + sin(r) * cos(p) * sin(y);
    q.z() = cos(r) * cos(p) * sin(y) - sin(r) * sin(p) * cos(y);
    return q; 
}

std::vector<Point2D> TrackingMpc::generateLemniscateTrajectory(double a, double t_step) {
    std::vector<Point2D> trajectory;

    // 生成 3 圈，从中间点开始
    double start_t = -M_PI / 2.0;       // 从交叉点开始
    double end_t   = start_t + 3 * 2 * M_PI;  // 三圈

    int num_points = static_cast<int>((end_t - start_t) / t_step);

    for (int i = 0; i <= num_points; ++i) {
        double t = start_t + i * t_step;
        double denom = 1 + std::pow(std::sin(t), 2);
        double x = a * std::cos(t) / denom;
        double y = a * std::cos(t) * std::sin(t) / denom;
        trajectory.push_back({x, y});
    }

    return trajectory;
}

std::vector<double> TrackingMpc::computeArcLength(const std::vector<Point2D>& traj) {
    std::vector<double> arc_len;
    arc_len.push_back(0.0);
    for (size_t i = 1; i < traj.size(); ++i) {
        double dx = traj[i].x - traj[i - 1].x;
        double dy = traj[i].y - traj[i - 1].y;
        double ds = std::sqrt(dx * dx + dy * dy);
        arc_len.push_back(arc_len.back() + ds);
    }
    return arc_len;
}
bool firsttime=true;
void TrackingMpc::createTrajectory()
{
    while (ros::ok())
    {
     
        if (!is_at_startposition_) //飞行准备
        {
            while (!let_mpc_run_)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }

            if (firsttime)
            {
                std::this_thread::sleep_for(std::chrono::seconds(1));
                firsttime=false;
            }

            static double target_x = 0.0;           // 缓启动
            static double target_y = 0.0;
            static double target_z = 0.0;
            

            static int target_count = 0;
            int resolution = 300;

            if(target_count < resolution)
            {
                target_x += current_target_states_.target_x / static_cast<double>(resolution);
                target_y += current_target_states_.target_y / static_cast<double>(resolution);
                target_z += current_target_states_.target_z / static_cast<double>(resolution);
                target_count++;
            }
            
            // 计算当前位置与目标位置的水平距离和垂直距离
            float horizontal_distance = 0.0;
            float z_distance = 0.0;
            float ex = current_states_[0] - current_target_states_.target_x;
            float ey = current_states_[1] - current_target_states_.target_y;
            float ez = current_states_[2] - current_target_states_.target_z;

            horizontal_distance = sqrtf(powf(ex, 2) + powf(ey, 2));
            z_distance = fabs(ez);  // 修正：计算z轴距离（绝对值）
            
            //更新 MPC 参考轨迹
            reference_states_ = (Eigen::Matrix<real_t, kCostStateSize, 1>() << 
            target_x, target_y, target_z, 
            current_target_states_.target_q.w(), current_target_states_.target_q.x(),
            current_target_states_.target_q.y(), current_target_states_.target_q.z(),
            current_target_states_.target_vx, current_target_states_.target_vy, 
            current_target_states_.target_vz).finished().replicate(1, ACADO_N + 1);

            //printf("distance: x- %f\t   y- %f\t  z- %f\t \n", ex,ey,ez);
            //printf(" ez- %f\t   tz- %f  \t \n", ez,target_z);
            
            if(horizontal_distance < 0.1 && z_distance <  0.1)
            {
                ROS_INFO("is_at_startposition_!");
                std::this_thread::sleep_for(std::chrono::seconds(6));  // 悬停6秒，稳定状态

                if(trajectory_type_ == TRAJECTORY_LEMNISCATE)
                {
                    raw_traj_ = generateLemniscateTrajectory(lemniscate_a_, 0.001);// 生成8字形轨迹
                    arc_len_ = computeArcLength(raw_traj_);// 计算轨迹弧长（用于速度规划）
                }
                is_at_startposition_ = true; // 标记已到达初始位置
            }
        } 
        else
        {
            switch (trajectory_type_)
            {
                case TRAJECTORY_NONE:
                {
                    reference_states_ = (Eigen::Matrix<real_t, kCostStateSize, 1>() << 
                        current_target_states_.target_x, current_target_states_.target_y,
                        current_target_states_.target_z, 
                        current_target_states_.target_q.w(), current_target_states_.target_q.x(),
                        current_target_states_.target_q.y(), current_target_states_.target_q.z(),
                        current_target_states_.target_vx, current_target_states_.target_vy, 
                        current_target_states_.target_vz).finished().replicate(1, ACADO_N + 1);
                    break;
                }

                case TRAJECTORY_point:  // 定点
                {
   
                    //生成 MPC 的多步参考轨迹（预测未来所有时间步的目标）
                   
                        int i = 0;
                        current_target_states_.target_x = 0;
                        current_target_states_.target_y = 0;
                        current_target_states_.target_z = 1;

                        reference_states_.block(0,i,kCostStateSize,ACADO_N + 1-i) = (Eigen::Matrix<real_t, kCostStateSize, 1>() << 
                        current_target_states_.target_x, 
                        current_target_states_.target_y, 
                        current_target_states_.target_z, 
                        current_target_states_.target_q.w(), 
                        current_target_states_.target_q.x(),
                        current_target_states_.target_q.y(), 
                        current_target_states_.target_q.z(),
                        current_target_states_.target_vx, 
                        current_target_states_.target_vy, 
                        current_target_states_.target_vz).finished().replicate(1, ACADO_N + 1-i);
                    break;
                }
                 default:
                    break;
            }
        }

        pd_msg_.data[0] = reference_states_(0, 0) ;  // 赋值 d_x 估计值
        pd_msg_.data[1] = reference_states_(1, 0) ;  // 赋值 d_y 估计值
        pd_msg_.data[2] = reference_states_(2, 0) ;  // 赋值 d_z 估计值
        pd_pub_.publish(pd_msg_);  // 发布干扰消息
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}



Eigen::Vector3f TrackingMpc::getTargetPosition()
{
    return Eigen::Vector3f(current_target_states_.target_x, 
        current_target_states_.target_y, current_target_states_.target_z);
}


void TrackingMpc::dataLog(double time)
{
    Eigen::VectorXd pos_data = Eigen::VectorXd::Zero(18);

    pos_data << current_states_(0), current_states_(1), current_states_(2),
                current_target_states_.target_x,
                current_target_states_.target_y, current_target_states_.target_z,
                current_states_(0)-current_target_states_.target_x,
                current_states_(1)-current_target_states_.target_y,
                current_states_(2)-current_target_states_.target_z,
                attitude_(0)-current_target_states_.target_euler(0),
                attitude_(1)-current_target_states_.target_euler(1),
                attitude_(2)-current_target_states_.target_euler(2),
                attitude_(0), attitude_(1), attitude_(2),
                current_target_states_.target_euler(0),
                current_target_states_.target_euler(1), current_target_states_.target_euler(2);

                
    pos_logger_.logData(time, pos_data);
}


void TrackingMpc::saveToCSV() {
    if (history_log_.empty()) {
        ROS_WARN("No data to save!");
        return;
    }

    // [修改] 生成带时间戳的文件名
    auto now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(now);

    std::stringstream ss;
    ss << std::put_time(std::localtime(&in_time_t), "%Y%m%d_%H%M%S"); // 格式: 20231027_153000

    // 文件名: Experiment_Case_X_20231027_153000.csv
    // 注意路径，你可以改成 "/home/go2/ws_my/data/..." 确保有权限
    //std::string filename = file_path+"Experiment_Case_" + std::to_string(hmpc_flag) + "_" + ss.str() + ".csv";
    std::string filename = file_path+"Experiment_Case_" + std::to_string(hmpc_flag) + ".csv";
    std::ofstream file(filename);

    if (!file.is_open()) {
        ROS_ERROR_STREAM("Failed to open file: " << filename);
        return;
    }

    // 写入表头
    file << "time,px,py,pz,wx,wy,wz,F_mpc,dF_dob,F_total,F_sat,d_x,d_y,d_z,qw,qx,qy,qz\n";
    file << std::fixed << std::setprecision(6);

    // 写入数据
    for (const auto& d : history_log_) {
        file << d.time << ","
             << d.px << "," << d.py << "," << d.pz << ","
             << d.wx << "," << d.wy << "," << d.wz << ","
             << d.F_mpc << ","
             << d.dF_dob << ","
             << d.F_total << ","
             << d.F_sat << ","
             << d.d_x << "," << d.d_y << "," << d.d_z << ","
             << d.qw << "," << d.qx << "," << d.qy << "," << d.qz << "\n"; 
    }

    file.close();
    ROS_INFO_STREAM("Data saved to " << filename << ". Total points: " << history_log_.size());
    
    // 清空防止重复保存
    history_log_.clear();
}