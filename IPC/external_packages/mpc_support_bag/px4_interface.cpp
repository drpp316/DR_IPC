#include "px4_interface.h"

OffboardMode::OffboardMode(TrackingMpc* mpc_ros_application):
            nh_(),
            rate_(20.0),
            mpc_ros_application_(mpc_ros_application),
            is_mpc_start_(false),
            min_thrust_(0.0),
            max_thrust_(0.0)               
{

    bodyrate_thrust_publisher_ = nh_.advertise<mavros_msgs::AttitudeTarget>
        ("/mavros/setpoint_raw/attitude", 1);
    // set a safe defult thrust value
    bodyrate_thrust_.type_mask = bodyrate_thrust_.IGNORE_ATTITUDE;
    bodyrate_thrust_.body_rate.x = 0.0;
    bodyrate_thrust_.body_rate.y = 0.0;
    bodyrate_thrust_.body_rate.z = 0.0;
    bodyrate_thrust_.thrust = 0.1;
 
    state_subscriber_ = nh_.subscribe<mavros_msgs::State>
        ("/mavros/state", 10, &OffboardMode::px4CurrentState, this);
    
}

void OffboardMode::shutdown()
{
    ros::waitForShutdown();
    set_offboard_mode_thread_.join();
    pub_mavros_control_thread_.join();
    
    delete mpc_ros_application_;
}

int OffboardMode::mainLoop()
{
    ros::AsyncSpinner spinner(0);
    spinner.start();

    //获取 MPC 控制量的约束边界（推力范围）
    Eigen::Matrix<real_t, ACADO_NU, 1> max_control, min_control;
    max_control = mpc_ros_application_->getControlUpperBound();
    min_control = mpc_ros_application_->getControlLowerBound();


    min_thrust_ = min_control(ACADO_NU-1);
    max_thrust_ = max_control(ACADO_NU-1);

    ROS_INFO("min thrust: %f, max thrust: %f", min_thrust_, max_thrust_);



    //两个独立线程
    mpc_ros_application_->let_mpc_run_ = true;
    pub_mavros_control_thread_ = std::thread(&OffboardMode::pubMavrosControl, this);

    shutdown();

    return 0;
}


void OffboardMode::pubMavrosControl()
{
    while(ros::ok())
    {
        // ROS_INFO("Publishing bodyrate trust");
        // ROS_INFO_THROTTLE(2, "Publishing bodyrate trust");
        // 1. 获取 MPC 控制器的错误状态
        MpcError px4_error;
        px4_error = mpc_ros_application_->getMpcError();

        //发布错误码
        if(is_mpc_start_)
        {
            switch (px4_error)
            {
            case MPC_OK:
                break;
            
            case DATA_LOST:
                {
                    ROS_ERROR("Data lost! Exit.");// 数据丢失（如传感器数据未更新）
                    ros::shutdown();
                }
                break;
            case CONTACT_DETECTED:
                {
                    ROS_WARN("Contact detected! Exit.");  // 检测到碰撞（如无人机碰到障碍物）
                    ros::shutdown();
                }
                break;
            default:
                break;
            }
        }
        
        // 3. 生成并发布飞控可识别的控制指令
        updateMavrosControl(mpc_ros_application_->getCurrentControl());
        
         // 4. 控制发布频率（每 5 毫秒一次，即 200Hz，满足飞控对控制频率的要求）
        //if(mpc_ros_application_->get_hmpc_flag()>=2) std::this_thread::sleep_for(std::chrono::microseconds(mpc_ros_application_->get_DOBUS()));  // 2000 Hz
        //else  
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

void OffboardMode::updateMavrosControl( 
    const Eigen::Ref<const Eigen::Matrix<real_t, ACADO_NU, 1>> control)
{
    ros::Time current_time = ros::Time::now();
    if(mpc_ros_application_->let_mpc_run_)
    {
        if(is_mpc_start_ == false)
        {
            mpc_ros_application_->mpcStart();// 调用 TrackingMpc 的 mpcStart() 启动 MPC（如启动 MPC 求解线程）
            is_mpc_start_ = true;
        }
    
        // 1. 将 MPC 原始控制量赋值给飞控指令消息（AttitudeTarget 类型）
        bodyrate_thrust_.body_rate.x = control(0); // 角速度 wx（MPC 输出第 0 个元素）
        bodyrate_thrust_.body_rate.y = control(1); // 角速度 wy（MPC 输出第 1 个元素）
        bodyrate_thrust_.body_rate.z = control(2); // 角速度 wz（MPC 输出第 2 个元素）


        //float tmp_delta_T=0;  int flag_dc=mpc_ros_application_->get_hmpc_flag();
        float tmp_delta_T=0;int flag_dc=1;
         // 关键：将 MpcRosApplication* 转换为 TrackingMpc*
        TrackingMpc* tracking_mpc = dynamic_cast<TrackingMpc*>(mpc_ros_application_);
        if (tracking_mpc == nullptr ) {  // 转换失败（如指针未正确关联），直接返回避免崩溃
            ROS_ERROR("Failed to cast mpc_ros_application_ to TrackingMpc*!");
        }
        else if( flag_dc==1){
            //不在此处干扰补偿  delta_T = 0;
        }
        else{  //flag_dc>=2 
        }

        mpc_ros_application_->set_delta_T(tmp_delta_T);
        float T=control(3)+tmp_delta_T;
        bodyrate_thrust_.thrust = controlNormalization(T, min_thrust_, max_thrust_);

        float tmp_T_sat=get_T_sat();  //饱和 补偿值
        mpc_ros_application_->set_T_sat(tmp_T_sat);

        //printf("****  inter: u0- %f u- %f  u_sat- %f \r\n",control(3),T,tmp_T_sat);

        bodyrate_thrust_.header.stamp = current_time;
        // 3. 设置消息时间戳并发布（飞控订阅该消息获取控制指令
        bodyrate_thrust_publisher_.publish(bodyrate_thrust_);

        // printf("Thrust: %f, Bodyrate: %f, %f, %f\n", bodyrate_thrust_.thrust,
        //     bodyrate_thrust_.body_rate.x, 
        //     bodyrate_thrust_.body_rate.y, 
        //     bodyrate_thrust_.body_rate.z);
    }
    else
    {
        if(is_mpc_start_ == true)
        {
            mpc_ros_application_->mpcOff();
            is_mpc_start_ = false;
        }
        // pose_.header.stamp = current_time;
        // Eigen::Vector3f target_position;
        // target_position = mpc_ros_application_->getTargetPosition();
        // pose_.pose.position.x = target_position(0);
        // pose_.pose.position.y = target_position(1);
        // pose_.pose.position.z = target_position(2);
        // pos_publisher_.publish(pose_);
    }
}

void OffboardMode::px4CurrentState(const mavros_msgs::State::ConstPtr& msg)
{
    current_state_ = *msg;
    // ROS_INFO("Recive px4 current state...");
}



float OffboardMode::controlNormalization(float signal, float min, float max)
{
    if(signal>max)  signal=max;
    if(signal<min)  signal=min;
    T_sat=signal;
    return (signal - min)/(max - min);
}

