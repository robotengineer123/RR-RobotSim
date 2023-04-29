#include <verification_controller/verification_controller.h>

using namespace verification_controller;

bool VerificationController::init(hardware_interface::RobotHW* robot_hw,
                    ros::NodeHandle&             root_nh,
                    ros::NodeHandle&             controller_nh)
{
    typedef hardware_interface::VelocityJointInterface VelIface;
    typedef hardware_interface::PositionJointInterface PosIface;
    
    // get multiple types of hardware_interface
    VelIface *vel_joint_if = robot_hw->get<VelIface>(); // vel for wheels
    PosIface *pos_joint_if = robot_hw->get<PosIface>(); // pos for steers

    std::string r_drive_name_, l_drive_name_, r_steer_name_, l_steer_name_;
    controller_nh.getParam("right_drive_joint", r_drive_name_);
    controller_nh.getParam("left_drive_joint", l_drive_name_);
    controller_nh.getParam("right_steer_joint", r_steer_name_);
    controller_nh.getParam("left_steer_joint", l_steer_name_);

    r_drive_jh_ = vel_joint_if->getHandle(r_drive_name_);
    l_drive_jh_ = vel_joint_if->getHandle(l_drive_name_);
    r_steer_jh_ = pos_joint_if->getHandle(r_steer_name_);
    l_steer_jh_ = pos_joint_if->getHandle(l_steer_name_);

    odom_sub_ = controller_nh.subscribe("/rr_robot/odom", 1, &VerificationController::OdomCallback, this);
    yaw_pub = controller_nh.advertise<std_msgs::Float64>("/yaw", 50);
    velX_pub = controller_nh.advertise<std_msgs::Float64>("/velX", 50);

    nhp_ = ros::NodeHandle("~");

    if (!nhp_.hasParam("pidY/p"))
        nhp_.setParam("pidY/p", 100.0);
    if (!nhp_.hasParam("pidY/i"))
        nhp_.setParam("pidY/i", 10.0);
    if (!nhp_.hasParam("pidY/d"))
        nhp_.setParam("pidY/d", 20.0);
    if (!nhp_.hasParam("pidY/i_clamp_min"))
        nhp_.setParam("pidY/i_clamp_min", -1.5);
    if (!nhp_.hasParam("pidY/i_clamp_max"))
        nhp_.setParam("pidY/i_clamp_max", 1.5);

    nhp_.setParam("publish_state", true);

    pidY.init(ros::NodeHandle(nhp_, "pidY"), false);

    std::vector<float> Time = time_vector();
    std::vector<float> Yaw = yaw_vector();

    return true;
}

void VerificationController::Brake()
{
    const double steer_pos = 0.0;
    const double wheel_vel = 0.0;

    r_drive_jh_.setCommand(wheel_vel);
    l_drive_jh_.setCommand(wheel_vel);
    r_steer_jh_.setCommand(steer_pos);
    l_steer_jh_.setCommand(steer_pos);
}

void VerificationController::starting(const ros::Time& time)
{
    Brake();
    odom_buf_.initRT(odom_cmd);
}

void VerificationController::update(const ros::Time& time, const ros::Duration& period)
{
    odom = *(odom_buf_.readFromRT());

    // get yaw and velocity
    tf::Quaternion q(
        odom.pose.pose.orientation.x,
        odom.pose.pose.orientation.y,
        odom.pose.pose.orientation.z,
        odom.pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    m.getRPY(currentRoll, currentPitch, currentYaw);
    
    currentVel = odom.twist.twist.linear.x;

    // PID 
    double yaw = pidY.updatePid(currentYaw - yaw_desi_, time - last_time);
    
    double velocity_r = vel_desi_/radius;
    double velocity_l = vel_desi_/radius;

    double pid_combined_r = velocity_r + yaw/2;
    double pid_combined_l = velocity_l - yaw/2;
    
    last_time = time;

    // Publish scanner message
    currentYawMsg.data = currentYaw;
    yaw_pub.publish(currentYawMsg);
    currentVelMsg.data = currentVel;
    velX_pub.publish(currentVelMsg);

    // Set commands
    r_drive_jh_.setCommand(pid_combined_r);
    l_drive_jh_.setCommand(pid_combined_l);

    r_steer_jh_.setCommand(steer_desi_);
    l_steer_jh_.setCommand(steer_desi_);
}

std::vector<float> VerificationController::time_vector()
{
    // define variables
    std::string Time_str, Yaw_str;//variables from file are here
    std::vector<float> Time;
    std::vector<float> Yaw;

    //number of lines
    int i = 0;

    std::ifstream coeff {file}; //opening the file.
    if (coeff.is_open()) //if the file is open
    {
      //ignore first line
      std::string line;
      getline(coeff, line);

      while (!coeff.eof()) //while the end of file is NOT reached
      {
        //I have 4 sets {alpha, CD, CL, CY} so use 4 getlines
        getline(coeff, Time_str, ',');
        Time.push_back(stof(Time_str));
        getline(coeff, Yaw_str, '\n');
        Yaw.push_back(stof(Yaw_str));
        
        i += 1; //increment number of lines
      }
      coeff.close(); //closing the file
      std::cout << "Number of entries: " << i-1 << std::endl;
    }
    else std::cout << "Unable to open file"; //if the file is not open output

    return Time;
}

std::vector<float> VerificationController::yaw_vector()
{
    // define variables
    std::string Time_str, Yaw_str;//variables from file are here
    std::vector<float> Time;
    std::vector<float> Yaw;

    //number of lines
    int i = 0;

    std::ifstream coeff {file}; //opening the file.
    if (coeff.is_open()) //if the file is open
    {
      //ignore first line
      std::string line;
      getline(coeff, line);

      while (!coeff.eof()) //while the end of file is NOT reached
      {
        //I have 4 sets {alpha, CD, CL, CY} so use 4 getlines
        getline(coeff, Time_str, ',');
        Time.push_back(stof(Time_str));
        getline(coeff, Yaw_str, '\n');
        Yaw.push_back(stof(Yaw_str));
        
        i += 1; //increment number of lines
      }
      coeff.close(); //closing the file
      std::cout << "Number of entries: " << i-1 << std::endl;
    }
    else std::cout << "Unable to open file"; //if the file is not open output

    return Yaw;
}

void VerificationController::stopping(const ros::Time& time)
{
    Brake();
}

void VerificationController::OdomCallback(nav_msgs::OdometryConstPtr cmd)
{
    odom_cmd.pose.pose = cmd->pose.pose;
    odom_cmd.twist.twist = cmd->twist.twist;
    odom_buf_.writeFromNonRT(odom_cmd);
}