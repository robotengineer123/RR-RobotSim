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

    odom_sub_ = controller_nh.subscribe("/odom", 1, &VerificationController::OdomCallback, this);
    yaw_pub = controller_nh.advertise<std_msgs::Float64>("/yaw", 50);
    velX_pub = controller_nh.advertise<std_msgs::Float64>("/velX", 50);

    pidV.initPid(40.0, 25.0, 5.5, 2.5, -2.5);
    pidY.initPid(4.5, 1.5, 1.5, 0.3, -0.3);

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
    double velocity_r = pidV.updatePid(currentVel - vel_desi_, time - last_time);
    double velocity_l = pidV.updatePid(currentVel - vel_desi_, time - last_time);   
    double yaw = pidY.updatePid(currentYaw - yaw_desi_, time - last_time);
    
    double pid_combined_r = velocity_r + yaw;
    double pid_combined_l = velocity_r - yaw;
    
    last_time = time;

    // Publish scanner message
    currentYawMsg.data = currentYaw;
    yaw_pub.publish(currentYawMsg);
    currentVelMsg.data = currentVel;
    velX_pub.publish(currentVelMsg);

    // Set commands
    r_drive_jh_.setCommand(pid_combined_r);
    l_drive_jh_.setCommand(pid_combined_l);

    r_steer_jh_.setCommand(steer_desi_angle_);
    l_steer_jh_.setCommand(steer_desi_angle_);
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