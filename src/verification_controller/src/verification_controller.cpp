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

    odom_sub_ = controller_nh.subscribe("odom", 1, &VerificationController::OdomCallback, this);

    pidV.initPid(6.0, 1.0, 2.0, 0.3, -0.3);
    pidY.initPid(6.0, 1.0, 2.0, 0.3, -0.3);

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
    odom_buf_pose_.initRT(odom_pose);
    odom_buf_twist_.initRT(odom_twist);
}

void VerificationController::update(const ros::Time& time, const ros::Duration& period)
{
    odom_pose = *(odom_buf_pose_.readFromRT());
    odom_twist = *(odom_buf_twist_.readFromRT());

    // get yaw and velocity
    tf::Quaternion q(
        odom_pose.pose.orientation.x,
        odom_pose.pose.orientation.y,
        odom_pose.pose.orientation.z,
        odom_pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    m.getRPY(currentRoll, currentPitch, currentYaw);
    
    currentVel = odom_twist.twist.linear.x;

    // PID
    double velocity_r = pidV.updatePid(currentVel - vel_desi_, time - last_time);
    double velocity_l = pidV.updatePid(currentVel - vel_desi_, time - last_time);   
    double yaw = pidY.updatePid(currentYaw - yaw_desi_, time - last_time);
    
    double pid_combined_r = velocity_r + yaw;
    double pid_combined_l = velocity_r - yaw;
    
    last_time = time;

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
    odom_pose = cmd->pose;
    odom_twist = cmd->twist;
    odom_buf_pose_.writeFromNonRT(odom_pose);
    odom_buf_twist_.writeFromNonRT(odom_twist);
}