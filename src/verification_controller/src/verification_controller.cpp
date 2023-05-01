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

    //if (!nhp_.hasParam("pidY/p"))
    //    nhp_.setParam("pidY/p", 100.0);
    //if (!nhp_.hasParam("pidY/i"))
    //    nhp_.setParam("pidY/i", 10.0);
    //if (!nhp_.hasParam("pidY/d"))
    //    nhp_.setParam("pidY/d", 20.0);
    //if (!nhp_.hasParam("pidY/i_clamp_min"))
    //    nhp_.setParam("pidY/i_clamp_min", -1.5);
    //if (!nhp_.hasParam("pidY/i_clamp_max"))
    //    nhp_.setParam("pidY/i_clamp_max", 1.5);

    if (!nhp_.hasParam("pidY/p"))
        nhp_.setParam("pidY/p", 1.0);
    if (!nhp_.hasParam("pidY/i"))
        nhp_.setParam("pidY/i", 0.5);
    if (!nhp_.hasParam("pidY/d"))
        nhp_.setParam("pidY/d", 0.5);
    if (!nhp_.hasParam("pidY/i_clamp_min"))
        nhp_.setParam("pidY/i_clamp_min", -0.3);
    if (!nhp_.hasParam("pidY/i_clamp_max"))
        nhp_.setParam("pidY/i_clamp_max", 0.3);

    nhp_.setParam("publish_state", true);

    pidY.init(ros::NodeHandle(nhp_, "pidY"), false);

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

voidVerific ationController::update(const ros::Time& time, const ros::Duration& period)
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

    // After yaw_start seconds use yaw from experimental results
    if ((time.toSec() > yaw_start) && (time.toSec() < Time[Time.size() - 1]))
    {
        float yaw_time = time.toSec() - yaw_start;
        long idx_closest = search_closest(Time, yaw_time);
        yaw_desi_ = Yaw[idx_closest];
    }

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

// Search for closest value in vector and output its index
long VerificationController::search_closest(const std::vector<float>& sorted_array, float x)
{
    auto iter_geq = std::lower_bound(
        sorted_array.begin(), 
        sorted_array.end(), 
        x
    );

    if (iter_geq == sorted_array.begin()) {
        return 0;
    }

    float a = *(iter_geq - 1);
    float b = *(iter_geq);

    if (std::fabs(x - a) < std::fabs(x - b)) {
        return iter_geq - sorted_array.begin() - 1;
    }

    return iter_geq - sorted_array.begin();

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