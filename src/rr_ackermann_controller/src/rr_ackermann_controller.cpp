#include <rr_ackermann_controller/rr_ackermann_controller.h>
#include <tf/transform_datatypes.h>

using namespace rr_ackermann_controller;

bool RrAckermannController::init(hardware_interface::RobotHW *robot_hw,
                                 ros::NodeHandle &root_nh,
                                 ros::NodeHandle &controller_nh)
{
    typedef hardware_interface::VelocityJointInterface VelIface;
    typedef hardware_interface::PositionJointInterface PosIface;

    // get multiple types of hardware_interface
    VelIface *vel_joint_if = robot_hw->get<VelIface>(); // vel for wheels
    PosIface *pos_joint_if = robot_hw->get<PosIface>(); // pos for steers

    // Vehicle parameters
    controller_nh.getParam("wheel_base", wheel_base_);
    controller_nh.getParam("track_width", track_width_);

    // joints
    std::string r_drive_name_, l_drive_name_, r_steer_name_, l_steer_name_, radius_topic, odom_topic;
    controller_nh.getParam("right_drive_joint", r_drive_name_);
    controller_nh.getParam("left_drive_joint", l_drive_name_);
    controller_nh.getParam("right_steer_joint", r_steer_name_);
    controller_nh.getParam("left_steer_joint", l_steer_name_);

    // topics
    controller_nh.getParam("radius_topic", radius_topic);
    controller_nh.getParam("odom_topic", odom_topic);

    // Get joint handles
    r_drive_jh_ = vel_joint_if->getHandle(r_drive_name_);
    l_drive_jh_ = vel_joint_if->getHandle(l_drive_name_);
    r_steer_jh_ = pos_joint_if->getHandle(r_steer_name_);
    l_steer_jh_ = pos_joint_if->getHandle(l_steer_name_);

    // Init subscribers
    twist_sub_ = controller_nh.subscribe("cmd_vel", 1, &RrAckermannController::CmdVelCallback, this);
    radius_sub_ = controller_nh.subscribe(radius_topic, 1, &RrAckermannController::RadiusCallback, this);
    odom_sub_ = root_nh.subscribe(odom_topic, 1, &RrAckermannController::OdomCallback, this);

    yaw_pid_.init(controller_nh);

    controller_nh.param("has_velocity_limits", speed_limiter_.has_velocity_limits, speed_limiter_.has_velocity_limits);
    controller_nh.param("has_acceleration_limits", speed_limiter_.has_acceleration_limits, speed_limiter_.has_acceleration_limits);
    controller_nh.param("has_jerk_limits", speed_limiter_.has_jerk_limits, speed_limiter_.has_jerk_limits);
    controller_nh.param("max_velocity", speed_limiter_.max_velocity, speed_limiter_.max_velocity);
    controller_nh.param("min_velocity", speed_limiter_.min_velocity, speed_limiter_.max_velocity);
    controller_nh.param("max_acceleration", speed_limiter_.max_acceleration, speed_limiter_.max_acceleration);
    controller_nh.param("min_acceleration", speed_limiter_.min_acceleration, speed_limiter_.max_acceleration);
    controller_nh.param("max_jerk", speed_limiter_.max_jerk, speed_limiter_.max_jerk);
    controller_nh.param("min_jerk", speed_limiter_.min_jerk, speed_limiter_.max_jerk);

    controller_nh.param("has_steer_limit", steer_limiter_.has_steer_limits, steer_limiter_.has_steer_limits);
    controller_nh.param("max_steer", steer_limiter_.max_steer, steer_limiter_.max_steer);
    controller_nh.param("min_steer", steer_limiter_.min_steer, steer_limiter_.min_steer);

    return true;
}

void RrAckermannController::Brake()
{
    const double steer_pos = 0.0;
    const double wheel_vel = 0.0;

    r_drive_jh_.setCommand(wheel_vel);
    l_drive_jh_.setCommand(wheel_vel);
    r_steer_jh_.setCommand(steer_pos);
    l_steer_jh_.setCommand(steer_pos);
}

void RrAckermannController::starting(const ros::Time &time)
{
    Brake();
    vel_cmd_.stamp = ros::Time::now();
    vel_buf_.initRT(vel_cmd_);
    radius_buf_.initRT(0.3);
    yaw_buf_.initRT(0);
    yaw_vel_buf_.initRT(0);
}

void RrAckermannController::update(const ros::Time &time, const ros::Duration &period)
{
    VelCmd vel = *(vel_buf_.readFromRT());
    double radius = *(radius_buf_.readFromRT());

    const double dt = (time - vel.stamp).toSec();
    if (dt > cmd_vel_timeout)
    {
        vel.ang = 0;
        vel.lin = 0;
    }

    // limit linear speed
    speed_limiter_.limit(vel.lin, last_v0_, last_v1_, period.toSec());
    last_v1_ = last_v0_;
    last_v0_ = vel.lin;

    InvKinResult ikr = InvKinWithSteerLimit(vel.ang, vel.lin, radius);

    // limit steer angle
    ikr.l_fw_angle;
    ikr.r_fw_angle;

    double yaw_cmd = ComputeYawCmd(period);

    r_drive_jh_.setCommand(ikr.rot_vel + yaw_cmd);
    l_drive_jh_.setCommand(ikr.rot_vel - yaw_cmd);

    r_steer_jh_.setCommand(ikr.r_fw_angle);
    l_steer_jh_.setCommand(ikr.l_fw_angle);
}

RrAckermannController::InvKinResult RrAckermannController::InvKinWithSteerLimit(double yaw_vel, double lin_vel, double radius)
{
    typedef RrAckermannController::InvKinResult IKR;
    if (lin_vel == 0)
        return IKR{0.0, 0.0, 0.0};

    double steer = std::atan(wheel_base_ * yaw_vel / lin_vel);
    steer_limiter_.LimitSteer(steer);
    double steer_i = 0;
    double steer_o = 0;
    double rot_vel = lin_vel / radius;
    IKR ikr{0, 0, rot_vel};

    if (not steer == 0)
    {
        double r = wheel_base_ / std::tan(steer);

        double steer_i = std::atan(wheel_base_ / (r - track_width_ / 2.0));
        double steer_o = std::atan(wheel_base_ / (r + track_width_ / 2.0));

        if (r <= 0)
            ikr = IKR{steer_i, steer_o, rot_vel};
        else
            ikr = IKR{steer_o, steer_i, rot_vel};
    }
    return ikr;
}

double rr_ackermann_controller::RrAckermannController::ComputeYawCmd(const ros::Duration &period)
{
    double yaw_sp = (vel_buf_.readFromRT())->ang;
    double yaw_ref = *(yaw_vel_buf_.readFromRT());

    double yaw_cmd = yaw_pid_.computeCommand(yaw_sp - yaw_ref, period);
    return yaw_cmd;
}

void RrAckermannController::stopping(const ros::Time &time)
{
    Brake();
}

void RrAckermannController::CmdVelCallback(geometry_msgs::TwistConstPtr cmd)
{
    vel_cmd_.lin = cmd->linear.x;
    vel_cmd_.ang = cmd->angular.z;
    vel_cmd_.stamp = ros::Time::now();
    vel_buf_.writeFromNonRT(vel_cmd_);
}

void RrAckermannController::RadiusCallback(std_msgs::Float64ConstPtr cmd)
{
    double radius = cmd->data;
    if (radius == 0)
        radius = 0.3;
    radius_buf_.writeFromNonRT(radius);
}

void rr_ackermann_controller::RrAckermannController::DesiredYawCallback(std_msgs::Float64ConstPtr cmd)
{
    yaw_cmd_ = cmd->data;
    yaw_buf_.writeFromNonRT(yaw_cmd_);
}

void rr_ackermann_controller::RrAckermannController::OdomCallback(nav_msgs::OdometryConstPtr cmd)
{
    geometry_msgs::PoseWithCovariance pose_msg = cmd->pose;
    auto twist = cmd->twist;
    yaw_vel_buf_.writeFromNonRT(twist.twist.angular.z);
}
