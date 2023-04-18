#include <rr_ackermann_controller/rr_ackermann_controller.h>
#include <tf/transform_datatypes.h>
#include <boost/assign.hpp>

using namespace rr_ackermann_controller;

bool RrAckermannController::init(hardware_interface::RobotHW *robot_hw,
                                 ros::NodeHandle &root_nh,
                                 ros::NodeHandle &controller_nh)
{
    typedef hardware_interface::VelocityJointInterface VelIface;
    typedef hardware_interface::PositionJointInterface PosIface;
    typedef hardware_interface::JointStateInterface StateIface;

    // Get node name
    const std::string complete_ns = controller_nh.getNamespace();
    std::size_t id = complete_ns.find_last_of("/");
    name_ = complete_ns.substr(id + 1);

    // get multiple types of hardware_interface
    VelIface *vel_joint_if = robot_hw->get<VelIface>(); // vel for wheels
    PosIface *pos_joint_if = robot_hw->get<PosIface>(); // pos for steers
    StateIface *state_joint_if = robot_hw->get<StateIface>();

    // Vehicle parameters
    controller_nh.getParam("wheel_base", wheel_base_);
    controller_nh.getParam("track_width", track_width_);

    // joints
    std::string r_drive_name, l_drive_name, r_steer_name, l_steer_name, bot_encoder_name, radius_topic, odom_topic;
    controller_nh.getParam("right_drive_joint", r_drive_name);
    controller_nh.getParam("left_drive_joint", l_drive_name);
    controller_nh.getParam("right_steer_joint", r_steer_name);
    controller_nh.getParam("left_steer_joint", l_steer_name);
    controller_nh.getParam("bot_wheel_encoder_joint", bot_encoder_name);


    // topics
    controller_nh.getParam("radius_topic", radius_topic);
    controller_nh.getParam("odom_topic", odom_topic);

    // Get joint handles
    r_drive_jh_ = vel_joint_if->getHandle(r_drive_name);
    l_drive_jh_ = vel_joint_if->getHandle(l_drive_name);
    r_steer_jh_ = pos_joint_if->getHandle(r_steer_name);
    l_steer_jh_ = pos_joint_if->getHandle(l_steer_name);
    bot_encoder_jh_ = state_joint_if->getHandle(bot_encoder_name);

    // Init subscribers
    twist_sub_ = controller_nh.subscribe("cmd_vel", 1, &RrAckermannController::CmdVelCallback, this);
    radius_sub_ = controller_nh.subscribe(radius_topic, 1, &RrAckermannController::RadiusCallback, this);
    odom_sub_ = root_nh.subscribe(odom_topic, 1, &RrAckermannController::OdomCallback, this);

    yaw_pid_.init(controller_nh);

    // Limits
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

    InitOdom(controller_nh);
    setOdomPubFields(root_nh, controller_nh);
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
    // COMPUTE AND PUBLISH ODOMETRY

    double wheel_pos = bot_encoder_jh_.getPosition();
    double steer_pos = (r_steer_jh_.getPosition() + l_steer_jh_.getPosition()) / 2;

    if (std::isnan(wheel_pos) || std::isnan(steer_pos))
        return;

    // Estimate linear and angular velocity using joint information
    steer_pos = steer_pos * steer_pos_multiplier_;
    odometry_.update(wheel_pos, steer_pos, time);

    // Publish odometry message
    if (last_state_publish_time_ + publish_period_ < time)
    {
        last_state_publish_time_ += publish_period_;
        // Compute and store orientation info
        const geometry_msgs::Quaternion orientation(
            tf::createQuaternionMsgFromYaw(odometry_.getHeading()));

        // Populate odom message and publish
        if (odom_pub_->trylock())
        {
            odom_pub_->msg_.header.stamp = time;
            odom_pub_->msg_.pose.pose.position.x = odometry_.getX();
            odom_pub_->msg_.pose.pose.position.y = odometry_.getY();
            odom_pub_->msg_.pose.pose.orientation = orientation;
            odom_pub_->msg_.twist.twist.linear.x = odometry_.getLinear();
            odom_pub_->msg_.twist.twist.angular.z = odometry_.getAngular();
            odom_pub_->unlockAndPublish();
        }

        // Publish tf /odom frame
        if (enable_odom_tf_ && tf_odom_pub_->trylock())
        {
            geometry_msgs::TransformStamped &odom_frame = tf_odom_pub_->msg_.transforms[0];
            odom_frame.header.stamp = time;
            odom_frame.transform.translation.x = odometry_.getX();
            odom_frame.transform.translation.y = odometry_.getY();
            odom_frame.transform.rotation = orientation;
            tf_odom_pub_->unlockAndPublish();
        }
    }

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

void rr_ackermann_controller::RrAckermannController::InitOdom(ros::NodeHandle controller_nh)
{
    controller_nh.param("enable_odom_tf", enable_odom_tf_, enable_odom_tf_);
    ROS_INFO_STREAM_NAMED(name_, "Publishing to tf is " << (enable_odom_tf_ ? "enabled" : "disabled"));

    controller_nh.param("base_frame_id", base_frame_id_, base_frame_id_);
    ROS_INFO_STREAM_NAMED(name_, "Base frame_id set to " << base_frame_id_);

    controller_nh.param("odom_frame_id", odom_frame_id_, odom_frame_id_);
    ROS_INFO_STREAM_NAMED(name_, "Odometry frame_id set to " << odom_frame_id_);

    double publish_rate;
    controller_nh.param("publish_rate", publish_rate, 50.0);
    ROS_INFO_STREAM_NAMED(name_, "Controller state will be published at "
                                     << publish_rate << "Hz.");
    publish_period_ = ros::Duration(1.0 / publish_rate);

    double wheel_base_multiplier, wheel_radius_multiplier;
    controller_nh.param("wheel_base_multiplier", wheel_base_multiplier, 1.);
    ROS_INFO_STREAM_NAMED(name_, "Wheel separation height will be multiplied by "
                                     << wheel_base_multiplier << ".");

    controller_nh.param("wheel_radius_multiplier", wheel_radius_multiplier, 1.);
    ROS_INFO_STREAM_NAMED(name_, "Wheel radius will be multiplied by "
                                     << wheel_radius_multiplier << ".");

    controller_nh.param("steer_pos_multiplier", steer_pos_multiplier_, 1.);
    ROS_INFO_STREAM_NAMED(name_, "Steer pos will be multiplied by "
                                     << steer_pos_multiplier_ << ".");

    int velocity_rolling_window_size = 10;
    controller_nh.param("velocity_rolling_window_size", velocity_rolling_window_size, velocity_rolling_window_size);
    ROS_INFO_STREAM_NAMED(name_, "Velocity rolling window size of "
                                     << velocity_rolling_window_size << ".");

    double wheel_radius;
    controller_nh.getParam("wheel_radius", wheel_radius);

    const double ws_h = wheel_base_multiplier * wheel_base_;
    const double wr = wheel_radius_multiplier * wheel_radius;
    odometry_.setVelocityRollingWindowSize(velocity_rolling_window_size);
    odometry_.setWheelParams(ws_h, wr);
}

void RrAckermannController::setOdomPubFields(ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh)
{
    // Get and check params for covariances
    XmlRpc::XmlRpcValue pose_cov_list;
    controller_nh.getParam("pose_covariance_diagonal", pose_cov_list);
    ROS_ASSERT(pose_cov_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
    ROS_ASSERT(pose_cov_list.size() == 6);
    for (int i = 0; i < pose_cov_list.size(); ++i)
        ROS_ASSERT(pose_cov_list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);

    XmlRpc::XmlRpcValue twist_cov_list;
    controller_nh.getParam("twist_covariance_diagonal", twist_cov_list);
    ROS_ASSERT(twist_cov_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
    ROS_ASSERT(twist_cov_list.size() == 6);
    for (int i = 0; i < twist_cov_list.size(); ++i)
        ROS_ASSERT(twist_cov_list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);

    // Setup odometry realtime publisher + odom message constant fields
    odom_pub_.reset(new realtime_tools::RealtimePublisher<nav_msgs::Odometry>(controller_nh, "odom", 100));
    odom_pub_->msg_.header.frame_id = odom_frame_id_;
    odom_pub_->msg_.child_frame_id = base_frame_id_;
    odom_pub_->msg_.pose.pose.position.z = 0;
    odom_pub_->msg_.pose.covariance = boost::assign::list_of
        (static_cast<double>(pose_cov_list[0]))(0)(0)(0)(0)(0)
        (0)(static_cast<double>(pose_cov_list[1]))(0)(0)(0)(0)
        (0)(0)(static_cast<double>(pose_cov_list[2]))(0)(0)(0)
        (0)(0)(0)(static_cast<double>(pose_cov_list[3]))(0)(0)
        (0)(0)(0)(0)(static_cast<double>(pose_cov_list[4]))(0)
        (0)(0)(0)(0)(0)(static_cast<double>(pose_cov_list[5]));
    odom_pub_->msg_.twist.twist.linear.y = 0;
    odom_pub_->msg_.twist.twist.linear.z = 0;
    odom_pub_->msg_.twist.twist.angular.x = 0;
    odom_pub_->msg_.twist.twist.angular.y = 0;
    odom_pub_->msg_.twist.covariance = boost::assign::list_of
        (static_cast<double>(twist_cov_list[0]))(0)(0)(0)(0)(0)
        (0)(static_cast<double>(twist_cov_list[1]))(0)(0)(0)(0)
        (0)(0)(static_cast<double>(twist_cov_list[2]))(0)(0)(0)
        (0)(0)(0)(static_cast<double>(twist_cov_list[3]))(0)(0)
        (0)(0)(0)(0)(static_cast<double>(twist_cov_list[4]))(0)
        (0)(0)(0)(0)(0)(static_cast<double>(twist_cov_list[5]));
    tf_odom_pub_.reset(new realtime_tools::RealtimePublisher<tf::tfMessage>(root_nh, "/tf", 100));
    tf_odom_pub_->msg_.transforms.resize(1);
    tf_odom_pub_->msg_.transforms[0].transform.translation.z = 0.0;
    tf_odom_pub_->msg_.transforms[0].child_frame_id = base_frame_id_;
    tf_odom_pub_->msg_.transforms[0].header.frame_id = odom_frame_id_;
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
