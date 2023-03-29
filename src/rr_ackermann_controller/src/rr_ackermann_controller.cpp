#include <rr_ackermann_controller/rr_ackermann_controller.h>

using namespace rr_ackermann_controller;

bool RrAckermannController::init(hardware_interface::RobotHW* robot_hw,
                    ros::NodeHandle&             root_nh,
                    ros::NodeHandle&             controller_nh)
{
    typedef hardware_interface::VelocityJointInterface VelIface;
    typedef hardware_interface::PositionJointInterface PosIface;
    
    // get multiple types of hardware_interface
    VelIface *vel_joint_if = robot_hw->get<VelIface>(); // vel for wheels
    PosIface *pos_joint_if = robot_hw->get<PosIface>(); // pos for steers

    //Vehicle parameters
    controller_nh.getParam("wheel_base", wheel_base_);
    controller_nh.getParam("track_width", track_width_);


    std::string r_drive_name_, l_drive_name_, r_steer_name_, l_steer_name_, radius_topic;
    controller_nh.getParam("right_drive_joint", r_drive_name_);
    controller_nh.getParam("left_drive_joint", l_drive_name_);
    controller_nh.getParam("right_steer_joint", r_steer_name_);
    controller_nh.getParam("left_steer_joint", l_steer_name_);
    controller_nh.getParam("radius_topic", radius_topic);

    r_drive_jh_ = vel_joint_if->getHandle(r_drive_name_);
    l_drive_jh_ = vel_joint_if->getHandle(l_drive_name_);
    r_steer_jh_ = pos_joint_if->getHandle(r_steer_name_);
    l_steer_jh_ = pos_joint_if->getHandle(l_steer_name_);

    twist_sub_ = controller_nh.subscribe("cmd_vel", 1, &RrAckermannController::CmdVelCallback, this);
    radius_sub_ = controller_nh.subscribe(radius_topic, 1, &RrAckermannController::RadiusCallback, this);



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

void RrAckermannController::starting(const ros::Time& time)
{
    Brake();
    vel_cmd.stamp = ros::Time::now();
    vel_buf_.initRT(vel_cmd);
    radius_buf_.initRT(0.3);
}

void RrAckermannController::update(const ros::Time& time, const ros::Duration& period)
{
    VelCmd vel = *(vel_buf_.readFromRT());

    const double dt = (time - vel.stamp).toSec();
    if (dt > cmd_vel_timeout)
    {
        vel.ang = 0;
        vel.lin = 0;
    } 

    double radius = *(radius_buf_.readFromRT());
    InvKinResult ikr = InvKin(vel.ang, vel.lin, radius);
    
    r_drive_jh_.setCommand(ikr.rot_vel);
    l_drive_jh_.setCommand(ikr.rot_vel);

    r_steer_jh_.setCommand(ikr.r_fw_angle);
    l_steer_jh_.setCommand(ikr.l_fw_angle);
}
void RrAckermannController::stopping(const ros::Time& time)
{
    Brake();
}

void RrAckermannController::CmdVelCallback(geometry_msgs::TwistConstPtr cmd)
{
    vel_cmd.lin = cmd->linear.x;
    vel_cmd.ang = cmd->angular.z;
    vel_cmd.stamp = ros::Time::now();
    vel_buf_.writeFromNonRT(vel_cmd);
}

void RrAckermannController::RadiusCallback(std_msgs::Float64ConstPtr cmd)
{
    double radius = cmd->data;
    if (radius == 0)
        radius = 0.3;
    radius_buf_.writeFromNonRT(radius);
}

RrAckermannController::InvKinResult RrAckermannController::InvKin(double yaw_vel, double lin_vel, double radius)
{
    typedef RrAckermannController::InvKinResult IKR;
    if (lin_vel == 0)
    return IKR{0.0, 0.0, 0.0};
    
    double steer = std::atan(wheel_base_*yaw_vel/lin_vel);
    double steer_i = 0;
    double steer_o = 0;
    double rot_vel = lin_vel/radius;
    IKR ikr{0, 0, rot_vel};

    if (not steer==0)
    {
        double r = wheel_base_/std::tan(steer);

        double steer_i = std::atan(wheel_base_/(r - track_width_/2.0));
        double steer_o = std::atan(wheel_base_/(r + track_width_/2.0));

        if (r  <= 0)
            ikr = IKR{steer_i, steer_o, rot_vel};
        else
            ikr = IKR{steer_o, steer_i, rot_vel};
    }
    return ikr;
}