#pragma once
#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <nav_msgs/Odometry.h>

#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>
#include <std_msgs/Float64.h>

namespace rr_ackermann_controller {
    
  class RrAckermannController 
  : public controller_interface::MultiInterfaceController<
    hardware_interface::PositionJointInterface,
    hardware_interface::VelocityJointInterface>
  {
    struct InvKinResult
    {
      double l_fw_angle;
      double r_fw_angle;
      double rot_vel;
    };
  public:
    virtual bool init(hardware_interface::RobotHW* robot_hw,
                    ros::NodeHandle&             root_nh,
                    ros::NodeHandle&             controller_nh);
    

    void starting(const ros::Time& time);
    void update(const ros::Time& time, const ros::Duration& period);
    void stopping(const ros::Time& time);
  
  private:
    void CmdVelCallback(geometry_msgs::TwistConstPtr cmd);
    void RadiusCallback(std_msgs::Float64ConstPtr cmd);

    InvKinResult InvKin(double yaw_vel, double lin_vel, double radius);
    void Brake();

    // subscribers
    ros::Subscriber twist_sub_;
    ros::Subscriber radius_sub_;

    // car specifications
    double wheel_base_;
    double track_width_;

    // joints
    hardware_interface::JointHandle r_drive_jh_;
    hardware_interface::JointHandle l_drive_jh_;
    hardware_interface::JointHandle r_steer_jh_;
    hardware_interface::JointHandle l_steer_jh_;

    struct VelCmd
    {
      double lin;
      double ang;
      ros::Time stamp;

      VelCmd() : lin(0.0), ang(0.0), stamp(0.0) {}
    };
    realtime_tools::RealtimeBuffer<VelCmd> vel_buf_;
    realtime_tools::RealtimeBuffer<double> radius_buf_;
    VelCmd vel_cmd; 
  
    double cmd_vel_timeout = 0.5;  //timeout tolerance in seconds
  };
  PLUGINLIB_EXPORT_CLASS(rr_ackermann_controller::RrAckermannController, controller_interface::ControllerBase)
}