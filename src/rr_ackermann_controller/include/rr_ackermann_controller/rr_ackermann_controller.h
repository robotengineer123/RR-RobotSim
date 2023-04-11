#pragma once
#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/imu_sensor_interface.h>
#include <pluginlib/class_list_macros.h>
#include <control_toolbox/pid.h>

#include <nav_msgs/Odometry.h>

#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>
#include <std_msgs/Float64.h>

#include <memory>


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
    InvKinResult InvKin(double yaw_vel, double lin_vel, double radius);
    double ComputeYawCmd(const ros::Duration& period);
    void Brake();
    double Clamp(double value, double hi, double lo) 
    {
      if (value > hi)
        return hi;
      if (value < lo)
        return lo;
      return value;
    }

    void CmdVelCallback(geometry_msgs::TwistConstPtr cmd);
    void RadiusCallback(std_msgs::Float64ConstPtr cmd);
    void DesiredYawCallback(std_msgs::Float64ConstPtr cmd);
    void OdomCallback(nav_msgs::OdometryConstPtr cmd);


    // subscribers
    ros::Subscriber twist_sub_;
    ros::Subscriber radius_sub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber sp_yaw_sub_;


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
    VelCmd vel_cmd_; 
    double odom_yaw_cmd_;
    double radius_cmd_;
    double yaw_cmd_;
    realtime_tools::RealtimeBuffer<VelCmd> vel_buf_;
    realtime_tools::RealtimeBuffer<double> yaw_vel_buf_;
    realtime_tools::RealtimeBuffer<double> radius_buf_;
    realtime_tools::RealtimeBuffer<double> yaw_buf_;

  
    double cmd_vel_timeout = 0.5;  //timeout tolerance in seconds

    control_toolbox::Pid yaw_pid_;

  };
  PLUGINLIB_EXPORT_CLASS(rr_ackermann_controller::RrAckermannController, controller_interface::ControllerBase)
}