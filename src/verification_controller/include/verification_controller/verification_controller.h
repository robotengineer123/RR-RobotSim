#pragma once
#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <nav_msgs/Odometry.h>

#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>
#include <std_msgs/Float64.h>

#include <control_toolbox/pid.h>

#include <tf/transform_datatypes.h>

#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/TwistWithCovariance.h>

namespace verification_controller {
    
  class VerificationController 
  : public controller_interface::MultiInterfaceController<
    hardware_interface::PositionJointInterface,
    hardware_interface::VelocityJointInterface>
  {

  public:
    virtual bool init(hardware_interface::RobotHW* robot_hw,
                    ros::NodeHandle&             root_nh,
                    ros::NodeHandle&             controller_nh);
    

    void starting(const ros::Time& time);
    void update(const ros::Time& time, const ros::Duration& period);
    void stopping(const ros::Time& time);
  
  private:
    void OdomCallback(nav_msgs::OdometryConstPtr cmd);
    void Brake();

    // subscribers
    ros::Subscriber odom_sub_;

    // joints
    hardware_interface::JointHandle r_drive_jh_;
    hardware_interface::JointHandle l_drive_jh_;
    hardware_interface::JointHandle r_steer_jh_;
    hardware_interface::JointHandle l_steer_jh_;

    // pid
    double yaw_desi_ = 0.05;
    double vel_desi_ = 0.2;
    double steer_desi_ = 0.01;
    double steer_desi_angle_ = steer_desi_*180/3.14;
    double currentVel;
    double currentRoll, currentPitch, currentYaw;
    control_toolbox::Pid pidV;
    control_toolbox::Pid pidY;
    ros::Time last_time = ros::Time::now();
    
    // odometry
    realtime_tools::RealtimeBuffer<geometry_msgs::PoseWithCovariance> odom_buf_pose_;
    realtime_tools::RealtimeBuffer<geometry_msgs::TwistWithCovariance> odom_buf_twist_;
    // callback
    geometry_msgs::PoseWithCovariance odom_pose_;
    geometry_msgs::TwistWithCovariance odom_twist_;
    // pid
    geometry_msgs::PoseWithCovariance odom_pose;
    geometry_msgs::TwistWithCovariance odom_twist;
  
  };
  PLUGINLIB_EXPORT_CLASS(verification_controller::VerificationController, controller_interface::ControllerBase)
}