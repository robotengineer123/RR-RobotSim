#pragma once
#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <nav_msgs/Odometry.h>

#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>
#include <std_msgs/Float64.h>
#include <stdio.h>

#include <control_toolbox/pid.h>

#include <tf/transform_datatypes.h>


#include <iostream>
#include <string>
#include <iomanip>
#include <sstream>
#include <vector>
#include <fstream>

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
    std::vector<float> time_vector();
    std::vector<float> yaw_vector();
  
  private:
    void OdomCallback(nav_msgs::OdometryConstPtr cmd);
    void Brake();

    // subscribers/publishers
    ros::Subscriber odom_sub_;
    ros::Publisher yaw_pub;
    ros::Publisher velX_pub;
    std_msgs::Float64 currentYawMsg;
    std_msgs::Float64 currentVelMsg;

    // joints
    hardware_interface::JointHandle r_drive_jh_;
    hardware_interface::JointHandle l_drive_jh_;
    hardware_interface::JointHandle r_steer_jh_;
    hardware_interface::JointHandle l_steer_jh_;

    // pid
    double radius = 0.2;

    std::string file = "exp1.csv";
    double yaw_desi_ = 0.0;
    double steer_desi_angle_ = 6.0;
    double vel_desi = 0.05;

    double vel_offset = 0.004;
    double vel_desi_ = vel_desi + vel_offset;
    double steer_desi_ = steer_desi_angle_*3.14/180;
    double currentVel;
    double currentRoll, currentPitch, currentYaw;
    control_toolbox::Pid pidY;
    ros::Time last_time = ros::Time::now();
    
    // odometry
    realtime_tools::RealtimeBuffer<nav_msgs::Odometry> odom_buf_;
    // callback
    nav_msgs::Odometry odom_cmd;
    // pid
    nav_msgs::Odometry odom;

    // Node handle
    ros::NodeHandle nhp_;
  };
  PLUGINLIB_EXPORT_CLASS(verification_controller::VerificationController, controller_interface::ControllerBase)
}