#pragma once
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <rr_hardware_interface/motor_driver.h>
#include <std_msgs/Float64.h>
#include <rr_ethercat_motor/MotorState.h> 
#include <ros/ros.h>

class RopeDriveDriver : public MotorDriver
{
public:
    RopeDriveDriver(
        ros::NodeHandle &nh, 
        hardware_interface::JointStateInterface& state_if,
        hardware_interface::VelocityJointInterface& vel_if,
        const std::string& joint_name,
        const std::string& motor_state_topic,
        const std::string& command_topic);
    bool ReadState();
    bool SendCommand();

private:
    void StateCallback(rr_ethercat_motor::MotorStateConstPtr msg);

    ros::Subscriber state_sub_;
    ros::Publisher cmd_pub_;

    std_msgs::Float64 cmd_msg_;
    
    double pos_state_ = 0;
    double vel_state_ = 0;
    double eff_state_ = 0;
};