#pragma once
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <rr_hardware_interface/motor_driver.h>
#include <std_msgs/Float64.h>
#include <ros/ros.h>
#include <rr_wheels/FloatTrigger.h>
#include <rr_wheels/WheelStatus.h>

class TopWheelDriver : public MotorDriver
{
public:
    TopWheelDriver(
        ros::NodeHandle &nh, 
        hardware_interface::JointStateInterface& state_if,
        hardware_interface::PositionJointInterface& pos_if,
        const std::string& joint_name,
        const std::string& state_topic,
        const std::string& cmd_service);
    bool ReadState();
    bool SendCommand();

private:
    void StateCallback(rr_wheels::WheelStatusConstPtr msg);

    ros::Subscriber state_sub_;
    ros::ServiceClient cmd_srv_client_;
    rr_wheels::FloatTrigger cmd_request_;
    
    double pos_state_ = 0;
    double vel_state_ = 0; //not available from hw
    double eff_state_ = 0; //not available from hw
};