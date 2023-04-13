#pragma once
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include "motor_driver.h"

class RopeDriveDriver : public MotorDriver
{
public:
    RopeDriveDriver(
        ros::NodeHandle &nh, 
        hardware_interface::InterfaceManager* if_manager,
        const std::string& joint_name);
    bool ReadState();
    bool SendCommand();

private:
    hardware_interface::JointStateInterface joint_state_if_;
    hardware_interface::VelocityJointInterface joint_vel_if_;

    double cmd_;
    
    double pos_state_;
    double vel_state_;
    double eff_state_;
};