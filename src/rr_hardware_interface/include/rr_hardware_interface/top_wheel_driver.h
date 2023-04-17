#pragma once
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <rr_hardware_interface/motor_driver.h>

class TopWheelDriver : public MotorDriver
{
public:
    TopWheelDriver(
        ros::NodeHandle &nh, 
        hardware_interface::InterfaceManager* if_manager,
        const std::string& joint_name);
    bool ReadState();
    bool SendCommand();

private:
    hardware_interface::JointStateInterface joint_state_if_;
    hardware_interface::PositionJointInterface joint_pos_if_;

    double cmd_;
    
    double pos_state_;
    double vel_state_;
    double eff_state_;
};