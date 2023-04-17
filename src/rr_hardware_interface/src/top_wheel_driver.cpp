#include <rr_hardware_interface/top_wheel_driver.h>

TopWheelDriver::TopWheelDriver(
    ros::NodeHandle &nh, 
    hardware_interface::InterfaceManager* if_manager, 
    const std::string &joint_name)
{
    joint_state_if_.registerHandle(hardware_interface::JointStateHandle(
        joint_name, &pos_state_, &vel_state_, &eff_state_));
    
    joint_pos_if_.registerHandle(hardware_interface::JointHandle(
        joint_state_if_.getHandle(joint_name), 
        &cmd_));

    if_manager->registerInterface(&joint_state_if_);
    if_manager->registerInterface(&joint_pos_if_);
}

bool TopWheelDriver::ReadState()
{
    return true;
}

bool TopWheelDriver::SendCommand()
{
    return true;
}
