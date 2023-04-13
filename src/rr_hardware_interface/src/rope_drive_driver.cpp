#include <rr_hardware_interface/rope_drive_driver.h>
#include <rr_hardware_interface/rr_hardware_interface.h>


RopeDriveDriver::RopeDriveDriver(
    ros::NodeHandle &nh, 
    hardware_interface::InterfaceManager* if_manager,
    const std::string& joint_name)
{
    joint_state_if_.registerHandle(hardware_interface::JointStateHandle(
        joint_name, &pos_state_, &vel_state_, &eff_state_));
    
    joint_vel_if_.registerHandle(hardware_interface::JointHandle(
        joint_state_if_.getHandle(joint_name), 
        &cmd_));

    if_manager->registerInterface(&joint_state_if_);
    if_manager->registerInterface(&joint_vel_if_);
}

bool RopeDriveDriver::ReadState()
{
    return true;
}

bool RopeDriveDriver::SendCommand()
{
    return true;
}
