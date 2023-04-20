#include <rr_hardware_interface/rope_drive_driver.h>
#include <rr_hardware_interface/rr_hardware_interface.h>
#include <rr_hardware_interface/rope_drive_driver.h>

RopeDriveDriver::RopeDriveDriver(
    ros::NodeHandle &nh, 
    hardware_interface::JointStateInterface& state_if,
    hardware_interface::VelocityJointInterface& vel_if,
    const std::string& joint_name,
    const std::string& motor_state_topic,
    const std::string& command_topic)
{
    state_if.registerHandle(hardware_interface::JointStateHandle(
        joint_name, &pos_state_, &vel_state_, &eff_state_));
    
    vel_if.registerHandle(hardware_interface::JointHandle(
        state_if.getHandle(joint_name), 
        &cmd_msg_.data));

    // We communicate with hardware through an external publisher and subscriber
    // TODO: Communicate directly with hardware instead
    state_sub_ = nh.subscribe(motor_state_topic, 1, &RopeDriveDriver::StateCallback, this);
    cmd_pub_ = nh.advertise<std_msgs::Float64>(command_topic, 1);
}

// State is read in the callback 
bool RopeDriveDriver::ReadState()
{
    return true;
}

bool RopeDriveDriver::SendCommand()
{
    cmd_pub_.publish(std_msgs::Float64(cmd_msg_));
    return true;
}

void RopeDriveDriver::StateCallback(rr_ethercat_motor::MotorStateConstPtr msg)
{
    pos_state_ = msg->position;
    vel_state_ = msg->velocity;
    eff_state_ = msg->effort;
}
