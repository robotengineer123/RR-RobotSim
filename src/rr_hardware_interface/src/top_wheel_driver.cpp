#include <rr_hardware_interface/top_wheel_driver.h>
#include <ros/callback_queue.h>

TopWheelDriver::TopWheelDriver(
        ros::NodeHandle &nh, 
        hardware_interface::JointStateInterface& state_if,
        hardware_interface::PositionJointInterface& pos_if,
        const std::string& joint_name,
        const std::string& state_topic,
        const std::string& cmd_service)
{
    state_if.registerHandle(hardware_interface::JointStateHandle(
        joint_name, &pos_state_, &vel_state_, &eff_state_));
    
    pos_if.registerHandle(hardware_interface::JointHandle(
        state_if.getHandle(joint_name), 
        &cmd_request_.request.value));

    state_sub_ = nh.subscribe(state_topic, 1, &TopWheelDriver::StateCallback, this);
    cmd_srv_client_ = nh.serviceClient<rr_wheels::FloatTrigger>(cmd_service);
}

bool TopWheelDriver::ReadState()
{
    return true;
}

bool TopWheelDriver::SendCommand()
{
    return cmd_srv_client_.call(cmd_request_) and cmd_request_.response.success;
}

void TopWheelDriver::StateCallback(rr_wheels::WheelStatusConstPtr msg)
{
    pos_state_ = msg->angle;
}