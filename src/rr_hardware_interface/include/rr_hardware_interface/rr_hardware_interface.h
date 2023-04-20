#pragma once
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

#include <vector>
#include <array>
#include <rr_hardware_interface/top_wheel_driver.h>
#include <rr_hardware_interface/rope_drive_driver.h>
#include <ros/callback_queue.h>

namespace rr_hardware_interface
{
    class RRHardwareInterface : public hardware_interface::RobotHW
    {
    public:
        RRHardwareInterface() = default;
        bool init(ros::NodeHandle &root_nh, ros::NodeHandle &robot_hw_nh);

        bool read(const ros::Time time, const ros::Duration period);

        bool write(const ros::Time time, const ros::Duration period);
    
    private:
        void LoadParamWErrorHandling(ros::NodeHandle &nh, const std::string& name, std::string& store_var);
        
        hardware_interface::JointStateInterface state_if_;
        hardware_interface::PositionJointInterface pos_if_;
        hardware_interface::VelocityJointInterface vel_if_;

        std::array<std::unique_ptr<MotorDriver>, 4> drivers_;
        ros::CallbackQueue queue_;
    };
}