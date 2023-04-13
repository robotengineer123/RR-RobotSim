#include <rr_hardware_interface/rr_hardware_interface.h>

namespace rr_hardware_interface
{
    bool RRHardwareInterface::init(ros::NodeHandle &root_nh, ros::NodeHandle &robot_hw_nh)
    {
        // Load joint names from the parameter server
        std::string le_top_wheel_name, te_top_wheel_name, le_rope_drive_name, te_rope_drive_name;
        LoadParamWErrorHandling(robot_hw_nh, "le_top_wheel_yaw_joint", le_top_wheel_name);
        LoadParamWErrorHandling(robot_hw_nh, "te_top_wheel_yaw_joint", te_top_wheel_name);
        LoadParamWErrorHandling(robot_hw_nh, "le_rope_drive_joint", le_rope_drive_name);
        LoadParamWErrorHandling(robot_hw_nh, "te_rope_drive_joint", te_rope_drive_name);

        drivers_[0] = std::unique_ptr<TopWheelDriver>(new TopWheelDriver(
            robot_hw_nh, this, le_top_wheel_name));
        drivers_[1] = std::unique_ptr<TopWheelDriver>(new TopWheelDriver(
            robot_hw_nh, this, te_top_wheel_name));
        drivers_[2] = std::unique_ptr<RopeDriveDriver>(new RopeDriveDriver(
            robot_hw_nh, this, le_rope_drive_name));
        drivers_[3] = std::unique_ptr<RopeDriveDriver>(new RopeDriveDriver(
            robot_hw_nh, this, te_rope_drive_name));

        return true;
    }

    bool RRHardwareInterface::read(const ros::Time time, const ros::Duration period)
    {
        for (auto& driver : drivers_)
            driver->ReadState();
        return true;
    }

    bool RRHardwareInterface::write(const ros::Time time, const ros::Duration period)
    {
        for (auto& driver : drivers_)
            driver->SendCommand();
        return true;
    }

    void rr_hardware_interface::RRHardwareInterface::LoadParamWErrorHandling(ros::NodeHandle &nh, const std::string& name, std::string& store_var)
    {
        if (!nh.getParam(name, store_var))
        {
            throw std::runtime_error("Cannot find required parameter '" + name + "' on the parameter server.");
        }
    }
}