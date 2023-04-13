#include <controller_manager/controller_manager.h>
#include <rr_hardware_interface/rr_hardware_interface.h>
#include <ros/ros.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "rr_hardware_interface");
    ros::AsyncSpinner spinner(3);
    spinner.start();

    ros::NodeHandle root_nh;
    ros::NodeHandle robot_nh("~");

    rr_hardware_interface::RRHardwareInterface robot_hw;
    controller_manager::ControllerManager cm(&robot_hw, robot_nh);

    // timers
    ros::Time timestamp;
    ros::Duration period;
    auto stopwatch_last = std::chrono::steady_clock::now();
    auto stopwatch_now = stopwatch_last;

    robot_hw.init(root_nh, robot_nh);

    ros::Rate loop_rate(100);

    while(ros::ok())
    {
        if(!robot_hw.read(timestamp, period))
        {
            ROS_FATAL_NAMED("rr_hardware_interface", 
                            "Failed to read state from robot. Shutting down");
            ros::shutdown();
        }

        // update time since last read
        timestamp = ros::Time::now();
        stopwatch_now = std::chrono::steady_clock::now();
        period.fromSec(std::chrono::duration_cast<std::chrono::duration<double>>(
            stopwatch_now - stopwatch_last).count());
        stopwatch_last = stopwatch_now;

        // update controllers
        cm.update(timestamp, period);
        robot_hw.write(timestamp, period);

        loop_rate.sleep();
    }

    spinner.stop();
    ROS_INFO_NAMED("rr_hardware_interface", "Shutting down.");
    
    return 0;
}
