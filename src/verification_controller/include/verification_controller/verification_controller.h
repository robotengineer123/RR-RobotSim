#pragma once
#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <nav_msgs/Odometry.h>

#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>
#include <std_msgs/Float64.h>
#include <stdio.h>

#include <control_toolbox/pid.h>

#include <tf/transform_datatypes.h>


#include <iostream>
#include <string>
#include <iomanip>
#include <sstream>
#include <vector>
#include <fstream>

#include <functional> 

namespace verification_controller {
    
  class VerificationController 
  : public controller_interface::MultiInterfaceController<
    hardware_interface::PositionJointInterface,
    hardware_interface::VelocityJointInterface>
  {

  public:
    virtual bool init(hardware_interface::RobotHW* robot_hw,
                    ros::NodeHandle&             root_nh,
                    ros::NodeHandle&             controller_nh);
    

    void starting(const ros::Time& time);
    void update(const ros::Time& time, const ros::Duration& period);
    void stopping(const ros::Time& time);
    long search_closest(const std::vector<float>& sorted_array, float x);
  
  private:
    void OdomCallback(nav_msgs::OdometryConstPtr cmd);
    void Brake();

    // subscribers/publishers
    ros::Subscriber odom_sub_;
    ros::Publisher yaw_pub;
    ros::Publisher velX_pub;
    std_msgs::Float64 currentYawMsg;
    std_msgs::Float64 currentVelMsg;

    // joints
    hardware_interface::JointHandle r_drive_jh_;
    hardware_interface::JointHandle l_drive_jh_;
    hardware_interface::JointHandle r_steer_jh_;
    hardware_interface::JointHandle l_steer_jh_;

    // pid
    double radius = 0.2;

    std::string file = "/home/mathias/dev/RR-RobotSim/src/verification_controller/src/exp2.csv"; // File containing yaw data to mimic
    double yaw_desi_ = 0.0;
    double steer_desi_angle_ = 0.0;

    float yaw_start = 30.0;

    double vel_desi = 0.05;
    double vel_offset = 0.004; // Velocity offset to reach wanted velocity
    double vel_desi_ = vel_desi + vel_offset;
    double steer_desi_ = steer_desi_angle_*3.14/180;
    double currentVel;
    double currentRoll, currentPitch, currentYaw;
    control_toolbox::Pid pidY;
    control_toolbox::Pid pidV;
    ros::Time last_time = ros::Time::now();
    
    // odometry
    realtime_tools::RealtimeBuffer<nav_msgs::Odometry> odom_buf_;
    // callback
    nav_msgs::Odometry odom_cmd;
    // pid
    nav_msgs::Odometry odom;

    // Node handle
    ros::NodeHandle nhp_;

    std::vector<float> time_vector()
    {
        // define variables
        std::string Time_str, Yaw_str;//variables from file are here
        std::vector<float> Time_;

        //number of lines
        int i = 0;

        std::ifstream coeff(file); //opening the file.
        if (coeff.is_open()) //if the file is open
        {
          //ignore first line
          std::string line;
          getline(coeff, line);

          while (!coeff.eof()) //while the end of file is NOT reached
          {
            //2 sets {Time, Yaw} so use 2 getlines
            getline(coeff, Time_str, ',');
            Time_.push_back(stof(Time_str));
            getline(coeff, Yaw_str, '\n');
            
            i += 1; //increment number of lines
          }
          coeff.close(); //closing the file
        }
        else ROS_INFO("Unable to open file");

        return Time_;
    }

    std::vector<float> yaw_vector()
    {
        // define variables
        std::string Time_str, Yaw_str;//variables from file are here

        std::vector<float> Yaw_;

        //number of lines
        int i = 0;

        std::ifstream coeff(file); //opening the file.
        if (coeff.is_open()) //if the file is open
        {
          //ignore first line
          std::string line;
          getline(coeff, line);

          while (!coeff.eof()) //while the end of file is NOT reached
          {
            //2 sets {Time, Yaw} so use 2 getlines
            getline(coeff, Time_str, ',');
            getline(coeff, Yaw_str, '\n');
            Yaw_.push_back(stof(Yaw_str));
            
            i += 1; //increment number of lines
          }
          coeff.close(); //closing the file
        }
        else ROS_INFO("Unable to open file");

        return Yaw_;
    }

    std::vector<float> Time = time_vector();
    std::vector<float> Yaw = yaw_vector();

  };
  PLUGINLIB_EXPORT_CLASS(verification_controller::VerificationController, controller_interface::ControllerBase)
}