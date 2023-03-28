#pragma once
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Vector3.hh>
#include <ros/ros.h>
#include <rr_msgs/MotorState.h>
#include <std_msgs/Float64.h>

namespace WinchPlugin
{
    class Winch
    {
    public:
        Winch(ros::NodeHandle *nh_ptr,
            gazebo::physics::LinkPtr winch_link,
            ignition::math::Vector3d fixed_pos,
            std::string radius_topic,
            int winch_axis,
            bool rot_dir_switch=false);
        
        void SetStrainCurvePolyCoeffs(ignition::math::Vector3d poly_coeffs);
        void UpdateRopeTension();


    private:
        void EffRadiusCallback(std_msgs::Float64ConstPtr msg);
        void UpdateLenghts();
        double RopeStiffness(double strain);
        double UnwindStep();
        double PosAngle(double angle);

        ros::NodeHandle* nh;

        ros::Subscriber eff_r_sub;

        double eff_radius_ = 0.2;
        double prev_rot = 0;
        
        int rot_dir_;
        bool first_read = true;
        
        ignition::math::Vector3d rope_dir_;

        double rope_len_ = 0;
        double dist_to_fixed_ = 0;

        gazebo::physics::LinkPtr winch_link_;
        gazebo::physics::JointPtr winch_joint_;

        int winch_axis_ = 1;
        ignition::math::Vector3d fixed_pos_{10, 0.0, 0.0};
        ignition::math::Vector3d poly_coeffs_{848.64823567, 188.91463659,  13.92308499};
    };
}
