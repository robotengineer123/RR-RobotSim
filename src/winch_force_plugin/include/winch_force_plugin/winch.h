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
            gazebo::physics::LinkPtr rope_link, 
            ignition::math::Vector3d fixed_pos, 
            std::string motor_state_topic, 
            std::string radius_topic, 
            int prismatic_axis, 
            bool rot_dir_switch=false);
        
        void SetStrainCurvePolyCoeffs(ignition::math::Vector3d poly_coeffs);
        void UpdateState();
        double GetSimPosition();
        double GetRealPosition();
        const ignition::math::Vector3d& GetRopeDir();
        void ApplyRopeTension(double tension);

        void EffRadiusCallback(const std_msgs::Float64ConstPtr &msg);
        void MotorCallback(const rr_msgs::MotorStateConstPtr &msg);

    private:
        double RopeStiffness(double strain);
        void UpdateStiffness();

        ros::NodeHandle* nh;

        ros::Subscriber eff_r_sub;
        ros::Subscriber motor_sub;

        double eff_radius_ = 0.2;
        double encoder_pos_ = 0;
        double prev_encoder_pos_ = 0;
        
        int rot_dir_;
        bool first_read = true;
        
        ignition::math::Vector3d rope_dir_;

        double real_rope_len_ = 0;
        double sim_rope_len_ = 0;
        double sim_vel = 0;
        double real_vel = 0;

        gazebo::physics::LinkPtr rope_link_;
        gazebo::physics::JointPtr joint_;

        int axis_ = 1;
        ignition::math::Vector3d fixed_pos_{10, 0.0, 0.0};
        ignition::math::Vector3d poly_coeffs_{848.64823567, 188.91463659,  13.92308499};
    };
}
