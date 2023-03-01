#include <functional>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Vector3.hh>
#include <std_msgs/Float64.h>
#include <array>
#include <iostream>
#include <ros/ros.h>
#include <rr_msgs/MotorState.h>
#include <math.h>
#include <unordered_map>

namespace gazebo
{
    class WinchForcePlugin : public ModelPlugin
    {
    public:
        void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
        {
            model_ = _parent;
            sdf_ = _sdf;

            LoadSdfParams();
            winch_link = joint->GetParent();
            rope_link = joint->GetChild();

            StartNode();
            eff_r_sub = nh->subscribe(eff_r_topic_, 1, &WinchForcePlugin::EffRadiusCallback, this);
            torque_sub = nh->subscribe(torque_topic_, 1, &WinchForcePlugin::TorqueCallback, this);
            ros::spinOnce();

            ROS_INFO("Winch plugin loaded for model: %s", model_->GetName().c_str());

            // Listen to the update event. This event is broadcast every
            // simulation iteration.
            update_connection_ = event::Events::ConnectWorldUpdateBegin(
                std::bind(&WinchForcePlugin::OnUpdate, this));
        }

        // Called by the world update start event
        void OnUpdate()
        {
            ros::spinOnce();

            ignition::math::Vector3d winch_pos = winch_link->WorldCoGPose().Pos();
            rope_length = fixed_pos.Distance(winch_pos);
            
            joint->SetStiffness(1 ,normalized_stiffness/rope_length);

            ignition::math::Vector3d rope_force = GlobRopeForce();

            rope_link->AddForceAtRelativePosition(rope_force, {0, 0, 0});
            //ROS_INFO("Force is: %lf, %lf, %lf    torque is: %lf, %lf, %lf", force[0], force[1], force[2], torque[0], torque[1], torque[2]);
        }


        void EffRadiusCallback(const std_msgs::Float64ConstPtr &msg)
        {
            eff_radius_ = msg->data;
        }

        void TorqueCallback(const rr_msgs::MotorStateConstPtr &msg)
        {
            torque_= msg->effort;
        }

    private:

        void StartNode()
        {
            // initialize Ros node
            int argc = 0;
            char **argv = NULL;
            ros::init(argc, argv, "winch plugin", ros::init_options::AnonymousName);
            nh = std::make_unique<ros::NodeHandle>();
        }

        void LoadSdfParams()
        {
            normalized_stiffness = LoadParam<double>("one_meter_stiffness");
            fixed_pos = LoadParam<ignition::math::Vector3d>("rope_fixture");
            eff_r_topic_ = LoadParam<std::string>("radius_topic");
            torque_topic_ = LoadParam<std::string>("torque_topic");
            joint = model_->GetJoint(
                LoadParam<std::string>("rope_joint"));
        }

        template <class T>
        T LoadParam(std::string par_name)
        {
            if (not sdf_->HasElement(par_name))
            {
                std::cerr << "Winch plugin needs " << par_name << " to be specified\n";
                throw std::invalid_argument(par_name);
            }
            return sdf_->Get<T>(par_name);
        }

        ignition::math::Vector3d GlobRopeForce()
        {
            ignition::math::Vector3d winch_pos = winch_link->WorldCoGPose().Pos();
            ignition::math::Quaterniond winch_quat = winch_link->WorldCoGPose().Rot();

            ignition::math::Vector3d rope_dir = (fixed_pos - winch_pos).Normalize();

            double cos_angle = std::abs(winch_quat.RotateVector({0, 1, 0}).Dot(rope_dir));
            ignition::math::Vector3d rope_force = rope_dir* (torque_*eff_radius_/cos_angle);
            return rope_force;
        }

    private:
        physics::ModelPtr model_;
        sdf::ElementPtr sdf_;
        event::ConnectionPtr update_connection_; // Pointer to the update event connection

        std::unique_ptr<ros::NodeHandle> nh;
        
        ros::Subscriber eff_r_sub;
        ros::Subscriber torque_sub;
        std::string eff_r_topic_;
        std::string torque_topic_;
        double eff_radius_ = 0.2;
        double torque_ = 0;

        double prev_rot;

        physics::LinkPtr winch_link;
        physics::LinkPtr rope_link;
        physics::JointPtr joint;


        ignition::math::Vector3d fixed_pos{0.0, 0.0, 0.0};
        double normalized_stiffness = 1;
        double rope_length = 0;
    };

    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(WinchForcePlugin)
} // namespace gazebo