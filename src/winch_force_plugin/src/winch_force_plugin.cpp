#include <functional>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Vector3.hh>
#include <std_msgs/Float64.h>
#include <array>
#include <iostream>
#include <ros/ros.h>
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

            rope_length = fixed_pos.Distance(winch_link->WorldCoGPose().Pos());

            StartNode();
            eff_r_sub = nh->subscribe(eff_r_topic_, 1, &WinchForcePlugin::EffRadiusCallback, this);
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

            rope_length += UnwindStep();
            ignition::math::Vector3d force = SpringForce();
            
            ignition::math::Quaterniond quat = model_->WorldPose().Rot();
            ignition::math::Vector3d torque_arm = ignition::math::Vector3d(0, 0, 1);
            torque_arm = quat.RotateVector(torque_arm);

            ignition::math::Vector3d torque = torque_arm.Cross(force);

            winch_link->AddForceAtRelativePosition(force, {0, 0, 0});
            winch_link->AddTorque(torque);
            //ROS_INFO("Force is: %lf, %lf, %lf    torque is: %lf, %lf, %lf", force[0], force[1], force[2], torque[0], torque[1], torque[2]);
        }


        void EffRadiusCallback(const std_msgs::Float64ConstPtr &msg)
        {
            eff_radius_ = msg->data;
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
            rot_axis = LoadParam<char>("rot_axis");
            fixed_pos = LoadParam<ignition::math::Vector3d>("rope_fixture");
            eff_r_topic_ = LoadParam<std::string>("radius_topic");
            winch_link = model_->GetLink(
                LoadParam<std::string>("winch_link"));
        }

        template <class T>
        T LoadParam(std::string par_name)
        {
            if (not sdf_->HasElement("winch_link"))
            {
                std::cerr << "Winch plugin needs " << par_name << " to be specified\n";
                throw std::invalid_argument(par_name);
            }
            return sdf_->Get<T>(par_name);
        }

        ignition::math::Vector3d SpringForce()
        {
            ignition::math::Vector3d winch_pos = winch_link->WorldCoGPose().Pos();
            double dist = fixed_pos.Distance(winch_pos);
            ignition::math::Vector3d direction = (fixed_pos - winch_pos).Normalize();

            double stiffness = normalized_stiffness / rope_length;

            ignition::math::Vector3d force(0, 0, 0);
            if (rope_length < dist)
            {
                force = (dist - rope_length)*normalized_stiffness * direction;
            }
            
            return force;
        }

        double UnwindStep()
        {
            double cur_rot = PosAngle(CurrentRot());
            double angle_diff = cur_rot - prev_rot;
            prev_rot = cur_rot;
            if (std::abs(angle_diff) > M_PI) // then we wrapped around the circle
            {
                angle_diff = -std::abs(angle_diff) / angle_diff * (2 * M_PI - std::abs(angle_diff));
            }
            double unwind_l = angle_diff * eff_radius_;
            return unwind_l;
        }

        double PosAngle(double angle)
        {
            if (angle < 0)
                angle = 2 * M_PI - angle;
            return angle;
        }

        double CurrentRot()
        {
            double rot;
            switch (rot_axis)
            {
            case 'x':
                rot = winch_link->WorldCoGPose().Pitch();
                break;

            case 'y':
                rot = winch_link->WorldCoGPose().Roll();
                break;

            case 'z':
                rot = winch_link->WorldCoGPose().Yaw();
                break;

            default:
                std::cerr << "Invalid axis of rotation";
            }
            return rot;
        }

    private:
        physics::ModelPtr model_;
        sdf::ElementPtr sdf_;
        event::ConnectionPtr update_connection_; // Pointer to the update event connection

        std::unique_ptr<ros::NodeHandle> nh;
        
        ros::Subscriber eff_r_sub;
        std::string eff_r_topic_;
        double eff_radius_ = 0.2;

        double prev_rot;

        ignition::math::Vector3d fixed_pos{0.0, 0.0, 0.0};
        physics::LinkPtr winch_link;
        double normalized_stiffness = 1;
        char rot_axis;
        double cur_torque;
        double rope_length;
    };

    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(WinchForcePlugin)
} // namespace gazebo