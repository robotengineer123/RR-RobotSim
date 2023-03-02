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
#include <rr_msgs/MotorState.h>

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
            motor_sub = nh->subscribe(motor_topic, 1, &WinchForcePlugin::MotorCallback, this);
            ros::spinOnce();
            
            sim_pos = fixed_pos.Distance(rope_link->WorldCoGPose().Pos());
            real_pos_ = sim_pos;
            prev_encoder_pos_ = encoder_pos_;

            ROS_INFO("Winch plugin loaded for model: %s", model_->GetName().c_str());

            // Listen to the update event. This event is broadcast every
            // simulation iteration.
            update_connection_ = event::Events::ConnectWorldUpdateBegin(
                std::bind(&WinchForcePlugin::OnUpdate, this));
        }

        void EffRadiusCallback(const std_msgs::Float64ConstPtr &msg)
        {
            eff_radius_ = msg->data;
        }

        void MotorCallback(const rr_msgs::MotorStateConstPtr &msg)
        {
            encoder_pos_ = msg->position;
            encoder_vel_ = msg->velocity;
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
            motor_topic = LoadParam<std::string>("motor_topic");

            kp = LoadParam<std::string>("kp");
            kd = LoadParam<std::string>("kd");
            ki = LoadParam<std::string>("ki");

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

    public:
        // Called by the world update start event
        void OnUpdate()
        {
            ros::spinOnce();

            joint->SetStiffness(1, normalized_stiffness / real_pos_);

            double pid_output = PID();

            ApplyForce(pid_output);
        }

    private:
        double PID()
        {
            real_pos_ +=  eff_radius_ * (encoder_pos_ - prev_encoder_pos_);
            double real_vel =  eff_radius_ * encoder_vel_;

            double sim_pos = fixed_pos.Distance(rope_link->WorldCoGPose().Pos());
            double sim_vel = rope_link->WorldLinearVel().Length();
            
            pos_error_ =  real_pos_ - sim_pos;
            vel_error_ = real_vel - sim_vel; 
            integral_term_ += ki*pos_error_;

            double pid_output = pos_error_*kp + vel_error_*kd + integral_term_;

            // The winch can only apply force to the robot by reeling in rope.
            // if the rope needs to be longer, the winch should stop applying force
            // and let gravity move the robot down
            if (pid_output < 0)
            {
                pid_output = 0;
            }
            return pid_output;
        }

        void ApplyForce(double magnitude)
        {
            ignition::math::Vector3d winch_pos = winch_link->WorldCoGPose().Pos();
            ignition::math::Vector3d rope_dir = (fixed_pos - winch_pos).Normalize();

            ignition::math::Vector3d force = rope_dir * magnitude;
            rope_link->AddForceAtRelativePosition(force, {0, 0, 0});
        }

    private:
        physics::ModelPtr model_;
        sdf::ElementPtr sdf_;
        event::ConnectionPtr update_connection_; // Pointer to the update event connection

        std::unique_ptr<ros::NodeHandle> nh;

        ros::Subscriber eff_r_sub;
        ros::Subscriber motor_sub;
        std::string eff_r_topic_;
        std::string motor_topic;
        double eff_radius_ = 0.2;
        double encoder_pos_ = 0;
        double prev_encoder_pos_;

        double encoder_vel_ = 0;
        double acceleration_ = 0;

        double prev_time;
        double init_dist;
        double real_pos_ = 0;
        double sim_pos = 0;
        double sim_vel = 0;
        double real_vel = 0;

        double kp;
        double ki = 0;
        double kd = 0;
        double integral_term_ = 0;
        double pos_error_ = 0;
        double vel_error_ = 0;

        physics::LinkPtr winch_link;
        physics::LinkPtr rope_link;
        physics::JointPtr joint;

        ignition::math::Vector3d fixed_pos{0.0, 0.0, 0.0};
        double normalized_stiffness = 1;
    };

    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(WinchForcePlugin)
} // namespace gazebo