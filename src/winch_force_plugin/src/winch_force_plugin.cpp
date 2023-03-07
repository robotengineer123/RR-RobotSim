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
            winch_link_ = joint_->GetParent();
            rope_link_ = joint_->GetChild();

            for(auto& link : model_->GetLinks()) 
                tot_mass_+=link->GetInertial()->Mass();
            
            if (rot_dir_switch)
                winch_dir = -1;
            else
                winch_dir = 1;

            StartNode();
            eff_r_sub = nh->subscribe(eff_r_topic_, 1, &WinchForcePlugin::EffRadiusCallback, this);
            motor_sub = nh->subscribe(motor_topic, 1, &WinchForcePlugin::MotorCallback, this);
            
            ros::spinOnce();

            sim_rope_len_ = fixed_pos.Distance(rope_link_->WorldCoGPose().Pos());
            real_rope_len_ = sim_rope_len_;
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
            prev_encoder_pos_ = encoder_pos_;
            encoder_pos_ = msg->position * winch_dir;
            encoder_vel_ = msg->velocity * winch_dir;
            encoder_not_read = false;
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
            fixed_pos = LoadParam<ignition::math::Vector3d>("rope_fixture");
            poly_coeffs = LoadParam<ignition::math::Vector3d>("rope_stiff_poly_coeffs");
            eff_r_topic_ = LoadParam<std::string>("radius_topic");
            motor_topic = LoadParam<std::string>("motor_topic");
            rot_dir_switch = LoadParam<bool>("rot_dir_switch");
            axis_ = LoadParam<int>("rope_joint_axis");

            kp = LoadParam<double>("kp");
            kd = LoadParam<double>("kd");
            ki = LoadParam<double>("ki");

            joint_ = model_->GetJoint(
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
            if (std::abs(encoder_pos_-prev_encoder_pos_) > 5)
            {
                return;
            }
            
            ignition::math::Vector3d winch_pos = winch_link_->WorldCoGPose().Pos();
            rope_dir_ = (fixed_pos - winch_pos).Normalize();

            double spring_ref = joint_->GetSpringReferencePosition(axis_);
            double spring_pos = rope_link_->RelativePose().Pos()[axis_];
            double elongation = std::abs(spring_ref - spring_pos);
            double stiffness = RopeStiffnessCurve(elongation);

            joint_->SetStiffness(axis_, stiffness);

            double pid_output = PID();

            ApplyForce(pid_output);
        }

    private:
        double PID()
        {
            real_rope_len_ +=  winch_dir*eff_radius_ * (encoder_pos_ - prev_encoder_pos_);
            double real_vel =  winch_dir*eff_radius_ * encoder_vel_;

            double sim_rope_len = fixed_pos.Distance(rope_link_->WorldCoGPose().Pos());
            double sim_vel = rope_link_->WorldLinearVel().Length();

            ROS_INFO("real: %lf sim: %lf", real_rope_len_, sim_rope_len);
            
            pos_error_ =  sim_rope_len - real_rope_len_;
            vel_error_ = sim_vel - real_vel; 
            integral_term_ += ki*pos_error_;

            double cos_angle = std::abs(rope_dir_.Dot(gravity_dir_));
            double static_force = tot_mass_*9.82/cos_angle/2;

            double pid_output = pos_error_*kp + vel_error_*kd + integral_term_ + static_force;

            // The winch can only apply force to the robot by reeling in rope.
            // if the rope needs to be longer, the winch should stop applying force
            // and let gravity move the robot down
            if (pid_output < 0)
            {
                pid_output = 0;
                integral_term_ = 0;
            }
            return pid_output;
        }

        void ApplyForce(double magnitude)
        {
            ignition::math::Vector3d force = rope_dir_ * magnitude;
            rope_link_->AddForceAtRelativePosition(force, {0, 0, 0});
        }

        double RopeStiffnessCurve(double elongation)
        {
            ignition::math::Vector3d x{2*elongation, 1, 0};

            double stiffness = x.Dot(poly_coeffs)*70/sim_rope_len_;
            return stiffness;
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
        bool encoder_not_read = true;
        double encoder_pos_ = 0;
        double prev_encoder_pos_;
        bool rot_dir_switch = false;
        ignition::math::Vector3d poly_coeffs{848.64823567, 188.91463659,  13.92308499};
        int winch_dir;
        double tot_mass_ = 0;
        ignition::math::Vector3d rope_dir_ ;
        ignition::math::Vector3d gravity_dir_{-1, 0 , 0};

        double encoder_vel_ = 0;
        double acceleration_ = 0;

        double sim_rope_len_ = 0;
        double real_rope_len_ = 0;
        double sim_vel = 0;
        double real_vel = 0;

        double kp;
        double ki = 0;
        double kd = 0;
        double integral_term_ = 0;
        double pos_error_ = 0;
        double vel_error_ = 0;

        int axis_;

        physics::LinkPtr winch_link_;
        physics::LinkPtr rope_link_;

        physics::JointPtr joint_;

        ignition::math::Vector3d fixed_pos{10, 0.0, 0.0};
    };

    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(WinchForcePlugin)
} // namespace gazebo