#include <functional>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Vector3.hh>
#include <ros/ros.h>
#include <ctime>
#include <rr_msgs/MotorState.h>
#include "winch_force_plugin/winch.h"
#include "winch_force_plugin/pid.h"


namespace gazebo
{
    class WinchForcePlugin : public ModelPlugin
    {
    public:
        void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
        {
            model_ = _parent;
            sdf_ = _sdf;

            for(auto& link : model_->GetLinks()) 
                tot_mass_+=link->GetInertial()->Mass();

            gravity_dir_ = model_->GetWorld()->Gravity().Normalize();
            
            StartNode();
            InitWinches();
            InitPIDs();

            ros::spinOnce();

            update_connection_ = event::Events::ConnectWorldUpdateBegin(
                std::bind(&WinchForcePlugin::OnUpdate, this));

            ROS_INFO("Winch plugin loaded for model: %s", model_->GetName().c_str());
        }

    private:
        void InitWinches()
        {
            ignition::math::Vector3d poly_coeffs = LoadParam<ignition::math::Vector3d>("rope_stiff_poly_coeffs"); 

            right_winch_ptr_ = std::make_unique<WinchPlugin::Winch>(
                nh_.get(),
                model_->GetLink(LoadParam<std::string>("right_rope_link")),
                LoadParam<ignition::math::Vector3d>("right_fixed_pos"),
                LoadParam<std::string>("right_motor_topic"),
                LoadParam<std::string>("right_radius_topic"),
                LoadParam<int>("right_prismatic_axis"),
                LoadParam<bool>("right_rot_dir_switch")
                );
            right_winch_ptr_->SetStrainCurvePolyCoeffs(poly_coeffs);

            left_winch_ptr_ = std::make_unique<WinchPlugin::Winch>(
                nh_.get(),
                model_->GetLink(LoadParam<std::string>("right_rope_link")),
                LoadParam<ignition::math::Vector3d>("left_fixed_pos"),
                LoadParam<std::string>("left_motor_topic"),
                LoadParam<std::string>("left_radius_topic"),
                LoadParam<int>("left_prismatic_axis"),
                LoadParam<bool>("left_rot_dir_switch")
                );
            left_winch_ptr_->SetStrainCurvePolyCoeffs(poly_coeffs);
        }


        void InitPIDs()
        {
            double kp = LoadParam<double>("kp");
            double kd = LoadParam<double>("kd");
            double ki = LoadParam<double>("ki");

            right_pid_ = WinchPlugin::PID(
                    kp,
                    kd,
                    ki,
                    std::bind(&WinchPlugin::Winch::GetRealPosition, right_winch_ptr_.get())
                    );

            left_pid_ = WinchPlugin::PID(
                    kp,
                    kd,
                    ki,
                    std::bind(&WinchPlugin::Winch::GetRealPosition, left_winch_ptr_.get())
                    );
        }

        void StartNode()
        {
            // initialize Ros node
            int argc = 0;
            char **argv = NULL;
            ros::init(argc, argv, "winch plugin", ros::init_options::AnonymousName);
            nh_ = std::make_unique<ros::NodeHandle>();
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
            ApplyForce(*right_winch_ptr_, right_pid_);
            ApplyForce(*left_winch_ptr_, left_pid_);
        }

    private:

        void ApplyForce(WinchPlugin::Winch& winch, WinchPlugin::PID& pid)
        {
            winch.UpdateState();
            double pid_output = pid.Compute(winch.GetSimPosition());
            auto static_tension = StaticRopeTension(winch.GetRopeDir());
            double rope_tension = pid_output + static_tension;
            
            if (rope_tension < 0)
            {
                if (pid.integral < - static_tension) 
                    pid.integral = - static_tension;

                return;
            }
            winch.ApplyRopeTension(rope_tension);
        }

        double StaticRopeTension(const ignition::math::Vector3d& rope_dir) const
        {
            double cos_angle = std::abs(rope_dir.Dot(gravity_dir_));
            double static_force = tot_mass_*9.82/cos_angle/2;
            return static_force;
        }


        physics::ModelPtr model_;
        sdf::ElementPtr sdf_;
        event::ConnectionPtr update_connection_;

        std::unique_ptr<ros::NodeHandle> nh_;

        double tot_mass_ = 0;
        ignition::math::Vector3d gravity_dir_;

        std::unique_ptr<WinchPlugin::Winch> right_winch_ptr_;
        std::unique_ptr<WinchPlugin::Winch> left_winch_ptr_;
        WinchPlugin::PID right_pid_;
        WinchPlugin::PID left_pid_;
    };

    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(WinchForcePlugin)
} // namespace gazebo