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
            
            StartNode();
            InitWinches();

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
                model_->GetLink(LoadParam<std::string>("right_winch_link")),
                LoadParam<ignition::math::Vector3d>("right_fixed_pos"),
                LoadParam<std::string>("right_radius_topic"),
                LoadParam<int>("right_winch_axis"),
                LoadParam<bool>("right_rot_dir_switch")
                );
            right_winch_ptr_->SetStrainCurvePolyCoeffs(poly_coeffs);
            right_winch_ptr_->SetDamping(LoadParam<double>("damping"));


            left_winch_ptr_ = std::make_unique<WinchPlugin::Winch>(
                nh_.get(),
                model_->GetLink(LoadParam<std::string>("left_winch_link")),
                LoadParam<ignition::math::Vector3d>("left_fixed_pos"),
                LoadParam<std::string>("left_radius_topic"),
                LoadParam<int>("left_winch_axis"),
                LoadParam<bool>("left_rot_dir_switch")
                );
            left_winch_ptr_->SetStrainCurvePolyCoeffs(poly_coeffs);
            left_winch_ptr_->SetDamping(LoadParam<double>("damping"));

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
                ROS_WARN("Winch plugin needs %s to be specified\n", par_name.c_str());
                throw std::invalid_argument(par_name);
            }
            return sdf_->Get<T>(par_name);
        }

    public:

        // Called by the world update start event
        void OnUpdate()
        {
            ros::spinOnce();
            right_winch_ptr_->UpdateRopeTension();
            left_winch_ptr_->UpdateRopeTension();
        }

    private:

        physics::ModelPtr model_;
        sdf::ElementPtr sdf_;
        event::ConnectionPtr update_connection_;

        std::unique_ptr<ros::NodeHandle> nh_;

        std::unique_ptr<WinchPlugin::Winch> right_winch_ptr_;
        std::unique_ptr<WinchPlugin::Winch> left_winch_ptr_;
    };

    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(WinchForcePlugin)
} // namespace gazebo