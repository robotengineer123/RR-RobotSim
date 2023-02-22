#include <functional>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Vector3.hh>
#include <array>
#include <iostream>
#include <ros/ros.h>
#include <math.h>

namespace gazebo {
class WinchForcePlugin : public ModelPlugin {
public:
    void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {
        // Store the pointer to the model
        model_ = _parent;

        if (_sdf->HasElement("spring_stiffness"))
        {
            stiffness = _sdf->Get<double>("spring_stiffness");
        }

        if (_sdf->HasElement("rot_axis"))
        {
            rot_axis = _sdf->Get<char>("rot_axis");
        }

        if (_sdf->HasElement("init_radius"))
        {
            eff_radius = _sdf->Get<double>("init_radius");
        }

        if (_sdf->HasElement("rope_fixture"))
        {
            fixed_pos = _sdf->Get<ignition::math::Vector3d>("rope_fixture");
        }

        std::string winch;
        if (not _sdf->HasElement("winch_link"))
        {
            std::cerr << "Winch plugin needs the \"winch_link\" to be specified";
            throw std::invalid_argument("winch_link");
        }
        winch = _sdf->Get<std::string>("winch_link");
        winch_link = model_->GetLink(winch);

        init_dist = fixed_pos.Distance(winch_link->WorldCoGPose().Pos());
        prev_rot = CurrentRot(winch_link, rot_axis);
        unwinded_rope_l = init_dist;
        ROS_WARN("Winch plugin loaded to model: %s on link: %s", model_->GetName().c_str(), winch.c_str());
        // Listen to the update event. This event is broadcast every
        // simulation iteration.
        update_connection_ = event::Events::ConnectWorldUpdateBegin(
            std::bind(&WinchForcePlugin::OnUpdate, this));
    }

  // Called by the world update start event
    void OnUpdate() 
    {
        ignition::math::Vector3d winch_pos = winch_link->WorldCoGPose().Pos();
        ignition::math::v6::Quaterniond winch_rot = winch_link->Rot();
        double dist = fixed_pos.Distance(winch_pos);
        ignition::math::Vector3d direction = (fixed_pos-winch_pos).Normalize();

        unwinded_rope_l += UnwindStep();
        
        ignition::math::Vector3d force(0, 0, 0);
        if (unwinded_rope_l < dist)
        {
            force = (dist-unwinded_rope_l)*stiffness*direction;
        }
        ignition::math::Vector3d torque_arm();
        ignition::math::Vector3d torque = force.Cross();

        physics::Joint_V joint = winch_link->GetParentJoints().front()


        winch_link->SetForceAtRelativePosition(force);

        winch_link->SetTorque(ignition::math::Vector3d(1000000.0, 1000.0, 0.0));
    }

    
private:
    double UnwindStep()
    {
        double cur_rot = PosAngle(CurrentRot());
        double angle_diff = cur_rot - prev_rot;
        prev_rot = cur_rot;
        if (std::abs(angle_diff) > M_PI) //then we wrapped around the circle
        {
            angle_diff = -std::abs(angle_diff)/angle_diff*(2*M_PI - std::abs(angle_diff));
        }
        eff_radius += 2*eff_radius/(2*M_PI)*angle_diff;  //effective radius changes by rope diameter for each revolution
        double unwind_l = angle_diff * eff_radius;
        return unwind_l;
    }

    double PosAngle(double angle)
    {
        if (angle<0)
            angle = 2*M_PI - angle;
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

    // Pointer to the update event connection
    event::ConnectionPtr update_connection_;

    ignition::math::Vector3d fixed_pos{0.0, 0.0, 0.0};
    physics::LinkPtr winch_link;
    double stiffness = 1;
    double init_dist;
    char rot_axis;
    double prev_rot;
    double eff_radius;
    double unwinded_rope_l;
};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(WinchForcePlugin)
} // namespace gazebo