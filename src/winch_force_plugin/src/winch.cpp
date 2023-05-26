#include "winch_force_plugin/winch.h"
#include <math.h>

using namespace WinchPlugin;

Winch::Winch(
    ros::NodeHandle *nh_ptr,
    gazebo::physics::LinkPtr winch_link,
    ignition::math::Vector3d fixed_pos,
    std::string radius_topic,
    int winch_axis,
    bool rot_dir_switch)
    : nh(nh_ptr),
      winch_link_(winch_link),
      fixed_pos_(fixed_pos),
      winch_axis_(winch_axis)
{
    winch_joint_ = winch_link_->GetParentJoints()[0];

    eff_r_sub = nh->subscribe(radius_topic, 1, &Winch::EffRadiusCallback, this);

    //The rope len is initialized as if completely stretched but with no tension
    rope_len_ = fixed_pos_.Distance(winch_link_->WorldCoGPose().Pos());  

    if (rot_dir_switch)
        rot_dir_ = -1;
    else
        rot_dir_ = 1;

    prev_rot = winch_joint_->Position(winch_axis_);
}

/// @brief sets the polynomial-coefficients for the Force/Strain curve
/// used to define the rope stiffness.
/// @param c2 x²
/// @param c1 x¹
/// @param c0 x⁰
void Winch::SetStrainCurvePolyCoeffs(ignition::math::Vector3d poly_coeffs)
{
    poly_coeffs_ = poly_coeffs;
}

void Winch::EffRadiusCallback(std_msgs::Float64ConstPtr msg)
{
    eff_radius_ = msg->data;
}

void Winch::UpdateLenghts()
{
    rope_len_ += UnwindStep();
    dist_to_fixed_ = fixed_pos_.Distance(winch_link_->WorldCoGPose().Pos());
    ignition::math::Vector3d winch_pos = winch_link_->WorldCoGPose().Pos();
    rope_dir_ = (fixed_pos_ - winch_pos).Normalize();
}

void Winch::UpdateRopeTension()
{
    UpdateLenghts();

    double elongation = dist_to_fixed_ - rope_len_;
    
    if (elongation < 0)
        return;

    double strain = elongation / dist_to_fixed_;
    double diff_strain = strain - prev_strain; 
    double damp_force = DampRopeForce();
    double elastic_force = ElaticRopeForce(strain);
    double force_mag =  damp_force+elastic_force;
    ignition::math::Vector3d force = rope_dir_ * force_mag;


    // express force vector in the winch coordinatesystem
    ignition::math::Quaterniond quat = winch_link_->WorldCoGPose().Rot();
    ignition::math::Vector3d loc_force = quat.RotateVector(force);

    // find the projection of the force onto the plane of rotation of the winch joint
    ignition::math::Vector3d rot_plane_normal{0,0,0};
    rot_plane_normal[winch_axis_] = 1;
    ignition::math::Vector3d force_inplane = loc_force - loc_force.Dot(rot_plane_normal)*rot_plane_normal;


    double torque = -force_inplane.Length()*eff_radius_;

    winch_link_->AddForceAtRelativePosition(force, {0, 0, 0});  //Add Force at rotational joint

    winch_joint_->SetForce(winch_axis_, torque);
}

double Winch::ElaticRopeForce(double strain)
{
    ignition::math::Vector3d x{std::pow(strain,2), strain, 1};
    double force = x.Dot(poly_coeffs_);
    return force;
}

double Winch::DampRopeForce()
{
    double body_vel = -rope_dir_.Dot(winch_link_->WorldLinearVel());  //Velocity along rope direction
    double rope_vel = winch_joint_->GetVelocity(winch_axis_)*rot_dir_*eff_radius_;
    return (body_vel-rope_vel)*damp_factor_;
}

double Winch::UnwindStep()
{
    double cur_rot = winch_joint_->Position(winch_axis_);
    double angle_diff = rot_dir_*(cur_rot - prev_rot);
    prev_rot = cur_rot;
    double unwind_l = angle_diff * eff_radius_;
    return unwind_l;
}