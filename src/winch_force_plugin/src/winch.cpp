#include "winch_force_plugin/winch.h"

using namespace WinchPlugin;

Winch::Winch(
    ros::NodeHandle *nh_ptr,
    gazebo::physics::LinkPtr rope_link,
    ignition::math::Vector3d fixed_pos,
    std::string motor_state_topic,
    std::string radius_topic,
    int prismatic_axis,
    bool rot_dir_switch)
    : nh(nh_ptr),
    rope_link_(rope_link),
    fixed_pos_(fixed_pos),
    axis_(prismatic_axis)
{
    joint_ = rope_link_->GetParentJoints()[0];

    eff_r_sub = nh->subscribe(radius_topic, 1, &Winch::EffRadiusCallback, this);
    motor_sub = nh->subscribe(motor_state_topic, 1, &Winch::MotorCallback, this);

    real_rope_len_ = fixed_pos_.Distance(rope_link_->WorldCoGPose().Pos());

    if (rot_dir_switch)
        rot_dir_ = -1;
    else
        rot_dir_ = 1;
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

void Winch::EffRadiusCallback(const std_msgs::Float64ConstPtr &msg)
{
    eff_radius_ = msg->data;
}

void Winch::MotorCallback(const rr_msgs::MotorStateConstPtr &msg)
{
    prev_encoder_pos_ = encoder_pos_;
    encoder_pos_ = msg->position;
    if (first_read)
    {
        prev_encoder_pos_ = encoder_pos_;
        first_read = false;
    }
}

void Winch::UpdateState()
{
    real_rope_len_ += rot_dir_ * eff_radius_ * (encoder_pos_ - prev_encoder_pos_);
    sim_rope_len_ = fixed_pos_.Distance(rope_link_->WorldCoGPose().Pos());
    ignition::math::Vector3d winch_pos = rope_link_->GetParentJointsLinks()[0]->WorldCoGPose().Pos();
    rope_dir_ = (fixed_pos_ - winch_pos).Normalize();
}

double Winch::GetRealPosition()
{
    return real_rope_len_;
}

double Winch::GetSimPosition()
{
    return sim_rope_len_;
}

const ignition::math::Vector3d& Winch::GetRopeDir()
{
    return rope_dir_;
}

void Winch::ApplyRopeTension(double tension)
{
    ignition::math::Vector3d force = rope_dir_ * tension;
    rope_link_->AddForceAtWorldPosition(force, fixed_pos_);
}

void Winch::UpdateStiffness()
{
    double spring_ref = joint_->GetSpringReferencePosition(axis_);
    double spring_pos = rope_link_->RelativePose().Pos()[axis_];
    double strain = std::abs(spring_ref - spring_pos) / sim_rope_len_;
    double stiffness = RopeStiffness(strain);
    joint_->SetStiffness(axis_, stiffness);
}

double Winch::RopeStiffness(double strain)
{
    ignition::math::Vector3d x{2*strain, 1, 0};
    double stiffness = x.Dot(poly_coeffs_);
    return stiffness;
}