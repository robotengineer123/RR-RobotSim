#include <functional>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Vector3.hh>
#include <string>
#include <math.h>
#include <iostream>
#include <sstream>
#include <std_msgs/String.h>
#include <rr_msgs/MotorState.h>
#include <ros/ros.h>
#include <stdio.h>
#include <vector>

namespace gazebo {
class WinchTorquePlugin : public ModelPlugin {

public:
    void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {
        
        // Store the pointer to the model
        model = _parent;

        // Start plugin nodes
        StartNodes();

        // Listen to topics (left and right motor states)
        left_motor_sub = nh_L->subscribe("/left_motor/motor_state", 1, &WinchTorquePlugin::LeftWinchTorqueCallback, this);
        right_motor_sub = nh_R->subscribe("/right_motor/motor_state", 1, &WinchTorquePlugin::RightWinchTorqueCallback, this);
        
        // Publish to topics of the simulation
        //left_motor_pub = nh_L->advertise<std_msgs::Float64>("/rr_robot/rope_drive_le_joint_controller/command", 1);
        //right_motor_pub = nh_R->advertise<std_msgs::Float64>("/rr_robot/rope_drive_te_joint_controller/command", 1);
        
        ros::spinOnce();

        ROS_INFO("Winch torque plugin loaded for model: %s", model->GetName().c_str());

        // Listen to the update event. This event is broadcast every
        // simulation iteration.
        updateConnection = event::Events::ConnectWorldUpdateBegin(
            std::bind(&WinchTorquePlugin::OnUpdate, this));
    }

    // Called by the world update start event
    void OnUpdate() {

        ros::spinOnce();
        
        //model->GetLink("rope_drive_le_joint")->SetTorque(ignition::math::Vector3d(0, left_motor_effort, 0));
        //model->GetLink("rope_drive_te_joint")->SetTorque(ignition::math::Vector3d(0, right_motor_effort, 0));

        //ROS_INFO("Left motor effort: %f", left_motor_effort);
        //ROS_INFO("Right motor effort: %f", right_motor_effort);

    }

    void LeftWinchTorqueCallback(const rr_msgs::MotorStateConstPtr &msg)
    {
        left_motor_effort = msg->effort;
    }
    
    void RightWinchTorqueCallback(const rr_msgs::MotorStateConstPtr &msg)
    {
        right_motor_effort = msg->effort;
    }

private:

    void StartNodes()
    {
        // initialize Ros node
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "Left winch torque plugin", ros::init_options::AnonymousName);
        nh_L = std::make_unique<ros::NodeHandle>();

        // initialize Ros node
        ros::init(argc, argv, "Right winch torque plugin", ros::init_options::AnonymousName);
        nh_R = std::make_unique<ros::NodeHandle>();
    }

    // Pointer to the model
    physics::ModelPtr model;

    // Pointer to the update event connection
    event::ConnectionPtr updateConnection;

    // Node handle, subscriber and publisher
    std::unique_ptr<ros::NodeHandle> nh_L;
    std::unique_ptr<ros::NodeHandle> nh_R;
    ros::Subscriber left_motor_sub;
    ros::Subscriber right_motor_sub;
    ros::Publisher left_motor_pub;
    ros::Publisher right_motor_pub;

    // Member variables
    float left_motor_effort;
    float right_motor_effort;


};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(WinchTorquePlugin)
} // namespace gazebo