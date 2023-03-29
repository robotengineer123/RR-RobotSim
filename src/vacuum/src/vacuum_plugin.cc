#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Vector3.hh>
#include <string>
#include <math.h>
#include <iostream>
#include <sstream>
#include <std_msgs/String.h>
#include <ros/ros.h>
#include <stdio.h>
#include <vector>

namespace gazebo {
class VacuumPlugin : public ModelPlugin {

public:
    void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {
        
        // Store the pointer to the model
        model = _parent;

        // Load values specified in robot.gazebo file
        edge = _sdf->Get<double>("edge");
        center = _sdf->Get<double>("center");
        mu = _sdf->Get<double>("mu");
        t = _sdf->Get<double>("t");
        k_v = _sdf->Get<double>("k_v");
        
        // Get links
        vacuum_link = model->GetLink("vacuum_sheet");
        dummy_link = model->GetLink("dummy");

        // Start plugin node
        StartNode();

        // Listen to topic
        vac_sub = nh->subscribe("/vacuum_system/status_string", 1, &VacuumPlugin::VacuumSystemCallback, this);
        ros::spinOnce();

        ROS_INFO("Vacuum plugin loaded for model: %s", model->GetName().c_str());

        // Listen to the update event. This event is broadcast every
        // simulation iteration.
        updateConnection = event::Events::ConnectWorldUpdateBegin(
            std::bind(&VacuumPlugin::OnUpdate, this));
    }

    // Called by the world update start event
    void OnUpdate() {

        ros::spinOnce();

        // Get vacuum percentage of edge and center vacuum
        vac = extractFloatWords(vacuum_system);
        
        edge_vac = vac[0];
        center_vac = vac[1];

        // [% vacuum] converted to [Pa] and multiplied to Area [m2] to obtain [N]
        N_center = (center_vac*1013)*center;
        N_side = (edge_vac*1013)*edge;
        N = N_center + N_side;

        // Get pose of vacuum sheet
        auto vac_pose = vacuum_link->WorldPose();

        // Compression of vacuum sheet
        d_v = t - vac_pose.Z();

        // Normal force from vacuum sheet
        N_v = k_v*d_v;

        // Force compressing suspension
        N_s = N - N_v;

        // Friction force
        F = mu*N_v;
        
        // Get resultant force on robot with no friction force
        ignition::math::Vector3d F_res = dummy_link->RelativeForce();
        
        // Check if all vacuum is off -> set forces to zero
        // Check if resultant force in plane is below the friction force -> Stand still
        F_res_plane = std::sqrt(std::pow(F_res.X(), 2) + std::pow(F_res.Y(), 2));

        //if ((not center_vac) && (not edge_vac)) {
        //    model->GetLink("dummy")->SetForce(ignition::math::Vector3d(0, 0, 0));
        //}
        if (F_res_plane == 0.0) {
            dummy_link->SetForce(ignition::math::Vector3d(0, 0, -N_s));
        }
        else if (F_res_plane < F) {
            dummy_link->SetForce(ignition::math::Vector3d(-F_res.X(), -F_res.Y(), -N_s));
        } 
        else {
            dummy_link->SetForce(ignition::math::Vector3d(-F*F_res.X()/F_res_plane, -F*F_res.Y()/F_res_plane, -N_s));
        }

    }

    void VacuumSystemCallback(const std_msgs::StringConstPtr &msg)
    {
        vacuum_system = msg->data;
    }

    // Function to find two doubles in a string
    std::vector<double> extractFloatWords(std::string str)
    {
        std::stringstream ss;
    
        /* Storing the whole string into string stream */
        ss << str;
    
        /* Running loop till the end of the stream */
        std::string temp;
        double found;
        std::vector<double> arr;
        while (!ss.eof() && arr.size() < 2) {
    
            /* extracting word by word from stream */
            ss >> temp;
    
            /* Checking the given word is integer or not */
            if (std::stringstream(temp) >> found)
                arr.push_back(found);
            }
        return arr;
    }

private:

    void StartNode()
    {
        // initialize Ros node
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "vacuum plugin", ros::init_options::AnonymousName);
        nh = std::make_unique<ros::NodeHandle>();
    }

    // Pointer to the model
    physics::ModelPtr model;

    // Pointer to the update event connection
    event::ConnectionPtr updateConnection;

    // Node handle, subscriber and string for topic to subscribe to
    std::unique_ptr<ros::NodeHandle> nh;
    ros::Subscriber vac_sub;

    // Member variables
    // Vacuum percentage array
    std::vector<double> vac;
    // Normal force compressing vacuum
    double N_v;
    // Normal force compressing suspension
    double N_s;
    // Total normal force
    double N;
    // Normal forces from center and edge vacuum
    double N_side;
    double N_center;
    // Friction force between vacuum sheet and blade
    double F;
    // Resultant force in the plane
    double F_res_plane;
    // Vacuum percent
    double edge_vac;
    double center_vac;
    // Edge and center vacuum cell areas
    double edge;
    double center;
    // Friction coefficient between vacuum sheet and blade
    double mu;
    // Vacuum topic message
    std::string vacuum_system = "\n  edge_vacuum: VAC 10.8%\n  center_vacuum: VAC 6.8%";
    // Thickness of vacuum sheet
    double t;
    // Stiffness of vacuum sheet
    double k_v;
    // Compression of vacuum sheet
    double d_v;
    // Links
    physics::LinkPtr vacuum_link;
    physics::LinkPtr dummy_link;

};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(VacuumPlugin)
} // namespace gazebo