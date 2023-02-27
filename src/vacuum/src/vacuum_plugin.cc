#include <functional>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Vector3.hh>
#include <string>
#include <math.h>

namespace gazebo {
class VacuumPlugin : public ModelPlugin {

public:
    void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {
        
        // Store the pointer to the model
        this->model = _parent;

        // Load values specified in robot.gazebo file
        edge = _sdf->Get<double>("edge");
        center = _sdf->Get<double>("center");
        mu = _sdf->Get<double>("mu");
        N_v = _sdf->Get<double>("N_v");

        // Start plugin node
        StartNode()
        ROS_INFO("Vacuum plugin loaded for model: %s", model->GetName().c_str());

        // Listen to the update event. This event is broadcast every
        // simulation iteration.
        this->updateConnection = event::Events::ConnectWorldUpdateBegin(
            std::bind(&VacuumPlugin::OnUpdate, this));
    }

    // Called by the world update start event
    void OnUpdate() {
        // Add normal forces if vacuum is on
        if (center_vac)
        {
            center_vac_pct = vac_pct;
        }
        if (edge_vac)
        {
            edge_vac_pct = vac_pct;
        }
        
        // [% vacuum] converted to [Pa] and multiplied to Area [m2] to obtain [N]
        N_center = (center_vac_pct*1013)*center;
        N_side = (edge_vac_pct*1013)*edge;
        N = N_center + N_side;

        // Force compressing suspension
        N_s = N - N_v;

        // Friction force
        F = mu*N_v;
        
        // Get resultant force on robot with no friction force
        ignition::math::Vector3d F_res = this->model->GetLink("dummy")->RelativeForce();
        
        // Check if all vacuum is off -> set forces to zero
        // Check if resultant force in plane is below the friction force -> Stand still
        F_res_plane = std::sqrt(std::pow(F_res.X(), 2) + std::pow(F_res.Y(), 2));
        if ((not center_vac) && (not edge_vac)) {
            this->model->GetLink("dummy")->SetForce(ignition::math::Vector3d(0, 0, 0));
        }
        else if (F_res_plane < F) {
            this->model->GetLink("dummy")->SetForce(ignition::math::Vector3d(-F_res.X(), -F_res.Y(), -N));
        } 
        else {
            this->model->GetLink("dummy")->SetForce(ignition::math::Vector3d(-F*F_res.X()/F_res_plane, -F*F_res.Y()/F_res_plane, -N));
        }
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

    // Member variables
    // Force compressing suspension
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
    
    // Whether vacuum is on or off
    bool center_vac = true;
    bool edge_vac = true;
    double vac_pct = 10;
    double edge_vac_pct;
    double center_vac_pct;

};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(VacuumPlugin)
} // namespace gazebo