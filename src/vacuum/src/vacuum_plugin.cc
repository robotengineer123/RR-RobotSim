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
        
        // If vacuum values are specified in .gazebo file
        if (_sdf->HasElement("vacuum_inner"))
        {
            vacuum_inner = _sdf->Get<bool>("vacuum_inner");
        }
        if (_sdf->HasElement("vacuum_inner"))
        {
            vacuum_outer = _sdf->Get<bool>("vacuum_outer");
        }

        // Store the pointer to the model
        this->model = _parent;

        // Listen to the update event. This event is broadcast every
        // simulation iteration.
        this->updateConnection = event::Events::ConnectWorldUpdateBegin(
            std::bind(&VacuumPlugin::OnUpdate, this));
    }

    // Called by the world update start event
    void OnUpdate() {
        // Initialize normal forces
        N_v = 0.0;
        N_s = 0.0;
        
        // Add normal forces if vacuum is on
        if (vacuum_inner)
        {
            N_v = k_v*d_v;
            N_s += 10000;
        }
        if (vacuum_outer)
        {
            N_v = k_v*d_v;
            N_s += 20000;
        }
        
        // Compute total normal force and friction force
        N = N_v + N_s;
        F = N_v*mu;
        
        // Get resultant force on robot with no friction force
        ignition::math::Vector3d F_res = this->model->GetLink("dummy")->RelativeForce();
        
        // Check if all vacuum is off -> set forces to zero
        // Check if resultant force in plane is below the friction force -> Stand still
        F_res_plane = std::sqrt(std::pow(F_res.X(), 2) + std::pow(F_res.Y(), 2));
        if ((not vacuum_inner) && (not vacuum_outer)) {
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
    // Pointer to the model
    physics::ModelPtr model;

    // Pointer to the update event connection
    event::ConnectionPtr updateConnection;

    // Member variables
    // Friction coefficient between vacuum sheet and blade
    double mu = 1.0;
    // Stiffness of vacuum sheet (v) and suspension (s)
    double k_v = 1.0;
    double k_s = 2.0;
    // Compression of vacuum sheet (v) and suspension (s)
    double d_v = 0.05;
    // Normal force on from vacuum sheet, suspension onto blade and total
    double N_v;
    double N_s;
    double N;
    // Friction force between vacuum sheet and blade
    double F;
    // Resultant force in the plane
    double F_res_plane;
    // Whether vacuum is on or off
    bool vacuum_inner = true;
    bool vacuum_outer = true;

};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(VacuumPlugin)
} // namespace gazebo