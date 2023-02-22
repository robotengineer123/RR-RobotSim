#include <functional>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Vector3.hh>

namespace gazebo {
class VacuumPlugin : public ModelPlugin {

float N = 10000.0;
float mu = 1.0;
float F;

public:
    void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/) {
        // Store the pointer to the model
        this->model = _parent;
        // Listen to the update event. This event is broadcast every
        // simulation iteration.
        this->updateConnection = event::Events::ConnectWorldUpdateBegin(
            std::bind(&VacuumPlugin::OnUpdate, this));
    }

// Called by the world update start event
public:
    void OnUpdate() {
        F = N*mu;
        this->model->GetLink("dummy")->SetForce(ignition::math::Vector3d(0., 0., 0.));
    }

// Pointer to the model
private:
    physics::ModelPtr model;

// Pointer to the update event connection
private:
    event::ConnectionPtr updateConnection;
};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(VacuumPlugin)
} // namespace gazebo