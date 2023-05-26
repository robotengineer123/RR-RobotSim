#include <contact_pub/contact_pub.h>
#include <std_msgs/Bool.h>

using namespace gazebo;
GZ_REGISTER_SENSOR_PLUGIN(ContactPlugin)

/////////////////////////////////////////////////
ContactPlugin::ContactPlugin() : SensorPlugin()
{
}

/////////////////////////////////////////////////
ContactPlugin::~ContactPlugin()
{
}

/////////////////////////////////////////////////
void ContactPlugin::Load(sensors::SensorPtr _sensor, sdf::ElementPtr /*_sdf*/)
{
  StartNode();
  pub_ = nh_->advertise<std_msgs::Bool>("is_bot_wheel_contacting", 1);
  this->parentSensor =
    std::dynamic_pointer_cast<sensors::ContactSensor>(_sensor);

  // Make sure the parent sensor is valid.
  if (!this->parentSensor)
  {
    gzerr << "ContactPlugin requires a ContactSensor.\n";
    return;
  }

  // Connect to the sensor update event.
  this->updateConnection = this->parentSensor->ConnectUpdated(
      std::bind(&ContactPlugin::OnUpdate, this));

  // Make sure the parent sensor is active.
  this->parentSensor->SetActive(true);

}

/////////////////////////////////////////////////
void ContactPlugin::OnUpdate()
{
  // Get all the contacts.
  msgs::Contacts contacts = this->parentSensor->Contacts();
  std_msgs::Bool msg;
  msg.data = not (contacts.contact_size() == 0);
  pub_.publish(msg);
}

void ContactPlugin::StartNode()
{
    // initialize Ros node
    int argc = 0;
    char **argv = NULL;
    ros::init(argc, argv, "contact_pub_node");
    nh_ = std::make_unique<ros::NodeHandle>();
}
