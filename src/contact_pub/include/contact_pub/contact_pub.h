#pragma once
#include <string>

#include <gazebo/gazebo.hh>
#include <gazebo/sensors/sensors.hh>
#include <ros/ros.h>


namespace gazebo
{
  /// \brief An example plugin for a contact sensor.
  class ContactPlugin : public SensorPlugin
  {
    void StartNode();
    /// \brief Constructor.
    public: ContactPlugin();

    /// \brief Destructor.
    public: virtual ~ContactPlugin();

    /// \brief Load the sensor plugin.
    /// \param[in] _sensor Pointer to the sensor that loaded this plugin.
    /// \param[in] _sdf SDF element that describes the plugin.
    public: virtual void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf);

    /// \brief Callback that receives the contact sensor's update signal.
    private: virtual void OnUpdate();

    /// \brief Pointer to the contact sensor
    private: sensors::ContactSensorPtr parentSensor;

    /// \brief Connection that maintains a link between the contact sensor's
    /// updated signal and the OnUpdate callback.
    private: 
        event::ConnectionPtr updateConnection;
        std::unique_ptr<ros::NodeHandle> nh_;
        ros::Publisher pub_;
  };
}