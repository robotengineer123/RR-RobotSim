#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <string>
#include <math.h>
#include <stdio.h>


namespace gazebo {
class LaserPlugin : public ModelPlugin {

public:
    void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {
        // Store pointer to the model
        model = _parent;
        
        // Width of turbine blade
        blade_width = _sdf->Get<float>("blade_width");
        half_blade_width = blade_width/2;
        
        // Populate the LaserScan message
        laser_frequency = _sdf->Get<float>("laser_frequency");
        scan.header.frame_id = _sdf->Get<std::string>("header_frame_id");
        scan.angle_min = _sdf->Get<float>("scan_angle_min");
        scan.angle_max = _sdf->Get<float>("scan_angle_max");
        scan.range_min = _sdf->Get<float>("scan_range_min");
        scan.range_max = _sdf->Get<float>("scan_range_max");

        scan.angle_increment = (scan.angle_max - scan.angle_min) / num_readings;
        scan.time_increment = (1 / laser_frequency) / (num_readings);

        // Set the model name and reference for obtaining the pose
        //modelstate.model_name = model->GetName().c_str();
        //setmodelstate.request.model_state = modelstate;

        // Initialize node, service client and laser publisher
        StartNode();
        scan_pub = nh->advertise<sensor_msgs::LaserScan>("scan", 50);
        ros::spinOnce();

        ROS_INFO("Laser scan plugin loaded for model: %s", model->GetName().c_str());

        // Listen to the update event. This event is broadcast every
        // simulation iteration.
        updateConnection = event::Events::ConnectWorldUpdateBegin(
            std::bind(&LaserPlugin::OnUpdate, this));
    }

    // Called by the world update start event
    void OnUpdate() {
        
        ros::spinOnce();

        // Pose of the robot
        auto rr_pose = model->WorldPose();
        y = rr_pose.Y();
        beta = rr_pose.Yaw();

        // Distance to edges
        le_dist = half_blade_width - y;
        te_dist = half_blade_width + y;

        // Generate some fake data for our laser scan
        for(unsigned int i = 0; i < num_readings; ++i){
        angle = beta + i*scan.angle_increment;
        abs_angle = fabs(angle);
        if (abs_angle<1.55) {
            ranges[i] = fabs(te_dist/(cos(angle)));
        }
        else if (abs_angle>1.59) {
            ranges[i] = fabs(le_dist/(cos(angle)));
        }
        else {
            ranges[i] = 50;
        }
        }
        
        // Scan timing
        ros::Time scan_time = ros::Time::now();
        scan.header.stamp = scan_time;

        // Scan ranges
        scan.ranges.resize(num_readings);
        for(unsigned int i = 0; i < num_readings; ++i){
            scan.ranges[i] = ranges[i];
        }

        // Publish scanner message
        scan_pub.publish(scan);
        }

private:

    void StartNode()
    {
        // initialize Ros node
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "laser_publisher_node", ros::init_options::AnonymousName);
        nh = std::make_unique<ros::NodeHandle>();
    }

    // Pointer to the model
    physics::ModelPtr model;

    // Pointer to the update event connection
    event::ConnectionPtr updateConnection;

    // Node handle, subscriber and string for topic to subscribe to
    std::unique_ptr<ros::NodeHandle> nh;

    // Blade and robot
    float blade_width;
    float half_blade_width;
    float x;
    double y;
    double beta;
    float le_dist;
    float te_dist;
    float angle;
    float abs_angle;

    // Laser scan variables
    constexpr static int num_readings = 100;
    float laser_frequency;
    float ranges[num_readings];
    float intensities[num_readings] = {};

    //gazebo_msgs::SetModelState setmodelstate;
    //gazebo_msgs::ModelState modelstate;

    ros::Publisher scan_pub;

    // Populate the LaserScan message
    sensor_msgs::LaserScan scan;

    };

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(LaserPlugin)
}