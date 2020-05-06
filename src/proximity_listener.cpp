#include <string>
#include <memory>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Point.h"
#include "Eigen/Dense"
#include "Eigen/Geometry"
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include "ros_robotic_skin/PointArray.h"

int num_callbacks = 0;
class ProximityListener
{
private:
    int num_sensors;
    float distance_threshold;
    std::unique_ptr<Eigen::Vector3d[]> live_points;

    void sensorCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
    std::unique_ptr<ros::Subscriber[]> sub;
    std::unique_ptr<Eigen::Vector3d[]> translation1;
    std::unique_ptr<Eigen::Vector3d[]> translation2;
    std::unique_ptr<Eigen::Quaterniond[]> rotation;
    std::unique_ptr<tf::StampedTransform[]> transform;
    tf::TransformListener listener;
public:
    ros::NodeHandle n;
    ProximityListener(int argc, char **argv, int num_sensors, float distance_threshold);
    ~ProximityListener();
    void start(int argc, char **argv);
};

ProximityListener::ProximityListener(int argc, char **argv, int num_sensors, float distance_threshold)
{
    distance_threshold = distance_threshold;
    this->live_points = std::make_unique<Eigen::Vector3d[]>(num_sensors);
    this->num_sensors = num_sensors;
    this->sub = std::make_unique<ros::Subscriber[]>(num_sensors);
    this->translation1 = std::make_unique<Eigen::Vector3d[]>(num_sensors);
    this->translation2 = std::make_unique<Eigen::Vector3d[]>(num_sensors);
    this->rotation = std::make_unique<Eigen::Quaterniond[]>(num_sensors);
    this->transform = std::make_unique<tf::StampedTransform[]>(num_sensors);
    for (int i = 0; i < num_sensors; i++)
    {
        sub[i] = n.subscribe<sensor_msgs::LaserScan>("proximity_data" + std::to_string(i), 1, &ProximityListener::sensorCallback, this);
    }
}

ProximityListener::~ProximityListener()
{
}

void ProximityListener::sensorCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    int sensor_number = std::stoi(scan->header.frame_id.substr(scan->header.frame_id.find_first_of("0123456789"), scan->header.frame_id.length() -1));

    try
    {
        listener.lookupTransform("/world", "/proximity_link" + std::to_string(sensor_number),
                                 ros::Time(0), transform[sensor_number]);
        translation1[sensor_number] << transform[sensor_number].getOrigin().getX(),
                                       transform[sensor_number].getOrigin().getY(),
                                       transform[sensor_number].getOrigin().getZ();
        translation2[sensor_number] << scan->ranges[0],
                                       0.0,
                                       0.0;
        rotation[sensor_number].w() = transform[sensor_number].getRotation().getW();
        rotation[sensor_number].x() = transform[sensor_number].getRotation().getX();
        rotation[sensor_number].y() = transform[sensor_number].getRotation().getY();
        rotation[sensor_number].z() = transform[sensor_number].getRotation().getZ();

        translation1[sensor_number] = translation1[sensor_number] + (rotation[sensor_number] * translation2[sensor_number]);
        live_points[sensor_number] = translation1[sensor_number];
        ROS_DEBUG("Number of callbacks: %d", num_callbacks++);
        ROS_DEBUG("Sensor number: %d", sensor_number);
        ROS_DEBUG("Point in space: [%f]", translation1[146].z());



    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }
}

void ProximityListener::start(int argc, char **argv)
{

    ros::Publisher pub = n.advertise<ros_robotic_skin::PointArray>("live_points", 1);
    ros::AsyncSpinner spinner(0);
    spinner.start();
    ros::Duration(2.0).sleep();

    ROS_INFO("2s sleep done ");

    geometry_msgs::Point points[num_sensors];
    ros_robotic_skin::PointArray msg;

    ros::Rate rate(50.0);
    while (ros::ok())
    {
        for (int i = 0; i < num_sensors; i++)
        {
            points[i].x = live_points[i].x();
            points[i].y = live_points[i].y();
            points[i].z = live_points[i].z();
            msg.point.push_back(points[i]);
        }
        pub.publish<ros_robotic_skin::PointArray>(msg);
        rate.sleep();
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "proximity_listener");
    ProximityListener proximity_listener(argc, argv, 162, 0.03);
    proximity_listener.start(argc, argv);

    return 0;
}