#include <string>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include "Eigen/Dense"
#include "Eigen/Geometry"
#include <tf/transform_listener.h>

int i = 0;
class ProximityListener
{
private:
    int num_sensors;
    float distance_threshold;
    void sensorCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
    ros::Subscriber sub[];
    tf::TransformListener listener;
public:
    ProximityListener(int argc, char **argv, int num_sensors, float distance_threshold);
    ~ProximityListener();
};

ProximityListener::ProximityListener(int argc, char **argv, int num_sensors, float distance_threshold)
{
    sub = ros::Subscriber()[num_sensors];
    distance_threshold = distance_threshold;
    ros::NodeHandle n;
    for (int i = 0; i < 162; i++)
    {
        sub[i] = n.subscribe<sensor_msgs::LaserScan>("proximity_data" + std::to_string(i), 1, &ProximityListener::sensorCallback, this);
    }
}

ProximityListener::~ProximityListener()
{
}

void ProximityListener::sensorCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    tf::StampedTransform transform;
    try
    {
        listener.lookupTransform("/world", "/proximity_link0",
                                 ros::Time(0), transform);
        Eigen::Vector3d result;
        Eigen::Vector3d v(transform.getOrigin().getX(),
                          transform.getOrigin().getY(),
                          transform.getOrigin().getZ());
        Eigen::Vector3d w(scan->ranges[0], 0.0, 0.0);
        Eigen::Quaterniond r(transform.getRotation().getW(),
                             transform.getRotation().getX(),
                             transform.getRotation().getY(),
                             transform.getRotation().getZ());
        result = v + (r * w);
        ROS_DEBUG("Number of callbacks: %d", i++);

    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "proximity_listener");
    ros::AsyncSpinner spinner(0);
    ProximityListener proximity_listener(argc, argv, 162, 0.03);
    spinner.start();
    ros::waitForShutdown();
    return 0;
}