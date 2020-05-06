#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include "Eigen/Dense"
#include "Eigen/Geometry"
 #include <tf/transform_listener.h>

class ProximityListener
{
private:
    int num_sensors;
    float distance_threshold;
    void sensorCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
    ros::Subscriber sub;
    tf::TransformListener listener;
public:
    ProximityListener(int argc, char **argv, int num_sensors, float distance_threshold);
    ~ProximityListener();
};

ProximityListener::ProximityListener(int argc, char **argv, int num_sensors, float distance_threshold)
{
    num_sensors = num_sensors;
    distance_threshold = distance_threshold;
    ros::NodeHandle n;
    sub = n.subscribe<sensor_msgs::LaserScan>("proximity_data0", 1, &ProximityListener::sensorCallback, this);
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
        std::cout << "[" << v << "]" << std::endl;
        std::cout << "[" << r * w << "]" << std::endl;
        std::cout << "[" << result << "]" << std::endl;
        std::cout << "-------------------" << std::endl;

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
    ProximityListener proximity_listener(argc, argv, 162, 0.03);
    ros::spin();
    return 0;
}