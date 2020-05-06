
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "Eigen/Geometry"
#include "ros_robotic_skin/PointArray.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"

class ProximityVisualizer
{
private:
    void Callback(const ros_robotic_skin::PointArray::ConstPtr& msg);
    ros::NodeHandle n;
    ros::Subscriber sub;
    ros::Publisher pub;
    visualization_msgs::Marker marker;
    visualization_msgs::MarkerArray marker_array;
public:

    ProximityVisualizer();
    ~ProximityVisualizer();
    void start();
};

ProximityVisualizer::ProximityVisualizer()
{
    sub = n.subscribe<ros_robotic_skin::PointArray>("live_points", 1, &ProximityVisualizer::Callback, this);
    pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    // Same for all markers
    marker.ns = "Live Points";
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.05;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
    marker.header.frame_id = "/world";
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
}

ProximityVisualizer::~ProximityVisualizer()
{
}

void ProximityVisualizer::Callback(const ros_robotic_skin::PointArray::ConstPtr& msg)
{
    marker.header.stamp = ros::Time::now();
    for (int i = 0; i < 162; i++)
    {
        // Check that it's not 'nan' or 'inf'
        marker.id = i;
        marker.pose.position.x = msg->points[i].x;
        marker.pose.position.y = msg->points[i].y;
        marker.pose.position.z = msg->points[i].z;
        marker_array.markers.push_back(marker);
    }
    pub.publish<visualization_msgs::MarkerArray>(marker_array);
}

void ProximityVisualizer::start()
{
    ros::spin();
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "proximity_visualizer");
    ProximityVisualizer proximity_visualizer;
    proximity_visualizer.start();
}