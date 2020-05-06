
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "Eigen/Geometry"
#include "ros_robotic_skin/PointArray.h"
#include "visualization_msgs/Marker.h"

visualization_msgs::Marker marker;
int id = 0;
double x, y, z;

void Callback(const ros_robotic_skin::PointArray msg)
{
    ROS_INFO("Message received");
    x = msg.points[id].x;
    y = msg.points[id].y;
    z = msg.points[id].z;

}

int main(int argc, char **argv)
{


    ros::init(argc, argv, "proximity_visualizer");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe<ros_robotic_skin::PointArray>("live_points", 1, Callback);
    ros::Publisher pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

    ros::Duration(3.0).sleep();
    ROS_INFO("Let's go");
    ros::Rate rate(50.0);
    ros::spinOnce();
    while (ros::ok())
    {
        marker.id = id;
        marker.pose.position.x = x; //x;
        marker.pose.position.y = y; //y;
        marker.pose.position.z = z; //z;
        marker.header.stamp = ros::Time::now();
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

        pub.publish<visualization_msgs::Marker>(marker);
        ros::spinOnce();
    }

}