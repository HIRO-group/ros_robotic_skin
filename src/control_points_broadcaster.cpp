#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>

int NUMBER_OF_CONTROL_POINTS = 5;

void publish_control_point(tf::TransformBroadcaster& br,
                           tf::Transform& transform,
                           ros::Publisher& marker_pub,
                           int link_number,
                           float z,
                           float sphere_radius )
{
    int i;
    switch (link_number)
    {
        case 3: i = 1; break;
        case 5: i = 2; break;
        default: break;
    }
    std::string name;
    std::string parent;
    name = "control_point" + std::to_string(i);
    parent = "panda_link" + std::to_string(link_number);
    transform.setOrigin( tf::Vector3(0.0, 0.0, z) );
    transform.setRotation( tf::Quaternion(0, 0, 0, 1) );
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), parent, name));
}

void visualize_sphere(ros::Publisher& marker_pub, visualization_msgs::Marker& marker, int control_point_number)
{
  // Marker setup
    marker.header.frame_id = "/control_point" + std::to_string(control_point_number);
    marker.header.stamp = ros::Time::now();
    marker.ns = "basic_shapes";
    marker.id = control_point_number;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    // Position and orientation
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    float radius = 0.15;

    marker.scale.x = radius;
    marker.scale.y = radius;
    marker.scale.z = radius;

    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 0.5;

    marker.lifetime = ros::Duration();

    marker_pub.publish(marker);
}


int main(int argc, char** argv){
    ros::init(argc, argv, "control_point_broadcaster");
    ros::NodeHandle node;

    tf::TransformBroadcaster br;
    tf::Transform transform;

    ros::Rate rate(50.0);

    ros::Publisher marker_pub = node.advertise<visualization_msgs::Marker>("visualization_markers", 1);
    visualization_msgs::Marker marker;


    while (node.ok())
    {


        for (int i = 0; i < NUMBER_OF_CONTROL_POINTS; i++)
        {
            visualize_sphere(marker_pub, marker, i);
        }
        rate.sleep();
    }
    return 0;
};