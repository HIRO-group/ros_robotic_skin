
#include "ros/ros.h"
#include "ros_robotic_skin/PointArray.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "signal.h"
#include <ros/master.h>

void on_shutdown(int sig);
class ProximityVisualizer {
 private:
    int num_sensors;
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
    void stop();
};

ProximityVisualizer::ProximityVisualizer() {
    signal(SIGINT, on_shutdown);
    if (!n.getParam("/num_sensors", num_sensors)) {
        ROS_ERROR("Can't get number of sensors from the parameter server");
        ros::shutdown();
    }
    sub = n.subscribe<ros_robotic_skin::PointArray>("live_points", 1, &ProximityVisualizer::Callback, this);
    pub = n.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1);
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

    marker.header.stamp = ros::Time::now();
    marker.id = 0;
    marker.pose.position.x = 0.0;
    marker.pose.position.y = 0.0;
    marker.pose.position.z = 0.0;
}

ProximityVisualizer::~ProximityVisualizer() {

}

void ProximityVisualizer::Callback(const ros_robotic_skin::PointArray::ConstPtr& msg) {
    // Delete all points from previous callback
    marker.action = visualization_msgs::Marker::DELETEALL;
    marker_array.markers.push_back(marker);
    pub.publish<visualization_msgs::MarkerArray>(marker_array);
    marker_array.markers.clear();

    // Add new points
    marker.action = visualization_msgs::Marker::ADD;
    marker.header.stamp = ros::Time::now();
    for (int i = 0; i < msg->points.size(); i++) {
        // Check that it's not 'nan' or 'inf'
        marker.id = i;
        marker.pose.position.x = msg->points[i].x;
        marker.pose.position.y = msg->points[i].y;
        marker.pose.position.z = msg->points[i].z;
        marker_array.markers.push_back(marker);
    }
    pub.publish<visualization_msgs::MarkerArray>(marker_array);
    marker_array.markers.clear();
}

void ProximityVisualizer::start() {
    ros::spin();
}

void ProximityVisualizer::stop() {
    ROS_INFO("Shutting down...");
    marker.action = visualization_msgs::Marker::DELETEALL;
    marker_array.markers.push_back(marker);
    pub.publish<visualization_msgs::MarkerArray>(marker_array);
    ROS_INFO("Deleted all points");
    ros::shutdown();
}

void on_shutdown(int sig) {
    ProximityVisualizer end_visualizer;
    end_visualizer.stop();
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "proximity_visualizer", ros::init_options::NoSigintHandler);
    ProximityVisualizer proximity_visualizer;
    proximity_visualizer.start();
}
