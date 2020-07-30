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
#include <math.h>
#include <vector>

#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "signal.h"
#include <ros/master.h>

class ProximityListener
{
private:
    int num_sensors{0};
    int num_control_points{0};
    float distance_threshold{0.0};
    float floor_threshold{0.04};
    bool removeFloor{true};
    std::vector<float> sphere_radiuses{0.23, 0.24, 0.2, 0.237, 0.225, 0.20, 0.27, 0.3};
    std::unique_ptr<Eigen::Vector3d[]> live_points;
    std::vector<Eigen::Vector3d> final_points;

    void sensorCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
    std::unique_ptr<ros::Subscriber[]> sub;
    std::unique_ptr<Eigen::Vector3d[]> translation1;
    std::unique_ptr<Eigen::Vector3d[]> translation2;
    std::unique_ptr<Eigen::Vector3d[]> translation_control_points;
    std::unique_ptr<Eigen::Quaterniond[]> rotation;
    std::unique_ptr<tf::StampedTransform[]> transform;
    std::unique_ptr<tf::StampedTransform[]> transform_control_points;
    tf::TransformListener listener;
    bool isInSphere(Eigen::Vector3d);

    visualization_msgs::Marker marker;
    visualization_msgs::MarkerArray marker_array;
    std::vector<std::vector<Eigen::Vector3d>> buffer;
public:
    ros::NodeHandle n;
    ros::Publisher pub, pubVisualization;
    ProximityListener(int argc, char **argv, int num_sensors, int num_control_points, float distance_threshold, bool removeFloor, float floor_threshold);
    void start();
    void stop();
    void initializeMarker();
};

ProximityListener::ProximityListener(int argc, char **argv, int num_sensors, int num_control_points,
                                     float distance_threshold=0.0, bool removeFloor=true, float floor_threshold=0.04)
{

    pub = n.advertise<ros_robotic_skin::PointArray>("live_points", 1);
    pubVisualization = n.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1);

    distance_threshold = distance_threshold;
    removeFloor = removeFloor;
    floor_threshold = floor_threshold;
    this->live_points = std::make_unique<Eigen::Vector3d[]>(num_sensors);
    this->num_control_points = num_control_points;
    this->num_sensors = num_sensors;
    this->sub = std::make_unique<ros::Subscriber[]>(num_sensors);
    this->translation1 = std::make_unique<Eigen::Vector3d[]>(num_sensors);
    this->translation2 = std::make_unique<Eigen::Vector3d[]>(num_sensors);
    this->rotation = std::make_unique<Eigen::Quaterniond[]>(num_sensors);
    this->transform = std::make_unique<tf::StampedTransform[]>(num_sensors);
    this->transform_control_points = std::make_unique<tf::StampedTransform[]>(num_control_points);
    this->translation_control_points = std::make_unique<Eigen::Vector3d[]>(num_control_points);
    for (int i = 0; i < num_sensors; i++) {
        sub[i] = n.subscribe<sensor_msgs::LaserScan>("proximity_data" + std::to_string(i), 1, &ProximityListener::sensorCallback, this);
    }
    n.setParam("num_sensors", num_sensors);

    this->initializeMarker();
}

void ProximityListener::initializeMarker()
{
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




void ProximityListener::sensorCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    int sensor_number = std::stoi(scan->header.frame_id.substr(scan->header.frame_id.find_first_of("0123456789"), scan->header.frame_id.length() -1));

    try {
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

        live_points[sensor_number] = translation1[sensor_number] + (rotation[sensor_number] * translation2[sensor_number]);
    } catch (tf::TransformException ex) {
        ROS_ERROR("%s", ex.what());
        ros::Duration(1.0).sleep();
    }
}

void ProximityListener::start()
{
    ros::AsyncSpinner spinner(0);
    spinner.start();
    ROS_INFO("Loading Proximity Listener...");
    ros::Duration(2.0).sleep();
    ROS_INFO("Loaded");

    geometry_msgs::Point point;
    ros_robotic_skin::PointArray msg;

    ros::Rate rate(50.0);
    while (ros::ok())
    {
        final_points.clear();
        for (int i = 0; i < num_sensors; i++)
        {
            if (!(std::isnan(live_points[i].x()) || isInSphere(live_points[i]) || ( (live_points[i].z() < floor_threshold) && removeFloor)))
            {
                final_points.push_back(live_points[i]);
            }
        }
        if (buffer.size() == 10)
        {
            buffer.pop_back();
        }
        buffer.insert(buffer.begin(), final_points);
        // Delete all visualizations from previous period
        marker.action = visualization_msgs::Marker::DELETEALL;
        marker_array.markers.push_back(marker);
        pubVisualization.publish<visualization_msgs::MarkerArray>(marker_array);
        marker_array.markers.clear();

        // Preparation for adding new visualizations
        marker.action = visualization_msgs::Marker::ADD;
        marker.header.stamp = ros::Time::now();
        for (int i = 0; i < buffer.size(); i++)
        {
            for (int j = 0; j < buffer[i].size(); j++)
            {
                // Add points to the PointArray message
                point.x = buffer[i][j].x();
                point.y = buffer[i][j].y();
                point.z = buffer[i][j].z();
                msg.points.push_back(point);

                // Add points to the visualization message
                marker.id = 162*i + j;
                marker.pose.position.x = point.x;
                marker.pose.position.y = point.y;
                marker.pose.position.z = point.z;
                marker_array.markers.push_back(marker);
            }
        }

        // Publish
        pub.publish<ros_robotic_skin::PointArray>(msg);
        pubVisualization.publish<visualization_msgs::MarkerArray>(marker_array);

        // Clear all to get ready for next loop
        msg.points.clear();
        marker_array.markers.clear();
        rate.sleep();
    }
}

bool ProximityListener::isInSphere(Eigen::Vector3d point_in_space) {
    for (int i = 0; i < num_control_points; i++) {
        listener.lookupTransform("/world", "/control_point" + std::to_string(i),
                                 ros::Time(0), transform_control_points[i]);
        translation_control_points[i] << transform_control_points[i].getOrigin().getX(),
                                         transform_control_points[i].getOrigin().getY(),
                                         transform_control_points[i].getOrigin().getZ();
        if ((point_in_space - translation_control_points[i]).norm() < sphere_radiuses[i])
            return true;
    }
    return false;
}

void ProximityListener::stop()
{
    ROS_INFO("Shutting down proximity listener...");
    ROS_INFO("Deleting visualization points...");
    marker.action = visualization_msgs::Marker::DELETEALL;
    marker_array.markers.push_back(marker);
    pubVisualization.publish<visualization_msgs::MarkerArray>(marker_array);
    ROS_INFO("Deleted all visualization points");
    ros::shutdown();
}

int topic_count(std::string topic_substring)
{
    ros::master::V_TopicInfo topic_infos;
    ros::master::getTopics(topic_infos);

    int count = 0;
    for (ros::master::V_TopicInfo::iterator it = topic_infos.begin() ; it != topic_infos.end(); it++) {
        const ros::master::TopicInfo& info = *it;
        if (info.name.find(topic_substring) != std::string::npos) {
            count++;
        }
    }
    return count;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "proximity_listener");
    ProximityListener proximity_listener(argc, argv, topic_count("proximity_data"), 8);
    proximity_listener.start();

    return 0;
}
