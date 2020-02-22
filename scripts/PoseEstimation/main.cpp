#include <cmath>
#include <imu_filter_madgwick/imu_filter.h>
#include "ros/ros.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"
#include "sensor_msgs/Imu.h"
#define FILTER_ITERATIONS 10000


template <WorldFrame::WorldFrame FRAME>
void filterStationary(
        float Ax, float Ay, float Az,
        float Mx, float My, float Mz,
        double& q0, double& q1, double& q2, double& q3) {
    float dt = 0.1;
    float Gx = 0.0, Gy = 0.0, Gz = 0.0; // Stationary state => Gyro = (0,0,0)

    ImuFilter filter;
    filter.setDriftBiasGain(0.0);
    filter.setAlgorithmGain(0.1);

    // initialize with some orientation
    filter.setOrientation(q0,q1,q2,q3);
    filter.setWorldFrame(FRAME);

    for (int i = 0; i < FILTER_ITERATIONS; i++) {
        filter.madgwickAHRSupdate(Gx, Gy, Gz, Ax, Ay, Az, Mx, My, Mz, dt);
    }

    filter.getOrientation(q0,q1,q2,q3);
}

void msgCallback(const sensor_msgs::Imu::ConstPtr& msg){
    double q0, q1, q2, q3;
//    filterStationary((float)msg->linear_acceleration.x, (float)msg->linear_acceleration.y, (float)msg->linear_acceleration.z,
//                     (float)msg->angular_velocity.x, (float)msg->angular_velocity.y, (float)msg->angular_velocity.z,
//                     q0, q1, q2, q3); // Not working, need to figure out
    std::cout << "IMU2 Data: X: " << msg->linear_acceleration.x << " Y: " << msg->linear_acceleration.y << " Z: "
        << msg->linear_acceleration.z << std::endl;


}

int main(int argc, char** argv) {
    ros::init(argc,argv,"IMU_Pose_Estimator");
    ros::NodeHandle nh;
    ros::Subscriber cam_sub = nh.subscribe("/imu_data2",100,msgCallback);
    ros::spin();
    return 0;
}
