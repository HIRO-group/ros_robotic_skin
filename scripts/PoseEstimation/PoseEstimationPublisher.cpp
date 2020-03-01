#include "PoseEstimationPublisher.h"
#include <imu_filter_madgwick/imu_filter.h>
#define FILTER_ITERATIONS 10000

/*
 * The below part filterStationary is directly copied from imu_filter_madgwick example file:
 * https://github.com/ccny-ros-pkg/imu_tools/blob/indigo/imu_filter_madgwick/test/madgwick_test.cpp
 * */
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

template <WorldFrame::WorldFrame FRAME>
void filterStationary(float Ax, float Ay, float Az,
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
        filter.madgwickAHRSupdateIMU(Gx, Gy, Gz, Ax, Ay, Az, dt);
    }

    filter.getOrientation(q0,q1,q2,q3);
}

void PosePublisher::init() {
    // Here just starting the function which will initialize the callback from different IMU placed on Franka
    filter_.setWorldFrame(WorldFrame::ENU);
    get_imu_data();

}

void PosePublisher::get_imu_data() {
    // Loop through IMU numbers and subscribe to them with static class function imu_callback
    std::vector<int> imu_numbers {0};
//    for(int i: imu_numbers){
//        ros::Subscriber cam_sub = nh->subscribe("imu_data"+std::to_string(i),100, PosePublisher::imu_callback);
//    }
    ros::Subscriber cam_sub = nh->subscribe("imu/data_raw", 100, PosePublisher::imu_callback);
    ros::spin();
}

void PosePublisher::imu_callback(const sensor_msgs::Imu::ConstPtr &msg) {
    double q0, q1, q2, q3;
    const geometry_msgs::Vector3& ang_vel = msg->angular_velocity;
    const geometry_msgs::Vector3& lin_acc = msg->linear_acceleration;
    /*
     * AHRS Stuff goes below
     *
     */
    ros::Time time = msg->header.stamp;
    std::string imu_frame_ = msg->header.frame_id;
    if (!initialized || stateless){
        geometry_msgs::Quaternion init_q;
        StatelessOrientation::computeOrientation(WorldFrame::ENU, lin_acc, init_q);
        filter_.setOrientation(init_q.w, init_q.x, init_q.y, init_q.z);
    }
    if(!initialized){
        last_time_ = time;
        initialized = true;
    }
    double dt;
    dt = (time - last_time_).toSec();
    filter_.madgwickAHRSupdateIMU(
            ang_vel.x, ang_vel.y, ang_vel.z,
            lin_acc.x, lin_acc.y, lin_acc.z,
            dt);
    last_time_ = time;
    filter_.getOrientation(q0,q1,q2,q3);
    // AHRS Stuff end
    geometry_msgs::Quaternion calculated_quaternion;
    /*Why considering like that?:
     * Proof: https://github.com/ccny-ros-pkg/imu_tools/blob/indigo/imu_filter_madgwick/src/imu_filter_ros.cpp#L297
     * */
    calculated_quaternion.x = q1;
    calculated_quaternion.y = q2;
    calculated_quaternion.z = q3;
    calculated_quaternion.w =  q0;
    /*
     * Writing like this has two reasons
     * 1) You cannot re initialize imu0_pose and publish to it, when done in a loop nothing gets published and also
     * the topic isn't visible. Basically you can't initialize like this PosePublisher::nh->advertise<geometry_msgs::Quaternion>("imu0_pose", 100) again and again and publish to it
     * Hence the reason of hardcoding
     *
     * 2) This will make it fast rather than initializing again and again
     *
     * 3) Using if-else if will make it easier to read rather than switch
     * */
    imu0_pose.publish(calculated_quaternion);
//    std::cout << calculated_quaternion.x << "," << calculated_quaternion.y << "," << calculated_quaternion.z << "," << calculated_quaternion.w << std::endl;

}
