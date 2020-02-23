#include "PoseEstimationPublisher.h"
#include <imu_filter_madgwick/imu_filter.h>
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
    get_imu_data();

}

void PosePublisher::get_imu_data() {
    std::vector<int> imu_numbers {1, 2, 3, 4, 5, 6, 7};
    for(int i: imu_numbers){
        ros::Subscriber cam_sub = nh->subscribe("/imu_data"+std::to_string(i),100, imu_callback);
    }
    ros::spin();
}

void PosePublisher::imu_callback(const sensor_msgs::Imu::ConstPtr &msg) {
    double q0, q1, q2, q3;
    filterStationary<WorldFrame::ENU>((float)msg->linear_acceleration.x, (float)msg->linear_acceleration.y, (float)msg->linear_acceleration.z,
                                      q0, q1, q2, q3);
    geometry_msgs::Quaternion calculated_quaternion;
    /*Why am I considering like that?:
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
     * */
    if(msg->header.frame_id == "imu_data0"){
        imu0_pose.publish(calculated_quaternion);
    } else if(msg->header.frame_id == "imu_data1"){
        imu1_pose.publish(calculated_quaternion);
    } else if(msg->header.frame_id == "imu_data2"){
        imu2_pose.publish(calculated_quaternion);
    } else if(msg->header.frame_id == "imu_data3"){
        imu3_pose.publish(calculated_quaternion);
    } else if(msg->header.frame_id == "imu_data4"){
        imu4_pose.publish(calculated_quaternion);
    } else if(msg->header.frame_id == "imu_data5"){
        imu5_pose.publish(calculated_quaternion);
    } else if(msg->header.frame_id == "imu_data6"){
        imu6_pose.publish(calculated_quaternion);
    } else if(msg->header.frame_id == "imu_data7"){
        imu7_pose.publish(calculated_quaternion);
    }

}
