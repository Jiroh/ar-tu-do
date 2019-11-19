#include "imu.h"

ImuTest::ImuTest()
{
    m_imu_subscriber = m_node_handle.subscribe<sensor_msgs::Imu>(TOPIC_IMU, 100,
                                                                             &ImuTest::imuCallback, this);
}

void ImuTest::imuCallback(const sensor_msgs::Imu::ConstPtr& imu_msg)
{
    std::cout << "Linear acceleration: (x: " << imu_msg->linear_acceleration.x << " | y: " << imu_msg->linear_acceleration.y << " | z: " << imu_msg->linear_acceleration.z << ")" << std::endl;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "imu_test");
    ImuTest imu_test;
    ros::spin();

    return EXIT_SUCCESS;
}