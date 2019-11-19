#pragma once

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wfloat-equal"
#pragma GCC diagnostic ignored "-Wdouble-promotion"
#include <laser_geometry/laser_geometry.h>
#pragma GCC diagnostic pop

constexpr const char* TOPIC_IMU = "/imu";

/**
 * @brief This converter class converts a 2D laser scan
 * as defined by sensor_msgs/LaserScan into a point
 * cloud as defined by sensor_msgs/PointCloud2.
 *
 * The main purpose of this class is the transformation
 * of the LaserScan to the "base_link" of the racer model.
 */
class ImuTest
{
    public:
    ImuTest();

    private:
    ros::NodeHandle m_node_handle;
    ros::Subscriber m_imu_subscriber;

    void imuCallback(const sensor_msgs::Imu::ConstPtr& imu_msg);
};