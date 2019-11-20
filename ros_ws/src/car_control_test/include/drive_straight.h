#pragma once

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include "car_control_test/Drive.h"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wfloat-equal"
#pragma GCC diagnostic ignored "-Wdouble-promotion"
#pragma GCC diagnostic pop

constexpr const char* TOPIC_FOCBOX_SPEED = "/commands/motor/speed";


/**
 * @brief This Component gives the car the command
 * to drive straight forward. 
 */
class StraightDrive
{
    public:
    StraightDrive();
    void publishSpeed(double speed);
    float m_speed = 0;

    private:
    ros::NodeHandle m_node_handle;
    ros::Publisher m_straightdrive_publisher;
    ros::ServiceServer m_drive_with_speed;

    bool drive_with_speed(car_control_test::Drive::Request &request, car_control_test::Drive::Response &response);
};