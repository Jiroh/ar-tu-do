#pragma once

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>
#include "car_control_test/Drive.h"
#include "car_control_test/TargetSpeed.h"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wfloat-equal"
#pragma GCC diagnostic ignored "-Wdouble-promotion"
#pragma GCC diagnostic pop

constexpr const char* TOPIC_FOCBOX_SPEED = "/commands/motor/speed";
constexpr const char* TOPIC_ODOM = "/odom";


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
    int count = 0;

    private:
    ros::NodeHandle m_node_handle;
    ros::Publisher m_straightdrive_publisher;
    ros::ServiceServer m_drive_with_speed;
    ros::ServiceServer m_speed_after_distance;
    ros::Subscriber m_odom;

    float current_speed;

    bool drive_with_speed(car_control_test::Drive::Request &request, car_control_test::Drive::Response &response);

    bool speedAfterDistanceService(car_control_test::TargetSpeed::Request &request, car_control_test::TargetSpeed::Response &response);
    float calcBreakingDistance(float current_speed, float target_speed, float distance, float acceleration);

    void odomCallback(const nav_msgs::Odometry::ConstPtr& odom);
};