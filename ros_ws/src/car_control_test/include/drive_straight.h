#pragma once

#include "car_control_test/Drive.h"
#include "car_control_test/TargetSpeed.h"
#include "drive_msgs/gazebo_state_telemetry.h"
#include "drive_msgs/drive_param.h"
#include <deque>
#include <iomanip>
#include <iostream>
#include <gazebo_msgs/LinkState.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float64.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wfloat-equal"
#pragma GCC diagnostic ignored "-Wdouble-promotion"
#pragma GCC diagnostic pop

constexpr const char* TOPIC_DRIVE_PARAM = "/commands/drive_param";
constexpr const char* TOPIC_ODOM = "/odom";
constexpr const char* TOPIC_GAZEBO_STATE_TELEMETRY = "/gazebo/state_telemetry";
constexpr const char* TOPIC_IMU = "/imu";

const static float CAR_ACCELERATION = 6;
const static float CAR_DECCELERATION = 5; 

struct Point
{
    float x;
    float y;
};

/**
 * @brief This Component gives the car the command
 * to drive straight forward.
 */
class StraightDrive
{
    public:
    StraightDrive();
    /**
     * @param speed speed in m/s
     */
    void publishSpeed(double speed);
    float m_speed = 0;

    private:
    ros::NodeHandle m_node_handle;
    ros::Publisher m_straightdrive_publisher;
    ros::ServiceServer m_drive_with_speed;
    ros::ServiceServer m_speed_after_distance;
    ros::Subscriber m_odom;
    ros::Subscriber m_gazebo_link_states;
    ros::Subscriber m_imu;

    float m_target_rpm = 0;
    float m_current_rpm = 0;

    float linear_acceleration_x;
    float m_current_speed;
    float m_gazebo_current_car_speed;

    Point m_car_pos;
    std::deque<float> m_moving_average_linear_acceleration_x;
    std::deque<float> m_moving_average_wheel_speed;

    bool drive_with_speed(car_control_test::Drive::Request& request, car_control_test::Drive::Response& response);

    bool speedAfterDistanceService(car_control_test::TargetSpeed::Request& request,
                                   car_control_test::TargetSpeed::Response& response);

    void gazeboStateTelemetryCallback(const drive_msgs::gazebo_state_telemetry::ConstPtr& state_telemetry);
    void imuCallback(const sensor_msgs::Imu::ConstPtr& imu_data);

    float calcMean(std::deque<float> moving_average)
    {
        float sum = 0;
        for (float& elem : moving_average)
        {
            sum += elem;
        }
        return sum / (float)moving_average.size();
    }

    /*
    current_speed in m/s
    target_speed in m/s
    distance in m
    acceleration in m/s^2
    */
    static float calcBreakingDistance(float current_speed, float target_speed, float distance, float acceleration,
                                            float decceleration)
    {
        acceleration = std::abs(acceleration);
        decceleration = std::abs(decceleration);
        return (distance * (acceleration / (acceleration + decceleration))) +
            ((current_speed * current_speed - target_speed * target_speed) / (2 * acceleration + 2 * decceleration));
    }

    static float calcMaxVelocity(float current_speed, float target_speed, float distance, float acceleration, float decceleration) {
        return std::sqrt((2 * distance * acceleration * decceleration + current_speed * current_speed * decceleration + target_speed * target_speed * acceleration) / (acceleration + decceleration));
    }

    static float calcAccelerationTime(float current_speed, float target_speed, float distance, float acceleration, float decceleration) {
        return (calcMaxVelocity(current_speed, target_speed, distance, acceleration, decceleration) - current_speed) / acceleration;
    }

    static float calcDeccelerationTime(float current_speed, float target_speed, float distance, float acceleration, float decceleration) {
        return (calcMaxVelocity(current_speed, target_speed, distance, acceleration, decceleration) - target_speed) / decceleration;
    }

    static float calcTimeLimit(float current_speed, float target_speed, float distance, float acceleration, float decceleration)
    {
        float max_velocity = calcMaxVelocity(current_speed, target_speed, distance, acceleration, decceleration);
        float time_1 = (max_velocity - current_speed) / acceleration;
        float time_2 = (max_velocity - target_speed) / decceleration;
        return time_1 + time_2;
    }

    static float calcAccelerationSpeed(float starting_speed, float target_speed, float remaining_distance, float distance, float acceleration, float decceleration) {
        return acceleration*calcAccelerationTime(starting_speed, target_speed, distance-remaining_distance, acceleration, decceleration)+starting_speed;
    }

    static float calcDeccelerationSpeed(float starting_speed, float target_speed, float remaining_distance, float acceleration, float decceleration) {
        return acceleration*calcDeccelerationTime(starting_speed, target_speed, remaining_distance, acceleration, decceleration)+target_speed;
    }
};