#pragma once

#include "../../simulation/vesc_sim/include/car_config.h"
#include "car_control_test/Drive.h"
#include "car_control_test/TargetSpeed.h"
#include "drive_msgs/gazebo_state_telemetry.h"
#include <deque>
#include <gazebo_msgs/LinkState.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float64.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wfloat-equal"
#pragma GCC diagnostic ignored "-Wdouble-promotion"
#pragma GCC diagnostic pop

constexpr const char* TOPIC_FOCBOX_SPEED = "/commands/motor/speed";
constexpr const char* TOPIC_ODOM = "/odom";
constexpr const char* TOPIC_GAZEBO_STATE_TELEMETRY = "/gazebo/state_telemetry";
constexpr const char* TOPIC_IMU = "/imu";

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

    float linear_acceleration_x;
    float m_gazebo_current_car_speed;
    Point m_car_pos;
    std::deque<float> m_moving_average_linear_acceleration_x;
    std::deque<float> m_moving_average_wheel_speed;

    bool drive_with_speed(car_control_test::Drive::Request& request, car_control_test::Drive::Response& response);

    bool speedAfterDistanceService(car_control_test::TargetSpeed::Request& request,
                                   car_control_test::TargetSpeed::Response& response);
    float calcBreakingDistance(float current_speed, float target_speed, float distance, float acceleration,
                               float decceleration);
    float calcMaxSpeed(float current_speed, float target_speed, float distance, float acceleration,
                       float decceleration);

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
    speed: m/s
    */
    int convertSpeedToRpm(float speed)
    {
        return speed * car_config::TRANSMISSION / car_config::ERPM_TO_SPEED;
    }

    float calcTimeLimit(float current_speed, float target_speed, float distance, float acceleration,
                        float decceleration)
    {
        float max_velocity =
            std::sqrt((2 * distance * acceleration * decceleration + current_speed * current_speed * decceleration +
                       target_speed * target_speed * acceleration) /
                      (acceleration + decceleration));
        float time_1 = (max_velocity - current_speed) / acceleration;
        float time_2 = (max_velocity - target_speed) / decceleration;
        return time_1 + time_2;
    }
};