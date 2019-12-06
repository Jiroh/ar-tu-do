#include "drive_straight.h"
#include <iomanip>
#include <iostream>

StraightDrive::StraightDrive()
{
    this->m_straightdrive_publisher = m_node_handle.advertise<std_msgs::Float64>(TOPIC_FOCBOX_SPEED, 1);
    this->m_drive_with_speed =
        m_node_handle.advertiseService("drive_with_speed", &StraightDrive::drive_with_speed, this);
    this->m_speed_after_distance =
        m_node_handle.advertiseService("speed_after_distance", &StraightDrive::speedAfterDistanceService, this);
    this->m_gazebo_link_states = this->m_node_handle.subscribe(TOPIC_GAZEBO_STATE_TELEMETRY, 1,
                                                               &StraightDrive::gazeboStateTelemetryCallback, this);
    this->m_imu = this->m_node_handle.subscribe(TOPIC_IMU, 1, &StraightDrive::imuCallback, this);
}

void StraightDrive::publishSpeed(double speed)
{
    std_msgs::Float64 speed_message;
    speed_message.data = speed;
    this->m_straightdrive_publisher.publish(speed_message);
}

bool StraightDrive::drive_with_speed(car_control_test::Drive::Request& request,
                                     car_control_test::Drive::Response& response)
{
    response.actualSpeed = calcMean(m_moving_average_wheel_speed);
    m_speed = request.speed;
    return true;
}

/*
current_speed in m/s
target_speed in m/s
distance in m
acceleration in m/s^2
*/
float StraightDrive::calcBreakingDistance(float current_speed, float target_speed, float distance, float acceleration,
                                          float decceleration)
{
    acceleration = std::abs(acceleration);
    decceleration = std::abs(decceleration);
    return (distance * (acceleration / (acceleration + decceleration))) +
        ((current_speed * current_speed - target_speed * target_speed) / (2 * acceleration + 2 * decceleration));
}

float StraightDrive::calcMaxSpeed(float current_speed, float target_speed, float distance, float acceleration,
                                  float decceleration)
{
    return std::sqrt((2 * distance * acceleration * decceleration + current_speed * current_speed * decceleration +
                      target_speed * target_speed * acceleration) /
                     (acceleration + decceleration));
}

bool StraightDrive::speedAfterDistanceService(car_control_test::TargetSpeed::Request& request,
                                              car_control_test::TargetSpeed::Response& response)
{
    float braking_distance =
        calcBreakingDistance(calcMean(m_moving_average_wheel_speed), request.target_speed, request.distance, 8, 4.5);
    response.breaking_distance = braking_distance;
    response.braking_possible = braking_distance < request.distance;
    response.current_speed = calcMean(m_moving_average_wheel_speed);

    float safety_margin = 0.05 * request.distance;

    float remain_distance = request.distance - safety_margin;

    float old_speed = 0;

    std::cout << "lowest_possible_time: "
              << calcTimeLimit(calcMean(m_moving_average_wheel_speed), request.target_speed, request.distance, 8, 4.5)
              << std::endl;
    std::cout << "______________________________________________________________________________________" << std::endl;

    float total_time = 0;
    ros::Time time_old = ros::Time::now();
    ros::Time current_time;
    float delta_time = 0;

    float current_acceleration = 0;
    float current_acceleration_gazebo = 0;

    float rpm;

    Point car_pos_start = m_car_pos;
    while (calcMean(m_moving_average_wheel_speed) > request.target_speed * 1.1)
    {
        current_time = ros::Time::now();
        delta_time = (current_time.toNSec() - time_old.toNSec()) / 1e9;
        total_time += delta_time;
        time_old = current_time;
        if (old_speed != 0 && delta_time != 0 && calcMean(m_moving_average_wheel_speed) != old_speed)
        {
            current_acceleration = calcMean(m_moving_average_linear_acceleration_x);
            current_acceleration_gazebo = (calcMean(m_moving_average_wheel_speed) - old_speed) / delta_time;

            std::cout << "current_speed: " << calcMean(m_moving_average_wheel_speed) << ", old_speed: " << old_speed
                      << ", delta_time: " << std::setprecision(9) << delta_time << std::endl;
            remain_distance =
                remain_distance - (current_acceleration / 2 * (delta_time * delta_time)) - old_speed * delta_time;
            old_speed = calcMean(m_moving_average_wheel_speed);

            braking_distance = calcBreakingDistance(calcMean(m_moving_average_wheel_speed), request.target_speed,
                                                    remain_distance, 8, 4.5);
            std::cout << "accel: " << current_acceleration << ", r_dist: " << remain_distance + safety_margin
                      << ", b_dist:" << braking_distance << std::endl;
            std::cout << "gazebo_car_speed: " << m_gazebo_current_car_speed
                      << ", gazebo_acceleration: " << current_acceleration_gazebo << std::endl;
            std::cout << "----------------------------------------------------------------------" << std::endl;

            rpm = convertSpeedToRpm(calcMean(m_moving_average_wheel_speed));
            publishSpeed(std::max<float>(0.0,
                                         rpm +
                                             10000 *
                                                 ((remain_distance - braking_distance) /
                                                  (std::abs(remain_distance) + std::abs(braking_distance)))));
        }
        else if (old_speed == 0)
        {
            old_speed = calcMean(m_moving_average_wheel_speed);
        }
        for (size_t i = 0; i < 10; i++)
        {
            ros::spinOnce();
            ros::Duration(0.01).sleep();
        }
    }
    m_speed = convertSpeedToRpm(request.target_speed);
    total_time += delta_time;
    remain_distance = remain_distance - (current_acceleration / 2 * (delta_time * delta_time)) - old_speed * delta_time;
    float travelled_distance = std::sqrt((car_pos_start.x - m_car_pos.x) * (car_pos_start.x - m_car_pos.x) +
                                         (car_pos_start.y - m_car_pos.y) * (car_pos_start.y - m_car_pos.y));
    std::cout << "travelled_distance_gazebo: " << travelled_distance
              << ", travelled_distance_imu: " << request.distance - remain_distance - safety_margin << std::endl;
    std::cout << "total_time: " << total_time << std::endl;
    std::cout << "______________________________________________________________________________________" << std::endl;

    return true;
}

// void testBreakingPoint(float target_speed, float distance)
// {
//     float breaking_distance = calcBreakingDistance(current_speed, target_speed, distance, 10.0, 10.0);
//     // while () {

//     // }
// }

void StraightDrive::gazeboStateTelemetryCallback(const drive_msgs::gazebo_state_telemetry::ConstPtr& state_telemetry)
{
    m_gazebo_current_car_speed = state_telemetry->car_speed;
    m_moving_average_wheel_speed.push_back(state_telemetry->wheel_speed);
    if (m_moving_average_wheel_speed.size() > 3)
    {
        m_moving_average_wheel_speed.pop_front();
    }
    m_car_pos.x = state_telemetry->point_x;
    m_car_pos.y = state_telemetry->point_y;
}

void StraightDrive::imuCallback(const sensor_msgs::Imu::ConstPtr& imu_data)
{
    m_moving_average_linear_acceleration_x.push_back(imu_data->linear_acceleration.x);
    if (m_moving_average_linear_acceleration_x.size() > 5)
    {
        m_moving_average_linear_acceleration_x.pop_front();
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "drive_straight");
    StraightDrive drive_straight;
    while (ros::ok())
    {
        ros::spinOnce();
        drive_straight.publishSpeed(drive_straight.m_speed);
    }

    ros::spin();

    return EXIT_SUCCESS;
}