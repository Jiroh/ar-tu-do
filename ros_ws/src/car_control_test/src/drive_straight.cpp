#include "drive_straight.h"

StraightDrive::StraightDrive()
{
    this->m_straightdrive_publisher = m_node_handle.advertise<std_msgs::Float64>(TOPIC_FOCBOX_SPEED, 1);
    this->m_drive_with_speed = m_node_handle.advertiseService("drive_with_speed", &StraightDrive::drive_with_speed, this);
    this->m_speed_after_distance = m_node_handle.advertiseService("speed_after_distance", &StraightDrive::speedAfterDistanceService, this);
    this->m_odom =
        this->m_node_handle.subscribe(TOPIC_ODOM, 1, &StraightDrive::odomCallback, this);
}

void StraightDrive::publishSpeed(double speed)
{
    std_msgs::Float64 speed_message;
    speed_message.data = speed;
    this->m_straightdrive_publisher.publish(speed_message);
}

bool StraightDrive::drive_with_speed(car_control_test::Drive::Request &request, car_control_test::Drive::Response &response)
{
    response.actualSpeed = m_speed;
    m_speed = request.speed;
    std::cout<< "Forwards" << std::endl;
    return true;
}

/*
current_speed in m/s
target_speed in m/s
distance in m
acceleration in m/s^2
*/
float StraightDrive::calcBreakingDistance(float current_speed, float target_speed, float distance, float acceleration) 
{
    return (distance/2) + ((current_speed*current_speed-target_speed*target_speed)/(4*acceleration));
}

bool StraightDrive::speedAfterDistanceService(car_control_test::TargetSpeed::Request &request, car_control_test::TargetSpeed::Response &response)
{

    float breaking_distance = calcBreakingDistance(current_speed, request.target_speed, request.distance, 10.0);
    response.breaking_distance = breaking_distance;
    response.braking_possible = breaking_distance < request.distance;
    response.current_speed = current_speed;
    return true;
}

void StraightDrive::odomCallback(const nav_msgs::Odometry::ConstPtr& odom) 
{
    current_speed = odom->twist.twist.linear.x;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "drive_straight");
    StraightDrive drive_straight;
    while(ros::ok()){
        ros::spinOnce();
        drive_straight.publishSpeed(drive_straight.m_speed);
    }

    ros::spin();

    return EXIT_SUCCESS;
}