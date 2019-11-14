#include "drive_straight.h"
#include <ros/ros.h>

StraightDrive::StraightDrive()
{
    
    this->m_straightdrive_publisher = m_node_handle.advertise<std_msgs::Float64>(TOPIC_FOCBOX_SPEED, 1);
}

void StraightDrive::publishSpeed(double speed)
{
    std_msgs::Float64 speed_message;
    speed_message.data = speed;
    this->m_straightdrive_publisher.publish(speed_message);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "straight_drive");
    StraightDrive drive_straight;
    while(ros::ok()){
        drive_straight.publishSpeed(1216.0);
    }

    return EXIT_SUCCESS;
}