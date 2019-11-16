#include "drive_straight.h"
#include <ros/ros.h>

StraightDrive::StraightDrive()
{
    
    this->m_straightdrive_publisher = m_node_handle.advertise<std_msgs::Float64>(TOPIC_FOCBOX_SPEED, 1);
    this->m_straightdrive_forward = m_node_handle.advertiseService("drive_backwards", drive_backwards, this);
    this->m_straightdrive_forward = m_node_handle.advertiseService("drive_forwards", drive_forwards, this);
}

void StraightDrive::publishSpeed(double speed)
{
    std_msgs::Float64 speed_message;
    speed_message.data = speed;
    this->m_straightdrive_publisher.publish(speed_message);
}

bool drive_forwards(drive_srv::drive::Request& request, drive_srv::drive::Response& response){
    std::cout<< "Forwards" << std::endl;
}

bool drive_backwards(drive_srv::drive::Request& request, drive_srv::drive::Response& response){
    std::cout<< "Backwards" << std::endl;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "straight_drive");
    StraightDrive drive_straight;
    // while(ros::ok()){
    //     ros::spinOnce();
    //     drive_straight.publishSpeed(1216.0);
    // }

    drive_straight.publishSpeed(1216.0);
    ros::spin();

    return EXIT_SUCCESS;
}