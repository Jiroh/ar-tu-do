#include "drive_straight.h"

StraightDrive::StraightDrive()
{
    
    this->m_straightdrive_publisher = m_node_handle.advertise<std_msgs::Float64>(TOPIC_FOCBOX_SPEED, 1);
    this->m_drive_with_speed = m_node_handle.advertiseService("drive_with_speed", &StraightDrive::drive_with_speed, this);
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