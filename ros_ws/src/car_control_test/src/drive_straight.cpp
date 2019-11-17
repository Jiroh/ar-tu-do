#include "drive_straight.h"

StraightDrive::StraightDrive()
{
    
    this->m_straightdrive_publisher = m_node_handle.advertise<std_msgs::Float64>(TOPIC_FOCBOX_SPEED, 1);
    this->m_straightdrive_forward = m_node_handle.advertiseService("drive_backwards", &StraightDrive::drive_backwards, this);
    this->m_straightdrive_forward = m_node_handle.advertiseService("drive_forwards", &StraightDrive::drive_forwards, this);
}

void StraightDrive::publishSpeed(double speed)
{
    std_msgs::Float64 speed_message;
    speed_message.data = speed;
    this->m_straightdrive_publisher.publish(speed_message);
}

bool StraightDrive::drive_forwards(car_control_test::Drive::Request &request, car_control_test::Drive::Response &response)
{
    std::cout<< "Forwards" << std::endl;
    return true;
}

bool StraightDrive::drive_backwards(car_control_test::Drive::Request &request, car_control_test::Drive::Response &response)
{
    std::cout<< "Backwards" << std::endl;
    return true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "drive_straight");
    StraightDrive drive_straight;
    // while(ros::ok()){
    //     ros::spinOnce();
    //     drive_straight.publishSpeed(1216.0);
    // }

    ros::spin();

    return EXIT_SUCCESS;
}