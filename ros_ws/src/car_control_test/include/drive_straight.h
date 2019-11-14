#pragma once

#include <ros/ros.h>
#include <std_msgs/Float64.h>

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

    private:
    ros::NodeHandle m_node_handle;
    ros::Publisher m_straightdrive_publisher;

      /**
     * @brief takes speed and publishes it to gazebo/focbox
     */
   
    
};