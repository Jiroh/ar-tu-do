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
    ros::ServiceServer m_straightdrive_forward;
    ros::ServiceServer m_straightdrive_backward;

    bool drive_backwards(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
    bool drive_forwards(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)

      /**
     * @brief takes speed and publishes it to gazebo/focbox
     */
   
    
};