#include "laser_scan.h"
#include <laser_geometry/laser_geometry.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>

LaserScan::LaserScan()
{
    m_laserscan_subscriber = m_node_handle.subscribe<sensor_msgs::LaserScan>(TOPIC_LASER_SCAN, 100,
                                                                             &LaserScan::scanCallback, this);
    m_pointcloud_publisher = m_node_handle.advertise<sensor_msgs::PointCloud2>(TOPIC_LASER_SCAN_POINTCLOUD, 100, false);
}

void LaserScan::scanCallback(const sensor_msgs::LaserScan::ConstPtr& laserscan)
{
    sensor_msgs::PointCloud pointcloud;
    
    m_projector.transformLaserScanToPointCloud(MODEL_BASE_LINK, *laserscan, pointcloud, m_listener);
    //m_pointcloud_publisher.publish(pointcloud);

    int i = 0;
    
    for (auto& range : pointcloud.points)
    {
        std::cout << laserscan->scan_time <<std::endl;
        std::cout << "Punkt: "<<i<< "Koordinate ["<< range.x<<","<<range.y << ","<< range.z<<"]"<< std::endl;
        i++;
    }
    
   
    

    

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "laser_scan");
    LaserScan tf_laserscan_to_pointcloud;
    ros::spin();

    return EXIT_SUCCESS;
}