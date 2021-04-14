#pragma once
#include "ros/ros.h"
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/PointCloud.h>
#include <deque>


class FlappyController {
public:

    explicit FlappyController(ros::NodeHandlePtr h);
    void velCallback(const geometry_msgs::Vector3::ConstPtr& msg);
    void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);

private:
    ros::Subscriber m_sub_vel;
    ros::Subscriber m_sub_laser_scan;
    ros::Publisher m_pub_acc_cmd;
    ros::Publisher m_pub_point_cloud;
    ros::NodeHandlePtr m_handle_ptr;

    bool isPointGroundOrCeiling(const geometry_msgs::Point32& p);

    std::deque<geometry_msgs::Point32> m_points;
    geometry_msgs::Vector3 m_prev_speed;
    ros::Time m_prev_time;
    double m_vertical_offset_est;

};
