#pragma once
#include "ros/ros.h"
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/PointCloud.h>
#include <deque>

enum struct State{
    IDLE,
    FIND_FLOOR,
    FIND_CEILING,
    NAVIGATE,
};


struct SpeedLimit{
    double vx;
    double vy_up;
    double vy_down;
};

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
    ros::Publisher m_pub_gate_upper;
    ros::Publisher m_pub_gate_lower;
    ros::NodeHandlePtr m_handle_ptr;

    bool isSimilarPointInCloud(const geometry_msgs::Point32& p);
    bool isPointGroundOrCeiling(const geometry_msgs::Point32& p);
    ros::Time m_prev_laser_update;
    std::deque<geometry_msgs::Point32> m_points;

    geometry_msgs::Vector3 m_prev_speed;
    ros::Time m_prev_speed_update_time;
    double m_vertical_offset_est{0.0};

    State m_state{State::IDLE};

    double m_floor_offset;
    double m_ceiling_offset;

    ros::Timer m_timer;
    void checkTimeouts(const ros::TimerEvent& e);


    SpeedLimit getSpeedLimit();
    bool detectPipe();


    double m_pipe_start{0.0};
    double m_pipe_end{0.0};
    double m_pipe_gap_start{0.0};
    double m_pipe_gap_end{0.0};
    bool m_pipe_detected{false};

    static constexpr auto m_dist_threshold = 0.05;
    static constexpr auto m_global_speed_limit = 0.8;
    static constexpr auto m_slow_speed = 0.1;
    static constexpr auto m_margin = 0.25;
    static constexpr auto m_nom_acc = 0.1;
    static constexpr auto m_max_acc = 0.3;
    static constexpr auto m_pipe_width = 0.45;

};
