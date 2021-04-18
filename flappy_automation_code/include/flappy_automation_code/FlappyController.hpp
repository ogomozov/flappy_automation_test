#pragma once

#include <flappy_automation_code/Common.hpp>
#include <flappy_automation_code/SimplePerception.hpp>
#include <flappy_automation_code/PathPlanner.hpp>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/PointCloud.h>
#include <deque>


enum struct ControllerState{
    IDLE,
    FIND_FLOOR,
    FIND_CEILING,
    NAVIGATE,
};


struct SpeedLimit{
    double x_limit;
    double y_up_limit;
    double y_down_limit;
    double vx;
    double vy_up;
    double vy_down;
};


/** Main controller class
 *
 * Reads the speed and laser scan input and publishes the acceleration
 * command to guide the bird throught a corridor filled with "pipe" gates.
 *
 * Once started, the controller will try to detect the floor and ceiling of
 * the corridor by moving up and down and observing the bounds on the laser scan
 * readings.
 *
 * When both floor and ceiling are detected the controller will start to navigate
 * the corridor. There are two components to the navigation: horizontal speed control
 * and vertical speed control. The horizontal speed is controlled in such a way that
 * the bird advance to the right (positive x-axis) while avoiding possible collision
 * with pipes. The vertical speed control is done to aligne the bird with the gap in
 * the closest pipe gap. The anticollision is done by limiting the speed in such a
 * way that the stopping distance is always shorter than the distance from the bird to
 * the closest obstacle minus bird's half size and some security margin.
 *
 * To detect pipes a point cloud is accumulated from the laser scan. Then a simple AABB
 * clastering is done and clustered points are sorted along y-axis. Two consecutive
 * points with y-distance larger than the bird size plus some security margin are
 * considered a potential gap. The candidate closest to the bird position along
 * y-axis is chosen as the detected gap.
 *
 */
class FlappyController {
public:

    explicit FlappyController(ros::NodeHandlePtr h);

private:

    /*============== ROS interface ==============*/
    ros::Subscriber m_sub_vel;
    ros::Subscriber m_sub_laser_scan;
    ros::Publisher m_pub_acc_cmd;
    ros::Publisher m_pub_point_cloud;
    ros::Publisher m_pub_first_gate_upper;
    ros::Publisher m_pub_first_gate_lower;
    ros::Publisher m_pub_second_gate_upper;
    ros::Publisher m_pub_second_gate_lower;
    ros::Publisher m_pub_path;
    ros::NodeHandlePtr m_handle_ptr;

    /* Timer to check the timeouts on speed and laser scan readings. */
    ros::Timer timeout_check_timer;
    ros::Time m_prev_laser_update;
    ros::Time m_prev_speed_update_time;

    /*============== Parameters ==============*/
    //  Static parameters.
    // TODO move to ros node parameters.
    static constexpr auto m_dist_threshold = 0.05;
    static constexpr auto m_slow_speed = 0.5;
    static constexpr auto m_margin_y = 0.22;
    static constexpr auto m_margin_x = 0.3;
    static constexpr auto m_nom_acc = 2.5;
    static constexpr auto m_max_acc = 3.0;
    static constexpr auto m_max_view_distance = 5.0;
    static constexpr auto m_max_speed = 2.5;

    /*============== State variables ==============*/
    // State of the controller.
    ControllerState m_state{ControllerState::IDLE};

    // Measured speed
    geometry_msgs::Vector3 m_current_speed{};
    geometry_msgs::Vector3 m_prev_speed{};

    // Measured time step
    // TODO add actual measurment to the velCallback
    double m_delta_sec_avg{1.0/30.0};

    PathPlanner m_path_planner;
    SimplePerception m_perception;

    /*============== Methods ==============*/

    static constexpr NodeParams initParams();

    // Changes the state of the controller and perceived variables
    // due to perception and position change of the bird.
    void updateState(double delta_sec);

    // Publishes the visual representation of the perceived value to
    // m_pub_point_cloud, m_pub_gate_upper and m_pub_gate_lower.
    // Published topics are used in rviz.
    void publishVisuals();

    // Computes and published the command depending on the current state
    // and the perception.
    void publishCommand(double delta_sec);

    // Gets the position at wich we want to stop due to
    // gate tracking and anitcollision.
    Vector2d getTargetPos() const;

    // Returns maximum speed from which we can stop at distance with
    // m_nom_acc deceleration.
    double maxSpeedFromDistance(double distance);

    /*============== Callbacks ==============*/
    // Callback on the velocity update. The main callback where the
    // command is computed and published.
    void velCallback(const geometry_msgs::Vector3::ConstPtr& msg);

    // Callback on the laser scan. Populates the point cloud and
    // performs detection.
    void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);

    // Callback on the timeout timer. Checks the last time the
    // velocity and laser scan was received and switches state to IDLE if
    // there is a timeout.
    void checkTimeouts(const ros::TimerEvent& e);

};
