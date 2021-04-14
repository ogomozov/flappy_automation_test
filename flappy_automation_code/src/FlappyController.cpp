#include <flappy_automation_code/FlappyController.hpp>
#include <laser_geometry/laser_geometry.h>


FlappyController::FlappyController(ros::NodeHandlePtr h) :
    m_sub_vel{
      h->subscribe<geometry_msgs::Vector3>("/flappy_vel", 1, &FlappyController::velCallback, this)},
    m_sub_laser_scan{
      h->subscribe<sensor_msgs::LaserScan>("/flappy_laser_scan", 1, &FlappyController::laserScanCallback, this)},
    m_pub_acc_cmd{
      h->advertise<geometry_msgs::Vector3>("/flappy_acc", 1)},
    m_pub_point_cloud{
      h->advertise<sensor_msgs::PointCloud>("/flappy_point_cloud", 1)},
    m_prev_time{ros::Time::now()},
    m_vertical_offset_est{0.0},
    m_handle_ptr{std::move(h)}
{}


void FlappyController::velCallback(const geometry_msgs::Vector3::ConstPtr& msg) {

    // Estimate how have we moved wrt the last frame
    const auto new_time = ros::Time::now();
    const auto& new_speed = *msg;
    const auto delta = (new_time - m_prev_time).toSec();

    const auto dx = (m_prev_speed.x + new_speed.x) * 0.5 * delta;
    const auto dy = (m_prev_speed.y + new_speed.y) * 0.5 * delta;

    m_prev_time = new_time;
    m_prev_speed = new_speed;

    // Shift the points in the opposite direction
    m_vertical_offset_est += dy;
    for (auto& point : m_points){
        point.x -= dx;
        point.y -= dy;
    }

    // Destroy points that we have passed. Since we are moving only from left to right
    // the points with smaller x coordinate are always in the beginning of the deque.
    while((not m_points.empty() and m_points.front().x < -0.2))
        m_points.pop_front();

    // Publish command
    geometry_msgs::Vector3 acc_cmd;
    acc_cmd.x = 0;
    acc_cmd.y = 0;
    m_pub_acc_cmd.publish(acc_cmd);

    //TODO publish bird transform
}


void FlappyController::laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    // Convert rays to points
    auto point_cloud_msg = sensor_msgs::PointCloud{};
    laser_geometry::LaserProjection{}.projectLaser(*msg, point_cloud_msg);

    // Filter out points outside corridors bounds and add to the cloud
    // TODO do not add points in another point is at the same location
    auto& points = point_cloud_msg.points;
    std::copy_if(points.begin(), points.end(), std::back_inserter(m_points), [this](const auto& point){
        return not isPointGroundOrCeiling(point);
    });

    // Publish current cloud for debug
    point_cloud_msg.points.clear();
    std::copy(m_points.begin(), m_points.end(), std::back_inserter(point_cloud_msg.points));
    m_pub_point_cloud.publish(point_cloud_msg);
}


bool FlappyController::isPointGroundOrCeiling(const geometry_msgs::Point32& p) {
    // TODO make a proper ground detection
    return false;
};