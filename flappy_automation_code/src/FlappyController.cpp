#include <flappy_automation_code/FlappyController.hpp>
#include <flappy_automation_code/Hermite3.hpp>
#include <flappy_automation_code/PathPlanner.hpp>
#include <laser_geometry/laser_geometry.h>
#include <geometry_msgs/PolygonStamped.h>
#include <nav_msgs/Path.h>


FlappyController::FlappyController(ros::NodeHandlePtr h) :
    m_sub_vel{
      h->subscribe<geometry_msgs::Vector3>("/flappy_vel", 1, &FlappyController::velCallback, this)},
    m_sub_laser_scan{
      h->subscribe<sensor_msgs::LaserScan>("/flappy_laser_scan", 1, &FlappyController::laserScanCallback, this)},
    m_pub_acc_cmd{
      h->advertise<geometry_msgs::Vector3>("/flappy_acc", 1)},
    m_pub_point_cloud{
      h->advertise<sensor_msgs::PointCloud>("/flappy_point_cloud", 1)},
    m_pub_first_gate_upper{h->advertise<geometry_msgs::PolygonStamped>("/flappy_first_gate_upper", 1)},
    m_pub_first_gate_lower{h->advertise<geometry_msgs::PolygonStamped>("/flappy_first_gate_lower", 1)},
    m_pub_second_gate_upper{h->advertise<geometry_msgs::PolygonStamped>("/flappy_second_gate_upper", 1)},
    m_pub_second_gate_lower{h->advertise<geometry_msgs::PolygonStamped>("/flappy_second_gate_lower", 1)},
    m_pub_path{h->advertise<nav_msgs::Path>("/flappy_path", 1)},
    m_params{initParams(h)},
    m_path_planner(m_params),
    m_perception{m_params}
{
    timeout_check_timer = h->createTimer(
        ros::Duration(0.1),
        &FlappyController::checkTimeouts,
        this,
        false, // oneshot
        false  // autostart
    );
}

void FlappyController::velCallback(const geometry_msgs::Vector3::ConstPtr& msg) {
    m_current_speed = *msg;
    const auto current_time = ros::Time::now();
    const auto delta = (current_time - m_prev_speed_update_time).toSec();
    if (m_state == ControllerState::IDLE){
        m_path_planner.reset();
        m_perception.reset();
        publishVisuals();
        timeout_check_timer.start();
        m_state = ControllerState::FIND_FLOOR;
        ROS_INFO("Starting up. First lets find the floor.");
    } else {
        updateState(delta);
        publishVisuals();
        publishCommand(delta);
    }
    m_prev_speed_update_time = current_time;
    m_prev_speed = m_current_speed;
}


void FlappyController::laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    m_prev_laser_update = ros::Time::now();
    if (m_state == ControllerState::IDLE)
        return;

    // Convert rays to points
    auto point_cloud_msg = sensor_msgs::PointCloud{};
    laser_geometry::LaserProjection{}.projectLaser(*msg, point_cloud_msg);

    std::vector<Vector2d> new_points;
    new_points.reserve(point_cloud_msg.points.size());
    std::transform(
        point_cloud_msg.points.begin(),
        point_cloud_msg.points.end(),
        std::back_inserter(new_points), [](const auto& p){
            return Vector2d{p.x, p.y};
        });
    m_perception.addPoints(new_points);
}


void FlappyController::checkTimeouts(const ros::TimerEvent& e){
    const auto now = ros::Time::now();
    if ((now - m_prev_speed_update_time).toSec() > 0.2){
        ROS_ERROR("Timeout on speed update. Switching to idle.");
        m_state = ControllerState::IDLE;
        timeout_check_timer.stop();
        return;
    }
    if ((now - m_prev_laser_update).toSec() > 0.2){
        ROS_ERROR("Timeout on laser scan update. Switching to idle.");
        m_state = ControllerState::IDLE;
        timeout_check_timer.stop();
        return;
    }
}

double FlappyController::maxSpeedFromDistance(double distance){
    const auto v_sq = std::max(0.0, 2.0 * m_params.m_nom_acc * distance);
    return std::sqrt(v_sq);
}


void FlappyController::updateState(double delta_sec){

    const auto dx = (m_prev_speed.x + m_current_speed.x) * 0.5 * delta_sec;
    const auto dy = (m_prev_speed.y + m_current_speed.y) * 0.5 * delta_sec;
    m_perception.shift({-dx, -dy});

    const auto floor_offset = m_perception.getFloorOffset();
    if (m_state == ControllerState::FIND_FLOOR and floor_offset.has_value()){
        m_state = ControllerState::FIND_CEILING;
        ROS_INFO("Found floor at %f. Searching for ceiling now.", *floor_offset);
    }

    const auto ceiling_offset = m_perception.getCeilingOffset();
    if (m_state == ControllerState::FIND_CEILING and ceiling_offset.has_value()) {
        ROS_INFO("Found ceiling at %f. Starting navigation.", *ceiling_offset);
        m_perception.clearPoints();
        m_state = ControllerState::NAVIGATE;
    }
    if (m_state == ControllerState::NAVIGATE){
        const auto current_speed = Vector2d{
                m_current_speed.x,
                m_current_speed.y
        };
        m_path_planner.plan(current_speed, m_perception);
        m_delta_sec_avg = 0.8 * m_delta_sec_avg + 0.2 * delta_sec;
    }
}

void FlappyController::publishVisuals() {


    const auto floor_offset = m_perception.getFloorOffset();
    const auto ceiling_offset = m_perception.getCeilingOffset();
    if (floor_offset.has_value() and ceiling_offset.has_value()){

        const auto publish_pipe_polygons = [&](const auto& pipe, auto& upper_pub, auto& lower_pub){
            geometry_msgs::PolygonStamped upper_poly_msg;
            upper_poly_msg.header.frame_id = "world";
            geometry_msgs::PolygonStamped lower_poly_msg;
            lower_poly_msg.header.frame_id = "world";
            {
                auto& points = upper_poly_msg.polygon.points;
                points.resize(4);
                points[0].x = pipe.m_end;
                points[0].y = *ceiling_offset;
                points[1].x = pipe.m_start;
                points[1].y = *ceiling_offset;
                points[2].x = pipe.m_start;
                points[2].y = pipe.m_gap_end;
                points[3].x = pipe.m_end;
                points[3].y = pipe.m_gap_end;
            }
            {
                auto &points = lower_poly_msg.polygon.points;
                points.resize(4);
                points[0].x = pipe.m_end;
                points[0].y = pipe.m_gap_start;
                points[1].x = pipe.m_start;
                points[1].y = pipe.m_gap_start;
                points[2].x = pipe.m_start;
                points[2].y = *floor_offset;
                points[3].x = pipe.m_end;
                points[3].y = *floor_offset;
            }
            upper_pub.publish(upper_poly_msg);
            lower_pub.publish(lower_poly_msg);
        };

        const auto publish_empty_polygons = [](auto& upper_pub, auto& lower_pub){
            geometry_msgs::PolygonStamped empty_poly;
            empty_poly.header.frame_id = "world";
            upper_pub.publish(empty_poly);
            lower_pub.publish(empty_poly);
        };

        const auto& pipes = m_perception.getDetectedPipes();
        if (not pipes.empty()){
            publish_pipe_polygons(pipes[0], m_pub_first_gate_upper, m_pub_first_gate_lower);
        } else {
            publish_empty_polygons(m_pub_first_gate_upper, m_pub_first_gate_lower);
        }
        if (pipes.size() > 1){
            publish_pipe_polygons(pipes[1], m_pub_second_gate_upper, m_pub_second_gate_lower);
        } else {
            publish_empty_polygons(m_pub_second_gate_upper, m_pub_second_gate_lower);
        }
    }

    auto point_cloud_msg = sensor_msgs::PointCloud{};
    point_cloud_msg.header.frame_id = "world";
    const auto& points = m_perception.getPoints();
    std::transform(
        points.begin(),
        points.end(),
        std::back_inserter(point_cloud_msg.points),
        [](const auto& p){
            auto msg = geometry_msgs::Point32{};
            msg.x = p.x;
            msg.y = p.y;
            return msg;
        });
    m_pub_point_cloud.publish(point_cloud_msg);

    const auto path = m_path_planner.getPath();
    auto path_msg = nav_msgs::Path{};
    path_msg.header.frame_id = "world";
    std::transform(path.begin(), path.end(), std::back_inserter(path_msg.poses), [](const auto& p){
        auto pose_msg = geometry_msgs::PoseStamped{};
        pose_msg.pose.position.x = p.m_pos.x;
        pose_msg.pose.position.y = p.m_pos.y;
        return pose_msg;
    });
    m_pub_path.publish(path_msg);
}


void FlappyController::publishCommand(double delta_sec){
    // Compute desired speed as a function of the current state
    auto speedWanted = Vector2d{0.0, 0.0};
    if (m_state == ControllerState::FIND_FLOOR) {
        speedWanted.x = 0.0;
        speedWanted.y = -m_params.m_slow_speed;
    } else if (m_state == ControllerState::FIND_CEILING){
        speedWanted.x = 0.0;
        speedWanted.y = m_params.m_slow_speed;
    } else if (m_state == ControllerState::NAVIGATE){
        const auto target_pos = getTargetPos();
        speedWanted.x = std::min(m_params.m_max_speed, maxSpeedFromDistance(target_pos.x));
        speedWanted.y = std::min(m_params.m_max_speed, maxSpeedFromDistance(std::fabs(target_pos.y)));
        speedWanted.y = (target_pos.y > 0)? speedWanted.y : -speedWanted.y;
    }

    geometry_msgs::Vector3 acc_cmd;
    acc_cmd.x = (speedWanted.x - m_current_speed.x) / m_delta_sec_avg;
    acc_cmd.y = (speedWanted.y - m_current_speed.y) / m_delta_sec_avg;
    acc_cmd.x = std::clamp(acc_cmd.x, -m_params.m_max_acc, m_params.m_max_acc);
    acc_cmd.y = std::clamp(acc_cmd.y, -m_params.m_max_acc, m_params.m_max_acc);
    m_pub_acc_cmd.publish(acc_cmd);
}

NodeParams FlappyController::initParams(ros::NodeHandlePtr node_handle) const {
    auto res = NodeParams{};
    node_handle->getParam("/dist_threshold", res.m_dist_threshold);
    node_handle->getParam("/slow_speed", res.m_slow_speed);
    node_handle->getParam("/margin_y", res.m_margin_y);
    node_handle->getParam("/margin_x", res.m_margin_x);
    node_handle->getParam("/nom_acc", res.m_nom_acc);
    node_handle->getParam("/max_acc", res.m_max_acc);
    node_handle->getParam("/max_view_distance", res.m_max_view_distance);
    node_handle->getParam("/max_speed", res.m_max_speed);
    return res;
}

Vector2d FlappyController::getTargetPos() const {
    auto res = Vector2d{
            m_params.m_max_view_distance,
        0.0
    };

    const auto& pipes = m_perception.getDetectedPipes();
    if (not pipes.empty()){
        const auto& pipe = pipes.front();
        auto relative_pos = pipe.getRelativePosition(m_params.m_margin_x);
        if ( relative_pos == RelativePosition::Before and not pipe.isAlignedWithGap(m_params.m_margin_y)){
            res.x = std::min(res.x, pipe.m_start - m_params.m_margin_x);
        }
        if (relative_pos != RelativePosition::After){
            res.y = (pipe.m_gap_start + pipe.m_gap_end) * 0.5;
        }
        if (relative_pos == RelativePosition::Inside and not pipe.isAlignedWithGap(m_params.m_margin_y)) {
            res.x = 0.0;
        }
    }

    auto next_pipe_start =  m_perception.getNextPipeStart() ;
    if (next_pipe_start.has_value())
        res.x = std::min(res.x, *next_pipe_start - m_params.m_margin_x);

    return res;
}