#include <flappy_automation_code/FlappyController.hpp>
#include <laser_geometry/laser_geometry.h>
#include <geometry_msgs/PolygonStamped.h>


FlappyController::FlappyController(ros::NodeHandlePtr h) :
    m_sub_vel{
      h->subscribe<geometry_msgs::Vector3>("/flappy_vel", 1, &FlappyController::velCallback, this)},
    m_sub_laser_scan{
      h->subscribe<sensor_msgs::LaserScan>("/flappy_laser_scan", 1, &FlappyController::laserScanCallback, this)},
    m_pub_acc_cmd{
      h->advertise<geometry_msgs::Vector3>("/flappy_acc", 1)},
    m_pub_point_cloud{
      h->advertise<sensor_msgs::PointCloud>("/flappy_point_cloud", 1)},
    m_pub_gate_upper{h->advertise<geometry_msgs::PolygonStamped>("/flappy_gate_upper", 1)},
    m_pub_gate_lower{h->advertise<geometry_msgs::PolygonStamped>("/flappy_gate_lower", 1)},
    m_perception{initParams()},
    m_handle_ptr{std::move(h)}
{
    timeout_check_timer = m_handle_ptr->createTimer(
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
    const auto v_sq = std::max(0.0, 2.0 * m_nom_acc * distance);
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

    m_delta_sec_avg = 0.8 * m_delta_sec_avg + 0.2 * delta_sec;
}

void FlappyController::publishVisuals() {
    geometry_msgs::PolygonStamped upper_poly_msg;
    upper_poly_msg.header.frame_id = "world";
    geometry_msgs::PolygonStamped lower_poly_msg;
    lower_poly_msg.header.frame_id = "world";

    const auto pipe = m_perception.getFistPipe();
    const auto floor_offset = m_perception.getFloorOffset();
    const auto ceiling_offset = m_perception.getCeilingOffset();

    if (pipe.has_value() and floor_offset.has_value() and ceiling_offset.has_value()){
        {
            auto& points = upper_poly_msg.polygon.points;
            points.resize(4);
            points[0].x = (*pipe).m_end;
            points[0].y = *ceiling_offset;
            points[1].x = (*pipe).m_start;
            points[1].y = *ceiling_offset;
            points[2].x = (*pipe).m_start;
            points[2].y = (*pipe).m_gap_end;
            points[3].x = (*pipe).m_end;
            points[3].y = (*pipe).m_gap_end;
        }
        {
            auto &points = lower_poly_msg.polygon.points;
            points.resize(4);
            points[0].x = (*pipe).m_end;
            points[0].y = (*pipe).m_gap_start;
            points[1].x = (*pipe).m_start;
            points[1].y = (*pipe).m_gap_start;
            points[2].x = (*pipe).m_start;
            points[2].y = *floor_offset;
            points[3].x = (*pipe).m_end;
            points[3].y = *floor_offset;
        }
    }
    m_pub_gate_upper.publish(upper_poly_msg);
    m_pub_gate_lower.publish(lower_poly_msg);

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
}


void FlappyController::publishCommand(double delta_sec){
    // Compute desired speed as a function of the current state
    auto speedWanted = Vector2d{0.0, 0.0};
    if (m_state == ControllerState::FIND_FLOOR) {
        speedWanted.x = 0.0;
        speedWanted.y = -m_slow_speed;
    } else if (m_state == ControllerState::FIND_CEILING){
        speedWanted.x = 0.0;
        speedWanted.y = m_slow_speed;
    } else if (m_state == ControllerState::NAVIGATE){
        const auto target_pos = getTargetPos();
        speedWanted.x = std::min(m_max_speed, maxSpeedFromDistance(target_pos.x));
        speedWanted.y = std::min(m_max_speed, maxSpeedFromDistance(std::fabs(target_pos.y)));
        speedWanted.y = (target_pos.y > 0)? speedWanted.y : -speedWanted.y;
    }

    geometry_msgs::Vector3 acc_cmd;
    acc_cmd.x = (speedWanted.x - m_current_speed.x) / m_delta_sec_avg;
    acc_cmd.y = (speedWanted.y - m_current_speed.y) / m_delta_sec_avg;
    acc_cmd.x = std::clamp(acc_cmd.x, -m_max_acc, m_max_acc);
    acc_cmd.y = std::clamp(acc_cmd.y, -m_max_acc, m_max_acc);
    m_pub_acc_cmd.publish(acc_cmd);
    m_prev_acc = acc_cmd;
}

constexpr NodeParams FlappyController::initParams(){
    return NodeParams{
        m_dist_threshold,
        m_slow_speed,
        m_margin_y,
        m_margin_x,
        m_nom_acc,
        m_max_acc,
        m_max_view_distance
    };
}

Vector2d FlappyController::getTargetPos() const {
    auto res = Vector2d{
        m_max_view_distance,
        0.0
    };

    const auto pipe = m_perception.getFistPipe();
    if (pipe.has_value()){
        auto relative_pos = (*pipe).getRelativePosition(m_margin_x);
        if ( relative_pos == RelativePosition::Before and not (*pipe).isAlignedWithGap(m_margin_y)){
            res.x = std::min(res.x, (*pipe).m_start - m_margin_x);
        }
        if (relative_pos != RelativePosition::After){
            res.y = ((*pipe).m_gap_start + (*pipe).m_gap_end) * 0.5;
        }
    }

    auto next_pipe_start =  m_perception.getNextPipeStart() ;
    if (next_pipe_start.has_value())
        res.x = std::min(res.x, *next_pipe_start - m_margin_x);

    return res;
}