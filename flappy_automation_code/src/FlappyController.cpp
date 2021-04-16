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
    m_handle_ptr{std::move(h)}
{
    timeout_check_timer = m_handle_ptr->createTimer(ros::Duration(0.1), &FlappyController::checkTimeouts, this, false, false);
    m_v_max_x = maxSpeedFromDistance(m_max_view_distance);
    m_v_max_y = maxSpeedFromDistance(m_max_view_distance);
}

void FlappyController::velCallback(const geometry_msgs::Vector3::ConstPtr& msg) {
    m_current_speed = *msg;
    const auto current_time = ros::Time::now();
    const auto delta = (current_time - m_prev_speed_update_time).toSec();
    if (m_state == ControllerState::IDLE){
        resetState();
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

    // Filter out points outside corridors bounds and add to the cloud
    auto& points = point_cloud_msg.points;
    std::copy_if(points.begin(), points.end(), std::back_inserter(m_points_x_sorted), [this](const auto& point){
        return not isPointGroundOrCeiling(point) and not isSimilarPointInCloud(point);
    });
    std::sort(m_points_x_sorted.begin(), m_points_x_sorted.end(), [](const auto& a, const auto& b){
        return a.x < b.x;
    });

    if (not m_floor_detected)
        detectFloor();
    if (not m_ceiling_detected)
        detectCeiling();
    if (m_state == ControllerState::NAVIGATE)
        detectPipe();
}


bool FlappyController::isPointGroundOrCeiling(const geometry_msgs::Point32& p) {
    if (m_state == ControllerState::NAVIGATE){
        if (p.y < m_floor_offset + 0.1)
            return true;
        if (p.y > m_ceiling_offset - 0.1)
            return true;
    }
    return false;
};


bool FlappyController::isSimilarPointInCloud(const geometry_msgs::Point32& p){
    for (const auto& other : m_points_x_sorted){
        const auto dx = p.x - other.x;
        const auto dy = p.y - other.y;
        const auto dist = dx*dx + dy*dy;
        if (dist < m_dist_threshold*m_dist_threshold)
            return true;
    }
    return false;
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


SpeedLimit FlappyController::getSpeedLimit(){
    auto res = SpeedLimit{
        m_v_max_x,
        m_v_max_y,
        -m_v_max_y
    };

    // Distance that we would have traveled when the command will take action.
    const auto dx_pred = m_current_speed.x * m_delta_sec_avg;
    const auto dy_pred = m_current_speed.y * m_delta_sec_avg;

    const auto dist = m_max_view_distance - m_margin_x - dx_pred;
    res.vx = std::min(maxSpeedFromDistance(dist), res.vx);

    if (m_pipe_detected){
        const auto inside_with_gap_y_span =
                (m_pipe_gap_end > m_margin_y) and (m_pipe_gap_start < -m_margin_y);
        if (not inside_with_gap_y_span){
            const auto dist = m_pipe_start - m_margin_x - dx_pred;
            res.vx = std::min(maxSpeedFromDistance(dist), res.vx);
        }

        const auto dist = m_pipe_next_start - m_margin_x - dx_pred;
        res.vx = std::min(maxSpeedFromDistance(dist), res.vx);
    }

    if (m_floor_detected){
        const auto y_down_limit = m_floor_offset + m_margin_y - dy_pred;
        res.vy_down = std::max(-maxSpeedFromDistance(-y_down_limit), res.vy_down);
    }
    if (m_ceiling_detected){
        const auto y_up_limit = m_ceiling_offset - m_margin_y - dy_pred;
        res.vy_up = std::min(maxSpeedFromDistance(y_up_limit), res.vy_up);
    }

    return res;
}


bool FlappyController::detectPipe(){
    m_pipe_detected = false;
    if (m_state != ControllerState::NAVIGATE)
        return false;

    if (m_points_x_sorted.size() < 2)
        return false;

    auto pipe_start = m_points_x_sorted.begin();
    auto pipe_end = m_points_x_sorted.end();
    for (auto it = std::next(pipe_start); it != m_points_x_sorted.end(); ++it){
        const auto dist = it->x - std::prev(it)->x;
        if (dist > m_margin_x*2){
            if (it->x + m_margin_x < 0){
                pipe_start = it;
                continue;
            }
            pipe_end = it;
            break;
        }
    }
    m_pipe_start = pipe_start->x;
    m_pipe_end = std::prev(pipe_end)->x;
    m_pipe_next_start = (pipe_end != m_points_x_sorted.end())? pipe_end->x : m_max_view_distance;

    auto offsets = std::vector<double>{};
    offsets.reserve(std::distance(pipe_start, pipe_end));
    std::transform(pipe_start, pipe_end, std::back_inserter(offsets), [](const auto& p){
        return p.y;
    });
    offsets.emplace_back(m_ceiling_offset);
    offsets.emplace_back(m_floor_offset);
    std::sort(offsets.begin(), offsets.end());
    auto gaps = std::vector<PipeGap>{};
    for (auto it = offsets.begin(); it != std::prev(offsets.end()); ++it){
        const auto diff = *std::next(it) - *it;
        if (diff > m_margin_y*2) {
            gaps.emplace_back(*it, *std::next(it));
        }
    }
    if (gaps.empty()){
        ROS_ERROR("Found a pipe but no gap in it.");
        m_pipe_gap_start = 0.0;
        m_pipe_gap_end = 0.0;
        m_pipe_detected = true;
        return true;
    }
    const auto it = std::min_element(gaps.begin(), gaps.end(), [](const auto& a, const auto& b){
        return std::fabs(a.m_middle) < std::fabs(b.m_middle);
    });
    m_pipe_gap_start = it->m_lower_extent;
    m_pipe_gap_end = it->m_upper_extent;
    m_pipe_detected = true;
    return true;
}

double FlappyController::maxSpeedFromDistance(double distance){
    const auto v_sq = std::max(0.0, 2.0 * m_nom_acc * distance);
    return std::sqrt(v_sq);
}


void FlappyController::resetState(){
    // Reset detection flags
    m_floor_detected = false;
    m_ceiling_detected = false;
    m_pipe_detected = false;

    // Reset point cloud
    m_points_x_sorted.clear();
}


void FlappyController::updatePerception(double dx, double dy) {
    // Shift everything
    if (m_floor_detected)
        m_floor_offset += dy;
    if (m_ceiling_detected)
        m_ceiling_offset += dy;
    for (auto& p : m_points_x_sorted) {
        p.x += dx;
        p.y += dy;
    }

    // Remove old points from the cloud
    while((not m_points_x_sorted.empty() and m_points_x_sorted.front().x < -m_margin_x))
        m_points_x_sorted.pop_front();
}


void FlappyController::updateState(double delta_sec){
    const auto dx = (m_prev_speed.x + m_current_speed.x) * 0.5 * delta_sec;
    const auto dy = (m_prev_speed.y + m_current_speed.y) * 0.5 * delta_sec;
    updatePerception(-dx, -dy);
    if (m_state == ControllerState::FIND_FLOOR and m_floor_detected){
        m_state = ControllerState::FIND_CEILING;
        ROS_INFO("Found floor at %f. Searching for ceiling now.", m_floor_offset);
    }
    if (m_state == ControllerState::FIND_CEILING and m_ceiling_detected) {
        ROS_INFO("Found ceiling at %f. Starting navigation.", m_ceiling_offset);
        m_points_x_sorted.clear();
        m_state = ControllerState::NAVIGATE;
    }
}

void FlappyController::publishVisuals() {
    geometry_msgs::PolygonStamped upper_poly_msg;
    upper_poly_msg.header.frame_id = "world";
    if (m_pipe_detected){
        auto& points = upper_poly_msg.polygon.points;
        points.resize(4);
        points[0].x = m_pipe_end;
        points[0].y = m_ceiling_offset;
        points[1].x = m_pipe_start;
        points[1].y = m_ceiling_offset;
        points[2].x = m_pipe_start;
        points[2].y = m_pipe_gap_end;
        points[3].x = m_pipe_end;
        points[3].y = m_pipe_gap_end;
    }
    m_pub_gate_upper.publish(upper_poly_msg);

    geometry_msgs::PolygonStamped lower_poly_msg;
    lower_poly_msg.header.frame_id = "world";
    if (m_pipe_detected){
        auto& points = lower_poly_msg.polygon.points;
        points.resize(4);
        points[0].x = m_pipe_end;
        points[0].y = m_pipe_gap_start;
        points[1].x = m_pipe_start;
        points[1].y = m_pipe_gap_start;
        points[2].x = m_pipe_start;
        points[2].y = m_floor_offset;
        points[3].x = m_pipe_end;
        points[3].y = m_floor_offset;
    }
    m_pub_gate_lower.publish(lower_poly_msg);

    auto point_cloud_msg = sensor_msgs::PointCloud{};
    point_cloud_msg.header.frame_id = "world";
    std::copy(m_points_x_sorted.begin(), m_points_x_sorted.end(), std::back_inserter(point_cloud_msg.points));
    m_pub_point_cloud.publish(point_cloud_msg);
}


bool FlappyController::detectFloor(){
    const auto it = std::min_element(m_points_x_sorted.begin(), m_points_x_sorted.end(), [](const auto& a, const auto& b){
        return a.y < b.y;
    });
    if (it != m_points_x_sorted.end() and it->y < 0){
        m_floor_offset = it->y;
        m_floor_detected = true;
        return true;
    }
    m_floor_detected = false;
    return false;
}


bool FlappyController::detectCeiling(){
    const auto it = std::max_element(m_points_x_sorted.begin(), m_points_x_sorted.end(), [](const auto& a, const auto& b){
        return a.y < b.y;
    });
    if (it != m_points_x_sorted.end() and it->y > 0){
        m_ceiling_offset = it->y;
        m_ceiling_detected = true;
        return true;
    }
    m_ceiling_detected = false;
    return false;
}



void FlappyController::publishCommand(double delta_sec){
    // Compute desired speed as a function of the current state
    auto vx_des = 0.0;
    auto vy_des = 0.0;
    if (m_state == ControllerState::FIND_FLOOR) {
        vx_des = 0.0;
        vy_des = -m_slow_speed;
    }
    if (m_state == ControllerState::FIND_CEILING){
        vx_des = 0.0;
        vy_des = m_slow_speed;
    }
    if (m_state == ControllerState::NAVIGATE){
        if (not (m_pipe_detected) or m_pipe_end < -m_margin_x) {
            vx_des =  m_v_max_x;
            vy_des = 0.0;
        } else {
            // Predicted distance that we will have traveled when the command is applied.
            const auto y_pred = m_current_speed.y * m_delta_sec_avg;
            const auto y_target = (m_pipe_gap_start + m_pipe_gap_end) * 0.5 - y_pred;
            const auto v_max = maxSpeedFromDistance(std::fabs(y_target));
            vy_des = (y_target>0)? v_max : -v_max;

            if (m_pipe_start > - m_margin_x and std::fabs(y_target) > 0.1){
                assert(std::fabs(vy_des) > 0);
                const auto dt = y_target / vy_des;
                const auto x_target = m_pipe_start - m_margin_x;
                assert(dt > 0);
                vx_des = std::min(x_target / dt, m_v_max_x);
            } else {
                vx_des = m_v_max_x;
            }
        }
    }

    // Find target speed by limiting the desired one by anitcollision
    const auto speed_limit = getSpeedLimit();
    const auto vx_prev = m_current_speed.x;
    const auto vy_prev = m_current_speed.y;
    const auto vx_next = std::clamp(vx_des, 0.0, speed_limit.vx);
    const auto vy_next = std::clamp(vy_des, speed_limit.vy_down, speed_limit.vy_up);

    // Compute and publish the accelerations required to reach target speed
    geometry_msgs::Vector3 acc_cmd;
    assert(delta_sec > 0);
    acc_cmd.x = std::clamp((vx_next-vx_prev)/delta_sec,-m_max_acc, m_max_acc);
    acc_cmd.y = std::clamp((vy_next-vy_prev)/delta_sec,-m_max_acc, m_max_acc);
    m_pub_acc_cmd.publish(acc_cmd);
}