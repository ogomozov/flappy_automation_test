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
    m_timer = m_handle_ptr->createTimer(ros::Duration(0.1), &FlappyController::checkTimeouts, this, false, false);
}


void FlappyController::velCallback(const geometry_msgs::Vector3::ConstPtr& msg) {

    if (m_state == State::IDLE){
        m_state = State::FIND_FLOOR;
        m_prev_speed_update_time = ros::Time::now();
        m_prev_speed = *msg;
        m_vertical_offset_est = 0.0;
        m_points.clear();
        m_timer.start();
        ROS_INFO("Starting up. First lets find the floor.");
        return;
    }

    // Estimate how have we moved wrt the last frame
    const auto new_time = ros::Time::now();
    const auto& new_speed = *msg;
    const auto delta = (new_time - m_prev_speed_update_time).toSec();

    const auto dx = (m_prev_speed.x + new_speed.x) * 0.5 * delta;
    const auto dy = (m_prev_speed.y + new_speed.y) * 0.5 * delta;

    m_prev_speed_update_time = new_time;
    m_prev_speed = new_speed;

    // Shift the points in the opposite direction
    m_vertical_offset_est += dy;
    for (auto& point : m_points){
        point.x -= dx;
        point.y -= dy;
    }

    // Destroy points that we have passed. The points are sorted by x in the laser callback.
    while((not m_points.empty() and m_points.front().x < -m_margin))
        m_points.pop_front();

    // Draw outline
    if (m_state == State::NAVIGATE){
        geometry_msgs::PolygonStamped upper_poly_msg;
        upper_poly_msg.header.frame_id = "world";
        if (m_pipe_detected){
            auto& points = upper_poly_msg.polygon.points;
            points.resize(4);
            points[0].x = m_pipe_end;
            points[0].y = m_ceiling_offset - m_vertical_offset_est;
            points[1].x = m_pipe_start;
            points[1].y = m_ceiling_offset - m_vertical_offset_est;
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
            points[2].y = m_floor_offset - m_vertical_offset_est;
            points[3].x = m_pipe_end;
            points[3].y = m_floor_offset - m_vertical_offset_est;
        }
        m_pub_gate_lower.publish(lower_poly_msg);


    }


    // Check if we found the floor
    if (m_state == State::FIND_FLOOR){
        const auto it = std::min_element(m_points.begin(), m_points.end(), [](const auto& a, const auto& b){
            return a.y < b.y;
        });
        if (it != m_points.end() and it->y < 0){
            m_floor_offset = m_vertical_offset_est + it->y;
            m_state = State::FIND_CEILING;
            ROS_INFO("Found floor at %f. Switching to finding ceiling.", m_floor_offset);
        }
    }

    if (m_state == State::FIND_CEILING){
        const auto it = std::max_element(m_points.begin(), m_points.end(), [](const auto& a, const auto& b){
            return a.y < b.y;
        });

        if (it != m_points.end() and it->y > 0){
            m_ceiling_offset = m_vertical_offset_est + it->y;
            m_state = State::NAVIGATE;
            m_points.clear();
            ROS_INFO("Found ceiling at %f. Starting navigation", m_ceiling_offset);
        }
    }


    // Publish command
    auto vx_des = 0.0;
    auto vy_des = 0.0;
    if (m_state == State::FIND_FLOOR) {
        vx_des = 0.0;
        vy_des = -m_slow_speed;
    }
    if (m_state == State::FIND_CEILING){
        vx_des =  0.0;
        vy_des =  m_slow_speed;
    }
    if (m_state == State::NAVIGATE){
        vx_des =  m_global_speed_limit;
        if (not (m_pipe_detected) or m_pipe_end < -m_margin) {
            vy_des = 0.0;
        } else {
            const auto y_target = (m_pipe_gap_start + m_pipe_gap_end) * 0.5;
            vy_des = 0.8 * y_target / delta;
        }

    }

    const auto speed_limit = getSpeedLimit();
    const auto vx_prev = m_prev_speed.x;
    const auto vy_prev = m_prev_speed.y;

    const auto vx_next = std::clamp(vx_des, 0.0, speed_limit.vx);
    const auto vy_next = std::clamp(vy_des, speed_limit.vy_down, speed_limit.vy_up);

    geometry_msgs::Vector3 acc_cmd;
    assert(delta > 0);
    acc_cmd.x = std::clamp((vx_next-vx_prev)/delta,-m_max_acc, m_max_acc);
    acc_cmd.y = std::clamp((vy_next-vy_prev)/delta,-m_max_acc, m_max_acc);
    m_pub_acc_cmd.publish(acc_cmd);

    //TODO publish bird transform
}


void FlappyController::laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    if (m_state == State::IDLE)
        return;

    m_prev_laser_update = ros::Time::now();
    // Convert rays to points
    auto point_cloud_msg = sensor_msgs::PointCloud{};
    laser_geometry::LaserProjection{}.projectLaser(*msg, point_cloud_msg);

    // Filter out points outside corridors bounds and add to the cloud
    auto& points = point_cloud_msg.points;
    std::copy_if(points.begin(), points.end(), std::back_inserter(m_points), [this](const auto& point){
        return not isPointGroundOrCeiling(point) and not isSimilarPointInCloud(point);
    });
    std::sort(m_points.begin(), m_points.end(), [](const auto& a, const auto& b){
        return a.x < b.x;
    });

    m_pipe_detected = detectPipe();

    // Publish current cloud for debug
    point_cloud_msg.points.clear();
    std::copy(m_points.begin(), m_points.end(), std::back_inserter(point_cloud_msg.points));
    m_pub_point_cloud.publish(point_cloud_msg);
}


bool FlappyController::isPointGroundOrCeiling(const geometry_msgs::Point32& p) {
    // TODO make a proper ground detection
    if (m_state == State::NAVIGATE){
        if (p.y + m_vertical_offset_est < m_floor_offset + 0.1)
            return true;
        if (p.y + m_vertical_offset_est > m_ceiling_offset - 0.1)
            return true;
    }
    return false;
};


bool FlappyController::isSimilarPointInCloud(const geometry_msgs::Point32& p){
    for (const auto& other : m_points){
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
        m_state = State::IDLE;
        m_timer.stop();
        ROS_ERROR("Timeout on speed update. Switching to idle.");
        return;
    }
    if ((now - m_prev_laser_update).toSec() > 0.2){
        m_state = State::IDLE;
        m_timer.stop();
        ROS_ERROR("Timeout on laser scan update. Switching to idle.");
        return;
    }
}


SpeedLimit FlappyController::getSpeedLimit(){

    auto res = SpeedLimit{
        m_global_speed_limit,
        m_global_speed_limit,
        -m_global_speed_limit
    };

    if (m_points.empty())
        return res;

    const auto max_speed_from_dist = [&](auto dist){
        const auto v_sq = std::max(0.0, 2.0 * m_nom_acc * dist);
        return std::sqrt(v_sq);
    };

    for (const auto& p : m_points){
        if (p.y < m_margin and p.y > -m_margin)
            res.vx = std::min(max_speed_from_dist(p.x - m_margin), res.vx);
        if (p.x < m_margin and p.y > 0)
            res.vy_up = std::min(max_speed_from_dist(p.y - m_margin), res.vy_up);
        if (p.x < m_margin and p.y <= 0)
            res.vy_down = std::max(-max_speed_from_dist(-p.y - m_margin), res.vy_down);
    }

    return res;
}



bool FlappyController::detectPipe(){
    if (m_state != State::NAVIGATE)
        return false;

    if (m_points.empty())
        return false;

    m_pipe_start = m_points.front().x;
    const auto pipe_end = std::lower_bound(m_points.begin(), m_points.end(), m_pipe_start + m_pipe_width, [](const auto& p, auto v){
        return p.x < v;
    });
    if (pipe_end == m_points.end())
        return false;
    m_pipe_end = std::prev(pipe_end)->x;

    auto offsets = std::vector<double>{};
    offsets.reserve(std::distance(m_points.begin(), pipe_end));
    std::transform(m_points.begin(), pipe_end, std::back_inserter(offsets), [](const auto& p){
        return p.y;
    });
    std::sort(offsets.begin(), offsets.end());
    for (auto it = offsets.begin(); it != std::prev(offsets.end()); ++it){
        const auto diff = *std::next(it) - *it;
        if (diff > m_margin*2) {
            m_pipe_gap_start = *it;
            m_pipe_gap_end = *std::next(it);
        }

    }

    return true;
}