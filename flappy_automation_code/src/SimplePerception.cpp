#include <flappy_automation_code/SimplePerception.hpp>
#include <algorithm>
#include <vector>
#include <cmath>

SimplePerception::SimplePerception(const NodeParams& params) : m_params{params}{
}

void SimplePerception::reset() {
    m_floor_detected = false;
    m_ceiling_detected = false;
    m_pipe_detected = false;
    clearPoints();
}

void SimplePerception::clearPoints() {
    m_points_x_sorted.clear();
}

std::optional<Pipe> SimplePerception::getFistPipe() const {
    if (not m_pipe_detected)
        return std::nullopt;
    return Pipe{
        m_pipe_start,
        m_pipe_end,
        m_pipe_gap_start,
        m_pipe_gap_end
    };
}

std::optional<double> SimplePerception::getNextPipeStart() const {
    if (m_pipe_detected)
        return m_pipe_next_start;
    return m_params.m_max_view_distance;
}

std::optional<double> SimplePerception::getFloorOffset() const {
    if (m_floor_detected)
        return m_floor_offset;
    return std::nullopt;
}

std::optional<double> SimplePerception::getCeilingOffset() const {
    if (m_ceiling_detected)
        return m_ceiling_offset;
    return std::nullopt;
}

/*
template<typename ConstIter>
void SimplePerception::addPoints(ConstIter it_begin, ConstIter it_end){
 */
void SimplePerception::addPoints(const std::vector<Vector2d> &new_points) {
    // TODO check if ConstIter is actually iterator to Vector2d
    auto it_begin = new_points.begin();
    auto it_end = new_points.end();
    std::copy_if(it_begin, it_end, std::back_inserter(m_points_x_sorted), [this](const auto& p){
        return not (isSimilarPointInCloud(p) or isPointGroundOrCeiling(p));
    });
    std::sort(m_points_x_sorted.begin(), m_points_x_sorted.end(), [](const auto& a, const auto& b){
        return a.x < b.x;
    });

    if (not m_floor_detected)
        detectFloor();
    if (not m_ceiling_detected)
        detectCeiling();
    if (m_floor_detected and m_ceiling_detected)
        detectPipe();
}


void SimplePerception::shift(const Vector2d& delta){
    if (m_floor_detected)
        m_floor_offset += delta.y;
    if (m_ceiling_detected)
        m_ceiling_offset += delta.y;

    for (auto& p : m_points_x_sorted){
        p.x += delta.x;
        p.y += delta.y;
    }

    while((not m_points_x_sorted.empty() and m_points_x_sorted.front().x < -m_params.m_margin_x))
        m_points_x_sorted.pop_front();

    detectPipe();
}


bool SimplePerception::isSimilarPointInCloud(const Vector2d& p){
    for (const auto& other : m_points_x_sorted){
        const auto dx = p.x - other.x;
        const auto dy = p.y - other.y;
        const auto dist_squared = dx*dx + dy*dy;
        if (dist_squared < pow2(m_params.m_dist_threshold))
            return true;
    }
    return false;
}


bool SimplePerception::isPointGroundOrCeiling(const Vector2d& p) {
    if (m_floor_detected and p.y < m_floor_offset + m_params.m_dist_threshold*2.0)
        return true;
    if (m_ceiling_detected and p.y > m_ceiling_offset - m_params.m_dist_threshold*2.0)
        return true;
    return false;
}

bool SimplePerception::detectPipe(){
    m_pipe_detected = false;
    if (not (m_floor_detected and m_ceiling_detected))
        return false;
    if (m_points_x_sorted.size() < 2)
        return false;

    auto pipe_start = m_points_x_sorted.begin();
    auto pipe_end = m_points_x_sorted.end();
    for (auto it = std::next(pipe_start); it != m_points_x_sorted.end(); ++it){
        const auto dist = it->x - std::prev(it)->x;
        if (dist > m_params.m_margin_x*2){
            if (it->x + m_params.m_margin_x < 0){
                pipe_start = it;
                continue;
            }
            pipe_end = it;
            break;
        }
    }
    m_pipe_start = pipe_start->x;
    m_pipe_end = std::prev(pipe_end)->x;
    m_pipe_next_start = (pipe_end != m_points_x_sorted.end())? pipe_end->x : m_params.m_max_view_distance;

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
        if (diff > m_params.m_margin_y*2) {
            gaps.emplace_back(*it, *std::next(it));
        }
    }
    if (gaps.empty()){
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

bool SimplePerception::detectFloor(){
    const auto it = std::min_element(
            m_points_x_sorted.begin(),
            m_points_x_sorted.end(),
            [](const auto& a, const auto& b){
                return a.y < b.y;
            });

    if (it != m_points_x_sorted.end() and it->y < 0){
        m_floor_offset = it->y;
        m_floor_detected = true;
    } else {
        m_floor_detected = false;
    }
    return m_floor_detected;
}

bool SimplePerception::detectCeiling(){
    const auto it = std::max_element(
            m_points_x_sorted.begin(),
            m_points_x_sorted.end(),
            [](const auto& a, const auto& b){
                return a.y < b.y;
            });

    if (it != m_points_x_sorted.end() and it->y > 0){
        m_ceiling_offset = it->y;
        m_ceiling_detected = true;
    } else {
        m_ceiling_detected = false;
    }
    return m_ceiling_detected;
}

const std::deque<Vector2d>& SimplePerception::getPoints() const {
    return m_points_x_sorted;
}
