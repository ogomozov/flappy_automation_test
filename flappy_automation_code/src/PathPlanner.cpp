#include <flappy_automation_code/PathPlanner.hpp>
#include <ros/ros.h>

void PathPlanner::plan(const Vector2d& current_speed, const SimplePerception& perception) {
    m_path.clear();
    m_path.emplace_back(Vector2d{
        0.0,
        0.0
    });

    const auto add_segment_to = [&](const auto& p){
        m_path.emplace_back(p);
    };

    const auto& pipes = perception.getDetectedPipes();
    for (auto it = pipes.begin(); it != pipes.end(); ++it){
        const auto gap_median = (it->m_gap_start + it->m_gap_end) * 0.5;
        const auto relative_pos = it->getRelativePosition(m_params.m_margin_x);
        const auto is_aligned = it->isAlignedWithGap(m_params.m_margin_y);
        if (relative_pos == RelativePosition::Before) {
            add_segment_to(Vector2d{
                it->m_start - m_params.m_margin_x,
                gap_median
            });
        } else if (relative_pos == RelativePosition::Inside and not is_aligned) {
            add_segment_to(Vector2d{
                m_path.back().x,
                gap_median
            });
        }
        if (relative_pos != RelativePosition::After){
            add_segment_to(Vector2d{
                it->m_end + m_params.m_margin_x,
                gap_median
            });
        }
    }
    add_segment_to(Vector2d{
        m_params.m_max_view_distance - m_params.m_margin_x,
        m_path.back().y
    });
}


void PathPlanner::reset() {
    m_path.clear();
}

const Path& PathPlanner::getPath() const {
    return m_path;
}
