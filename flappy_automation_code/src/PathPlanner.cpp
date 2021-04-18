#include <flappy_automation_code/PathPlanner.hpp>
#include <ros/ros.h>

void PathPlanner::plan(const Vector2d& current_speed, const SimplePerception& perception) {
    m_path.clear();
    m_path.emplace_back(
        Vector2d{
            0.0,
            0.0
        },
        normalized(current_speed)
    );

    const auto add_segment_to = [&](const auto& p, const auto& v_dir){
        const auto p0 = m_path.back().m_pos;
        const auto p1 = p;
        const auto spline_len_est = std::max(0.001, len(p1 - p0));
        const auto scale = m_params.m_max_speed / spline_len_est;
        const auto scale_inv = 1.0 / scale;
        const auto v0 = m_path.back().m_speed * scale_inv;
        const auto v1 = v_dir * scale_inv;

        const auto spline = Hermite3{
            p0,
            p1,
            v0,
            v1
        };

        constexpr auto num_sub_segments = 10;
        constexpr auto step = 1.0 / (num_sub_segments-1);
        for (int i = 0; i < num_sub_segments; ++i){
            const auto s = i * step;
            m_path.emplace_back(
                spline.posAt(s),
                spline.parametricSpeedAt(s) * scale
            );
        }
    };

    const auto& pipes = perception.getDetectedPipes();
    for (auto it = pipes.begin(); it != pipes.end(); ++it){
        const auto gap_median = (it->m_gap_start + it->m_gap_end) * 0.5;
        const auto relative_pos = it->getRelativePosition(m_params.m_margin_x);
        const auto is_aligned = it->isAlignedWithGap(m_params.m_margin_y);
        if (relative_pos == RelativePosition::Before) {
            add_segment_to(
                Vector2d{
                    it->m_start - m_params.m_margin_x,
                    gap_median
                },
                Vector2d{
                    1.0,
                    0.0
                }
            );
        } else if (relative_pos == RelativePosition::Inside and not is_aligned) {
            add_segment_to(
               Vector2d{
                  m_path.back().m_pos.x,
                  gap_median
               },
               Vector2d{
                  0.0,
                  0.0
               });
        }
        if (relative_pos != RelativePosition::After){
            add_segment_to(
                Vector2d{
                    it->m_end + m_params.m_margin_x,
                    gap_median
                },
                Vector2d{
                    1.0,
                    0.0
                }
            );
        }
    }
    add_segment_to(
        Vector2d{
            m_params.m_max_view_distance - m_params.m_margin_x,
            m_path.back().m_pos.y
        },
        Vector2d{
            1.0,
            0.0
        }
    );
}


void PathPlanner::reset() {
    m_path.clear();
}

const Path& PathPlanner::getPath() const {
    return m_path;
}
