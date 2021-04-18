#pragma once
#include <flappy_automation_code/Common.hpp>
#include <deque>
#include <optional>
#include <vector>


class SimplePerception{
public:

    explicit SimplePerception(const NodeParams& params);

    void reset();

    const std::vector<Pipe>& getDetectedPipes() const;
    std::optional<double> getNextPipeStart() const;
    std::optional<double> getFloorOffset() const;
    std::optional<double> getCeilingOffset() const;
    const std::deque<Vector2d>& getPoints() const;

    void addPoints(const std::vector<Vector2d>& new_points);
    void shift(const Vector2d& delta);
    void clearPoints();
private:

    NodeParams m_params;

    /*============= States =============*/

    // Detection flags
    bool m_floor_detected{false};
    bool m_ceiling_detected{false};

    // Perceived values.
    double m_floor_offset{0};
    double m_ceiling_offset{0};
    double m_pipe_start{0};
    double m_pipe_end{0};
    double m_pipe_next_start{0};
    double m_pipe_gap_start{0};
    double m_pipe_gap_end{0};
    std::deque<Vector2d> m_points_x_sorted{};
    std::vector<Pipe> m_pipes;

    /*============= Methods =============*/

    // Checks that a similar point is already in the point cloud.
    bool isSimilarPointInCloud(const Vector2d& p);

    // Checks that the point belongs to the detected floor or the ceiling.
    bool isPointGroundOrCeiling(const Vector2d& p);

    // Try to detect the pipes.
    void detectPipes();

    // Try to detect the floor. If successful sets the m_floor_detected flag
    // to true, updates the perceived values and returns true.
    bool detectFloor();

    // Try to detect the ceiling. If successful sets the m_ceiling_detected flag
    // to true, updates the perceived values and returns true.
    bool detectCeiling();

    using Iterator = typename std::deque<Vector2d>::iterator;
    Pipe detectPipe(Iterator start, Iterator end, double gap_guess);

};
