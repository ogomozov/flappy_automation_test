#pragma once
#include <flappy_automation_code/Common.hpp>
#include <deque>
#include <optional>
#include <vector>

/*
 * Simple perception based.
 * Accumulates the point cloud from laser readings and try to
 * cluster the points into AABBs that represent the pipes
 * in the game.
 */
class SimplePerception{
public:

    explicit SimplePerception(const NodeParams& params);

    // Resets the point cloud, floor and ceiling offsets and the detected pipes.
    void reset();

    // Returns reference to an array of pipes sorted by x offset from the bird.
    const std::vector<Pipe>& getDetectedPipes() const;

    // Returns the start of the pipe next to the one that the bird is currently passing.
    std::optional<double> getNextPipeStart() const;

    // If floor is detected returns its offset.
    std::optional<double> getFloorOffset() const;

    // If the ceiling is detected returns its offset.
    std::optional<double> getCeilingOffset() const;

    // Returns a container with the points sorted by x offset from the bird.
    const std::deque<Vector2d>& getPoints() const;

    // Addes the points to the cloud.
    void addPoints(const std::vector<Vector2d>& new_points);

    // Shift the detected points, gates and offsets according to the delta shift of the bird.
    void shift(const Vector2d& delta);

    // Clear the point cloud and reset detected pipes.
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

    // Detect and construct a single pipe from the point ranging from start to end
    // with a hint on the pipes gap vertical offset.
    using Iterator = typename std::deque<Vector2d>::iterator;
    Pipe detectPipe(Iterator start, Iterator end, double gap_guess);

};
