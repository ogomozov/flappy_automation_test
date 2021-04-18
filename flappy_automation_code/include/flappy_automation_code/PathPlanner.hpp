#pragma once

#include <flappy_automation_code/Common.hpp>
#include <flappy_automation_code/Hermite3.hpp>
#include <flappy_automation_code/SimplePerception.hpp>
#include <vector>

struct PathSample{
    PathSample(const Vector2d& pos, const Vector2d& speed) : m_pos{pos}, m_speed{speed}{}
    Vector2d m_pos;
    Vector2d m_speed;
};

using Path = std::vector<PathSample>;


/**
 *  Simple path planner based on cubic splines.
 *  It is currently unfinished. Currently splines assumes that
 *  we advance with the maximum speed and doesn't take into account
 *  the v > 0 constraint. To make it work we need to optimize the
 *  speed profile and consequently the curvature along the spline.
 *  This can be done through optimal control methods, for exemple
 *  the Model Predictive Control.
 *
 */
class PathPlanner{
public:

    explicit PathPlanner(const NodeParams& params) : m_params{params} {}

    // Reset the stored path.
    void reset();

    // Update the path according to the new speed and the perception.
    void plan(const Vector2d& current_speed, const SimplePerception& perception);

    // Returns last planned path
    const Path& getPath() const;

private:
    NodeParams m_params;
    Path m_path;
};
