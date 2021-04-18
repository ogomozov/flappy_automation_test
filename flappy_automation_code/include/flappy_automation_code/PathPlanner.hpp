#pragma once

#include <flappy_automation_code/Common.hpp>
#include <flappy_automation_code/Hermite3.hpp>
#include <flappy_automation_code/SimplePerception.hpp>
#include <vector>

using Path = std::vector<Vector2d>;

class PathPlanner{
public:

    PathPlanner(const NodeParams& params) : m_params{params} {}

    void reset();
    void plan(const Vector2d& current_speed, const SimplePerception& perception);
    const Path& getPath() const;

private:
    NodeParams m_params;
    Path m_path;
};
