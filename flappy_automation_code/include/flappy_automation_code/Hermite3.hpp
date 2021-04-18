#pragma once
#include <flappy_automation_code/Common.hpp>

class Hermite3{
public:

    Hermite3(
        const Vector2d& p0,
        const Vector2d& p1,
        const Vector2d& v0,
        const Vector2d& v1
    ) : m_a0{p0},
        m_a1{v0},
        m_a2{3.0 * (p1 - p0) - (v1 + 2.0 * v0)},
        m_a3{p1 - p0 - v0 - m_a2}
    {
    }

    // p(s)
    Vector2d posAt(double s) const {
        const auto s2 = s * s;
        const auto s3 = s2 * s;
        return  m_a3 * s3 + m_a2 * s2 + m_a1 * s + m_a0;
    }

    // dp(s)/ds
    Vector2d parametricSpeedAt(double s) const {
        auto s2 = s * s;
        return 3.0 * m_a3 * s2 + 2.0 * m_a2 * s + m_a1;
    }

    // d2p(s)/ds2
    Vector2d parametricAcceleration(double s) const {
        return 6.0 * m_a3 * s + 2.0 * m_a2;
    }

private:
    Vector2d m_a0;
    Vector2d m_a1;
    Vector2d m_a2;
    Vector2d m_a3;
};
