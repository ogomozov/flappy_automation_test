#pragma once
#include <cassert>

struct Vector2d{
    double x;
    double y;
};

constexpr Vector2d operator+(const Vector2d& a, const Vector2d& b) {
    return Vector2d{
        a.x + b.x,
        a.y + b.y
    };
}

constexpr Vector2d operator-(const Vector2d& a, const Vector2d& b) {
    return Vector2d{
        a.x - b.x,
        a.y - b.y
    };
}

constexpr Vector2d operator*(double scalar, const Vector2d& vector) {
    return Vector2d{
            vector.x * scalar,
            vector.y * scalar
    };
}

constexpr Vector2d operator*(const Vector2d& vector, double scalar) {
    return scalar * vector;
}

struct NodeParams{
    double m_dist_threshold;
    double m_slow_speed;
    double m_margin_y;
    double m_margin_x;
    double m_nom_acc;
    double m_max_acc;
    double m_max_view_distance;
};

struct PipeGap{
    PipeGap(double lower_extent, double upper_extent) :
            m_lower_extent{lower_extent},
            m_upper_extent{upper_extent},
            m_middle{0.5*(lower_extent + upper_extent)}
    {
        assert(m_upper_extent >= m_lower_extent);
    }
    double m_lower_extent;
    double m_upper_extent;
    double m_middle;
};

enum struct RelativePosition{
    Before,
    Inside,
    After
};

struct Pipe{
    Pipe(double start, double end, double gap_start, double gap_end) :
        m_start{start},
        m_end{end},
        m_gap_start{gap_start},
        m_gap_end{gap_end}
    {
        assert(m_end >= m_end);
        assert(m_gap_end >= m_gap_end);
    }

    inline bool isAlignedWithGap(double margin_y) const {
        assert(margin_y >= 0);
        return m_gap_end > margin_y and m_gap_start < -margin_y;
    }

    bool isPassingPipe(double margin_x) const {
        assert(margin_x >= 0);
        return margin_x > m_start and -margin_x < m_end;
    }

    bool isCollidingWithPipe(const Vector2d& margins) const {
        return isPassingPipe(margins.x) and not isAlignedWithGap(margins.y);
    }

    RelativePosition getRelativePosition(double margin_x) const {
        assert(margin_x >= 0);
        if (isPassingPipe(margin_x))
            return RelativePosition::Inside;
        if (m_end < -margin_x)
            return RelativePosition::After;
        return RelativePosition::Before;
    }

    double m_start;
    double m_end;
    double m_gap_start;
    double m_gap_end;
};

template<typename T>
constexpr T pow2(T v) {
    return v * v;
}