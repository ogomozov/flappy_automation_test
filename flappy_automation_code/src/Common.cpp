#include <flappy_automation_code/Common.hpp>

double len(const Vector2d &v) {
    return std::sqrt(pow2(v.x) + pow2(v.y));
}

Vector2d normalized(const Vector2d& v) {
    constexpr auto precision = 1.0e-3;
    const auto v_norm = len(v);
    if (v_norm < precision)
        return Vector2d{0.0, 0.0};
    const auto norm_inv = 1.0 / v_norm;
    return v * norm_inv;
}