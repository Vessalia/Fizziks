#pragma once
#include <Fizziks/Vec.h>

#include <limits>
#include <cmath>

namespace Fizziks
{
constexpr val_t PI = 3.141592653589793238462643383279502884197169399375105820974944592307816406L;
constexpr val_t TWO_PI = 2 * PI;

inline val_t clamp_angle(val_t angle)
{
    return angle - TWO_PI * std::floor(angle / TWO_PI);
}

inline Vec2 vec_max() 
{
    return
    {
        std::numeric_limits<val_t>::max(),
        std::numeric_limits<val_t>::max()
    };
}

inline Vec2 vec_min() 
{
    return
    {
        std::numeric_limits<val_t>::lowest(),
        std::numeric_limits<val_t>::lowest()
    };
}

template<typename T>
T inline constexpr fizzmax() { return std::numeric_limits<T>::max(); }

template<typename T>
T inline constexpr fizzmin() { return std::numeric_limits<T>::min(); }

inline constexpr val_t deg2rad(val_t deg)
{
    constexpr val_t factor = PI / 180;
    return deg * factor;
}

inline constexpr val_t rad2deg(val_t rad)
{
    constexpr val_t factor = 180 / PI;
    return rad * factor;
}

FIZZIKS_API int mod(int a, int b);
FIZZIKS_API val_t crossproduct(const Vec2& a, const Vec2& b);
FIZZIKS_API Vec2 crossproduct(val_t w, const Vec2& r);
// (a x b) x c = b(a.c) - a(b.c)
FIZZIKS_API Vec2 lefttriplecross(const Vec2& a, const Vec2& b, const Vec2& c);
// a x (b x c) = b(a.c) - c(a.b)
FIZZIKS_API Vec2 righttriplecross(const Vec2& a, const Vec2& b, const Vec2& c);
}
