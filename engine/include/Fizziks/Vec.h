#pragma once
#include <Fizziks.h>
#include <cmath>

namespace Fizziks
{    
struct FIZZIKS_API Vec2
{
    val_t x = 0;
    val_t y = 0;

    static constexpr Vec2 Zero() { return Vec2{ 0, 0 }; }

    val_t dot(const Vec2& other) const;

    val_t norm() const;
    val_t squaredNorm() const;

    Vec2 normalized() const;
    Vec2& normalize();
    Vec2 rotated(const val_t angle) const;
    Vec2& rotate(const val_t angle);

    bool operator==(const Vec2& v) const { return x == v.x && y == v.y; };
    bool operator!=(const Vec2& other) const { return !(*this == other); };

    Vec2 operator+(const Vec2& other) const;
    Vec2 operator-(const Vec2& other) const;
    Vec2 operator*(const val_t scalar) const;
    Vec2 operator/(const val_t scalar) const;
    Vec2& operator+=(const Vec2& other);
    Vec2& operator-=(const Vec2& other);
    Vec2& operator*=(const val_t scalar);
    Vec2& operator/=(const val_t scalar);
};

FIZZIKS_API Vec2 operator-(const Vec2& v);
FIZZIKS_API Vec2 operator*(const val_t scalar, const Vec2& v);

struct FIZZIKS_API Mat2
{
    val_t m00 = 0, m01 = 0;
    val_t m10 = 0, m11 = 0;

    static constexpr Mat2 Identity()
    {
        Mat2 mat;
        mat.m00 = 1; mat.m01 = 0;
        mat.m10 = 0; mat.m11 = 1;
        return mat;
    }

    static constexpr Mat2 Rotation(const val_t angle)
    {
        Mat2 mat;
        val_t c = std::cos(angle);
        val_t s = std::sin(angle);
        mat.m00 = c;  mat.m01 = -s;
        mat.m10 = s;  mat.m11 =  c;
        return mat;
    }

    bool operator==(const Mat2& m) const { return m00 == m.m00 && m01 == m.m01 && m10 == m.m10 && m11 == m.m11; }
    bool operator!=(const Mat2& other) const { return !(*this == other); }

    Vec2 operator*(const Vec2& vec) const;
};
}
