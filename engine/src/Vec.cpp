#include <Fizziks/Vec.h>
#include <Fizziks/VecOps.h>
#include <cmath>

namespace Fizziks
{
// Vec2
Vec2 Vec2::lerped(const Vec2& other, val_t t) const { return internal::ops::lerp(*this, other, t); }
Vec2& Vec2::lerp(const Vec2& other, val_t t)        { *this = internal::ops::lerp(*this, other, t); return *this; }

val_t Vec2::dot(const Vec2& other)   const { return internal::ops::dot(*this, other); }
val_t Vec2::cross(const Vec2& other) const { return internal::ops::cross(*this, other); }
Vec2 Vec2::cross(val_t z)            const { return internal::ops::cross(*this, z); }

val_t Vec2::norm()        const { return internal::ops::norm(*this); }
val_t Vec2::squaredNorm() const { return internal::ops::squaredNorm(*this); }

Vec2 Vec2::normalized()         const { return internal::ops::normalized(*this); }
Vec2 Vec2::rotated(val_t angle) const { return internal::ops::rotated(*this, angle); }
Vec2& Vec2::normalize()               { internal::ops::normalize(*this); return *this; }
Vec2& Vec2::rotate(val_t angle)       { internal::ops::rotate(*this, angle); return *this; }

val_t Vec2::angle() const { return std::atan2(y, x); }

Vec2 Vec2::operator+(const Vec2& other) const { return internal::ops::add(*this, other); }
Vec2 Vec2::operator-(const Vec2& other) const { return internal::ops::sub(*this, other); }
Vec2 Vec2::operator*(val_t scalar)      const { return internal::ops::mul(*this, scalar); }
Vec2 Vec2::operator/(val_t scalar)      const { return internal::ops::div(*this, scalar); }

Vec2& Vec2::operator+=(const Vec2& other) { *this = internal::ops::add(*this, other); return *this; }
Vec2& Vec2::operator-=(const Vec2& other) { *this = internal::ops::sub(*this, other); return *this; }
Vec2& Vec2::operator*=(val_t scalar)      { *this = internal::ops::mul(*this, scalar); return *this; }
Vec2& Vec2::operator/=(val_t scalar)      { *this = internal::ops::div(*this, scalar); return *this; }

Vec2 operator-(const Vec2& v)               { return internal::ops::negate(v); }
Vec2 operator*(val_t scalar, const Vec2& v) { return internal::ops::mul(v, scalar); }
}

namespace Fizziks
{
// Mat2
Vec2 Mat2::col(int i) const { return internal::ops::col(*this, i); }
Vec2 Mat2::row(int i) const { return internal::ops::row(*this, i); }

Mat2 Mat2::operator+(const Mat2& other) const { return internal::ops::add(*this, other); }
Mat2 Mat2::operator-(const Mat2& other) const { return internal::ops::sub(*this, other); }
Mat2 Mat2::operator*(val_t scalar)      const { return internal::ops::mul(*this, scalar); }
Vec2 Mat2::operator*(const Vec2& vec)   const { return internal::ops::mul(*this, vec); }
Mat2 Mat2::operator*(const Mat2& mat)   const { return internal::ops::mul(*this, mat); }
Mat2 Mat2::operator/(val_t scalar)      const { return internal::ops::div(*this, scalar); }

Mat2& Mat2::operator+=(const Mat2& other) { *this = internal::ops::add(*this, other); return *this; }
Mat2& Mat2::operator-=(const Mat2& other) { *this = internal::ops::sub(*this, other); return *this; }
Mat2& Mat2::operator*=(val_t scalar)      { *this = internal::ops::mul(*this, scalar); return *this; }
Mat2& Mat2::operator*=(const Mat2& mat)   { *this = internal::ops::mul(*this, mat); return *this; }
Mat2& Mat2::operator/=(val_t scalar)      { *this = internal::ops::div(*this, scalar); return *this; }

Mat2 Mat2::transposed() const { return internal::ops::transposed(*this); }
Mat2& Mat2::transpose()       { internal::ops::transpose(*this); return *this; }

val_t Mat2::determinant() const { return internal::ops::determinant(*this); }
Mat2 Mat2::inverse()      const { return internal::ops::inverse(*this); }
Mat2& Mat2::invert()            { internal::ops::invert(*this); return *this; }

Mat2 operator-(const Mat2& m)               { return internal::ops::negate(m); }
Mat2 operator*(val_t scalar, const Mat2& m) { return internal::ops::mul(m, scalar); }
}