#include <Fizziks/Vec.h>
#include <Fizziks/VecMap.h>

namespace Fizziks
{
val_t Vec2::dot(const Vec2& other) const
{
    return internal::map(*this).dot(internal::map(other));
}

val_t Vec2::norm() const
{
    return internal::map(*this).norm();
}

val_t Vec2::squaredNorm() const
{
    return internal::map(*this).squaredNorm();
}

Vec2 Vec2::normalized() const
{
    return internal::map(internal::map(*this).normalized());
}

Vec2& Vec2::normalize()
{
    internal::map(*this).normalize();
    return *this;
}

Vec2 Vec2::rotated(const val_t angle) const
{
    return internal::map((internal::Rotation2p(angle) * internal::map(*this)).eval());
}

Vec2& Vec2::rotate(const val_t angle)
{
    internal::map(*this) = internal::Rotation2p(angle) * internal::map(*this);
    return *this;
}

Vec2 Vec2::operator+(const Vec2& other) const
{
    return internal::map((internal::map(*this) + internal::map(other)).eval());
}

Vec2 Vec2::operator-(const Vec2& other) const
{
    return internal::map((internal::map(*this) - internal::map(other)).eval());
}

Vec2 Vec2::operator*(const val_t scalar) const
{
    return internal::map((internal::map(*this) * scalar).eval());
}

Vec2 Vec2::operator/(const val_t scalar) const
{
    return internal::map((internal::map(*this) / scalar).eval());
}

Vec2& Vec2::operator+=(const Vec2& other)
{
    internal::map(*this) += internal::map(other);
    return *this;
}

Vec2& Vec2::operator-=(const Vec2& other)
{
    internal::map(*this) -= internal::map(other);
    return *this;
}

Vec2& Vec2::operator*=(const val_t scalar)
{
    internal::map(*this) *= scalar;
    return *this;
}

Vec2& Vec2::operator/=(const val_t scalar)
{
    internal::map(*this) /= scalar;
    return *this;
}

Vec2 operator-(const Vec2& v)
{
    return internal::map((-internal::map(v)).eval());
}

Vec2 operator*(const val_t scalar, const Vec2& v)
{
    return v * scalar;
}

Vec2 Mat2::operator*(const Vec2& vec) const
{
    return internal::map((internal::map(*this) * internal::map(vec)).eval());
}
}
