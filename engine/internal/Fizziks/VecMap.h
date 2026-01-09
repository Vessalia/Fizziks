#pragma once

// This file is the glue between the user and Eigen/GLM vector/matrix types.

#include <Fizziks/Fizziks.h>
#include <Fizziks/Vec.h>

#ifdef FIZZIKS_USE_GLM
#else
#include <Eigen/Dense>
#endif

namespace Fizziks::internal
{
#ifdef FIZZIKS_USE_GLM

using Vector2p   = glm::vec<2, val_t>;
using Matrix2p   = glm::mat<2, 2, val_t>;
using Rotation2p = glm::mat<2, 2, val_t>;

inline const Vector2p map(const Vec2& vec)
{
    return reinterpret_cast<const Vector2p&>(vec);
}
inline const Vec2 map(const Vector2p& vec)
{
    return reinterpret_cast<const Vec2&>(vec);
}

inline Vector2p map(Vec2& vec)
{
    return reinterpret_cast<Vector2p&>(vec);
}
inline Vec2 map(Vector2p& vec)
{
    return reinterpret_cast<Vec2&>(vec);
}

inline Vector2p vec_zero()
{
    return Vector2p(0, 0);
}

#else

#ifdef FIZZIKS_PRECISION_MODE
    using Vector2p = Eigen::Vector2d;
#else
    using Vector2p = Eigen::Vector2f;
#endif
using Matrix2p   = Eigen::Matrix<val_t, 2, 2>;
using Rotation2p = Eigen::Rotation2D<val_t>;

inline Eigen::Map<const Vector2p> map(const Vec2& vec)
{
    return Eigen::Map<const Vector2p, Eigen::Unaligned>(reinterpret_cast<const val_t*>(&vec.x));
}
inline Eigen::Map<Vector2p> map(Vec2& vec)
{
    return Eigen::Map<Vector2p, Eigen::Unaligned>(reinterpret_cast<val_t*>(&vec.x));
}
inline Vec2 map(const Vector2p& vec)
{
    Vec2 result;
    map(result) = vec;
    return result;
}

inline Eigen::Map<const Matrix2p> map(const Mat2& mat)
{
    return Eigen::Map<const Matrix2p, Eigen::Unaligned>(reinterpret_cast<const val_t*>(&mat.m00));
}
inline Eigen::Map<Matrix2p> map(Mat2& mat)
{
    return Eigen::Map<Matrix2p, Eigen::Unaligned>(reinterpret_cast<val_t*>(&mat.m00));
}
inline Mat2 map(const Matrix2p& mat)
{
    Mat2 result;
    map(result) = mat;
    return result;
}

inline const Vector2p vec_zero()
{
    return Vector2p::Zero();
}

#endif
}
