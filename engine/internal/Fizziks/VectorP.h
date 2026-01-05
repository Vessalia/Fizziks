#pragma once

#include <Types.h>

#ifdef FIZZIKS_USE_GLM
#else
#include <Eigen/Dense>
#endif

namespace Fizziks::internal
{
#ifdef FIZZIKS_USE_GLM

using Vector2p   = glm::vec<2, val_t>;
using Vector3p   = glm::vec<3, val_t>;
using Matrix2p   = glm::mat<2, 2, val_t>;
using Matrix3p   = glm::mat<3, 3, val_t>;
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

inline const Vector3p map(const Vec3& vec)
{
    return reinterpret_cast<const Vector3p&>(vec);
}
inline const Vec3 map(const Vector3p& vec)
{
    return reinterpret_cast<const Vec3&>(vec);
}

inline Vector3p map(Vec3& vec)
{
    return reinterpret_cast<Vector3p&>(vec);
}
inline Vec3 map(Vector3p& vec)
{
    return reinterpret_cast<Vec3&>(vec);
}

inline Vector2p rotate(const Vec2& vec, const val_t angle)
{
    return rotate(map(vec), angle);
}

inline Vector2p rotate(const Vector2p& vec, const val_t angle)
{
    val_t c = std::cos(angle);
    val_t s = std::sin(angle);
    Rotation2p rot = Rotation2p(c, -s,
                                s,  c);
    return rot * vec;
}

inline Vector2p vec_zero()
{
    return Vector2p(0, 0);
}
inline Vector3p vec3_zero()
{
    return Vector3p(0, 0, 0);
}

#else

#ifdef FIZZIKS_PRECISION_MODE
    using Vector2p = Eigen::Vector2d;
    using Vector3p = Eigen::Vector3d;
#else
    using Vector2p = Eigen::Vector2f;
    using Vector3p = Eigen::Vector3f;
#endif
using Matrix2p   = Eigen::Matrix<val_t, 2, 2>;
using Matrix3p   = Eigen::Matrix<val_t, 3, 3>;
using Rotation2p = Eigen::Rotation2D<val_t>;

inline Eigen::Map<const Vector2p> map(const Vec2& vec)
{
    return Eigen::Map<const Vector2p>(reinterpret_cast<const val_t*>(&vec.x));
}
inline const Vec2 map(const Vector2p& vec)
{
    Vec2 result;
    Eigen::Map<const Vector2p, Eigen::Unaligned>(reinterpret_cast<val_t*>(&result.x)) = vec;
    return result;
}

inline Eigen::Map<Vector2p> map(Vec2& vec)
{
    return Eigen::Map<Vector2p, Eigen::Unaligned>(reinterpret_cast<val_t*>(&vec.x));
}
inline Vec2 map(Vector2p& vec)
{
    Vec2 result;
    Eigen::Map<Vector2p, Eigen::Unaligned>(reinterpret_cast<val_t*>(&result.x)) = vec;
    return result;
}

inline Eigen::Map<const Vector3p> map(const Vec3& vec)
{
    return Eigen::Map<const Vector3p, Eigen::Unaligned>(reinterpret_cast<const val_t*>(&vec.x));
}
inline const Vec3 map(const Vector3p& vec)
{
    Vec3 result;
    Eigen::Map<const Vector3p, Eigen::Unaligned>(reinterpret_cast<val_t*>(&result.x)) = vec;
    return result;
}

inline Eigen::Map<Vector3p> map(Vec3& vec)
{
    return Eigen::Map<Vector3p, Eigen::Unaligned>(reinterpret_cast<val_t*>(&vec.x));
}
inline Vec3 map(Vector3p& vec)
{
    Vec3 result;
    Eigen::Map<Vector3p, Eigen::Unaligned>(reinterpret_cast<val_t*>(&result.x)) = vec;
    return result;
}

inline Vector2p rotate(const Vec2& vec, const val_t angle)
{
    return rotate(map(vec), angle);
}

inline Vector2p rotate(const Vector2p& vec, const val_t angle)
{
    return Rotation2p(angle) * vec;
}

inline const Vector2p vec_zero()
{
    return Vector2p::Zero();
}   
inline const Vector3p vec3_zero()
{
    return Vector3p::Zero();
}

#endif

val_t crossproduct(const Vector2p& a, const Vector2p& b);
Vector2p crossproduct(val_t w, const Vector2p& r);
// (a x b) x c = b(a.c) - a(b.c)
Vector2p lefttriplecross(const Vector2p& a, const Vector2p& b, const Vector2p& c);
// a x (b x c) = b(a.c) - c(a.b)
Vector2p righttriplecross(const Vector2p& a, const Vector2p& b, const Vector2p& c);
};
