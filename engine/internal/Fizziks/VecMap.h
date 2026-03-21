#pragma once
// Internal glue between Fizziks types and the selected math backend.
// Never include this from public headers.
#include <Fizziks/Fizziks.h>
#include <Fizziks/Vec.h>
#include <bit>

#ifdef FIZZIKS_USE_GLM
    #ifdef GLM_FORCE_ROW_MAJOR
        #error "GLM_FORCE_ROW_MAJOR must not be defined — Fizziks requires column-major matrices"
    #endif
    #include <glm/glm.hpp>
#else
    #include <Eigen/Dense>
#endif

namespace Fizziks::internal
{
#ifdef FIZZIKS_USE_GLM

using Vector2p   = glm::vec<2, val_t>;
using Matrix2p   = glm::mat<2, 2, val_t>;
using Rotation2p = glm::mat<2, 2, val_t>;

static_assert(sizeof(Vec2)  == sizeof(Vector2p),  "Vec2 and Vector2p size mismatch");
static_assert(sizeof(Mat2)  == sizeof(Matrix2p),  "Mat2 and Matrix2p size mismatch");
static_assert(alignof(Vec2) == alignof(Vector2p), "Vec2 and Vector2p alignment mismatch");
static_assert(alignof(Mat2) == alignof(Matrix2p), "Mat2 and Matrix2p alignment mismatch");

inline Vector2p map(const Vec2& vec) { return std::bit_cast<Vector2p>(vec); }
inline Vec2     map(const Vector2p& vec) { return std::bit_cast<Vec2>(vec); }
inline Matrix2p map(const Mat2& mat) { return std::bit_cast<Matrix2p>(mat); }
inline Mat2     map(const Matrix2p& mat) { return std::bit_cast<Mat2>(mat); }

inline Vector2p vec_zero() { return Vector2p(0, 0); }

#else

#ifdef FIZZIKS_PRECISION_MODE
    using Vector2p = Eigen::Vector2d;
#else
    using Vector2p = Eigen::Vector2f;
#endif
using Matrix2p = Eigen::Matrix<val_t, 2, 2, Eigen::ColMajor | Eigen::DontAlign>;
using Rotation2p = Eigen::Rotation2D<val_t>;

static_assert(sizeof(Vec2)  == sizeof(Vector2p),  "Vec2 and Vector2p size mismatch");
static_assert(sizeof(Mat2)  == sizeof(Matrix2p),  "Mat2 and Matrix2p size mismatch");
static_assert(alignof(Vec2) == alignof(Vector2p), "Vec2 and Vector2p alignment mismatch");
static_assert(alignof(Mat2) == alignof(Matrix2p), "Mat2 and Matrix2p alignment mismatch");

inline Eigen::Map<Vector2p>       map(Vec2& vec)       { return Eigen::Map<Vector2p>(reinterpret_cast<val_t*>(&vec.x)); }
inline Eigen::Map<Matrix2p>       map(Mat2& mat)       { return Eigen::Map<Matrix2p>(reinterpret_cast<val_t*>(&mat.m00)); }
inline Eigen::Map<const Vector2p> map(const Vec2& vec) { return Eigen::Map<const Vector2p>(reinterpret_cast<const val_t*>(&vec.x)); }
inline Eigen::Map<const Matrix2p> map(const Mat2& mat) { return Eigen::Map<const Matrix2p>(reinterpret_cast<const val_t*>(&mat.m00)); }

inline Vec2 map(const Vector2p& vec) { return { vec.x(), vec.y() }; }
inline Mat2 map(const Matrix2p& mat) { return { mat(0,0), mat(1,0), mat(0,1), mat(1,1) }; }

inline Vector2p vec_zero() { return Vector2p::Zero(); }

#endif
}
