#pragma once
// Internal backend abstraction.
#include <Fizziks/VecMap.h>
#include <cmath>

#ifdef FIZZIKS_USE_GLM
#include <glm/gtx/rotate_vector.hpp>
#include <glm/gtx/norm.hpp>
#endif

// ---------------------------------------------------------------------------
// Backend traits.
// Provides eval() and mat_cast() as no-ops on GLM, and the real thing on Eigen.
// ---------------------------------------------------------------------------
namespace Fizziks::internal
{
#ifdef FIZZIKS_USE_GLM

template<typename T> const T& eval(const T& x)     { return x; }
template<typename T> const T& mat_cast(const T& x) { return x; }

// Vec2
inline val_t    vec_dot       (const Vector2p& a, const Vector2p& b)          { return glm::dot(a, b); }
inline val_t    vec_norm      (const Vector2p& v)                             { return glm::length(v); }
inline val_t    vec_norm2     (const Vector2p& v)                             { return glm::length2(v); }
inline Vector2p vec_normalize (const Vector2p& v)                             { return glm::normalize(v); }
inline void     vec_normalize_inplace (Vec2& v)                               { map(v) = glm::normalize(map(v)); }
inline Vector2p vec_rotate    (const Vector2p& v, val_t a)                    { return glm::rotate(v, a); }
inline Vector2p vec_lerp      (const Vector2p& a, const Vector2p& b, val_t t) { return glm::mix(a, b, t); }

// Mat2
inline Vector2p mat_col       (const Matrix2p& m, int i)                      { return m[i]; }
inline Vector2p mat_row       (const Matrix2p& m, int i)                      { return { m[0][i], m[1][i] }; }
inline val_t    mat_det       (const Matrix2p& m)                             { return glm::determinant(m); }
inline Matrix2p mat_transpose (const Matrix2p& m)                             { return glm::transpose(m); }
inline Matrix2p mat_inverse   (const Matrix2p& m)                             { return glm::inverse(m); }

#else

template<typename T> auto      eval    (const T& x) { return x.eval(); }
template<typename T> Matrix2p  mat_cast(const T& x) { return static_cast<Matrix2p>(x); }

// Vec2
inline val_t    vec_dot       (const Vector2p& a, const Vector2p& b)          { return a.dot(b); }
inline val_t    vec_norm      (const Vector2p& v)                             { return v.norm(); }
inline val_t    vec_norm2     (const Vector2p& v)                             { return v.squaredNorm(); }
inline Vector2p vec_normalize (const Vector2p& v)                             { return v.normalized(); }
inline void     vec_normalize_inplace (Vec2& v)                               { map(v).normalize(); }
inline Vector2p vec_rotate    (const Vector2p& v, val_t a)                    { return (Rotation2p(a) * v).eval(); }
inline Vector2p vec_lerp      (const Vector2p& a, const Vector2p& b, val_t t) { return (a + t * (b - a)).eval(); }

// Mat2
inline Vector2p mat_col       (const Matrix2p& m, int i)                      { return m.col(i); }
inline Vector2p mat_row       (const Matrix2p& m, int i)                      { return m.row(i).transpose(); }
inline val_t    mat_det       (const Matrix2p& m)                             { return m.determinant(); }
inline Matrix2p mat_transpose (const Matrix2p& m)                             { return static_cast<Matrix2p>(m.transpose()); }
inline Matrix2p mat_inverse   (const Matrix2p& m)                             { return static_cast<Matrix2p>(m.inverse()); }

#endif
}

// ---------------------------------------------------------------------------
// ops -> backend-agnostic
// ---------------------------------------------------------------------------
namespace Fizziks::internal::ops
{
// Vec2
inline Vec2  lerp        (const Vec2& a, const Vec2& b, val_t t) { return map(vec_lerp(map(a), map(b), t)); }
inline val_t dot         (const Vec2& a, const Vec2& b)          { return vec_dot(map(a), map(b)); }
inline val_t cross       (const Vec2& a, const Vec2& b)          { return a.x * b.y - a.y * b.x; }
inline Vec2  cross       (const Vec2& v, val_t z)                { return {z * -v.y, z * v.x}; }
inline val_t norm        (const Vec2& v)                         { return vec_norm(map(v)); }
inline val_t squaredNorm (const Vec2& v)                         { return vec_norm2(map(v)); }
inline Vec2  normalized  (const Vec2& v)                         { return map(vec_normalize(map(v))); }
inline void  normalize   (Vec2& v)                               { vec_normalize_inplace(v); }
inline Vec2  rotated     (const Vec2& v, val_t a)                { return map(vec_rotate(map(v), a)); }
inline void  rotate      (Vec2& v, val_t a)                      { map(v) = vec_rotate(map(v), a); }
inline Vec2  add         (const Vec2& a, const Vec2& b)          { return map(eval(map(a) + map(b))); }
inline Vec2  sub         (const Vec2& a, const Vec2& b)          { return map(eval(map(a) - map(b))); }
inline Vec2  mul         (const Vec2& v, val_t s)                { return map(eval(map(v) * s)); }
inline Vec2  div         (const Vec2& v, val_t s)                { return map(eval(map(v) / s)); }
inline Vec2  negate      (const Vec2& v)                         { return map(eval(-map(v))); }

// Mat2
inline Vec2  col         (const Mat2& m, int i)                  { return map(mat_col(map(m), i)); }
inline Vec2  row         (const Mat2& m, int i)                  { return map(mat_row(map(m), i)); }
inline Mat2  add         (const Mat2& a, const Mat2& b)          { return map(mat_cast(eval(map(a) + map(b)))); }
inline Mat2  sub         (const Mat2& a, const Mat2& b)          { return map(mat_cast(eval(map(a) - map(b)))); }
inline Mat2  mul         (const Mat2& m, val_t s)                { return map(mat_cast(eval(map(m) * s))); }
inline Vec2  mul         (const Mat2& m, const Vec2& v)          { return map(eval(map(m) * map(v))); }
inline Mat2  mul         (const Mat2& a, const Mat2& b)          { return map(mat_cast(eval(map(a) * map(b)))); }
inline Mat2  div         (const Mat2& m, val_t s)                { return map(mat_cast(eval(map(m) / s))); }
inline Mat2  negate      (const Mat2& m)                         { return map(mat_cast(eval(-map(m)))); }
inline Mat2  transposed  (const Mat2& m)                         { return map(mat_transpose(map(m))); }
inline void  transpose   (Mat2& m)                               { map(m) = mat_transpose(map(m)); }
inline val_t determinant (const Mat2& m)                         { return mat_det(map(m)); }
inline Mat2  inverse     (const Mat2& m)                         { return map(mat_inverse(map(m))); }
inline void  invert      (Mat2& m)                               { map(m) = mat_inverse(map(m)); }
}