#pragma once
#include <Eigen/Dense>
#include <numeric>

#ifndef FIZZIKS_DEFINED
#define FIZZIKS_DEFINED
#endif

#if defined(_WIN32)
    #if defined(FIZZIKS_BUILDING_DLL)
        #define FIZZIKS_API __declspec(dllexport)
    #else
        #define FIZZIKS_API __declspec(dllimport)
    #endif
#else  
    #define FIZZIKS_API __attribute__((visibility("default")))
#endif

namespace Fizziks
{
#ifdef FIZZIKS_PRECISION_MODE
    typedef double val_t;
    typedef Eigen::Vector2d Vector2p;
    typedef Eigen::Vector3d Vector3p;
    typedef Eigen::Matrix2d Matrix2p;
    typedef Eigen::Matrix3d Matrix3p;
    typedef Eigen::Rotation2Dd Rotation2p;
#else
    typedef float val_t;
    typedef Eigen::Vector2f Vector2p;
    typedef Eigen::Vector3f Vector3p;
    typedef Eigen::Matrix2f Matrix2p;
    typedef Eigen::Matrix3f Matrix3p;
    typedef Eigen::Rotation2Df Rotation2p;
#endif

#define ASSERT_AND_CRASH(msg)\
do {\
    assert(false && msg);\
    int* crash = nullptr;\
    *crash;\
} while(0)\

constexpr val_t PI = EIGEN_PI;
constexpr val_t TWO_PI = 2 * PI;

inline Vector2p vec_max() {
    return Vector2p
    (
        std::numeric_limits<val_t>::max(),
        std::numeric_limits<val_t>::max()
    );
}

inline Vector2p vec_min() {
    return Vector2p
    (
        std::numeric_limits<val_t>::min(),
        std::numeric_limits<val_t>::min()
    );
}

template<typename T>
T constexpr fizzmax() { return std::numeric_limits<T>::max(); }

template<typename T>
T constexpr fizzmin() { return std::numeric_limits<T>::min(); }

int mod(int a, int b);
val_t crossproduct(const Vector2p& a, const Vector2p& b);
Vector2p crossproduct(val_t w, const Vector2p& r);
// (a x b) x c = b(a.c) - a(b.c)
Vector2p lefttriplecross(const Vector2p& a, const Vector2p& b, const Vector2p& c);
// a x (b x c) = b(a.c) - c(a.b)
Vector2p righttriplecross(const Vector2p& a, const Vector2p& b, const Vector2p& c);
};
