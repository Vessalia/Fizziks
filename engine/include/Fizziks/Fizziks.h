#pragma once
#include <Eigen/Dense>
#include <Types.h>
#include <limits>

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
#define ASSERT_AND_CRASH(msg)\
do {\
    assert(false && msg);\
    int* crash = nullptr;\
    *crash;\
} while(0)

constexpr val_t PI = EIGEN_PI;
constexpr val_t TWO_PI = 2 * PI;

inline Vec2 vec_max() {
    return
    {
        std::numeric_limits<val_t>::max(),
        std::numeric_limits<val_t>::max()
    };
}

inline Vec2 vec_min() {
    return
    {
        std::numeric_limits<val_t>::lowest(),
        std::numeric_limits<val_t>::lowest()
    };
}

template<typename T>
T constexpr fizzmax() { return std::numeric_limits<T>::max(); }

template<typename T>
T constexpr fizzmin() { return std::numeric_limits<T>::min(); }

int mod(int a, int b);
val_t crossproduct(const Vec2& a, const Vec2& b);
Vec2 crossproduct(val_t w, const Vec2& r);
// (a x b) x c = b(a.c) - a(b.c)
Vec2 lefttriplecross(const Vec2& a, const Vec2& b, const Vec2& c);
// a x (b x c) = b(a.c) - c(a.b)
Vec2 righttriplecross(const Vec2& a, const Vec2& b, const Vec2& c);
};
