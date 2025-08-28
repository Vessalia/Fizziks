#pragma once

#include <Eigen/Dense>
#include <numeric>

#ifndef FIZZIKS_DEFINED
#define FIZZIKS_DEFINED
#endif

namespace Fizziks
{
#ifdef FIZZIKS_PRECISION_MODE
    typedef double val_t;
    typedef Eigen::Vector2d Vector2p;
    typedef Eigen::Vector3d Vector3p;
    typedef Eigen::Matrix2d Matrix2p;
    typedef Eigen::Matrix3d Matrix3p;
#else
    typedef float val_t;
    typedef Eigen::Vector2f Vector2p;
    typedef Eigen::Vector3f Vector3p;
    typedef Eigen::Matrix2f Matrix2p;
    typedef Eigen::Matrix3f Matrix3p;
#endif

template<typename T>
T fizzmax()
{
    return std::numeric_limits<T>::max();
}

template<typename T>
T fizzmin()
{
    return std::numeric_limits<T>::min();
}

val_t getDist2(Vector2p v1, Vector2p v2);
val_t getDist2(Vector3p v1, Vector3p v2);
};
