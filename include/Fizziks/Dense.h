#pragma once

#include <Eigen/Dense>

#ifndef FIZZIKS_DEFINED
#define FIZZIKS_DEFINED
#endif

#ifdef FIZZIKS_PRECISION_MODE
    #define val_t double
    #define Vector2p Eigen::Vector2d
    #define Vector3p Eigen::Vector3d
    #define Matrix2p Eigen::Matrix2d
    #define Matrix3p Eigen::Matrix3d
#else
    #define val_t float
    #define Vector2p Eigen::Vector2f
    #define Vector3p Eigen::Vector3f
    #define Matrix2p Eigen::Matrix2f
    #define Matrix3p Eigen::Matrix3f
#endif
