#pragma once
#include <Eigen/Dense>

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

#ifdef FIZZIKS_PRECISION_MODE
    using val_t = double;
#else
    using val_t = float;
#endif
}
