#pragma once

namespace Fizziks
{    
#ifdef FIZZIKS_PRECISION_MODE
    typedef double val_t;
#else
    typedef float val_t;
#endif

struct Vec2
{
    val_t x = 0;
    val_t y = 0;
};

struct Mat2
{
    val_t m00 = 0, m01 = 0;
    val_t m10 = 0, m11 = 0;
};

struct Vec3
{
    val_t x = 0;
    val_t y = 0;
    val_t z = 0;
};

struct Mat3
{
    val_t m00 = 0, m01 = 0, m02 = 0;
    val_t m10 = 0, m11 = 0, m12 = 0;
    val_t m20 = 0, m21 = 0, m22 = 0;
};
};
