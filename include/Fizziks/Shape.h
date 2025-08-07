#pragma once
#include "Dense.h"

struct Circle
{
    val_t radius;
};

struct Rect
{
    val_t width;
    val_t height;
};

enum ShapeType
{
    CIRCLE, RECT
};

struct Shape
{
    ShapeType type;

    union
    {
        Circle circle;
        Rect rect;
    };
};
