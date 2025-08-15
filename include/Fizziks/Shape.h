#pragma once
#include "Dense.h"

namespace Fizziks
{
struct Circle
{
    val_t radius;
};

struct Rect
{
    val_t width, halfWidth;
    val_t height, halfHeight;
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

static Shape createCircle(val_t radius);
static Shape createRectangle(val_t width, val_t height);

static bool shapesOverlap(Shape s1, Vector2p p1, Shape s2, Vector2p p2);
};
