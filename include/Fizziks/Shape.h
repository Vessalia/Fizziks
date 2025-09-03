#pragma once
#include "Dense.h"

namespace Fizziks
{
struct Circle
{
    val_t radius;
};

struct AABB
{
    val_t halfWidth;
    val_t halfHeight;
};

enum ShapeType
{
    CIRCLE, AABB_t
};

struct Shape
{
    ShapeType type;

    union
    {
        Circle circle;
        AABB aabb;
    };
};

Shape createCircle(val_t radius);
Shape createAABB(val_t width, val_t height);

bool shapesOverlap(Shape s1, Vector2p p1, Shape s2, Vector2p p2);
};
