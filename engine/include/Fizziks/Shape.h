#pragma once
#include <Dense.h>

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

enum class ShapeType
{
    CIRCLE, AABB
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

struct Contact 
{
    Vector2p normal; 
    Vector2p contactPoint;  
    val_t penetration;  

    bool overlaps;
};

Shape createCircle(val_t radius);
Shape createAABB(val_t width, val_t height);

AABB getInscribingAABB(Shape s);

bool shapesOverlap(Shape s1, Vector2p p1, Shape s2, Vector2p p2);
Contact getShapeContact(Shape s1, Vector2p p1, Shape s2, Vector2p p2);
};
