#include "Shape.h"

namespace Fizziks
{
Shape createCircle(val_t radius)
{
    Shape circle;
    circle.type = ShapeType::CIRCLE;
    circle.circle.radius = radius;
    return circle;
}

Shape createAABB(val_t width, val_t height)
{
    Shape box;
    box.type = ShapeType::AABB_t;
    box.aabb.halfWidth  = width  / 2;
    box.aabb.halfHeight = height / 2;
    box.aabb;
    return box;
}

bool CircleOverlapsCircle(Circle c1, Vector2p p1, Circle c2, Vector2p p2)
{
    return getDist2(p1, p2) < ((c1.radius * c1.radius) + (c2.radius * c2.radius));
}

bool CircleOverlapsAABB(Circle c, Vector2p p1, AABB r, Vector2p p2)
{
    val_t closestX = std::clamp(p1.x(), p2.x() - r.halfWidth , p2.x() + r.halfWidth );
    val_t closestY = std::clamp(p1.y(), p2.y() - r.halfHeight, p2.y() + r.halfHeight);
    return Vector2p(closestX, closestY).squaredNorm() < c.radius * c.radius;
}

bool AABBOverlapsAABB(AABB r1, Vector2p p1, AABB r2, Vector2p p2)
{
    bool overlapX = abs(p1.x() - p2.x()) <= r1.halfWidth  + r2.halfWidth;
    bool overlapY = abs(p1.y() - p2.y()) <= r1.halfHeight + r2.halfHeight;
    return overlapX && overlapY;
}

bool shapesOverlap(Shape s1, Vector2p p1, Shape s2, Vector2p p2)
{
    if(s1.type == ShapeType::CIRCLE && s2.type == ShapeType::CIRCLE)
        return CircleOverlapsCircle(s1.circle, p1, s2.circle, p2);
    else if(s1.type == ShapeType::CIRCLE && s2.type == ShapeType::AABB_t)
        return CircleOverlapsAABB(s1.circle, p1, s2.aabb, p2);
    else if(s1.type == ShapeType::AABB_t && s2.type == ShapeType::CIRCLE)
        return CircleOverlapsAABB(s2.circle, p2, s1.aabb, p1);
    else if(s1.type == ShapeType::AABB_t && s2.type == ShapeType::AABB_t)
        return AABBOverlapsAABB(s1.aabb, p1, s2.aabb, p2);

    return false;
}
};
