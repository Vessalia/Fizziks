#include "Shape.h"

namespace Fizziks
{
static Shape createCircle(val_t radius)
{
    Shape circle;
    circle.type = ShapeType::CIRCLE;
    circle.circle.radius = radius;
    return circle;
}

static Shape createRectangle(val_t width, val_t height)
{
    Shape rect;
    rect.type = ShapeType::RECT;
    rect.rect.width = width;   rect.rect.halfWidth  = width  / 2;
    rect.rect.height = height; rect.rect.halfHeight = height / 2;
    return rect;
}

static bool circleOverlapsWithCircle(Circle c1, Vector2p p1, Circle c2, Vector2p p2)
{
    return getDist2(p1, p2) < ((c1.radius * c1.radius) + (c2.radius * c2.radius));
}

static bool circleOverlapsWithRect(Circle c, Vector2p p1, Rect r, Vector2p p2)
{
    val_t closestX = std::clamp(p1.x(), p2.x() - r.halfWidth , p2.x() + r.halfWidth );
    val_t closestY = std::clamp(p1.y(), p2.y() - r.halfHeight, p2.y() + r.halfHeight);
    return Vector2p(closestX, closestY).squaredNorm() < c.radius * c.radius;
}

static bool rectOverlapsWithRect(Rect r1, Vector2p p1, Rect r2, Vector2p p2)
{
    bool farRight  = p1.x() + r1.halfWidth  < p2.x() - r2.halfWidth;
    bool farLeft   = p1.x() - r1.halfWidth  > p2.x() + r2.halfWidth;
    bool farTop    = p1.y() + r1.halfHeight < p2.y() - r2.halfHeight;
    bool farBottom = p1.y() - r1.halfHeight > p2.y() + r2.halfHeight;
    return !(farRight || farLeft || farTop || farBottom);
}

static bool shapesOverlap(Shape s1, Vector2p p1, Shape s2, Vector2p p2)
{
    if(s1.type == ShapeType::CIRCLE && s2.type == ShapeType::CIRCLE)
        return circleOverlapsWithCircle(s1.circle, p1, s2.circle, p2);
    else if((s1.type == ShapeType::CIRCLE && s2.type == ShapeType::RECT)
         || (s1.type == ShapeType::RECT && s2.type == ShapeType::CIRCLE))
        {
            bool s1IsCircle = s1.type == ShapeType::CIRCLE;
            Circle c = s1IsCircle ? s1.circle : s2.circle; 
            Rect r = s1IsCircle ? s2.rect : s1.rect;
            Vector2p pc = s1IsCircle ? p1 : p2;
            Vector2p pr = s1IsCircle ? p2 : p1;
            return circleOverlapsWithRect(c, pc, r, pr);
        }
    else if(s1.type == ShapeType::RECT && s2.type == ShapeType::RECT)
        return rectOverlapsWithRect(s1.rect, p1, s2.rect, p2);

    return false;
}
};
