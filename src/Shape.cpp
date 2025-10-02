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
    return (p2 - p1).squaredNorm() < ((c1.radius * c1.radius) + (c2.radius * c2.radius));
}
Contact CircleContactsCircle(Circle c1, Vector2p p1, Circle c2, Vector2p p2)
{
    Contact contact;

    Vector2p sep = p2 - p1;
    contact.normal = sep.normalized(); // this could cause issues if perfectly overlap
    contact.penetration = (c1.radius + c2.radius) - sep.norm();
    contact.contactPoint = p1 + contact.normal * c1.radius;

    return contact;
}

bool AABBOverlapsAABB(AABB r1, Vector2p p1, AABB r2, Vector2p p2)
{
    bool overlapX = abs(p1.x() - p2.x()) <= r1.halfWidth  + r2.halfWidth;
    bool overlapY = abs(p1.y() - p2.y()) <= r1.halfHeight + r2.halfHeight;
    return overlapX && overlapY;
}
Contact AABBContactsAABB(AABB r1, Vector2p p1, AABB r2, Vector2p p2)
{
    Contact contact;

    Vector2p sep = p2 - p1;
    val_t overlapX = r1.halfWidth  + r2.halfWidth  - abs(p1.x() - p2.x());
    val_t overlapY = r1.halfHeight + r2.halfHeight - abs(p1.y() - p2.y());

    if(overlapX < overlapY)
    {
        contact.penetration = overlapX;
        contact.normal = { 0, sep.y() < 0 ? -1 : 1 }; // should default to reverse direction of motion
    }
    else
    {
        contact.penetration = overlapY;
        contact.normal = { sep.x() < 0 ? -1 : 1, 0 }; // should default to reverse direction of motion
    }

    contact.contactPoint = p1 + contact.normal * (contact.penetration * 0.5);

    return contact;
}

bool CircleOverlapsAABB(Circle c, Vector2p p1, AABB r, Vector2p p2)
{
    val_t closestX = std::clamp(p1.x(), p2.x() - r.halfWidth , p2.x() + r.halfWidth );
    val_t closestY = std::clamp(p1.y(), p2.y() - r.halfHeight, p2.y() + r.halfHeight);
    return Vector2p(closestX, closestY).squaredNorm() < c.radius * c.radius;
}
Contact CircleContactsAABB(Circle c, Vector2p p1, AABB r, Vector2p p2)
{
    Contact contact;

    val_t closestX = std::clamp(p1.x(), p2.x() - r.halfWidth , p2.x() + r.halfWidth );
    val_t closestY = std::clamp(p1.y(), p2.y() - r.halfHeight, p2.y() + r.halfHeight);

    Vector2p closest = Vector2p(closestX, closestY);
    Vector2p sep = p1 - closest;
    contact.normal = sep.normalized(); // this could cause issues if perfectly overlap
    contact.penetration = c.radius - sep.norm();
    contact.contactPoint = closest;

    return contact;
}

bool shapesOverlap(Shape s1, Vector2p p1, Shape s2, Vector2p p2)
{
    if(s1.type == CIRCLE && s2.type == CIRCLE)
        return CircleOverlapsCircle(s1.circle, p1, s2.circle, p2);
    else if(s1.type == AABB_t && s2.type == AABB_t)
        return AABBOverlapsAABB(s1.aabb, p1, s2.aabb, p2);
    else if(s1.type == CIRCLE && s2.type == AABB_t)
        return CircleOverlapsAABB(s1.circle, p1, s2.aabb, p2);
    else if(s1.type == AABB_t && s2.type == CIRCLE)
        return CircleOverlapsAABB(s2.circle, p2, s1.aabb, p1);

    return false;
}

// could use a template here to remove gross branching?
Contact getShapeContact(Shape s1, Vector2p p1, Shape s2, Vector2p p2)
{
    Contact contact;
    contact.overlaps = false;

    if(!shapesOverlap(s1, p1, s2, p2))
        return contact;

    contact.overlaps = true;
    if(s1.type == CIRCLE && s2.type == CIRCLE)
        contact = CircleContactsCircle(s1.circle, p1, s2.circle, p2);
    else if(s1.type == AABB_t && s2.type == AABB_t)
        contact = AABBContactsAABB(s1.aabb, p1, s2.aabb, p2);
    else if(s1.type == CIRCLE && s2.type == AABB_t)
        contact = CircleContactsAABB(s1.circle, p1, s2.aabb, p2);
    else if(s1.type == AABB_t && s2.type == CIRCLE)
        contact = CircleContactsAABB(s2.circle, p2, s1.aabb, p1);

    return contact;
}
};
