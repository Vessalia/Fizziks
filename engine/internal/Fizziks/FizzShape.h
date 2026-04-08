#pragma once
#include <Fizziks/Shape.h>

namespace Fizziks::internal
{
struct InternalShape
{
	std::variant<Circle, Polygon, Compound> data;
};

// handles convex and concave compound shapes
struct Compound
{
	std::vector<Shape> convexPieces;
};

InternalShape toInternal(const Shape& shape);
Shape toExternal(const InternalShape& shape);

FIZZIKS_API val_t getMoI(const InternalShape& shape, val_t mass);

FIZZIKS_API AABB getEncapsulatingAABB(const InternalShape& s, const Vec2& centroid, val_t rot, bool tight = true);

FIZZIKS_API bool shapesOverlap(const InternalShape& s1, const Vec2& p1, val_t r1, const InternalShape& s2, const Vec2& p2, val_t r2);
FIZZIKS_API Contact getShapeContact(const InternalShape& s1, const Vec2& p1, val_t r1, const InternalShape& s2, const Vec2& p2, val_t r2);
}
