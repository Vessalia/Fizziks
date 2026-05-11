#pragma once

#include <Fizziks/Fizziks.h>
#include <Fizziks/Shape.h>

#include <vector>
#include <variant>

namespace Fizziks::internal
{
struct Ellipse
{
	bool userCircle;
	val_t rx;
	val_t ry;
};

struct Polygon
{
	std::vector<Vec2> vertices;
	val_t effectiveRadius;
};

using Primitive = std::variant<Ellipse, Polygon>;

// handles convex and concave compound shapes
struct ConvexPiece
{
	Primitive shape;
	Vec2 offset;
	Mat2 rot;
};

struct Compound
{
	std::vector<ConvexPiece> pieces;
	val_t effectiveRadius;
	Shape external; // a bit heavy weight, but removes the issue of ambiguously rebuilding special shapes
};

using InternalShape = std::variant<Ellipse, Polygon, Compound>;

InternalShape toInternal(const Shape& shape);
Shape toExternal(const InternalShape& shape);

FIZZIKS_API val_t getMoI(const InternalShape& shape, val_t mass);

FIZZIKS_API bool shapesOverlap(const InternalShape& s1, const Vec2& p1, val_t r1, const InternalShape& s2, const Vec2& p2, val_t r2);
FIZZIKS_API Contact getShapeContact(const InternalShape& s1, const Vec2& p1, val_t r1, const InternalShape& s2, const Vec2& p2, val_t r2);
}
