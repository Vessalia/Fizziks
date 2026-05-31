#pragma once
#include <Fizziks/Fizziks.h>
#include <Fizziks/Vec.h>
#include <Fizziks/AABB.h>

#include <vector>
#include <variant>

// NOTE: positions and rotations here are relative to world-space, and about the centroid, not the user-based position
namespace Fizziks
{
struct Circle
{
	val_t radius;
};

struct Ellipse
{
	val_t rx;
	val_t ry;
};

struct Rect
{
	val_t width;
	val_t height;
};

struct Polygon
{
	std::vector<Vec2> vertices;
};

struct Capsule
{
	val_t capHeight;
	Rect body;
};

using Shape = std::variant<Circle, Ellipse, Rect, Polygon, Capsule>;

struct Contact 
{
	Vec2 contactPointWorldA, contactPointWorldB;
	Vec2 contactPointLocalA, contactPointLocalB;
	Vec2 normal;
	Vec2 tangent;
	val_t penetration;

	uint32_t featureA, featureB; // for tracking specific collisions

	bool overlaps;
};

FIZZIKS_API Circle createCircle(val_t radius);
FIZZIKS_API Ellipse createEllipse(val_t rx, val_t ry);
FIZZIKS_API Rect createRect(val_t width, val_t height);
FIZZIKS_API Polygon createPolygon(const std::vector<Vec2>& vertices);
FIZZIKS_API Capsule createCapsule(val_t capHeight, const Rect& body);

FIZZIKS_API val_t getMoI(const Shape& shape, val_t mass);

FIZZIKS_API AABB getBounds(const Shape& s, const Vec2& centroid, val_t rot, bool tight = true);

FIZZIKS_API bool shapesOverlap(const Shape& s1, const Vec2& p1, val_t r1, const Shape& s2, const Vec2& p2, val_t r2);
FIZZIKS_API Contact getShapeContact(const Shape& s1, const Vec2& p1, val_t r1, const Shape& s2, const Vec2& p2, val_t r2);
}
