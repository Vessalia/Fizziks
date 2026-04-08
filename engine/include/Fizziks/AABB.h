#pragma once
#include <Fizziks/Fizziks.h>
#include <Fizziks/Vec.h>

namespace Fizziks
{
struct FIZZIKS_API AABB
{
	Vec2 min, max;
	val_t hw = 0, hh = 0;

	bool operator==(const AABB&) const = default;

	val_t area() const { return 4 * hw * hh; }
	val_t perimeter() const { return 2 * (hw + hh); }
};

FIZZIKS_API AABB createAABB(val_t width, val_t height, const Vec2& pos);
FIZZIKS_API AABB createAABB(const Vec2& min, const Vec2& max);

FIZZIKS_API bool overlaps(const AABB& a, const AABB& b);
FIZZIKS_API bool contains(const AABB& a, const Vec2& point);
FIZZIKS_API bool contains(const AABB& a, const AABB& b);
FIZZIKS_API AABB merge(const AABB& a, const AABB& b);
}
