#include <Fizziks/AABB.h>

#include <algorithm>

namespace Fizziks
{
AABB createAABB(val_t width, val_t height, const Vec2& pos)
{
	const Vec2 extent = { width / 2, height / 2 };
	return { pos - extent, pos + extent, width / 2, height / 2 };
}

AABB createAABB(const Vec2& min, const Vec2& max)
{
	const Vec2 extent = max - min;
	return { min, max, extent.x / 2, extent.y / 2 };
}

bool overlaps(const AABB& a, const AABB& b)
{
	bool overlapX = a.min.x <= b.max.x && b.min.x <= a.max.x;
	bool overlapY = a.min.y <= b.max.y && b.min.y <= a.max.y;
	return overlapX && overlapY;
}
bool contains(const AABB& a, const Vec2& point)
{
	bool containX = a.min.x <= point.x && point.x <= a.max.x;
	bool containY = a.min.y <= point.y && point.y <= a.max.y;
	return containX && containY;
}
bool contains(const AABB& a, const AABB& b)
{
	bool containX = a.min.x <= b.min.x && b.max.x <= a.max.x;
	bool containY = a.min.y <= b.min.y && b.max.y <= a.max.y;
	return containX && containY;
}
AABB merge(const AABB& a, const AABB& b)
{
	const Vec2 min = { std::min(a.min.x, b.min.x), std::min(a.min.y, b.min.y) };
	const Vec2 max = { std::max(a.max.x, b.max.x), std::max(a.max.y, b.max.y) };
	return createAABB(min, max);
}
}
