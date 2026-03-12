#include <Fizziks/Ray.h>
#include <Fizziks/Vec.h>
#include <Fizziks/MathUtils.h>

namespace Fizziks
{
val_t raytest(const Ray& _ray, const AABB& aabb, const Vec2& pos)
{
	if (!_ray.dir.norm()) return -1;
	Ray ray = _ray;
	ray.dir.normalize();

	Vec2 low  = pos - Vec2(aabb.hw, aabb.hh); // bottom left
	Vec2 high = pos + Vec2(aabb.hw, aabb.hh); // bottom right

	// x slab intersections
	val_t tminX, tmaxX;
	if (ray.dir.x == 0)
	{
		if (ray.pos.x < low.x || ray.pos.x > high.x) return -1;
		tminX = fizzmin<val_t>(); tmaxX = fizzmax<val_t>();
	}
	else
	{
		tminX = (low.x  - ray.pos.x) / ray.dir.x;
		tmaxX = (high.x - ray.pos.x) / ray.dir.x;
		if (tminX > tmaxX) std::swap(tminX, tmaxX);
	}

	// y slab intersections
	val_t tminY, tmaxY;
	if (ray.dir.y == 0)
	{
		if (ray.pos.y < low.y || ray.pos.y > high.y) return -1;
		tminY = fizzmin<val_t>(); tmaxY = fizzmax<val_t>();
	}
	else
	{
		tminY = (low.y  - ray.pos.y) / ray.dir.y;
		tmaxY = (high.y - ray.pos.y) / ray.dir.y;
		if (tminY > tmaxY) std::swap(tminY, tmaxY);
	}

	val_t tclose = std::max(tminX, tminY); // furthest entry, need to enter both slabs to intersect the AABB
	val_t tfar   = std::min(tmaxX, tmaxY); // closest  exit , first slab exited exits the AABB

	if (tclose > tfar || tfar < 0) return -1;  // miss or behind

	return (tclose >= 0) ? tclose : 0;
}
}
