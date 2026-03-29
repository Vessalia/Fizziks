#include <Fizziks/Ray.h>
#include <Fizziks/Vec.h>
#include <Fizziks/MathUtils.h>
#include <Fizziks/FizzLog.h>

namespace Fizziks
{
val_t raytest(const Ray& _ray, const AABB& aabb)
{
	if (!_ray.dir.norm()) 
	{
		FIZZIKS_LOG_WARNING("Ray of length 0 is poorly defined and cannot properly test for intersections");
		return -1;
	}

	Ray ray = _ray;
	ray.dir.normalize();
	
	// x slab intersections
	val_t tminX, tmaxX;
	if (ray.dir.x == 0)
	{
		if (ray.pos.x < aabb.min.x || aabb.max.x < ray.pos.x) return -1;
		tminX = fizzmin<val_t>(); tmaxX = fizzmax<val_t>();
	}
	else
	{
		tminX = (aabb.min.x - ray.pos.x) / ray.dir.x;
		tmaxX = (aabb.max.x - ray.pos.x) / ray.dir.x;
		if (tminX > tmaxX) std::swap(tminX, tmaxX);
	}

	// y slab intersections
	val_t tminY, tmaxY;
	if (ray.dir.y == 0)
	{
		if (ray.pos.y < aabb.min.y || aabb.max.y < ray.pos.y) return -1;
		tminY = fizzmin<val_t>(); tmaxY = fizzmax<val_t>();
	}
	else
	{
		tminY = (aabb.min.y - ray.pos.y) / ray.dir.y;
		tmaxY = (aabb.max.y - ray.pos.y) / ray.dir.y;
		if (tminY > tmaxY) std::swap(tminY, tmaxY);
	}

	val_t tclose = std::max(tminX, tminY); // furthest entry, need to enter both slabs to intersect the AABB
	val_t tfar   = std::min(tmaxX, tmaxY); // closest  exit , first slab exited exits the AABB

	if (tfar < tclose || tfar < 0) 
	{
		FIZZIKS_LOG_INFO("Raycast miss");
		return -1;  // miss or behind
	}

	return (tclose >= 0) ? tclose : 0;
}
}
