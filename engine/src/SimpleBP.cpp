#include <Fizziks/SimpleBP.h>
#include <Fizziks/MathUtils.h>

namespace Fizziks::internal
{
void SimpleBP::replace(uint32_t prevID, uint32_t newID)
{
	if(bodies.contains(prevID)) 
	{
		bodies[newID] = bodies[prevID];
		remove(prevID);
	}
}

CollisionPairs SimpleBP::computePairs(void)
{
	CollisionPairs pairs;

	for (auto itA = bodies.begin(); itA != bodies.end(); ++itA) 
	{
		auto [idA, aabbA] = *itA;

		auto itB = itA; ++itB;
		for(; itB != bodies.end(); ++itB) 
		{
			auto [idB, aabbB] = *itB;

			if(overlaps(aabbA, aabbB))
				pairs.push_back({idA, idB});
		}
	}

	return pairs;
}

uint32_t SimpleBP::pick(const Vec2& point) const
{
	for (const auto& [id, aabb] : bodies)
	{
		if (contains(aabb, point))
			return id;
	}

	return fizzmax<uint32_t>();
}

std::vector<uint32_t> SimpleBP::query(const AABB& aabb) const
{
	std::vector<uint32_t> IDs;
	for (const auto& [id, o_aabb] : bodies)
	{
		if(overlaps(o_aabb, aabb))
			IDs.push_back(id);
	}

	return IDs;
}

RaycastResult SimpleBP::raycast(const Ray& _ray) const
{
	RaycastResult closest;
	closest.hit = false;
	Ray ray = _ray;
	if(ray.dir.norm() == 0) return closest;
	ray.dir.normalize();

	val_t closestT = fizzmax<val_t>();
	for(const auto& [id, aabb] : bodies)
	{
		// x slab intersections
		val_t tminX, tmaxX;
		if (ray.dir.x == 0) 
		{
			if (ray.pos.x < aabb.min.x || aabb.max.x < ray.pos.x) continue;
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
			if (ray.pos.y < aabb.min.y || aabb.max.y < ray.pos.y) continue;
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

		if (tclose > tfar || tfar < 0) continue; // miss or behind

		val_t hitT = (tclose >= 0) ? tclose : 0;
		if (hitT >= closestT) continue;

		const val_t one = val_t(1.0);
		val_t sx    = std::copysign(one, ray.dir.x);
		val_t sy    = std::copysign(one, ray.dir.y);
		val_t entry = (tminX > tminY) ? one : 0; // 1 if x, 0 if y
		val_t sign  = (tclose >= 0.0) ? -one : one; // -1 if start outside, +1 if start inside
		Vec2 normal = sign * Vec2(entry * sx, (one - entry) * sy);

		closest.hit    = true;
		closest.ID     = id;
		closest.normal = normal;
		closest.point  = ray.pos + hitT * ray.dir;
		closestT       = hitT;
	}

	return closest;
}

std::vector<AABB> SimpleBP::getDebugInfo() const
{
	std::vector<AABB> debug;
	for (const auto& [id, aabb] : bodies)
	{
		debug.push_back(aabb);
	}

	return debug;
}
}
