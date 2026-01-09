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

CollisionPairs& SimpleBP::computePairs(void)
{
    pairs.clear();

    for (auto itA = bodies.begin(); itA != bodies.end(); ++itA) 
    {
        auto [idA, entryA] = *itA;
        auto [aabbA, posA] = entryA;

        auto itB = itA; ++itB;
        for(; itB != bodies.end(); ++itB) 
        {
            auto [idB, entryB] = *itB;
            auto [aabbB, posB] = entryB;

            if(AABBOverlapsAABB(aabbA, posA, aabbB, posB))
                pairs.push_back({idA, idB});
        }
    }

    return pairs;
}

uint32_t SimpleBP::pick(const Vec2& point) const
{
    for (const auto& [id, entry] : bodies)
    {
        auto [aabb, pos] = entry;
        if (AABBContains(aabb, pos, point))
            return id;
    }

    return fizzmax<uint32_t>();
}

std::vector<uint32_t> SimpleBP::query(const AABB& aabb, const Vec2& pos) const
{
    std::vector<uint32_t> IDs;
    for (const auto& [id, entry] : bodies)
    {
        auto [o_aabb, o_pos] = entry;
        if(AABBOverlapsAABB(aabb, pos, o_aabb, o_pos))
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
    for(const auto& [id, entry] : bodies)
    {
        auto [aabb, pos] = entry;
        Vec2 low  = pos - Vec2(aabb.halfWidth, aabb.halfHeight); // bottom left
        Vec2 high = pos + Vec2(aabb.halfWidth, aabb.halfHeight); // top right

        // x slab intersections
        val_t tminX, tmaxX;
        if (ray.dir.x == 0) 
        {
            if (ray.pos.x < low.x || ray.pos.x > high.x) continue;
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
        if (ray.dir.x == 0) 
        {
            if (ray.pos.x < low.x || ray.pos.x > high.x) continue;
            tminY = fizzmin<val_t>(); tmaxY = fizzmax<val_t>();
        } 
        else 
        {
            tminY = (low.x  - ray.pos.x) / ray.dir.x;
            tmaxY = (high.x - ray.pos.x) / ray.dir.x;
            if (tminY > tmaxY) std::swap(tminY, tmaxY);
        }

        val_t tclose = std::max(tminX, tminY); // furthest entry, need to enter both slabs to intersect the AABB
        val_t tfar   = std::min(tmaxX, tmaxY); // closest  exit , first slab exited exits the AABB

        if (tclose > tfar || tfar < 0) continue; // miss or behind

        val_t hitT = (tclose >= 0) ? tclose : 0;
        if (hitT >= closestT) continue;

        val_t sx = std::copysign(1.0, ray.dir.x);
        val_t sy = std::copysign(1.0, ray.dir.x);
        val_t entry = (tminX > tminY) ? 1.0 : 0.0; // 1 if x, 0 if y
        val_t sign = (tclose >= 0.0) ? -1.0 : 1.0; // -1 if start outside, +1 if start inside
        Vec2 normal = sign * Vec2(entry * sx, (1.0 - entry) * sy);

        closest.hit = true;
        closest.ID = id;
        closest.normal = normal;
        closest.point = ray.pos + hitT * ray.dir;
    }

    return closest;
}
}
