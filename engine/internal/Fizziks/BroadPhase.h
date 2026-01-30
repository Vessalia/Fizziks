#pragma once
#include <Fizziks/Fizziks.h>
#include <Fizziks/Shape.h>
#include <Fizziks/Vec.h>
#include <Fizziks/Ray.h>

namespace Fizziks::internal
{
// more readable
using CollisionPair = std::pair<uint32_t, uint32_t>;
using CollisionPairs = std::vector<CollisionPair>;

class Broadphase
{
public:
    virtual ~Broadphase() = default;

    virtual uint32_t add(uint32_t ID, const AABB& aabb, const Vec2& at) = 0;
    virtual bool remove(uint32_t ID) = 0;
    virtual void replace(uint32_t prevID, uint32_t newID) = 0;
    virtual void update(uint32_t ID, const AABB& aabb, const Vec2& at) = 0;
    virtual CollisionPairs computePairs(void) = 0;
    //virtual void setLayer(uint32_t ID, uint32_t layer) = 0;
    
    // returns the collider ID that collides with a point
    virtual uint32_t pick(const Vec2& point) const = 0;

    // used to get the collider IDs that broadly collide with the given aabb
    virtual std::vector<uint32_t> query(const AABB& aabb, const Vec2& pos) const = 0;

    virtual RaycastResult raycast(const Ray& ray) const = 0;

    virtual std::vector<std::pair<AABB, Vec2>> getDebugInfo() const = 0;
};
}
