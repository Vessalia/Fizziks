#pragma once
#include <Dense.h>
#include <RigidDef.h>
#include <Ray.h>

namespace Fizziks
{
// more readable
typedef std::pair<uint32_t, uint32_t> CollisionPair;
typedef std::vector<CollisionPair> CollisionPairs;

class Broadphase
{
public:
    virtual ~Broadphase() = default;

    virtual uint32_t add(uint32_t ID, const AABB& aabb, const Vector2p& at) = 0;
    virtual bool remove(uint32_t ID) = 0;
    virtual void replace(uint32_t prevID, uint32_t newID) = 0;
    virtual void update(uint32_t ID, const AABB& aabb, const Vector2p& at) = 0;
    virtual const CollisionPairs& computePairs(void) = 0;
    
    // returns the collider ID that collides with a point
    virtual uint32_t pick(const Vector2p& point) const = 0;

    // used to get the collider IDs that broadly collide with the given aabb
    virtual std::vector<uint32_t> query(const AABB& aabb, const Vector2p& pos) const = 0;

    virtual RaycastResult raycast(const Ray& ray) const = 0;
};
};
