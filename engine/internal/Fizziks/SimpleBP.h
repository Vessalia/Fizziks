#pragma once
#include <Fizziks/Fizziks.h>
#include <Fizziks/Broadphase.h>

#include <unordered_map>

namespace Fizziks::internal
{
class SimpleBP : public Broadphase
{
public:
    virtual uint32_t add(uint32_t ID, const AABB& aabb, const Vec2& at)
    {
        bodies[ID] = { aabb, at };
        return ID;
    }

    virtual bool remove(uint32_t ID)
    {
        size_t oldSize = bodies.size();

        bodies.erase(ID);
    
        return bodies.size() != oldSize;
    }

    virtual void replace(uint32_t prevID, uint32_t newID);
     
    virtual void update(uint32_t ID, const AABB& aabb, const Vec2& at)
    {
        if(bodies.contains(ID));
        bodies[ID] = { aabb, at };
    }
     
    virtual CollisionPairs computePairs(void) const;
    virtual uint32_t pick(const Vec2& point) const;
    virtual std::vector<uint32_t> query(const AABB& aabb, const Vec2& pos) const;
    virtual RaycastResult raycast(const Ray& ray) const;

    virtual std::vector<std::pair<AABB, Vec2>> getDebugInfo() const;
 
private:
    using Entry = std::pair<AABB, Vec2>;

    std::unordered_map<uint32_t, Entry> bodies;
};
}
