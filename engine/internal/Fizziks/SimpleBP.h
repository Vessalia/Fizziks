#pragma once
#include <Fizziks.h>
#include <Broadphase.h>

namespace Fizziks
{
class SimpleBP : public Broadphase
{
public:
    virtual uint32_t add(uint32_t ID, const AABB& aabb, const Vector2p& at)
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
     
    virtual void update(uint32_t ID, const AABB& aabb, const Vector2p& at)
    {
        if(bodies.contains(ID));
        bodies[ID] = { aabb, at };
    }
     
    virtual CollisionPairs& computePairs(void);
    virtual uint32_t pick(const Vector2p& point) const;
    virtual std::vector<uint32_t> query(const AABB& aabb, const Vector2p& pos) const;
    virtual RaycastResult raycast(const Ray& ray) const;
 
private:
    typedef std::pair<AABB, Vector2p> Entry;

    std::unordered_map<uint32_t, Entry> bodies;
    CollisionPairs pairs;
};
};