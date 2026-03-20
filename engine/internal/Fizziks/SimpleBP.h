#pragma once
#include <Fizziks/Fizziks.h>
#include <Fizziks/Broadphase.h>

#include <unordered_map>

namespace Fizziks::internal
{
class SimpleBP : public Broadphase
{
public:
	virtual uint32_t add(uint32_t ID, const AABB& aabb)
	{
		bodies[ID] = aabb;
		return ID;
	}

	virtual bool remove(uint32_t ID)
	{
		size_t oldSize = bodies.size();
		bodies.erase(ID);

		return bodies.size() != oldSize;
	}

	virtual void replace(uint32_t prevID, uint32_t newID);
	 
	virtual void update(uint32_t ID, const AABB& aabb)
	{
		if(bodies.contains(ID))
			bodies[ID] = aabb;
	}
	 
	virtual CollisionPairs computePairs(void);
	virtual uint32_t pick(const Vec2& point) const;
	virtual std::vector<uint32_t> query(const AABB& aabb) const;
	virtual RaycastResult raycast(const Ray& ray) const;

	virtual std::vector<AABB> getDebugInfo() const;
 
private:
	std::unordered_map<uint32_t, AABB> bodies;
};
}
