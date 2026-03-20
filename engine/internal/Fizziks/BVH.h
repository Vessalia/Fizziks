#pragma once
#include <Fizziks/Fizziks.h>
#include <Fizziks/Broadphase.h>

#include <unordered_map>
#include <queue>

namespace Fizziks::internal
{
class BVH : public Broadphase
{
public:
	virtual uint32_t add(uint32_t ID, const AABB& aabb);
	virtual bool remove(uint32_t ID);
	virtual void replace(uint32_t prevID, uint32_t newID);
	virtual void update(uint32_t ID, const AABB& aabb);

	virtual CollisionPairs computePairs(void);
	virtual uint32_t pick(const Vec2& point) const;
	virtual std::vector<uint32_t> query(const AABB& aabb) const;
	virtual RaycastResult raycast(const Ray& ray) const;

	virtual std::vector<AABB> getDebugInfo() const;

private:
	struct InternalPair
	{
        CollisionPair pair;
        uint32_t indexA, indexB; // adjacency list back reference
	};

	static constexpr uint32_t INVALID = -1;
	struct Node
	{
        AABB bounds;
        uint32_t bodyID = INVALID;
        uint32_t parent = INVALID;
        uint32_t child1 = INVALID;
        uint32_t child2 = INVALID;
        bool isleaf = true;
	};

	uint32_t root = 0;
	std::vector<Node> nodes;
	std::unordered_map<uint32_t, uint32_t> indexFromID;

	std::queue<uint32_t> moveBuffer;
	std::unordered_map<uint32_t, std::vector<uint32_t>> pairMap; // adjacency list
	std::vector<InternalPair> internalPairs; // internal list used with adjacency lists
	CollisionPairs collPairs; // public list used by users

	uint32_t allocateLeaf(uint32_t ID, const AABB& bounds);
	uint32_t allocateInternalNode();
	val_t cost() const;
	val_t cost(const AABB& bounds) const;
	val_t deltaCost(uint32_t sibling, uint32_t node) const;
	AABB mergeBounds(uint32_t node1, uint32_t node2) const;
	uint32_t pickBestSibling(uint32_t nodeIndex) const;
	void removeNodeAt(uint32_t index);
	bool removeNode(uint32_t bodyID, bool temp = false);
	void refitAdd(uint32_t leaf);
	void refitRemove(uint32_t from);
	void rotate(uint32_t index);
	void addPair(uint32_t idA, uint32_t idB);
	void removePairs(uint32_t ID);
	void removePair(uint32_t index);
};
}
