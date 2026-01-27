#pragma once
#include <Fizziks/Fizziks.h>
#include <Fizziks/Broadphase.h>

#include <unordered_map>

namespace Fizziks::internal
{
class BVH : public Broadphase
{
public:
    virtual uint32_t add(uint32_t ID, const AABB& aabb, const Vec2& at);
    virtual bool remove(uint32_t ID);
    virtual void replace(uint32_t prevID, uint32_t newID);
    virtual void update(uint32_t ID, const AABB& aabb, const Vec2& at);

    virtual CollisionPairs computePairs(void);
    virtual uint32_t pick(const Vec2& point) const;
    virtual std::vector<uint32_t> query(const AABB& aabb, const Vec2& pos) const;
    virtual RaycastResult raycast(const Ray& ray) const;

    virtual std::vector<std::pair<AABB, Vec2>> getDebugInfo() const;

private:
    using Entry = std::pair<AABB, Vec2>;

    static constexpr uint32_t INVALID = -1;
    struct Node
    {
        Entry bounds;
        uint32_t bodyID = INVALID;
        uint32_t parent = INVALID;
        uint32_t child1 = INVALID;
        uint32_t child2 = INVALID;
        bool isleaf = true;
    };

    uint32_t root = 0;
    std::vector<Node> nodes;
    std::unordered_map<uint32_t, uint32_t> indexFromID;

    bool dirty = true;
    CollisionPairs collPairs;

    uint32_t allocateLeaf(uint32_t ID, const Entry& entry);
    uint32_t allocateInternalNode();
    val_t cost() const;
    val_t cost(const Entry& entry) const;
    val_t deltaCost(uint32_t sibling, uint32_t node) const;
    Entry mergeBounds(uint32_t node1, uint32_t node2) const;
    Entry mergeBounds(const Entry& e1, const Entry& e2) const;
    uint32_t pickBestSibling(uint32_t nodeIndex) const;
    void removeNodeAt(uint32_t index);
    void refitAdd(uint32_t leaf);
    void refitRemove(uint32_t from);
    void rotate(uint32_t index);
};
}
