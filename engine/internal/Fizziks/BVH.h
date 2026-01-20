#pragma once
#include <Fizziks/Fizziks.h>
#include <Fizziks/Broadphase.h>

#include <unordered_map>

namespace Fizziks::internal
{
class BVH : public Broadphase
{
public:
    BVH();
    virtual uint32_t add(uint32_t ID, const AABB& aabb, const Vec2& at);
    virtual bool remove(uint32_t ID);
    virtual void replace(uint32_t prevID, uint32_t newID);
    virtual void update(uint32_t ID, const AABB& aabb, const Vec2& at);

    virtual CollisionPairs& computePairs(void);
    virtual uint32_t pick(const Vec2& point) const;
    virtual std::vector<uint32_t> query(const AABB& aabb, const Vec2& pos) const;
    virtual RaycastResult raycast(const Ray& ray) const;

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

    uint32_t allocateLeaf(uint32_t ID, const Entry& entry);
    uint32_t allocateInternalNode();
    val_t cost(uint32_t from);
    uint32_t pickBestSibling(uint32_t nodeIndex);
    void removeNodeAt(uint32_t index);
    void refit(uint32_t from);
};
}
