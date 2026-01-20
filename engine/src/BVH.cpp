#include <Fizziks/BVH.h>
#include <Fizziks/MathUtils.h>

#include <queue>

namespace Fizziks::internal
{
uint32_t BVH::allocateLeaf(uint32_t ID, const Entry& entry)
{
    uint32_t index = nodes.size();
    Node leaf;
    leaf.bounds = entry;
    leaf.bodyID = ID;
    nodes.push_back(leaf);

    return index;
}

uint32_t BVH::allocateInternalNode()
{
    uint32_t index = nodes.size();
    Node node;
    node.isleaf = false;
    nodes.push_back(node);

    return index;
}

val_t BVH::cost() const
{
    val_t cost = 0;
    for (int i = 0; i < nodes.size(); ++i)
    {
        if (!nodes[i].isleaf && i != root) cost += nodes[i].bounds.first.area();
    }

    return cost;
}

val_t BVH::deltaCost(uint32_t sibling, uint32_t node) const
{
    return mergeBounds(sibling, node).first.area() - nodes[sibling].bounds.first.area();
}

BVH::Entry BVH::mergeBounds(uint32_t node1, uint32_t node2) const
{
    const auto& [b1, p1] = nodes[node1].bounds;
    const auto& [b2, p2] = nodes[node2].bounds;
    return merge(b1, p1, b2, p2);
}

uint32_t BVH::pickBestSibling(uint32_t nodeIndex) const
{
    struct Candidate
    {
        uint32_t index;
        val_t directCost;
        val_t inheritedCost;

        bool operator<(const Candidate& other) const { return cost() > other.cost(); } // want to prioritize small costs
        val_t cost() const { return directCost + inheritedCost; }
    };

    uint32_t bestSibling = root;
    val_t bestCost = fizzmax<val_t>();
    val_t rootCost = mergeBounds(root, nodeIndex).first.area();
    std::priority_queue<Candidate> pq; pq.push({ root, rootCost, 0 });

    while (!pq.empty())
    {
        Candidate c = pq.top(); pq.pop();
        val_t cost = c.cost();
        if (cost >= bestCost) continue; // this is branch and bound
        
        bestSibling = c.index;
        bestCost = cost;

        const Node& sibling = nodes[bestSibling];
        if (!sibling.isleaf)
        {
            const uint32_t& c1 = sibling.child1;
            const uint32_t& c2 = sibling.child2;
            val_t inherited = c.inheritedCost + deltaCost(c.index, nodeIndex);
            val_t cost1 = mergeBounds(c1, nodeIndex).first.area();
            val_t cost2 = mergeBounds(c2, nodeIndex).first.area();
            // since we don't know the cost of descending either child, we can't prune either here
            // ex: child1 is a node, child2 is a leaf, cost1 > cost2, but eventually the new leaf has
            // a lower cost in child1s subtree 
            if (cost1 + inherited < bestCost) pq.push({ c1, cost1, inherited });
            if (cost2 + inherited < bestCost) pq.push({ c2, cost2, inherited });
        }
    }

    return bestSibling;
}

void BVH::refit(uint32_t from)
{
    for (uint32_t i = from; i != INVALID; i = nodes[i].parent) 
    {
        const Entry newBounds = mergeBounds(nodes[i].child1, nodes[i].child2);
        if (newBounds == nodes[i].bounds) break;
        else nodes[i].bounds = newBounds;
    }
}

constexpr val_t FAT_FACTOR = 0.2;
static AABB fatten(const AABB& aabb)
{
    return { FAT_FACTOR * aabb.hw, FAT_FACTOR * aabb.hh, aabb.offset };
}

uint32_t BVH::add(uint32_t ID, const AABB& aabb, const Vec2& at)
{
    const Entry entry = { fatten(aabb), at };

    uint32_t leaf = allocateLeaf(ID, entry);

    if (nodes.size() == 1)
    {
        root = leaf;
        return ID;
    }

    // Stage 1: find the best sibling for the new leaf
    uint32_t sibling = pickBestSibling(leaf);

    // Stage 2: create a new parent
    int oldParent = nodes[sibling].parent;
    int newParent = allocateInternalNode();
    nodes[newParent].parent = oldParent;
    const auto& [siblingBox, siblingAt] = nodes[sibling].bounds;
    nodes[newParent].bounds = merge(entry.first, at, siblingBox, siblingAt);
    
    if (oldParent != BVH::INVALID) // the sibling wasn't the root
    {
        if (nodes[oldParent].child1 == sibling) 
            nodes[oldParent].child1 = newParent;
        else                                    
            nodes[oldParent].child2 = newParent;
    }
    else // the sibling was the root
    {
        root = newParent;
    }
    nodes[newParent].child1 = sibling;
    nodes[newParent].child2 = leaf;
    nodes[sibling].parent = newParent;
    nodes[leaf].parent = newParent;

    // Stage 3: walk back up the tree refitting AABBs
    refit(nodes[leaf].parent);

    return ID;
}

// Simply swap-pops a node from the tree. Clean-up is required after to preserve validity
void BVH::removeNodeAt(uint32_t index)
{
    if (nodes[index].isleaf) indexFromID.erase(nodes[index].bodyID);

    uint32_t last = nodes.size() - 1;
    if (index != last) 
    {
        Node& moved = nodes[last];
        nodes[index] = moved;

        // Fix parent
        if (moved.parent != INVALID) 
        {
            Node& p = nodes[moved.parent];
            if (p.child1 == last) p.child1 = index;
            if (p.child2 == last) p.child2 = index;
        }

        // Fix children
        if (moved.child1 != INVALID) nodes[moved.child1].parent = index;
        if (moved.child2 != INVALID) nodes[moved.child2].parent = index;

        // Fix body map
        if (moved.bodyID != INVALID) indexFromID[moved.bodyID] = index;
    }

    // don't remove ID mapping key, as this node may not be a leaf
    nodes.pop_back();
}

// users can only request to remove leaf nodes
bool BVH::remove(uint32_t bodyID)
{
    auto it = indexFromID.find(bodyID);
    if (it == indexFromID.end()) return false;

    uint32_t leaf = it->second;

    if (leaf == root) 
    {
        nodes.clear();
        indexFromID.clear();
        root = INVALID;
        return true;
    }

    uint32_t parent = nodes[leaf].parent;
    uint32_t sibling = (nodes[parent].child1 == leaf) ? nodes[parent].child2 : nodes[parent].child1;
    uint32_t grandparent = nodes[parent].parent;

    if (grandparent != INVALID) // the parent is not the root
    {
        Node& gp = nodes[grandparent];
        if (gp.child1 == parent) 
            gp.child1 = sibling;
        else 
            gp.child2 = sibling;

        nodes[sibling].parent = grandparent;
    } 
    else 
    {
        root = sibling;
        nodes[sibling].parent = INVALID;
    }

    // Remove both structural nodes
    // since we swap pop the back node, make sure we don't accidentally alter the index of the other node
    removeNodeAt(std::max(leaf, parent)); // possibly the back node, remove it first
    removeNodeAt(std::min(leaf, parent));

    // Refit upward
    refit(grandparent);

    return true;
}

void BVH::replace(uint32_t prevID, uint32_t newID)
{
    if (prevID == newID) return;

    auto it = indexFromID.find(prevID);
    if (it == indexFromID.end()) return;

    remove(newID);

    uint32_t index = it->second;
    nodes[index].bodyID = newID;
    indexFromID.erase(it);
    indexFromID[newID] = index;
}

void BVH::update(uint32_t ID, const AABB& aabb, const Vec2& at)
{
    auto it = indexFromID.find(ID);
    if (it == indexFromID.end()) return;

    uint32_t index = it->second;
    Node& node = nodes[index];
    if (contains(node.bounds.first, node.bounds.second, aabb, at)) return;

    Entry newBounds = { aabb, at };
    node.bounds = newBounds;
    if (node.parent == BVH::INVALID) return;
    else if (node.parent == root)
    {
        Node& parent = nodes[node.parent];
        parent.bounds = mergeBounds(parent.child1, parent.child2);
    }
    else
    {
        remove(ID);
        add(ID, aabb, at);
    }
}

CollisionPairs& BVH::computePairs(void)
{
    return {};
}

uint32_t BVH::pick(const Vec2& point) const
{
    return 0;
}

std::vector<uint32_t> BVH::query(const AABB& aabb, const Vec2& pos) const
{
    return {};
}

RaycastResult BVH::raycast(const Ray& ray) const
{
    return {};
}
}
