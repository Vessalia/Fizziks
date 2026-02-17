#include <Fizziks/BVH.h>
#include <Fizziks/MathUtils.h>

#include <queue>
#include <stack>

namespace Fizziks::internal
{
uint32_t BVH::allocateLeaf(uint32_t ID, const Entry& entry)
{
    Node leaf;
    leaf.bounds = entry;
    leaf.bodyID = ID;
    nodes.push_back(leaf);
    uint32_t index = nodes.size() - 1;
    indexFromID[ID] = index;

    return index;
}

uint32_t BVH::allocateInternalNode()
{
    Node node;
    node.isleaf = false;
    nodes.push_back(node);

    return nodes.size() - 1;
}

val_t BVH::cost() const
{
    val_t result = 0;
    for (int i = 0; i < nodes.size(); ++i)
    {
        if (!nodes[i].isleaf && i != root) result += cost(nodes[i].bounds);
    }

    return result;
}

val_t BVH::cost(const Entry& entry) const
{
    return entry.first.area();
}

val_t BVH::deltaCost(uint32_t sibling, uint32_t node) const
{
    return cost(mergeBounds(sibling, node)) - cost(nodes[sibling].bounds);
}

BVH::Entry BVH::mergeBounds(uint32_t node1, uint32_t node2) const
{
    const auto& [b1, p1] = nodes[node1].bounds;
    const auto& [b2, p2] = nodes[node2].bounds;
    return merge(b1, p1, b2, p2);
}

BVH::Entry BVH::mergeBounds(const Entry& e1, const Entry& e2) const
{
    return merge(e1.first, e1.second, e2.first, e2.second);
}

val_t epsilon = 0.001;
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
    val_t rootCost = cost(mergeBounds(root, nodeIndex));
    std::priority_queue<Candidate> pq; pq.push({ root, rootCost, 0 });

    while (!pq.empty())
    {
        Candidate c = pq.top(); pq.pop();
        val_t currCost = c.cost();
        if (currCost - bestCost >= -epsilon) continue; // this is branch and bound
        
        bestSibling = c.index;
        bestCost = currCost;

        const Node& sibling = nodes[bestSibling];
        if (!sibling.isleaf)
        {
            const uint32_t& child1 = sibling.child1;
            const uint32_t& child2 = sibling.child2;
            val_t inherited = c.inheritedCost + deltaCost(c.index, nodeIndex);
            pq.push({ child1, cost(mergeBounds(child1, nodeIndex)), inherited });
            pq.push({ child2, cost(mergeBounds(child2, nodeIndex)), inherited });
        }
    }

    return bestSibling;
}

void BVH::refitAdd(uint32_t leaf)
{
    for (uint32_t i = nodes[leaf].parent; i != INVALID; i = nodes[i].parent)
    {
        nodes[i].bounds = mergeBounds(nodes[i].child1, nodes[i].child2);
        rotate(i);
    }
}

void BVH::refitRemove(uint32_t from)
{
    for (uint32_t i = from; i != INVALID; i = nodes[i].parent) 
    {
        if (nodes[i].isleaf) continue;

        const Entry newBounds = mergeBounds(nodes[i].child1, nodes[i].child2);
        nodes[i].bounds = newBounds;
    }
}

/*
   G
  / \
 D    A
/ \  / \
...  B  C

From A, can swap D and B, or D and C.
Find what has the min cost on A.
Only As bounds change
*/
void BVH::rotate(uint32_t index)
{
    Node& A = nodes[index];
    if (A.isleaf || A.parent == INVALID) return; // no children to rotate for a leaf, or a tree of depth 1

    Node& G = nodes[A.parent];
    uint32_t& other = G.child1 == index ? G.child2 : G.child1;
    Node& D = nodes[other];
    Node& B = nodes[A.child1];
    Node& C = nodes[A.child2];

    val_t AO = cost(A.bounds);
    val_t DB = cost(mergeBounds(D.bounds, B.bounds));
    val_t DC = cost(mergeBounds(D.bounds, C.bounds));

    if (AO <= DB && AO <= DC)
    {
        return;
    }
    else if (DB <= DC)
    {
        C.parent = A.parent;
        D.parent = B.parent;
        std::swap(A.child2, other);
    }
    else
    {
        B.parent = A.parent;
        D.parent = C.parent;
        std::swap(A.child1, other);
    }

    A.bounds = mergeBounds(A.child1, A.child2);
}

constexpr val_t FAT_FACTOR = 1.05;
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
    refitAdd(leaf);

    moveBuffer.push(ID);

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

        if (last == root) root = index;
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
    uint32_t front = std::min(leaf, parent);
    uint32_t back = std::max(leaf, parent);
    removeNodeAt(back); // possibly the back node, remove it first
    if (sibling == nodes.size()) sibling = back; // sibling got swapped
    removeNodeAt(front);
    if (sibling == nodes.size()) sibling = front;

    // Refit upward (removeNodeAt may have invalidated parent, sibling, or grandparent)
    if (sibling != root) refitRemove(nodes[sibling].parent);

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

    Entry newBounds = { fatten(aabb), at };
    node.bounds = newBounds;
    if (node.parent == BVH::INVALID) return;
    else
    {
        remove(ID);
        add(ID, aabb, at);
    }
}

void BVH::removePairs(uint32_t ID)
{
    std::erase_if(collPairs, [&](const CollisionPair& pair) {
        return pair.first == ID || pair.second == ID;
    });
}

void BVH::addPair(uint32_t idA, uint32_t idB)
{
    if (idA > idB) std::swap(idA, idB);
    const CollisionPair pair = { idA, idB };
    collPairs.push_back(pair);
}

CollisionPairs BVH::computePairs(void)
{
    if (!moveBuffer.size()) return collPairs;

    std::vector<uint32_t> stack;
    stack.reserve(std::log2(nodes.size()));
    while (!moveBuffer.empty())
    {
        uint32_t movedID = moveBuffer.front(); moveBuffer.pop();

        removePairs(movedID);
        const uint32_t movedIndex = indexFromID[movedID];
        const Node& moved = nodes[movedIndex];
        const auto [box, pos] = moved.bounds;

        if (!nodes[root].isleaf)
        {
            stack.push_back(nodes[root].child1);
            stack.push_back(nodes[root].child2);
        }

        while (!stack.empty())
        {
            uint32_t index = stack.back(); stack.pop_back();
            if (index == movedIndex) continue;

            const Node& node = nodes[index];
            const auto [oBox, oPos] = node.bounds;
            if (!overlaps(box, pos, oBox, oPos)) continue;

            if (node.isleaf)
            {
                addPair(movedID, node.bodyID);
            }
            else
            {
                stack.push_back(node.child1);
                stack.push_back(node.child2);
            }
        }
    }

    return collPairs;
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

std::vector<std::pair<AABB, Vec2>> BVH::getDebugInfo() const
{
    std::vector<std::pair<AABB, Vec2>> debug;
    for (int i = 0; i < nodes.size(); ++i)
    {
        debug.push_back(nodes[i].bounds);
    }

    return debug;
}
}
