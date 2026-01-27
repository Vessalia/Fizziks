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

    return nodes.size() - 1;
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
    indexFromID[ID] = leaf;

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

    dirty = true;

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
    removeNodeAt(std::max(leaf, parent)); // possibly the back node, remove it first
    if (sibling == nodes.size()) sibling = std::max(leaf, parent); // sibling got swapped
    removeNodeAt(std::min(leaf, parent));
    if (sibling == nodes.size()) sibling = std::min(leaf, parent);

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

CollisionPairs BVH::computePairs(void)
{
    if (!dirty) return collPairs;

    if (indexFromID.size() < 2) return {};

    dirty = false;
    CollisionPairs pairs;
    std::stack<CollisionPair> stack;
    // there is at least 2 leaves in the tree
    stack.push({ nodes[root].child1, nodes[root].child2 }); // cross-traverse 
    stack.push({ nodes[root].child1, nodes[root].child1 }); // self-traverse
    stack.push({ nodes[root].child2, nodes[root].child2 }); // self-traverse
    while (!stack.empty())
    {
        const auto [i1, i2] = stack.top(); stack.pop();
        const Node& node1 = nodes[i1];
        const Node& node2 = nodes[i2];
        
        if (i1 != i2) // cross-traversal
        {
            const auto& [b1, p1] = node1.bounds;
            const auto& [b2, p2] = node2.bounds;
            bool overlapping = overlaps(b1, p1, b2, p2);
            if (!overlapping) continue;

            if (node1.isleaf && node2.isleaf) // leaf-leaf traversal
            {
                pairs.push_back({ node1.bodyID, node2.bodyID });
            }
            else if (node1.isleaf != node2.isleaf) // node-leaf / leaf-node traversal
            {
                const Node& node = node1.isleaf ? node2 : node1;
                uint32_t leaf = node1.isleaf ? i1 : i2;

                stack.push({ node.child1, leaf });
                stack.push({ node.child2, leaf });
            }
            else // node-node traversal
            {
                stack.push({ node1.child1, node2.child1 });
                stack.push({ node1.child1, node2.child2 });
                stack.push({ node1.child2, node2.child1 });
                stack.push({ node1.child2, node2.child2 });
            }
        }
        else if (i1 == i2 && !nodes[i1].isleaf) // node self-traversal
        {
            const Node& node = nodes[i1];
            stack.push({ node.child1, node.child1 });
            stack.push({ node.child1, node.child2 });
            stack.push({ node.child2, node.child2 });
        }
        // do nothing when self-traversing a leaf
    }

    collPairs = pairs;
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
