#include <Fizziks/BVH.h>
#include <Fizziks/MathUtils.h>

#include <queue>
#include <stack>

namespace Fizziks::internal
{
uint32_t BVH::allocateLeaf(uint32_t ID, const AABB& bounds)
{
	Node leaf;
	leaf.bounds = bounds;
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

val_t BVH::cost(const AABB& bounds) const
{
	return bounds.area();
}

val_t BVH::deltaCost(uint32_t sibling, uint32_t node) const
{
	return cost(mergeBounds(sibling, node)) - cost(nodes[sibling].bounds);
}

AABB BVH::mergeBounds(uint32_t node1, uint32_t node2) const
{
	return merge(nodes[node1].bounds, nodes[node2].bounds);
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

		const AABB newBounds = mergeBounds(nodes[i].child1, nodes[i].child2);
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
	val_t DB = cost(merge(D.bounds, B.bounds));
	val_t DC = cost(merge(D.bounds, C.bounds));

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
    Vec2 center = { aabb.min.x + aabb.hw, aabb.min.y + aabb.hh };
    return createAABB(aabb.hw * 2 * FAT_FACTOR, aabb.hh * 2 * FAT_FACTOR, center);
}

uint32_t BVH::add(uint32_t ID, const AABB& aabb)
{
	const AABB bounds = fatten(aabb);

	uint32_t leaf = allocateLeaf(ID, bounds);

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
	nodes[newParent].bounds = merge(bounds, nodes[sibling].bounds);
	
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

bool BVH::removeNode(uint32_t bodyID, bool temp)
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

	if (!temp)
	{
		removePairs(bodyID);
		pairMap.erase(bodyID);
	}

	return true;
}

// users can only request to remove leaf nodes
bool BVH::remove(uint32_t bodyID)
{
	return removeNode(bodyID);
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

	removePairs(prevID);
	moveBuffer.push(newID);
}

void BVH::update(uint32_t ID, const AABB& aabb)
{
	auto it = indexFromID.find(ID);
	if (it == indexFromID.end()) return;

	uint32_t index = it->second;
	Node& node = nodes[index];
	if (contains(node.bounds, aabb)) return;

	AABB newBounds = fatten(aabb);
	node.bounds = newBounds;
	if (node.parent == BVH::INVALID) return;
	else
	{
		auto& t = pairMap[ID];
		removeNode(ID, true);
		add(ID, aabb);
	}
}

void BVH::addPair(uint32_t idA, uint32_t idB)
{
	if (idA > idB) std::swap(idA, idB);

	const CollisionPair pair = { idA, idB };
	const InternalPair internalPair = { pair, static_cast<uint32_t>(pairMap[idA].size()), static_cast<uint32_t>(pairMap[idB].size()) };
	uint32_t idx = collPairs.size();

	collPairs.push_back(pair);
	internalPairs.push_back(internalPair);

	pairMap[idA].push_back(idx);
	pairMap[idB].push_back(idx);
}

void BVH::removePairs(uint32_t ID)
{
	if (!pairMap.contains(ID)) return; // don't create indexes if not needed

	auto& list = pairMap[ID];
	while (!list.empty())
	{
		uint32_t index = list.back();
		removePair(index);
	}
}

void BVH::removePair(uint32_t index)
{
	auto dead = internalPairs[index];
	auto [da, db] = dead.pair;

	auto fixAdj = [&](uint32_t body, uint32_t idx)
	{
		auto& list = pairMap[body];
		
		std::swap(list[idx], list.back());
		list.pop_back();

		if (idx < list.size()) // something moved into idx
		{
			uint32_t movedPairIdx = list[idx];
			auto& mp = internalPairs[movedPairIdx];
			if (mp.pair.first == body) mp.indexA = idx;
			else					   mp.indexB = idx;
		}
	};

	fixAdj(da, dead.indexA);
	fixAdj(db, dead.indexB);

	uint32_t last = internalPairs.size() - 1;

	if (index < last)
	{
		internalPairs[index] = internalPairs[last];
		collPairs[index] = collPairs[last];

		auto& moved = internalPairs[index];
		auto [a, b] = moved.pair;

		pairMap[a][moved.indexA] = index;
		pairMap[b][moved.indexB] = index;
	}

	internalPairs.pop_back();
	collPairs.pop_back();
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
			if (node.bodyID == movedID || !overlaps(moved.bounds, node.bounds)) continue;

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
	if (root == INVALID) return INVALID;

	std::stack<uint32_t> stack;
	stack.push(root);

	while (!stack.empty())
	{
		uint32_t idx = stack.top(); stack.pop();

		const Node& node = nodes[idx];

		if (!contains(node.bounds, point)) continue;
		if (node.isleaf) return node.bodyID;

		stack.push(node.child1);
		stack.push(node.child2);
	}

	return INVALID;
}

std::vector<uint32_t> BVH::query(const AABB& aabb) const
{
	std::vector<uint32_t> results;
	if (root == INVALID) return results;

	std::stack<uint32_t> stack;
	stack.push(root);

	while (!stack.empty())
	{
		uint32_t idx = stack.top(); stack.pop();

		const Node& node = nodes[idx];

		if (!overlaps(aabb, node.bounds)) continue;
		if (node.isleaf)
		{
			results.push_back(node.bodyID);
			continue;
		}

		stack.push(node.child1);
		stack.push(node.child2);
	}

	return results;
}

RaycastResult BVH::raycast(const Ray& ray) const
{
	RaycastResult res;
	res.hit = false;
	val_t closestT = fizzmax<val_t>();
	val_t maxT = ray.maxDist < 0 ? fizzmax<val_t>() : ray.maxDist;

	std::stack<uint32_t> stack;
	stack.push(root);

	while (!stack.empty())
	{
		uint32_t idx = stack.top(); stack.pop();

		const Node& node = nodes[idx];
		val_t t = raytest(ray, node.bounds);
		if (t < 0 || t > closestT || t > maxT) continue;
		if (node.isleaf)
		{
			// need to check against colliders here and recalc t
			if (t < closestT)
			{
				closestT = t;
				res.hit = true;
				res.ID = node.bodyID;
				res.point = ray.pos + ray.dir * t;
				// get the normal based on the collider
			}
		}
		else
		{
			stack.push(node.child1);
			stack.push(node.child2);
		}
	}

	return res;
}

std::vector<AABB> BVH::getDebugInfo() const
{
	std::vector<AABB> debug;
	for (int i = 0; i < nodes.size(); ++i)
	{
		debug.push_back(nodes[i].bounds);
	}

	return debug;
}
}
