#pragma once
#include <vector>
#include <Dense.h>
#include <BitArray.h>
#include <Shape.h>

namespace Fizziks
{
class UniformGrid2D
{
public:
    UniformGrid2D(size_t unitsX, size_t unitsY);
    UniformGrid2D() : UniformGrid2D(10, 10) { }

    bool insert(size_t entityID, Vector2p pos, AABB dim = createAABB(1, 1).aabb);
    bool remove(size_t entityID);
    bool replace(size_t prevID, size_t newID);
    bool update(size_t entityID, Vector2p newPos, AABB dim = createAABB(1, 1).aabb);
    BitArray neighbourhood(size_t entityID) const;

private:
    size_t entityCount;
    std::vector<BitArray> columns;
    std::vector<BitArray> rows;

    BitArray presence;

    size_t unitsX;
    size_t unitsY;

    bool validID(size_t entityID) const;
};
};
