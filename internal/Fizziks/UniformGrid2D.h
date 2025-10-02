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
    UniformGrid2D(size_t unitsX, size_t unitsY, size_t worldUnitsX, size_t worldUnitsY);

    bool insert(size_t entityID, Vector2p pos, AABB dim = createAABB(1, 1).aabb);
    bool remove(size_t entityID);
    bool replace(size_t prevID, size_t newID);
    bool update(size_t entityID, Vector2p newPos, AABB dim = createAABB(1, 1).aabb);
    BitArray neighbourhood(size_t entityID) const;
     
    BitArray castRay(Vector2p pos, Vector2p ray) const;
    BitArray castRay(Vector2p pos, Vector2p dir, val_t len) const;

    Vector2p world_to_grid_space(const Vector2p& vec) const;
    AABB world_to_grid_space(const AABB& aabb) const;

private:
    size_t entityCount;
    std::vector<BitArray> columns;
    std::vector<BitArray> rows;

    BitArray presence;

    size_t unitsX;
    size_t unitsY;

    size_t worldUnitsX;
    size_t worldUnitsY;

    bool valid_ID(size_t entityID) const;
    AABB compute_dim(size_t entityID) const;
    BitArray cast_ray(Vector2p from, Vector2p to) const;
};
};
