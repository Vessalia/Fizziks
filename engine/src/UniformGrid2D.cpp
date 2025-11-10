#include <UniformGrid2D.h>

namespace Fizziks
{
UniformGrid2D::UniformGrid2D(size_t unitsX, size_t unitsY, size_t worldUnitsX, size_t worldUnitsY)
    : unitsX(unitsX)
    , unitsY(unitsY)
    , worldUnitsX(worldUnitsX)
    , worldUnitsY(worldUnitsY)
    , entityCount(0)
{
    columns.resize(unitsX);
    rows.resize(unitsY);
}

bool UniformGrid2D::insert(size_t entityID, Vector2p pos, AABB dim)
{
    if(valid_ID(entityID)) return false; // probably need a more elegant solution to duplication

    int startX = floor(pos.x() - dim.halfWidth);  int endX = ceil(pos.x() + dim.halfWidth); // this is incorrect, objects on a cell boundary should not be considered in both cells
    int startY = floor(pos.y() - dim.halfHeight); int endY = ceil(pos.y() + dim.halfHeight);
    for(int x = startX; x < endX; ++x)
    {
        for(int y = startY; y < endY; ++y)
        {
            unsigned int cellX = mod(x, unitsX);
            unsigned int cellY = mod(y, unitsY);
            columns[cellX].set(entityID);
            rows[cellY].set(entityID);
        }
    }

    presence.set(entityID);
    ++entityCount;

    return true;
}

bool UniformGrid2D::remove(size_t entityID)
{
    if(!valid_ID(entityID)) return false;

    BitArray entity;
    entity.set(entityID);
 
    Vector2p pos;
    for(auto& column : columns)
    {
        if(column.read(entityID))
            column.clear(entityID);
    }

    for(auto& row : rows)
    {
        if(row.read(entityID))
            row.clear(entityID);
    }

    presence.clear(entityID);
    --entityCount;

    return true;
}

bool UniformGrid2D::replace(size_t prevID, size_t newID)
{
    if(!valid_ID(prevID) || valid_ID(newID)) return false;

    for(auto& column : columns)
    {
        if(column.read(prevID))
        {
            column.clear(prevID);
            column.set(newID);
        }
    }

    for(auto& row : rows)
    {
        if(row.read(prevID))
        {
            row.clear(prevID);
            row.set(newID);
        }
    }

    presence.clear(prevID);
    presence.set(newID);

    return true;
}

bool UniformGrid2D::update(size_t entityID, Vector2p newPos, AABB dim)
{
    if(!remove(entityID)) return false;
    return insert(entityID, newPos, dim);
}

// this   (row[i] & col[j]) | (row[i] & col[j + 1]) | (row[i + 1] & col[j]) | (row[i + 1] & col[j + 1]) | ...
// equals (row[i] | row[i + 1] | ...) & (col[j] | col[j + 1] | ...)
BitArray UniformGrid2D::neighbourhood(size_t entityID) const
{
    if(!valid_ID(entityID)) return BitArray::Zero;

    BitArray rowNeighbours;
    BitArray columnNeighbours;

    for(const auto& column : columns)
    {
        if(column.read(entityID))
            columnNeighbours |= column;
    }

    for(const auto& row : rows)
    {
        if(row.read(entityID))
            rowNeighbours |= row;
    }

    return rowNeighbours & columnNeighbours;
}

BitArray UniformGrid2D::castRay(Vector2p pos, Vector2p ray) const
{
    pos = world_to_grid_space(pos);
    ray = world_to_grid_space(ray);
    return cast_ray(pos, pos + ray);
}

BitArray UniformGrid2D::castRay(Vector2p pos, Vector2p dir, val_t len) const
{
    pos = world_to_grid_space(pos);
    return castRay(pos, dir * len);
}

bool UniformGrid2D::valid_ID(size_t entityID) const
{
    return entityCount > 0 && presence.read(entityID);
}

Vector2p UniformGrid2D::world_to_grid_space(const Vector2p& vec) const
{
    val_t xFactor = unitsX / (val_t)worldUnitsX;
    val_t yFactor = unitsY / (val_t)worldUnitsY;
    return { vec.x() * xFactor, vec.y() * yFactor }; 
}

AABB UniformGrid2D::world_to_grid_space(const AABB& aabb) const
{
    val_t xFactor = unitsX / (val_t)worldUnitsX;
    val_t yFactor = unitsY / (val_t)worldUnitsY;
    return { aabb.halfWidth * xFactor, aabb.halfHeight * yFactor };
}

AABB UniformGrid2D::compute_dim(size_t entityID) const
{
    int startX = -1; int endX = -1;
    int startY = -1; int endY = -1;

    for(int i = 0; i < columns.size(); ++i)
    {
        const auto& column = columns[i];
        if(startX < 0)
        {
            if(column.read(entityID))
                startX = i;
        }
        else if(endX < 0)
        {
            if(!column.read(entityID))
                endX = i;
        }
    }

    for(int i = 0; i < rows.size(); ++i)
    {
        const auto& row = rows[i];
        if(startY < 0)
        {
            if(row.read(entityID))
                startY = i;
        }
        else if(endY < 0)
        {
            if(!row.read(entityID))
                endY = i;
        }
    }

    int width  = endX - startX;
    int height = endY - startY;
    return createAABB(width, height);
}

BitArray UniformGrid2D::cast_ray(Vector2p from, Vector2p to) const
{
    val_t x1 = from.x(); val_t x2 = to.x();
    val_t y1 = from.y(); val_t y2 = to.y();
    int i = static_cast<int>(floor(x1));
    int j = static_cast<int>(floor(y1));
    int iend = static_cast<int>(floor(x2));
    int jend = static_cast<int>(floor(y2));

    val_t dx = x2 - x1;
    val_t dy = y2 - y1;

    int stepX = (dx > 0) ? 1 : (dx < 0) ? -1 : 0;
    int stepY = (dy > 0) ? 1 : (dy < 0) ? -1 : 0;

    const val_t INF = fizzmax<val_t>();
    val_t tMaxX = INF, tMaxY = INF;   // t at which we cross the first vertical/horizontal boundary
    val_t tDeltaX = INF, tDeltaY = INF; // how much t increases to cross one cell in x/y

    if (stepX != 0) 
    {
        // next vertical boundary x coordinate
        val_t nextBoundaryX = (stepX > 0) ? (i + 1.0) : i;
        tMaxX = (nextBoundaryX - x1) / dx;        // dx != 0 here
        tDeltaX = 1.0 / std::abs(dx);             // how much t per cell in x
    }

    if (stepY != 0) 
    {
        val_t nextBoundaryY = (stepY > 0) ? (j + 1.0) : j;
        tMaxY = (nextBoundaryY - y1) / dy;
        tDeltaY = 1.0 / std::abs(dy);
    }

    auto in_bounds = [&](int ii, int jj) -> bool {
        return ii >= 0 && ii < static_cast<int>(columns.size())
            && jj >= 0 && jj < static_cast<int>(rows.size());
    };

    BitArray passthrough;

    // include starting cell if inside grid
    if (in_bounds(i, j)) {
        passthrough |= columns[i] & rows[j];
    }

    // Walk until we reach the cell containing the end point (iend, jend)
    while ((i != iend) || (j != jend)) {
        // choose smallest tMax to step
        if (tMaxX < tMaxY) {
            if (tMaxX > 1.0) break; // next crossing would be past the segment end
            i += stepX;
            tMaxX += tDeltaX;
        } else if (tMaxY < tMaxX) {
            if (tMaxY > 1.0) break;
            j += stepY;
            tMaxY += tDeltaY;
        } else { // tMaxX == tMaxY: crossing exactly through a corner -> step both
            if (tMaxX > 1.0) break;
            i += stepX;
            j += stepY;
            tMaxX += tDeltaX;
            tMaxY += tDeltaY;
        }

        if (!in_bounds(i, j)) break; // stop if we left the grid
        passthrough |= columns[i] & rows[j];
    }

    // ensure we include the cell that contains the end point
    if (in_bounds(iend, jend)) passthrough |= columns[iend] & rows[jend];

    return passthrough;
}
};
