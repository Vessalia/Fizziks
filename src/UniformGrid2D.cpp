#include <UniformGrid2D.h>

namespace Fizziks
{
UniformGrid2D::UniformGrid2D(size_t unitsX, size_t unitsY)
    : unitsX(unitsX)
    , unitsY(unitsY)
    , entityCount(0)
{
    columns.resize(unitsX);
    rows.resize(unitsY);
}

bool UniformGrid2D::insert(size_t entityID, Vector2p pos, AABB dim)
{
    if(validID(entityID)) return false; // probably need a more elegant solution to duplication

    int startX = floor(pos.x() - dim.halfWidth);  int endX = floor(pos.x() + dim.halfWidth); // this is incorrect, objects on a cell boundary should not be considered in both cells
    int startY = floor(pos.y() - dim.halfHeight); int endY = floor(pos.y() + dim.halfHeight);
    for(int x = startX; x <= endX; ++x)
    {
        for(int y = startY; y <= endY; ++y)
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
    if(!validID(entityID)) return false;

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
    if(!validID(prevID) || validID(newID)) return false;

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

// this   (row[i] & col[i]) | (row[i] & col[j + 1]) | (row[i + 1] & col[j]) | (row[i + 1] & col[j + 1]) | ...
// equals (row[i] | row[i + 1] | ...) & (col[j] | col[j + 1] | ...)
BitArray UniformGrid2D::neighbourhood(size_t entityID) const
{
    if(validID(entityID)) return BitArray::Zero;

    BitArray rowNeighbours;
    BitArray columnNeighbours;

    for(const auto& row : rows)
    {
        if(row.read(entityID))
            rowNeighbours |= row;
    }

    for(const auto& column : columns)
    {
        if(column.read(entityID))
            rowNeighbours |= column;
    }

    return rowNeighbours & columnNeighbours;
}

bool UniformGrid2D::validID(size_t entityID) const
{
    return entityCount > 0 && presence.read(entityID);
}
};
