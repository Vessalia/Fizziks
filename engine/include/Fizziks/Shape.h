#pragma once
#include <Dense.h>
#include <variant>

// NOTE: positions and rotations here are relative to world-space, and about the centroid, not the user-based position
namespace Fizziks
{
struct Circle
{
    val_t radius;
};

struct Polygon
{
    std::vector<Vector2p> vertices;
};

enum class ShapeType
{
    CIRCLE, POLYGON
};

struct Shape
{
    ShapeType type;
    std::variant<Circle, Polygon> data;
};

// handles convex and concave compound shapes
struct Compound
{
    std::vector<Shape> convexPieces;
};

struct AABB
{
    val_t halfWidth;
    val_t halfHeight;
    
    Vector2p offset;
};

struct Contact 
{
    Vector2p normal; 
    Vector2p contactPoint;  
    val_t penetration;  

    bool overlaps;
};

val_t deg2rad(val_t deg);
val_t rad2deg(val_t rad);

Shape createCircle(val_t radius);
Shape createRect(val_t width, val_t height);
Shape createPolygon(const std::vector<Vector2p>& vertices);
AABB createAABB(val_t width, val_t height, const Vector2p& offset = Vector2p::Zero());

AABB getInscribingAABB(const Shape& s, const Vector2p& centroid, val_t rot);
bool AABBOverlapsAABB(const AABB& r1, const Vector2p& p1, const AABB& r2, const Vector2p& p2);
bool AABBContains(const AABB& aabb, const Vector2p& pos, const Vector2p& point);

bool shapesOverlap(const Shape& s1, const Vector2p& p1, val_t r1, const Shape& s2, const Vector2p& p2, val_t r2);
Contact getShapeContact(const Shape& s1, const Vector2p& p1, val_t r1, const Shape& s2, const Vector2p& p2, val_t r2);
};
