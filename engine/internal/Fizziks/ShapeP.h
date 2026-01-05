#pragma once
#include <Fizziks.h>
#include <variant>
#include <VectorP.h>

// NOTE: positions and rotations here are relative to world-space, and about the centroid, not the user-based position
namespace Fizziks::internal
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
    Vector2p contactPointWorldA, contactPointWorldB;
    Vector2p contactPointLocalA, contactPointLocalB;  
    Vector2p normal; 
    Vector2p tangent;
    val_t penetration;  

    uint32_t featureA, featureB; // for tracking specific collisions

    bool overlaps;
};

val_t deg2rad(const val_t deg);
val_t rad2deg(const val_t rad);

Shape createCircle(const val_t radius);
Shape createRect(const val_t width, const val_t height);
Shape createPolygon(const std::vector<Vector2p>& vertices);
AABB createAABB(const val_t width, const val_t height, const Vector2p& offset = vec_zero());

val_t getMoI(const Shape& shape, const val_t mass);

AABB getInscribingAABB(const Shape& s, const Vector2p& centroid, const val_t rot);
bool AABBOverlapsAABB(const AABB& r1, const Vector2p& p1, const AABB& r2, const Vector2p& p2);
bool AABBContains(const AABB& aabb, const Vector2p& pos, const Vector2p& point);

bool shapesOverlap(const Shape& s1, const Vector2p& p1, const val_t r1, const Shape& s2, const Vector2p& p2, const val_t r2);
Contact getShapeContact(const Shape& s1, const Vector2p& p1, const val_t r1, const Shape& s2, const Vector2p& p2, const val_t r2);
};
