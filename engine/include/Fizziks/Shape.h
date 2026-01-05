#pragma once
#include <Fizziks.h>
#include <Vec.h>
#include <variant>

// NOTE: positions and rotations here are relative to world-space, and about the centroid, not the user-based position
namespace Fizziks
{
struct FIZZIKS_API Circle
{
    val_t radius;
};

struct FIZZIKS_API Polygon
{
    std::vector<Vec2> vertices;
};

enum class FIZZIKS_API ShapeType
{
    CIRCLE, POLYGON
};

struct FIZZIKS_API Shape
{
    ShapeType type;
    std::variant<Circle, Polygon> data;
};

// handles convex and concave compound shapes
struct FIZZIKS_API Compound
{
    std::vector<Shape> convexPieces;
};

struct FIZZIKS_API AABB
{
    val_t halfWidth;
    val_t halfHeight;
    
    Vec2 offset;
};

struct FIZZIKS_API Contact 
{
    Vec2 contactPointWorldA, contactPointWorldB;
    Vec2 contactPointLocalA, contactPointLocalB;  
    Vec2 normal; 
    Vec2 tangent;
    val_t penetration;  

    uint32_t featureA, featureB; // for tracking specific collisions

    bool overlaps;
};

FIZZIKS_API constexpr val_t deg2rad(const val_t deg);
FIZZIKS_API constexpr val_t rad2deg(const val_t rad);

FIZZIKS_API Shape createCircle(const val_t radius);
FIZZIKS_API Shape createRect(const val_t width, const val_t height);
FIZZIKS_API Shape createPolygon(const std::vector<Vec2>& vertices);
FIZZIKS_API AABB createAABB(const val_t width, const val_t height, const Vec2& offset = { 0, 0 });

FIZZIKS_API val_t getMoI(const Shape& shape, const val_t mass);

FIZZIKS_API AABB getInscribingAABB(const Shape& s, const Vec2& centroid, const val_t rot);
FIZZIKS_API bool AABBOverlapsAABB(const AABB& r1, const Vec2& p1, const AABB& r2, const Vec2& p2);
FIZZIKS_API bool AABBContains(const AABB& aabb, const Vec2& pos, const Vec2& point);

FIZZIKS_API bool shapesOverlap(const Shape& s1, const Vec2& p1, const val_t r1, const Shape& s2, const Vec2& p2, const val_t r2);
FIZZIKS_API Contact getShapeContact(const Shape& s1, const Vec2& p1, const val_t r1, const Shape& s2, const Vec2& p2, const val_t r2);
}
