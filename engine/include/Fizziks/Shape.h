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

struct ContactKey
{
    uint32_t bodyAId, bodyBId;
    uint32_t collIdA, collIdB;
    uint32_t featureA, featureB;

    bool operator==(const ContactKey&) const = default;
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

val_t deg2rad(val_t deg);
val_t rad2deg(val_t rad);

Shape createCircle(val_t radius);
Shape createRect(val_t width, val_t height);
Shape createPolygon(const std::vector<Vector2p>& vertices);
AABB createAABB(val_t width, val_t height, const Vector2p& offset = Vector2p::Zero());

val_t getMoI(const Shape& shape, const val_t mass);

AABB getInscribingAABB(const Shape& s, const Vector2p& centroid, val_t rot);
bool AABBOverlapsAABB(const AABB& r1, const Vector2p& p1, const AABB& r2, const Vector2p& p2);
bool AABBContains(const AABB& aabb, const Vector2p& pos, const Vector2p& point);

bool shapesOverlap(const Shape& s1, const Vector2p& p1, val_t r1, const Shape& s2, const Vector2p& p2, val_t r2);
Contact getShapeContact(const Shape& s1, const Vector2p& p1, val_t r1, const Shape& s2, const Vector2p& p2, val_t r2);
};

namespace std
{
template<>
struct std::hash<Fizziks::ContactKey>
{
    std::size_t operator()(const Fizziks::ContactKey& ck) const
    {
        size_t h = 0;

        auto hashCombine = [&](uint32_t v)
            {
                h ^= std::hash<uint32_t>{}(v)+0x9e3779b9 + (h << 6) + (h >> 2);
            };

        hashCombine(ck.bodyAId);
        hashCombine(ck.bodyBId);
        hashCombine(ck.collIdA);
        hashCombine(ck.collIdB);
        hashCombine(ck.featureA);
        hashCombine(ck.featureB);

        return h;
    }
};
};
