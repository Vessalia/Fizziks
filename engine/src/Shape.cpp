#include "Shape.h"
#include <numbers>

namespace Fizziks
{
val_t deg2rad(val_t deg)
{
    return deg * std::numbers::pi_v<val_t> / 180;
}

val_t rad2deg(val_t rad)
{
    return rad * 180 / std::numbers::pi_v<val_t>;
}

// https://en.wikipedia.org/wiki/Centroid @ Of a polygon
Vector2p getCentroid(const std::vector<Vector2p>& vertices)
{
    Vector2p centroid = Vector2p::Zero();
    val_t area = 0;

    size_t n = vertices.size();
    for (size_t i = 0; i < n; ++i)
    {
        const Vector2p& v0 = vertices[i];
        const Vector2p& v1 = vertices[(i + 1) % n];

        val_t cross = crossproduct(v0, v1);
        centroid += (v0 + v1) * cross;
        area += cross;
    }

    return area != 0 ? (centroid / (3 * area)).eval() : Vector2p::Zero();
}

Shape createCircle(val_t radius)
{
    return { ShapeType::CIRCLE, Circle{ radius } };
}

Shape createRect(val_t width, val_t height)
{
    std::vector<Vector2p> vertices 
    {
        { -width / 2,  height / 2 },
        { -width / 2, -height / 2 },
        {  width / 2, -height / 2 },
        {  width / 2,  height / 2 }
    };
    return createPolygon(vertices);
}

Shape createPolygon(const std::vector<Vector2p>& vertices)
{
    auto centroid = getCentroid(vertices);
    auto verts = vertices;
    for (auto& vert : verts)
    {
        vert -= centroid;
    }
    return { ShapeType::POLYGON, Polygon{ verts } };
}

AABB createAABB(val_t width, val_t height, const Vector2p& offset)
{
    return { width / 2, height / 2, offset };
}

AABB getInscribingAABB(const Shape& s, const Vector2p& centroid, val_t rot)
{
    AABB aabb;
    Rotation2p rotation(rot);

    if(s.type == ShapeType::POLYGON)
    {
        auto& polygon = std::get<Polygon>(s.data);

        Vector2p min = fizzmax<Vector2p>(), max = fizzmin<Vector2p>();
        for (const auto& vertex : polygon.vertices)
        {
            auto transformed = rotation * vertex;
            min.x() = std::min(min.x(), transformed.x());
            min.y() = std::min(min.y(), transformed.y());
            max.x() = std::max(max.x(), transformed.x());
            max.y() = std::max(max.y(), transformed.y());
        }
        
        aabb.halfWidth  = (max.x() - min.x()) / 2;
        aabb.halfHeight = (max.y() - min.y()) / 2;

        aabb.offset = (min + max) / 2 - centroid;
    }
    else if(s.type == ShapeType::CIRCLE)
    {
        auto& circle = std::get<Circle>(s.data);
        aabb.halfWidth  = circle.radius;
        aabb.halfHeight = circle.radius;
    }

    return aabb;
}

bool AABBOverlapsAABB(const AABB& r1, const Vector2p& p1, const AABB& r2, const Vector2p& p2)
{
    bool overlapX = abs(p1.x() - p2.x()) <= r1.halfWidth  + r2.halfWidth;
    bool overlapY = abs(p1.y() - p2.y()) <= r1.halfHeight + r2.halfHeight;
    return overlapX && overlapY;
}
bool AABBContains(const AABB& aabb, const Vector2p& pos, const Vector2p& point)
{
    bool overlapX = abs(pos.x() - point.x()) <= aabb.halfWidth;
    bool overlapY = abs(pos.y() - point.y()) <= aabb.halfHeight;
    return overlapX && overlapY;
}

#pragma region CSO support

Vector2p getSupport(const Shape& shape, val_t rot, const Vector2p& direction)
{
    Vector2p result = Vector2p::Zero();
    Rotation2p rotation(-rot);
    Vector2p dir = rotation * direction;

    if(shape.type == ShapeType::CIRCLE)
    {
        const auto& c = std::get<Circle>(shape.data);
        result = dir.normalized() * c.radius;
    }
    else if (shape.type == ShapeType::POLYGON)
    {
        const auto& p = std::get<Polygon>(shape.data);
        val_t bestProj = -fizzmax<val_t>();
        
        for (const auto& v : p.vertices)
        {
            val_t proj = v.dot(dir);
            if(proj > bestProj)
            {
                bestProj = proj;
                result = v;
            }
        }
    }

    return result;
}

Vector2p getCSOSupport(const Shape& s1, const Vector2p& p1, val_t r1, 
                       const Shape& s2, const Vector2p& p2, val_t r2, 
                       const Vector2p& dir)
{
    Rotation2p rot1(r1), rot2(r2);
    Vector2p support1 = getSupport(s1, r1, dir);
    Vector2p support2 = getSupport(s2, r2, -dir);
    return rot1 * support1 + p1 - (rot2 * support2 + p2);
}

#pragma endregion

#pragma region GJK

Vector2p projToEdge (const Vector2p& P, const Vector2p& Q, const Vector2p& point) 
{
    Vector2p PQ = Q - P;
    val_t t = (point - P).dot(PQ) / PQ.dot(PQ);
    t = std::clamp(t, static_cast<val_t>(0), static_cast<val_t>(1));
    if      (t == 0) return P;
    else if (t == 1) return Q;
    else             return P + t * PQ; // avoid dealing with all the float math stuff
}

Vector2p closestPoint(const std::vector<Vector2p>& simplex, const Vector2p& point)
{
    size_t count = simplex.size();
    if (count == 0)
    {
        return fizzmax<Vector2p>(); // should never happen
    }
    if (count == 1)
    {
        return simplex[0];
    }
    else if (count == 2) 
    {
        return projToEdge(simplex[0], simplex[1], point);
    }
    else if (count >= 3)
    {
        bool inside = true;
        Vector2p A = simplex[0];
        Vector2p B = simplex[1];
        Vector2p C = simplex[2];
        bool winding = crossproduct(B - A, C - A) >= 0;
        for (size_t i = 0; i < count; ++i)
        {
            Vector2p P = simplex[i];
            Vector2p Q = simplex[(i + 1) % count];
            if ((crossproduct(Q - P, point - P) >= 0) != winding)
            {
                inside = false;
                break;
            }
        }
        if (inside) return point;

        Vector2p closest;
        val_t bestDist = fizzmax<val_t>();
        for (size_t i = 0; i < count; ++i)
        {
            Vector2p P = simplex[i];
            Vector2p Q = simplex[(i + 1) % count];
            Vector2p proj = projToEdge(P, Q, point);
            val_t dist = (point - proj).squaredNorm();
            if (dist < bestDist)
            {
                bestDist = dist;
                closest = proj;
            }
        }
        return closest;
    }
}

std::vector<Vector2p> reduceSimplex(const std::vector<Vector2p>& simplex, const Vector2p& point)
{
    if(simplex.size() == 1) return simplex;

    val_t bestDist = fizzmax<val_t>();
    Vector2p P, Q;
    Vector2p R;
    bool onEdge = false;

    for (size_t i = 0; i < simplex.size(); ++i)
    {
        Vector2p A = simplex[i];
        Vector2p B = simplex[(i + 1) % simplex.size()];
        Vector2p proj = projToEdge(A, B, point);
        val_t dist = (point - proj).squaredNorm();
        if (dist < bestDist)
        {
            P = A; Q = B; R = proj;
            bestDist = dist;
            onEdge = R != P && R != Q;
        }
    }

    if (onEdge) return { P, Q };
    else        return { R };
}

const val_t epsilon = 0.0001; // should probably be tunable?
const int maxIterations = 30;
bool shapesOverlap(const Shape& s1, const Vector2p& p1, val_t r1,
                   const Shape& s2, const Vector2p& p2, val_t r2)
{
    std::vector<Vector2p> simplex;
    const Vector2p origin = Vector2p::Zero();

    Vector2p direction = (origin - (p2 - p1));
    if (direction.squaredNorm() <= epsilon)
        direction = Vector2p(1, 0);

    Vector2p point = getCSOSupport(s1, p1, r1, s2, p2, r2, direction);
    simplex.push_back(point);
    Vector2p closest;
    int iterations = 0;
    while(iterations++ < maxIterations)
    {
        closest = closestPoint(simplex, origin);
        if (closest.squaredNorm() <= epsilon) return true;
        simplex = reduceSimplex(simplex, origin); // still contains closest
        direction = -closest; // -v = origin - v
        point = getCSOSupport(s1, p1, r1, s2, p2, r2, direction);
        if (point.dot(direction) <= 0) return false; // didn't pass origin -> must be outside
        simplex.push_back(point);
    }

    return false;
}

#pragma endregion

#pragma region EPA

Contact getShapeContact(const Shape& s1, const Vector2p& p1, val_t r1, const Shape& s2, const Vector2p& p2, val_t r2)
{
    Contact contact;
    contact.overlaps = false;

    if(!shapesOverlap(s1, p1, r1, s2, p2, r2))
        return contact;



    return contact;
}

#pragma endregion
};
