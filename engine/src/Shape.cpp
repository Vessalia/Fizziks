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

val_t getMoI(const Shape& shape, const val_t mass)
{
    val_t MoI = 0;
    if (shape.type == ShapeType::CIRCLE)
    {
        Circle c = std::get<Circle>(shape.data);
        MoI = 0.5 * mass * c.radius * c.radius;
    }
    else if (shape.type == ShapeType::POLYGON)
    {
        Polygon p = std::get<Polygon>(shape.data);

        val_t area = 0;
        val_t cx = 0, cy = 0;

        for(int i = 0; i < p.vertices.size(); ++i)
        {
            const auto& v0 = p.vertices[i];
            const auto& v1 = p.vertices[(i + 1) % p.vertices.size()];
            const val_t cross = crossproduct(v0, v1);

            area += cross;
            cx += (v0.x() + v1.x()) * cross;
            cy += (v0.y() + v1.y()) * cross;

            MoI += (v0.x() * v0.x() + v0.x() * v1.x() + v1.x() * v1.x() + 
                    v0.y() * v0.y() + v0.y() * v1.y() + v1.y() * v1.y()) * cross;
        }

        cx /= (3 * area);
        cy /= (3 * area);

        MoI = MoI / 12 - mass * (cx * cx + cy * cy);
    }

    return MoI;
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

        aabb.offset = Vector2p::Zero();
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
struct SupportVertex
{
    Vector2p CSO;
    Vector2p A, B;

    explicit operator Vector2p() const
    {
        return CSO;
    }
};

typedef std::vector<SupportVertex> Simplex;

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

SupportVertex getCSOSupport(const Shape& s1, const Vector2p& p1, val_t r1,
                            const Shape& s2, const Vector2p& p2, val_t r2, 
                            const Vector2p& dir)
{
    Rotation2p rot1(r1), rot2(r2);
    Vector2p support1 = getSupport(s1, r1,  dir);
    Vector2p support2 = getSupport(s2, r2, -dir);
    return { (rot1 * support1 + p1) - (rot2 * support2 + p2), support1, support2 };
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

    return fizzmax<Vector2p>(); // should never happen
}

Vector2p closestPoint(const Simplex& simplex, const Vector2p& point)
{
    std::vector<Vector2p> poly;
    for (const auto& [cso, a, b] : simplex)
    {
        poly.push_back(cso);
    }

    return closestPoint(poly, point);
}

Simplex reduceSimplex(const Simplex& simplex, const Vector2p& point)
{
    if(simplex.size() == 1) return simplex;

    val_t bestDist = fizzmax<val_t>();
    SupportVertex P, Q;
    SupportVertex R;
    bool onEdge = false;

    for (size_t i = 0; i < simplex.size(); ++i)
    {
        SupportVertex A = simplex[i];
        SupportVertex B = simplex[(i + 1) % simplex.size()];
        auto pA = A.CSO;
        auto pB = B.CSO;
        Vector2p proj = projToEdge(pA, pB, point);
        val_t dist = (point - proj).squaredNorm();
        if (dist < bestDist)
        {
            P = A; Q = B; 
            if      (proj == pA) R = A;
            else if (proj == pB) R = B;
            onEdge = proj != pA && proj != pB;
            bestDist = dist;
        }
    }

    if (onEdge) return { P, Q };
    else        return { R };
}

const val_t epsilon = 0.0001; // should probably be tunable?
const val_t epsilon2 = epsilon * epsilon;
const Vector2p origin = Vector2p::Zero();
const int maxIterations = 30;
bool shapesOverlap(const Shape& s1, const Vector2p& p1, val_t r1,
                   const Shape& s2, const Vector2p& p2, val_t r2)
{
    Simplex simplex;

    Vector2p direction = p2 - p1; // doesn't really matter
    if (direction.squaredNorm() <= epsilon2)
        direction = Vector2p(1, 0);

    auto point = getCSOSupport(s1, p1, r1, s2, p2, r2, direction);
    simplex.push_back(point);
    Vector2p closest = closestPoint(simplex, origin);
    if (closest.squaredNorm() <= epsilon2) return true;
    int iterations = 0;
    while(iterations++ < maxIterations)
    {
        simplex = reduceSimplex(simplex, origin); // still contains closest
        direction = -closest; // -v = origin - v
        point = getCSOSupport(s1, p1, r1, s2, p2, r2, direction);
        auto p = point.CSO;
        if (p != origin && p.dot(direction) <= epsilon2) return false; // didn't pass origin -> it must be outside
        simplex.push_back(point);
        closest = closestPoint(simplex, origin);
        if (closest.squaredNorm() <= epsilon2) return true;
    }

    return false;
}

#pragma endregion

#pragma region EPA

std::pair<bool, Simplex> getGJKSimplex(const Shape& s1, const Vector2p& p1, val_t r1,
                                       const Shape& s2, const Vector2p& p2, val_t r2)
{
    Simplex simplex;

    Vector2p direction = p2 - p1; // doesn't really matter
    if (direction.squaredNorm() <= epsilon2)
        direction = Vector2p(1, 0);

    auto point = getCSOSupport(s1, p1, r1, s2, p2, r2, direction);
    simplex.push_back(point);
    Vector2p closest = closestPoint(simplex, origin);
    if (closest.squaredNorm() <= epsilon2) return { true, simplex };
    int iterations = 0;
    while (iterations++ < maxIterations)
    {
        simplex = reduceSimplex(simplex, origin); // still contains closest
        direction = -closest; // -v = origin - v
        point = getCSOSupport(s1, p1, r1, s2, p2, r2, direction);
        auto p = point.CSO;
        if (p != origin && p.dot(direction) <= epsilon2) return { false, {} }; // didn't pass origin -> it must be outside
        simplex.push_back(point);
        closest = closestPoint(simplex, origin);
        if (closest.squaredNorm() <= epsilon2) return { true, simplex };
    }

    return { false, {} };
}

const Vector2p pos_x = { 1,  0};
const Vector2p neg_x = {-1,  0};
const Vector2p pos_y = { 0,  1};
const Vector2p neg_y = { 0, -1};
const std::array<Vector2p, 4> dirs = { pos_x, neg_x, pos_y, neg_y };
const std::array<Vector2p, 2> axes = { pos_x, pos_y };
Simplex blowupSimplex(const Simplex& simplex,
                      const Shape& s1, const Vector2p& p1, val_t r1,
                      const Shape& s2, const Vector2p& p2, val_t r2)
{
    if (simplex.size() < 1 || simplex.size() > 2) return simplex; // can only blow up a point or line

    Simplex result = simplex;
    switch(simplex.size())
    {
        case (1): // point
        {
            for (const Vector2p& dir : dirs)
            {
                const SupportVertex point = getCSOSupport(s1, p1, r1, s2, p2, r2, dir);
                if ((point.CSO - simplex[0].CSO).squaredNorm() >= epsilon2)
                {
                    result.push_back(point);
                    break;
                }
            }
        [[fallthrough]];
        } // fall-through: point -> line -> triangle
        case (2): // line
        {
            const Vector2p line = simplex[1].CSO - simplex[0].CSO;
            Vector2p perp = Vector2p(-line.y(), line.x()); // in 2D vs 3D, don't need to be careful about tangent vectors
            SupportVertex point = getCSOSupport(s1, p1, r1, s2, p2, r2, perp);
            if ((point.CSO - simplex[0].CSO).squaredNorm() < epsilon2)
            {
                point = getCSOSupport(s1, p1, r1, s2, p2, r2, -perp);
            }
            result.push_back(point);
        }
    }

    // enforce CCW winding
    Vector2p A = result[0].CSO, B = result[1].CSO, C = result[2].CSO;
    val_t cross = crossproduct(B - A, C - A);
    if (cross < 0) std::swap(result[0], result[1]);

    return result;
}

std::pair<size_t, Vector2p> closestFacet(const Simplex& simplex, const Vector2p& from)
{
    size_t insertIndex = 0;
    Vector2p dir;
    val_t bestDist = fizzmax<val_t>();
    for (size_t i = 0; i < simplex.size(); ++i)
    {
        const Vector2p P = simplex[i].CSO;
        const Vector2p Q = simplex[(i + 1) % simplex.size()].CSO;
        Vector2p point = projToEdge(P, Q, origin);
        val_t dist = point.squaredNorm();
        if (dist < bestDist)
        {
            bestDist = dist;
            dir = point;
            insertIndex = i;
        }
    }

    return { insertIndex, dir };
}

Contact getCircleCircleContact(const Circle& c1, const Vector2p& p1, val_t r1,
                               const Circle& c2, const Vector2p& p2, val_t r2)
{
    Contact contact;
    contact.overlaps = false;

    Vector2p d = p2 - p1;
    val_t dist2 = d.squaredNorm();
    val_t r = c1.radius + c2.radius;

    if (dist2 >= r * r) return contact;

    val_t dist = d.norm();
    Vector2p norm = dist > epsilon ? d / dist : Vector2p(0, 1);

    contact.overlaps = true;
    contact.normal = norm;
    contact.penetration = r - dist;
    contact.tangent = { -norm.y(), norm.x() };

    contact.contactPointWorldA = p1 + norm * c1.radius;
    contact.contactPointWorldB = p2 - norm * c2.radius;

    contact.contactPointLocalA = Rotation2p(-r1) * (contact.contactPointWorldA - p1);
    contact.contactPointLocalB = Rotation2p(-r2) * (contact.contactPointWorldB - p2);

    return contact;
}

Contact getShapeContact(const Shape& s1, const Vector2p& p1, val_t r1,
                        const Shape& s2, const Vector2p& p2, val_t r2)
{
    // circle-cirlce collision results in degenerate cases, handle this specially
    if (s1.type == ShapeType::CIRCLE && s2.type == ShapeType::CIRCLE)
    {
        return getCircleCircleContact(std::get<Circle>(s1.data), p1, r1,
                                      std::get<Circle>(s2.data), p2, r2);
    }

    Contact contact;

    auto [overlaps, simplex] = getGJKSimplex(s1, p1, r1, s2, p2, r2);
    contact.overlaps = overlaps;
    if (!overlaps) return contact;

    simplex = blowupSimplex(simplex, s1, p1, r1, s2, p2, r2);

    auto [insertIndex, dir] = closestFacet(simplex, origin);
    SupportVertex support = getCSOSupport(s1, p1, r1, s2, p2, r2, dir);
    SupportVertex lastSupport = simplex[insertIndex];
    while ((support.CSO - lastSupport.CSO).squaredNorm() > epsilon2)
    {
        simplex.insert(simplex.begin() + insertIndex, support);
        std::tie(insertIndex, dir) = closestFacet(simplex, origin);
        lastSupport = support;
        support = getCSOSupport(s1, p1, r1, s2, p2, r2, dir);
    }

    // closest edge of final simplex to the origin
    auto [vertIndex, _] = closestFacet(simplex, origin);
    size_t i0 = vertIndex, i1 = (vertIndex + 1) % simplex.size();
    SupportVertex SA = simplex[i0];
    SupportVertex SB = simplex[i1];
    Vector2p A = SA.CSO, B = SB.CSO;

    Vector2p edge = B - A;
    Vector2p normal(-edge.y(), edge.x()); // normal is perp to edge
    normal = normal.normalized();

    // ensure the normal points towards the origin
    if (normal.dot(A) > 0) normal = -normal;

    // penetration is distance from origin to edge along normal
    val_t penetration = std::abs(normal.dot(A));

    // compute contact point via projection onto edge
    float denom = edge.dot(edge);
    float t = denom > 0 ? -(A.dot(edge)) / denom : static_cast<val_t>(0);
    t = std::clamp(t, static_cast<val_t>(0), static_cast<val_t>(1));
    SupportVertex vert = {
        A    + t * edge,
        SA.A + t * (SB.A - SA.A),
        SA.B + t * (SB.B - SA.B)
    };

    contact.penetration = penetration;
    contact.normal = normal;
    contact.tangent = { -normal.y(), normal.x() };
    contact.contactPointLocalA = vert.A;
    contact.contactPointLocalB = vert.B;
    contact.contactPointWorldA = Rotation2p(r1) * vert.A + p1;
    contact.contactPointWorldB = Rotation2p(r2) * vert.B + p2;

    return contact;
}

#pragma endregion
};
