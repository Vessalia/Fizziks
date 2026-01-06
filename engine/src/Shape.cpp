#include <Shape.h>
#include <MathUtils.h>

namespace Fizziks
{
// https://en.wikipedia.org/wiki/Centroid @ Of a polygon
Vec2 getCentroid(const std::vector<Vec2>& vertices)
{
    Vec2 centroid = Vec2::Zero();
    val_t area = 0;

    size_t n = vertices.size();
    for (size_t i = 0; i < n; ++i)
    {
        const Vec2& v0 = vertices[i];
        const Vec2& v1 = vertices[(i + 1) % n];

        val_t cross = crossproduct(v0, v1);
        centroid += (v0 + v1) * cross;
        area += cross;
    }

    return area != 0 ? centroid / (3 * area) : Vec2::Zero();
}

Shape createCircle(const val_t radius)
{
    return { ShapeType::CIRCLE, Circle{ radius } };
}

Shape createRect(val_t width, val_t height)
{
    std::vector<Vec2> vertices 
    {
        { -width / 2,  height / 2 },
        { -width / 2, -height / 2 },
        {  width / 2, -height / 2 },
        {  width / 2,  height / 2 }
    };

    return createPolygon(vertices);
}

Shape createPolygon(const std::vector<Vec2>& vertices)
{
    auto centroid = getCentroid(vertices);
    auto verts = vertices;
    for (auto& vert : verts)
    {
        vert -= centroid;
    }
    return { ShapeType::POLYGON, Polygon{ verts } };
}

AABB createAABB(val_t width, val_t height, const Vec2& offset)
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
            cx += (v0.x + v1.x) * cross;
            cy += (v0.y + v1.y) * cross;

            MoI += (v0.x * v0.x + v0.x * v1.x + v1.x * v1.x + 
                    v0.y * v0.y + v0.y * v1.y + v1.y * v1.y) * cross;
        }

        cx /= (3 * area);
        cy /= (3 * area);

        MoI = MoI / 12 - mass * (cx * cx + cy * cy);
    }

    return MoI;
}

AABB getInscribingAABB(const Shape& s, const Vec2& centroid, val_t rot)
{
    AABB aabb;

    if(s.type == ShapeType::POLYGON)
    {
        auto& polygon = std::get<Polygon>(s.data);

        Vec2 min = vec_max(), max = vec_min();
        for (const auto& vertex : polygon.vertices)
        {
            auto transformed = vertex.rotated(rot);
            min.x = std::min(min.x, transformed.x);
            min.y = std::min(min.y, transformed.y);
            max.x = std::max(max.x, transformed.x);
            max.y = std::max(max.y, transformed.y);
        }
        
        aabb.halfWidth  = (max.x - min.x) / 2;
        aabb.halfHeight = (max.y - min.y) / 2;

        aabb.offset = (min + max) / 2 - centroid;
    }
    else if(s.type == ShapeType::CIRCLE)
    {
        auto& circle = std::get<Circle>(s.data);
        aabb.halfWidth  = circle.radius;
        aabb.halfHeight = circle.radius;

        aabb.offset = Vec2::Zero();
    }

    return aabb;
}

bool AABBOverlapsAABB(const AABB& r1, const Vec2& p1, const AABB& r2, const Vec2& p2)
{
    bool overlapX = abs(p1.x - p2.x) <= r1.halfWidth  + r2.halfWidth;
    bool overlapY = abs(p1.y - p2.y) <= r1.halfHeight + r2.halfHeight;
    return overlapX && overlapY;
}
bool AABBContains(const AABB& aabb, const Vec2& pos, const Vec2& point)
{
    bool overlapX = abs(pos.x - point.x) <= aabb.halfWidth;
    bool overlapY = abs(pos.y - point.y) <= aabb.halfHeight;
    return overlapX && overlapY;
}

#pragma region CSO support
struct SupportVertex
{
    Vec2 CSO;
    Vec2 A, B;

    explicit operator Vec2() const
    {
        return CSO;
    }

    bool operator==(const SupportVertex&) const = default;
};

typedef std::vector<SupportVertex> Simplex;

Vec2 getSupport(const Shape& shape, val_t rot, const Vec2& direction)
{
    Vec2 result = Vec2::Zero();
    Vec2 dir = direction.rotated(-rot);

    if(shape.type == ShapeType::CIRCLE)
    {
        result = dir.normalized() * std::get<Circle>(shape.data).radius;
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

SupportVertex getCSOSupport(const Shape& s1, const Vec2& p1, val_t r1,
                            const Shape& s2, const Vec2& p2, val_t r2, 
                            const Vec2& dir)
{
    Vec2 support1 = getSupport(s1, r1,  dir);
    Vec2 support2 = getSupport(s2, r2, -dir);
    return { (support1.rotated(r1) + p1) - (support2.rotated(r2) + p2), support1, support2 };
}

#pragma endregion

#pragma region GJK

Vec2 projToEdge (const Vec2& P, const Vec2& Q, const Vec2& point) 
{
    Vec2 PQ = Q - P;
    val_t t = (point - P).dot(PQ) / PQ.dot(PQ);
    t = std::clamp(t, static_cast<val_t>(0), static_cast<val_t>(1));
    if      (t == 0) return P;
    else if (t == 1) return Q;
    else             return P + t * PQ; // avoid dealing with all the float math stuff
}

void enforceCCWWinding(Simplex& simplex)
{
    if (simplex.size() < 3) return;
    else if (simplex.size() == 3) // need 3rd point to be the newest one (don't move) for GJK simplex reduction alg
    {
        Vec2 A = simplex[0].CSO, B = simplex[1].CSO, C = simplex[2].CSO;
        val_t cross = crossproduct(B - A, C - A);
        if (cross < 0) std::swap(simplex[0], simplex[1]);
    }
    else
    {
        val_t signedArea = 0;

        // Shoelace formula (signed area * 2)
        for (size_t i = 0; i < simplex.size(); ++i)
        {
            const Vec2& a = simplex[i].CSO;
            const Vec2& b = simplex[(i + 1) % simplex.size()].CSO;

            signedArea += crossproduct(a, b);
        }

        // If clockwise, reverse to make CCW
        if (signedArea < 0)
        {
            std::reverse(simplex.begin(), simplex.end());
        }
    }
}

Simplex reduceSimplex(const Simplex& simplex, Vec2* dir)
{
    if (simplex.size() == 1) return simplex;
    else if (simplex.size() == 2)
    {
        // B can't be the closest to the origin, we just tried to get closer
        // AB is closest if angle between it and A to origin is positive
        // else its on the far side of A -> A is the closest
        Vec2 B = simplex[0].CSO;
        Vec2 A = simplex[1].CSO;
        Vec2 AB = B - A;
        if (AB.dot(-A) > 0)
        {
            *dir = lefttriplecross(AB, -A, AB);
            return simplex;
        }
        else
        {
            *dir = -A;
            return { simplex[1] };
        }
    }
    else if (simplex.size() == 3)
    {
        // origin can't be closest to C or B, since BC was closer. Can't be BC, we just tried to get closer
        Vec2 C = simplex[0].CSO;
        Vec2 B = simplex[1].CSO;
        Vec2 A = simplex[2].CSO;
        Vec2 AB = B - A;
        Vec2 AC = C - A;
        // AB x AC -> into the page, so (AB x AC) x AC is perp to AC and outside the triangle
        if (lefttriplecross(AB, AC, AC).dot(-A) > 0)
        {
            if (AC.dot(-A) > 0) 
            {
                *dir = lefttriplecross(AC, -A, AC); // this is the same value as our outer if, but is more 3D friendly since this may point out of the trangles plane
                return { simplex[0], simplex[2] };
            }
            else if (AB.dot(-A) > 0) // it is possible with a very wide angle we can be infront of AB
            {
                *dir = lefttriplecross(AB, -A, AB);
                return { simplex[1], simplex[2] };
            }
            else
            {
                *dir = -A;
                return { simplex[2] };
            }
        }
        // Similar to above, check if we're outside closest to AB
        else if (righttriplecross(AB, AB, AC).dot(-A) > 0)
        {
            if (AB.dot(-A) > 0)
            {
                *dir = lefttriplecross(AB, -A, AB);
                return { simplex[1], simplex[2] };
            }
            else
            {
                *dir = -A;
                return { simplex[2] };
            }
        }
        // we're inside, the origin is contained!
        else
        {
            *dir = Vec2::Zero();
            return simplex;
        }
    }
    else
    {
        ASSERT_AND_CRASH("invalid state reached in GJK");
        return simplex;
    }
}

const val_t epsilon = 0.0001; // should probably be tunable?
const Vec2 origin = Vec2::Zero();
const int maxIterationsGJK = 30;
const int maxIterationsEPA = 30;
std::pair<bool, Simplex> getGJKSimplex(const Shape& s1, const Vec2& p1, val_t r1,
                                       const Shape& s2, const Vec2& p2, val_t r2)
{
    Simplex simplex;

    Vec2 direction = p2 - p1; // doesn't really matter
    if (direction.squaredNorm() == 0) direction = Vec2(1, 0);

    auto point = getCSOSupport(s1, p1, r1, s2, p2, r2, direction);
    simplex.push_back(point);
    if (point.CSO.squaredNorm() == 0) return { true, simplex };
    direction = -point.CSO;
    for (int i = 0; i < maxIterationsGJK; ++i)
    {
        point = getCSOSupport(s1, p1, r1, s2, p2, r2, direction);
        if (point.CSO.dot(direction) <= 0) return { false, {} }; // didn't pass origin -> it must be outside
        simplex.push_back(point); // need to insert at correct index
        enforceCCWWinding(simplex);
        simplex = reduceSimplex(simplex, &direction);
        if (direction == Vec2::Zero()) return { true, simplex };
    }

    return { false, {} };
}

bool shapesOverlap(const Shape& s1, const Vec2& p1, val_t r1,
                   const Shape& s2, const Vec2& p2, val_t r2)
{
    const auto [overlaps, _] = getGJKSimplex(s1, p1, r1, s2, p2, r2);
    return overlaps;
}

#pragma endregion

#pragma region EPA

const Vec2 pos_x = { 1,  0};
const Vec2 neg_x = {-1,  0};
const Vec2 pos_y = { 0,  1};
const Vec2 neg_y = { 0, -1};
const std::array<Vec2, 4> dirs = { pos_x, neg_x, pos_y, neg_y };
const std::array<Vec2, 2> axes = { pos_x, pos_y };
Simplex blowupSimplex(const Simplex& simplex,
                      const Shape& s1, const Vec2& p1, val_t r1,
                      const Shape& s2, const Vec2& p2, val_t r2)
{
    if (simplex.size() < 1 || simplex.size() > 2) return simplex; // can only blow up a point or line

    Simplex result = simplex;
    switch(simplex.size())
    {
        case (1): // point
        {
            for (const Vec2& dir : dirs)
            {
                const SupportVertex point = getCSOSupport(s1, p1, r1, s2, p2, r2, dir);
                if ((point.CSO - result[0].CSO).squaredNorm() >= epsilon)
                {
                    result.push_back(point);
                    break;
                }
            }
            if (result.size() < 2) result.push_back(getCSOSupport(s1, p1, r1, s2, p2, r2, dirs[0]));
        [[fallthrough]];
        } // fall-through: point -> line -> triangle
        case (2): // line
        {
            const Vec2 line = result[1].CSO - result[0].CSO;
            Vec2 perp = Vec2(-line.y, line.x); // in 2D vs 3D, don't need to be careful about tangent vectors
            SupportVertex point = getCSOSupport(s1, p1, r1, s2, p2, r2, perp);
            if ((point.CSO - result[0].CSO).squaredNorm() < epsilon)
            {
                point = getCSOSupport(s1, p1, r1, s2, p2, r2, -perp);
            }
            result.push_back(point);
        }
    }

    // enforce CCW winding
    enforceCCWWinding(result);
    return result;
}

// undefined cases for when point is outside of the simplex
std::tuple<std::vector<size_t>, Vec2> closestFacet(const Simplex& simplex, const Vec2& point)
{
    if (simplex.size() < 3) return { { }, Vec2::Zero() }; // should never happen, will crash EPA

    std::vector<size_t> bestEdge;
    val_t bestDist = fizzmax<val_t>();
    for (size_t i = 0; i < simplex.size(); ++i)
    {
        const size_t from = i, to = (i + 1) % simplex.size();
        const Vec2 P = simplex[from].CSO;
        const Vec2 Q = simplex[to].CSO;
        Vec2 proj = projToEdge(P, Q, point);
        val_t dist = (proj - point).squaredNorm();
        if (dist < bestDist)
        {
            bestDist = dist;
            bestEdge = { from, to };
        }
    }

    Vec2 A = simplex[bestEdge[0]].CSO;
    Vec2 B = simplex[bestEdge[1]].CSO;
    Vec2 edge = B - A;
    Vec2 dir(edge.y, -edge.x);   // perpendicular
    if (dir.dot(-A) > 0) dir = -dir;

    return { bestEdge, dir };
}

Contact getCircleCircleContact(const Circle& c1, const Vec2& p1, val_t r1,
                               const Circle& c2, const Vec2& p2, val_t r2)
{
    Contact contact;
    contact.overlaps = false;

    Vec2 d = p2 - p1;
    val_t dist2 = d.squaredNorm();
    val_t r = c1.radius + c2.radius;

    if (dist2 >= r * r) return contact;

    val_t dist = d.norm();
    Vec2 norm = dist > epsilon ? d / dist : Vec2(0, 1);

    contact.overlaps = true;
    contact.normal = norm;
    contact.penetration = r - dist;
    contact.tangent = { -norm.y, norm.x };

    contact.contactPointWorldA = p1 + norm * c1.radius;
    contact.contactPointWorldB = p2 - norm * c2.radius;

    contact.contactPointLocalA = (contact.contactPointWorldA - p1).rotated(r1);
    contact.contactPointLocalB = (contact.contactPointWorldB - p2).rotated(r2);

    contact.featureA = 0;
    contact.featureB = 0;

    return contact;
}

uint32_t getFeature(const Shape& shape, const Vec2& pos, const Vec2& normal)
{
    if (shape.type == ShapeType::CIRCLE) return 0;

    auto& vertices = std::get<Polygon>(shape.data).vertices;
    for (int i = 0; i < vertices.size(); ++i)
    {
        Vec2 A = vertices[i];
        Vec2 B = vertices[(i + 1) % vertices.size()];
        Vec2 AB = B - A;
        Vec2 AP = pos - A;

        val_t dot = AP.dot(AB);
        if (crossproduct(AB, AP) > epsilon                // make sure point is on feature edge
         || dot < 0 || dot > AB.dot(AB)                   // make sure point is between A and B
         || std::abs(AB.dot(normal)) > epsilon) continue; // normal check picks dominant feature at vertices

        return i;
    }

    return -1;
}

Contact getShapeContact(const Shape& s1, const Vec2& p1, val_t r1,
                        const Shape& s2, const Vec2& p2, val_t r2)
{
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

    auto [closestEdge, dir] = closestFacet(simplex, origin);
    SupportVertex support = getCSOSupport(s1, p1, r1, s2, p2, r2, dir);
    SupportVertex lastSupport = { vec_max(), vec_max(), vec_max() };
    int iterations = 0;
    while (iterations++ < maxIterationsEPA && (support.CSO - lastSupport.CSO).squaredNorm() > epsilon)
    {
        // since s1 and s2 is convex, so is their minkowski difference,
        // so we don't need to remove any vertices
        int insert = (closestEdge[0] + 1) % simplex.size();
        simplex.insert(simplex.begin() + insert, support);
        std::tie(closestEdge, dir) = closestFacet(simplex, origin);
        lastSupport = support;
        support = getCSOSupport(s1, p1, r1, s2, p2, r2, dir);
    }

    // closest edge of final simplex to the origin
    std::tie(closestEdge, dir) = closestFacet(simplex, origin);
    size_t i0 = closestEdge[0], i1 = closestEdge[1];
    SupportVertex SA = simplex[i0];
    SupportVertex SB = simplex[i1];
    Vec2 A = SA.CSO, B = SB.CSO;

    Vec2 edge = B - A;

    // compute contact point via projection onto edge
    val_t denom = edge.dot(edge);
    val_t t = denom > 0 ? -(A.dot(edge)) / denom : static_cast<val_t>(0);
    t = std::clamp(t, static_cast<val_t>(0), static_cast<val_t>(1));
    SupportVertex vert = 
    {
        A    + t * edge,
        SA.A + t * (SB.A - SA.A),
        SA.B + t * (SB.B - SA.B)
    };

    contact.penetration = vert.CSO.norm();
    contact.normal = vert.CSO.normalized();
    contact.tangent = { -contact.normal.y, contact.normal.x };
    contact.contactPointLocalA = vert.A;
    contact.contactPointLocalB = vert.B;
    contact.contactPointWorldA = vert.A.rotated(r1) + p1;
    contact.contactPointWorldB = vert.B.rotated(r2) + p2;
    contact.featureA = getFeature(s1, vert.A, contact.normal);
    contact.featureB = getFeature(s2, vert.B, -contact.normal);

    return contact;
}

#pragma endregion
}
