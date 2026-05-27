#include <Fizziks/Shape.h>
#include <Fizziks/FizzShape.h>
#include <Fizziks/MathUtils.h>
#include <Fizziks/FizzLog.h>
#include <Fizziks/Ray.h>

#include <algorithm>
#include <array>
#include <unordered_map>
#include <numeric>

namespace Fizziks
{
const Vec2 origin = Vec2::Zero();
const val_t epsilon = val_t(0.0001); // should probably be tunable?

// https://en.wikipedia.org/wiki/Centroid @ Of a polygon
Vec2 getCentroid(const std::vector<Vec2>& vertices)
{
	Vec2 centroid = origin;
	val_t area = 0;

	size_t n = vertices.size();
	for (size_t i = 0; i < n; ++i)
	{
		const Vec2& v0 = vertices[i];
		const Vec2& v1 = vertices[(i + 1) % n];

		val_t cross = v0.cross(v1);
		centroid += (v0 + v1) * cross;
		area += cross;
	}

	return area != 0 ? centroid / (3 * area) : origin;
}

Circle createCircle(val_t radius)
{
	return Circle{ radius };
}

Ellipse createEllipse(val_t rx, val_t ry)
{
	return Ellipse{ rx, ry };
}

Rect createRect(val_t width, val_t height)
{
	return Rect{ width, height };
}

Polygon createPolygon(const std::vector<Vec2>& vertices)
{
	return Polygon{ vertices };
}

Capsule createCapsule(val_t capHeight, const Rect& body)
{
	return Capsule { capHeight, body };
}

val_t getMoI(const Shape& shape, val_t mass)
{
	return internal::getMoI(internal::toInternal(shape), mass);
}

AABB getBounds(const Shape& s, const Vec2& centroid, val_t rot, bool tight)
{
	return getBounds(internal::toInternal(s), centroid, rot, tight);
}

bool shapesOverlap(const Shape& s1, const Vec2& p1, val_t r1,
				   const Shape& s2, const Vec2& p2, val_t r2)
{
	return shapesOverlap(internal::toInternal(s1), p1, r1,
						 internal::toInternal(s2), p2, r2);
}

Contact getShapeContact(const Shape& s1, const Vec2& p1, val_t rot1,
						const Shape& s2, const Vec2& p2, val_t rot2)
{
	return getShapeContact(internal::toInternal(s1), p1, rot1,
						   internal::toInternal(s2), p2, rot2);
}
}

// External to internal mapping functions
namespace Fizziks::internal
{
enum class WindingDirection {
CCW, CW, Degenerate
};

WindingDirection getWindingDirection(const std::vector<Vec2>& vertices)
{
	val_t signedArea = 0;

	int n = static_cast<int>(vertices.size());
	for (int i = 0; i < n; ++i)
	{
		Vec2 u = vertices[i];
		Vec2 v = vertices[(i + 1) % n];
		signedArea += u.cross(v);
	}

	if (signedArea == 0)      return WindingDirection::Degenerate;
	else if (signedArea > 0)  return WindingDirection::CCW;
	else                      return WindingDirection::CW;
}

ConvexPiece toConvexPiece(const std::vector<Vec2>& vertices)
{
	ConvexPiece piece;

	const Vec2 centroid = getCentroid(vertices);
	std::vector<Vec2> verts = vertices;
	val_t effectiveRadius = 0;
	for (auto& vert : verts)
	{
		vert -= centroid;
		effectiveRadius = std::max(effectiveRadius, vert.norm());
	}

	if (getWindingDirection(verts) == WindingDirection::CW)
	{
		std::reverse(verts.begin(), verts.end());
	}

	// should make sure no vertices are degenerate, no crossing, and no holes

	Polygon poly(verts, effectiveRadius);

	piece.shape = poly;
	piece.offset = centroid;
	piece.rot = Mat2::Rotation(0);

	return piece;
};

Ellipse toInternal(const Fizziks::Circle& c)
{
	return Ellipse{ c.radius, c.radius };
}

Ellipse toInternal(const Fizziks::Ellipse& e)
{
	return Ellipse{ e.rx, e.ry };
}

Polygon toInternal(const Fizziks::Rect& r)
{
	std::vector<Vec2> vertices
	{
		{ -r.width / 2,  r.height / 2 },
		{ -r.width / 2, -r.height / 2 },
		{  r.width / 2, -r.height / 2 },
		{  r.width / 2,  r.height / 2 }
	};

	return Polygon{ vertices, vertices[0].norm() };
}

bool isConvex(const Polygon& poly)
{
	int sign = 0;
	int size = static_cast<int>(poly.vertices.size());
	for (int i = 0; i < size; ++i)
	{
		auto& a = poly.vertices[i];
		auto& b = poly.vertices[(i + 1) % size];
		auto& c = poly.vertices[(i + 2) % size];

		val_t cross = (c - b).cross(b - a);
		if (std::abs(cross) > epsilon)
		{
			if (sign == 0) sign = (cross > 0) ? 1 : -1;
			else if ((cross > 0) != (sign > 0)) return false;
		}
	}

	return true;
}

std::vector<std::vector<uint32_t>> triangulate(const Polygon& poly)
{
	std::vector<std::vector<uint32_t>> triangulation;
	const std::vector<Vec2>& vertices = poly.vertices;
	std::vector<uint32_t> indices(vertices.size());
	std::iota(indices.begin(), indices.end(), 0);

	auto pointInTriangle = [](const Vec2& point, const Vec2 a, const Vec2 b, const Vec2 c)
	{
		return (b - a).cross(point - a) >= 0.0f &&
			   (c - b).cross(point - b) >= 0.0f &&
			   (a - c).cross(point - c) >= 0.0f;
	};

	auto isEar = [&](uint32_t i) -> bool
	{
		uint32_t prev = indices[(i + indices.size() - 1) % indices.size()];
		uint32_t curr = indices[i];
		uint32_t next = indices[(i + 1) % indices.size()];

		Vec2 A = vertices[curr], B = vertices[prev], C = vertices[next];
		Vec2 AB = B - A, AC = C - A;

		if (AB.cross(AC) <= 0) return false;

		for (int j = 0; j < indices.size(); ++j)
		{
			uint32_t idx = indices[j];
			if (idx == curr || idx == prev || idx == next) continue;
			if (pointInTriangle(vertices[idx], B, A, C)) return false;
		}

		return true;
	};

	while (indices.size() > 3)
	{
		for (int i = 0; i < indices.size(); ++i)
		{
			if (isEar(i))
			{
				triangulation.push_back({
					indices[(i + indices.size() - 1) % indices.size()],
					indices[i],
					indices[(i + 1) % indices.size()]
				});

				indices.erase(indices.begin() + i);
				break; // since we mutated indices, we need to reset our iteration
			}
		}
	}

	if (indices.size() == 3)
	{
		triangulation.push_back({
			indices[0],
			indices[1],
			indices[2]
		});
	}

	return triangulation;
}

// assume polygon is well formed at this point
Compound decomposePolygon(const Polygon& poly)
{
	std::vector<std::vector<uint32_t>> triangulation = triangulate(poly);

	auto makeEdge = [](uint32_t a, uint32_t b) -> Edge { return Edge(std::min(a,b), std::max(a,b)); };

	std::unordered_map<Edge, bool> borderEdges;
	for (uint32_t i = 0; i < static_cast<uint32_t>(poly.vertices.size()); ++i)
	{
		borderEdges[makeEdge(i, (i + 1) % poly.vertices.size())] = true;
	}

	auto getInternalEdges = [&](const std::vector<std::vector<uint32_t>>& polyList) -> std::unordered_map<Edge, std::vector<int>>
	{
		std::unordered_map<Edge, std::vector<int>> internalEdges;
		for (int t = 0; t < polyList.size(); t++)
		{
			const auto& tri = polyList[t];
			for (int i = 0; i < tri.size(); i++)
			{
				auto edge = makeEdge(tri[i], tri[(i + 1) % tri.size()]);
				if (!borderEdges.contains(edge))
				{
					internalEdges[edge].push_back(t);
				}
			}
		}

		return internalEdges;
	};

	auto mergePolygons = [&](const std::vector<uint32_t>& p1,
							 const std::vector<uint32_t>& p2,
							 const Edge& sharedEdge) -> std::vector<uint32_t>
	{
		std::vector<uint32_t> merged;
		merged.reserve(p1.size() + p2.size() - 2);

		int start1 = -1;
		for (int i = 0; i < p1.size(); ++i)
		{
			int j = (i + 1) % p1.size();
			const Edge edge = makeEdge(p1[i], p1[j]);
			if (edge == sharedEdge) { start1 = j; break; }
		}

		int start2 = -1;
		for (int i = 0; i < p2.size(); ++i)
		{
			if (p2[i] == p1[start1]) { start2 = (i + 1) % p2.size(); break; }
		}

		for (int i = 0; i < p1.size() - 1; ++i)
		{
			merged.push_back(p1[(start1 + i) % p1.size()]);
		}

		for (int i = 0; i < p2.size() - 1; ++i)
		{
			merged.push_back(p2[(start2 + i) % p2.size()]);
		}

		return merged;
	};

	auto isConvexPiece = [&](const std::vector<uint32_t>& piece) -> bool
	{
		int sign = 0;
		int size = static_cast<int>(piece.size());
		for (int i = 0; i < size; ++i)
		{
			const Vec2& a = poly.vertices[piece[i]];
			const Vec2& b = poly.vertices[piece[(i + 1) % size]];
			const Vec2& c = poly.vertices[piece[(i + 2) % size]];
			val_t cross = (c - b).cross(b - a);
			if (std::abs(cross) > epsilon)
			{
				if (sign == 0) sign = (cross > 0) ? 1 : -1;
				else if ((cross > 0) != (sign > 0)) return false;
			}
		}

		return true;
	};

	std::vector<std::vector<uint32_t>> current = triangulation;
	std::vector<std::vector<uint32_t>> next;
	next.reserve(current.size());

	bool changed = true;
	while (changed)
	{
		changed = false;
		std::vector<bool> merged(current.size(), false);
		auto internalEdges = getInternalEdges(current);

		for (auto& [edge, connected] : internalEdges)
		{
			if (connected.size() < 2) continue;
			int t1 = connected[0], t2 = connected[1];
			if (merged[t1] || merged[t2]) continue;

			std::vector<uint32_t> candidate = mergePolygons(current[t1], current[t2], edge);
			if (isConvexPiece(candidate))
			{
				next.push_back(std::move(candidate));
				merged[t1] = merged[t2] = true;
				changed = true;
			}
		}

		for (int i = 0; i < (int)current.size(); i++)
		{
			if (!merged[i])
			{
				next.push_back(current[i]);
			}
		}

		std::swap(current, next);
		next.clear();
	}

	Compound compound;
	for (int i = 0; i < current.size(); ++i)
	{
		std::vector<Vec2> vertices;
		for (int j : current[i])
		{
			vertices.push_back(poly.vertices[j]);
		}

		compound.pieces.push_back(toConvexPiece(vertices));
	}

	return compound;
}

// this is pretty awkward, should probably fix this up
ShapeType toInternal(const Fizziks::Polygon& p)
{
	ConvexPiece piece = internal::toConvexPiece(p.vertices);
	internal::Polygon poly = std::get<internal::Polygon>(piece.shape);

	if (isConvex(poly))
	{
		return poly;
	}
	else
	{
		return decomposePolygon(poly);
	}
}

Compound toInternal(const Fizziks::Capsule& cp)
{
	const Ellipse cap { cp.body.width, cp.capHeight };
	const Mat2 noRot = Mat2::Rotation(0);

	ConvexPiece topCap { cap, Vec2(0,  cp.body.height / 2), noRot };
	ConvexPiece body { toInternal(cp.body), Vec2::Zero(), noRot };
	ConvexPiece bottomCap { cap, Vec2(0, -cp.body.height / 2), noRot };

	return
	{
		{ topCap, body, bottomCap },
		std::max(cp.body.width / 2, cp.body.height / 2 + cap.ry)
	};
}

InternalShape toInternal(const Fizziks::Shape& shape)
{
	return std::visit([](const auto& s) -> InternalShape
	{
		InternalShape is;
		is.data = toInternal(s);
		is.external = s;
		return is;
	}, shape);
}
}

/*
Current list of InternalShape operations required to be supported:
- getMoI
- getBoundsFast
- getBoundsTight
- support
- getFeature

Current list of additional Primitive operations required to be supported:
- contains
- raycast
*/

namespace Fizziks::internal::ops
{
template<typename T>
concept HasEffectiveRadius = requires(T s) { s.effectiveRadius; };

template<HasEffectiveRadius T>
AABB getBoundsFast(const T& s, const Vec2& centroid)
{
	return createAABB(2 * s.effectiveRadius, 2 * s.effectiveRadius, centroid);
}

#pragma region Primitive ops

#pragma region Ellipse ops

bool contains(const Ellipse& e, const Vec2& p)
{
	val_t nx = p.x / e.rx;
	val_t ny = p.y / e.ry;
	return nx*nx + ny*ny <= 1;
}

// (ox + t * dx)^2 / a^2 + (oy + t * dy)^2 / b^2 = 1
// expand and solve quadratic equation At^2 + Bt + C = 0:
// A = (dx / a) ^ 2 + (dy / b) ^ 2
// B = 2 * (oxdx / a^2 + oydy / b^2)
// C = (ox / a) ^ 2 + (oy / b) ^ 2 - 1
RaycastResult raycast(const Ellipse& e, const Ray& ray)
{
	val_t a2 = 1 / (e.rx * e.rx);
	val_t b2 = 1 / (e.ry * e.ry);

	val_t A = ray.dir.x * ray.dir.x * a2 + ray.dir.y * ray.dir.y * b2;
	val_t B = 2 * (ray.pos.x * ray.dir.x * a2 + ray.pos.y * ray.dir.y * b2);
	val_t C = ray.pos.x * ray.pos.x * a2 + ray.pos.y * ray.pos.y * b2 - 1;

	val_t disc = B * B - 4 * A * C;
	if (disc < 0)
	{
		return { .hit = false };
	}
	else
	{
		val_t sqrtDisc = std::sqrt(disc);
		return
		{
			.hit = true,
			.entryT = (-B - sqrtDisc) / (2 * A),
			.exitT  = (-B + sqrtDisc) / (2 * A)
		};
	}
}

val_t getMoI(const Ellipse& e, val_t mass)
{
	return val_t(0.25) * mass * (e.rx * e.rx + e.ry * e.ry);
}

AABB getBoundsFast(const Ellipse& e, const Vec2& centroid)
{
	return createAABB(2 * e.rx, 2 * e.ry, centroid);
}

AABB getBoundsTight(const Ellipse& e, const Vec2& centroid, const Mat2& rot)
{
	return getBoundsFast(e, centroid);
}

// Reference: https://personal.math.ubc.ca/~CLP/CLP3/clp_3_mc/sec_Lagrange.html
// Maximize objective f = dir . p, where p is a point on e
// Constraint is g = (p.x/e.rx)^2 + (p.y/e.ry)^2 - 1 = 0
// Lagrange multiplier : L(p.x, p.y, λ) = f + λg, ∇L = 0
// ∇f = dir, ∇g = 2λ * (p.x/e.rx^2 + p.y/e.ry^2)
// so s = dir . (e.rx^2 , e.ry^2) / 2λ
// plug s into g to get λ: 1/2λ = 1 / sqrt((dir.x * e.rx)^2 + (dir.y * e.ry)^2)
// Note: if e.rx = e.ry = r, then 1/2λ = 1 / (r * ||dir||)
// then s = dir . (r^2, r^2) / (r * ||dir||) -> collapses to Circle case
Vec2 support(const Ellipse& e, const Vec2& dir)
{
	if (e.rx == e.ry)
	{
		return dir.normalized() * e.rx;
	}
	else
	{
		val_t lambdaFactor = 1 / sqrt(dir.x * dir.x * e.rx * e.rx + dir.y * dir.y * e.ry * e.ry);
		return Vec2(dir.x * e.rx * e.rx, dir.y * e.ry * e.ry) * lambdaFactor;
	}
}

constexpr int bucketCount = 16; // should definitely be based on size somehow
uint32_t getFeature(const Ellipse& e, const Vec2& pos, const Vec2& normal)
{
	if (e.rx == e.ry)
	{
		return 0;
	}
	else
	{
		float angle = std::atan2(normal.y, normal.x);
		int bucket = static_cast<int>((angle + PI) / (TWO_PI) * bucketCount) % bucketCount;
		return static_cast<uint32_t>(bucket);
	}
}

#pragma endregion

#pragma region Polygon ops

bool contains(const Polygon& p, const Vec2& pt)
{
	for (int i = 0; i < p.vertices.size(); ++i)
	{
		Vec2 A = p.vertices[i];
		Vec2 B = p.vertices[(i+1) % p.vertices.size()];
		if ((B - A).cross(pt - A) < 0) return false;
	}
	return true;
}

// For plane/line segment P: n . (r - r0) = 0
// letting r = o + t * d -> find the t solutions
// results in t = -(o - r0) . n / (d . n)
// repeat for every line segment in the polygon
RaycastResult raycast(const Polygon& p, const Ray& ray)
{
	val_t entryT = -fizzmax<val_t>();
	val_t exitT  =  fizzmax<val_t>();

	const auto& verts = p.vertices;
	uint32_t n = static_cast<uint32_t>(verts.size());
	for (uint32_t i = 0; i < n; ++i)
	{
		Vec2 a = verts[i], b = verts[(i + 1) % n];

		Vec2 edgeNormal = (b - a).perped();
		val_t distance = (ray.pos - a).dot(edgeNormal);
		val_t correctness = ray.dir.dot(edgeNormal);

		if (abs(correctness) < epsilon) continue;

		val_t t = -distance / correctness;

		if (correctness * distance > 0) // if winding is CCW then both are +ve, if CW then both -ve
			exitT  = std::min(exitT,  t);
		else
			entryT = std::max(entryT, t);

		if (entryT > exitT) return { .hit = false };
	}

	return { .hit = true, .entryT = entryT, .exitT = exitT };
}

val_t getMoI(const Polygon& p, val_t mass)
{
	val_t moi = 0;

	val_t area = 0;
	val_t cx = 0, cy = 0;

	for(int i = 0; i < p.vertices.size(); ++i)
	{
		const auto& v0 = p.vertices[i];
		const auto& v1 = p.vertices[(i + 1) % p.vertices.size()];
		const val_t cross = v0.cross(v1);

		area += cross;
		cx += (v0.x + v1.x) * cross;
		cy += (v0.y + v1.y) * cross;

		moi += (v0.x * v0.x + v0.x * v1.x + v1.x * v1.x +
		v0.y * v0.y + v0.y * v1.y + v1.y * v1.y) * cross;
	}

	cx /= (3 * area);
	cy /= (3 * area);

	return moi / 12 - mass * (cx * cx + cy * cy);
}

// has an effective radius, so no fast impl needed
AABB getBoundsTight(const Polygon& p, const Vec2& centroid, const Mat2& rot)
{
	Vec2 min = vec_max(), max = vec_min();
	for (const auto& vertex : p.vertices)
	{
		const auto transformed = rot * vertex;
		min.x = std::min(min.x, transformed.x);
		min.y = std::min(min.y, transformed.y);
		max.x = std::max(max.x, transformed.x);
		max.y = std::max(max.y, transformed.y);
	}

	return createAABB(min + centroid, max + centroid);
}

Vec2 support(const Polygon& p, const Vec2& dir)
{
	val_t bestProj = -fizzmax<val_t>();
	Vec2 best = origin;
	for (const auto& v : p.vertices)
	{
		val_t proj = v.dot(dir);
		if (proj > bestProj)
		{
			bestProj = proj;
			best = v;
		}
	}
	return best;
}

val_t facingWeight = val_t(1);
val_t proxWeight = val_t(0.1);

uint32_t getFeature(const Polygon& p, const Vec2& pos, const Vec2& normal)
{
	uint32_t bestIndex = 0;

	val_t bestScore = fizzmax<val_t>();
	for (uint32_t i = 0; i < static_cast<uint32_t>(p.vertices.size()); ++i)
	{
		Vec2 A = p.vertices[i];
		Vec2 B = p.vertices[(i + 1) % p.vertices.size()];
		Vec2 AB = B - A;
		Vec2 AP = pos - A;

		Vec2 edgeNormal = Vec2(AB.y, -AB.x);
		val_t edgeLen = edgeNormal.norm();
		if (edgeLen < epsilon) continue;

		val_t facing = edgeNormal.dot(normal) / edgeLen;
		val_t dist = std::abs(AB.cross(AP)) / edgeLen;
		val_t proximity = 1.0f / (1.0f + dist);

		val_t score = facingWeight * facing + proxWeight * proximity;
		if (score > bestScore)
		{
			bestScore = score;
			bestIndex = i;
		}
	}

	return bestIndex;
}

#pragma endregion

#pragma endregion

#pragma region Compound ops

// has an effective radius, so no fast impl needed
AABB getBoundsTight(const Compound& cp, const Vec2& centroid, const Mat2& rot)
{
	Vec2 min = vec_max(), max = vec_min();
	for (const ConvexPiece& piece : cp.pieces)
	{
		const Mat2 rotation = rot * piece.rot;
		const Vec2 offset = rot * piece.offset + centroid;

		std::visit([&](const auto& shape)
		{
			AABB box = getBoundsTight(shape, offset, rotation);
			min.x = std::min(min.x, box.min.x);
			min.y = std::min(min.y, box.min.y);
			max.x = std::max(max.x, box.max.x);
			max.y = std::max(max.y, box.max.y);
		}, piece.shape);
	}

	return createAABB(min, max);
}

constexpr val_t integrationStep = (val_t)(1.0 / 32);

// numerical integration approach to avoid overcounting areas
val_t getMoI(const Compound& c, val_t mass)
{
	AABB bounds = ops::getBoundsTight(c, Vec2::Zero(), Mat2::Identity());
	val_t cellSize = c.effectiveRadius * integrationStep;
	val_t cellArea = cellSize * cellSize;
	int hits = 0;
	val_t moiAccum = 0;

	for (val_t x = bounds.min.x + cellSize / 2; x < bounds.max.x - cellSize / 2; x += cellSize)
	{
		for (val_t y = bounds.min.y + cellSize / 2; y < bounds.max.y - cellSize / 2; y += cellSize)
		{
			Vec2 r(x, y);
			for (const auto& piece : c.pieces)
			{
				Vec2 local = piece.rot.transposed() * (r - piece.offset);
				bool inside = std::visit([&](const auto& s) {
					return contains(s, local);
				}, piece.shape);

				if (inside)
				{
					++hits;
					moiAccum += r.squaredNorm(); // integrate by r^2dm
					break;
				}
			}
		}
	}

	val_t area = hits * cellArea;
	val_t density = mass / area;
	val_t moi = density * cellArea * moiAccum;
	return moi;
}

Vec2 support(const ConvexPiece& piece, const Vec2& dir)
{
	Vec2 localDir = piece.rot.transposed() * dir;
	Vec2 localPoint = std::visit([&](const auto& shape) -> Vec2 { return support(shape, localDir); }, piece.shape);
	return piece.rot * localPoint + piece.offset;
}

Vec2 support(const Compound& c, const Vec2& dir)
{
	val_t bestProj = -fizzmax<val_t>();
	Vec2 best = origin;
	for (const ConvexPiece& piece : c.pieces)
	{
		Vec2 candidate = support(piece, dir);
		val_t proj = candidate.dot(dir);
		if (proj > bestProj)
		{
			bestProj = proj;
			best = candidate;
		}
	}

	return best;
}

RaycastResult raycast(const ConvexPiece& piece, const Ray& ray)
{
	Ray localRay(ray.pos - piece.offset, piece.rot.transposed() * ray.dir);
	return std::visit([&](const auto& shape) -> RaycastResult { return raycast(shape, localRay); }, piece.shape);
}

uint32_t getFeature(const Compound& cp, const Vec2& pos, const Vec2& normal)
{
	uint32_t bestPiece = 0;
	val_t bestT = 0;
	Ray ray(pos, normal);

	for (uint32_t i = 0; i < static_cast<uint32_t>(cp.pieces.size()); ++i)
	{
		const auto& piece = cp.pieces[i];
		RaycastResult cast = ops::raycast(piece, ray);
		if (cast.hit && cast.exitT > 0 && cast.exitT > bestT)
		{
			bestT = cast.exitT;
			bestPiece = i;
		}
	}

	const auto& piece = cp.pieces[bestPiece];

	Vec2 localPos = piece.rot.transposed() * (pos - piece.offset);
	Vec2 localNormal = piece.rot.transposed() * normal;

	uint32_t localFeature = std::visit([&](const auto& s) {
		return getFeature(s, localPos, localNormal);
	}, piece.shape);

	return (bestPiece << 24) | (localFeature & 0xFFFFFF);
}

#pragma endregion
}

namespace Fizziks::internal
{
const int maxIterationsGJK = 30;
const int maxIterationsEPA = 30;

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
using Simplex = std::vector<SupportVertex>;

struct Facet
{
	size_t from, to;
	Vec2 dir;
};

val_t getMoI(const InternalShape& shape, val_t mass)
{
	return std::visit([mass](const auto& s) -> val_t { return ops::getMoI(s, mass); }, shape.data);
}

AABB getBoundsFast(const ShapeType& shape, const Vec2& centroid)
{
	return std::visit([&centroid](const auto& s) -> AABB { return ops::getBoundsFast(s, centroid); }, shape);
}

AABB getBoundsTight(const ShapeType& shape, const Vec2& centroid, val_t rot)
{
	const Mat2 rotation = Mat2::Rotation(rot);
	return std::visit([&centroid, &rotation](const auto& s) -> AABB { return ops::getBoundsTight(s, centroid, rotation); }, shape);
}

AABB getBounds(const ShapeType& s, const Vec2& centroid, val_t rot, bool tight)
{
	if (tight) return getBoundsTight(s, centroid, rot);
	else	   return getBoundsFast(s, centroid);
}

AABB getBounds(const InternalShape& s, const Vec2& centroid, val_t rot, bool tight)
{
	return getBounds(s.data, centroid, rot, tight);
}

Vec2 getSupport(const ShapeType& shape, const Mat2& rot, const Vec2& direction)
{
	const Vec2 dir = rot.transposed() * direction;
	return std::visit([&dir](const auto& s) -> Vec2 { return ops::support(s, dir); }, shape);
}

SupportVertex getCSOSupport(const ShapeType& s1, const Vec2& p1, const Mat2& r1,
							const ShapeType& s2, const Vec2& p2, const Mat2& r2,
							const Vec2& dir)
{
	Vec2 support1 = getSupport(s1, r1,  dir);
	Vec2 support2 = getSupport(s2, r2, -dir);
	return { (r1 * support1 + p1) - (r2 * support2 + p2), support1, support2 };
}

uint32_t getFeature(const ShapeType& shape, const Vec2& pos, const Vec2& normal)
{
	return std::visit([&pos, &normal](const auto& s) -> uint32_t { return ops::getFeature(s, pos, normal); }, shape);
}

Shape toExternal(const InternalShape& shape)
{
	return shape.external;
}

#pragma region GJK

Vec2 projToEdge (const Vec2& P, const Vec2& Q, const Vec2& point)
{
	Vec2 PQ = Q - P;
	val_t denom = PQ.dot(PQ);
	val_t t = denom < epsilon ? 0 : (point - P).dot(PQ) / denom;
	t = std::clamp(t, static_cast<val_t>(0), static_cast<val_t>(1));
	if  	(t == 0) return P;
	else if (t == 1) return Q;
	else			 return P + t * PQ; // avoid dealing with all the float math stuff
}

void enforceCCWWinding(Simplex& simplex)
{
	if (simplex.size() < 3) return;
	else if (simplex.size() == 3) // need 3rd point to be the newest one (don't move) for GJK simplex reduction alg
	{
		Vec2 A = simplex[0].CSO, B = simplex[1].CSO, C = simplex[2].CSO;
		val_t cross = (B - A).cross(C - A);
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

			signedArea += a.cross(b);
		}

		// If clockwise, reverse to make CCW
		if (signedArea < 0)
		{
			std::reverse(simplex.begin(), simplex.end());
		}
	}
}

void reduceSimplex(Simplex& simplex, Vec2& dir)
{
	if (simplex.size() == 2)
	{
		// B can't be the closest to the origin, we just tried to get closer
		// AB is closest if angle between it and A to origin is positive
		// else its on the far side of A -> A is the closest
		Vec2 B = simplex[0].CSO;
		Vec2 A = simplex[1].CSO;
		Vec2 AB = B - A;
		if (AB.dot(-A) > 0)
		{
			dir = lefttriplecross(AB, -A, AB);
		}
		else
		{
			dir = -A;
			simplex.erase(simplex.begin()); // simplex = { simplex[1] };
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
				dir = lefttriplecross(AC, -A, AC); // this is the same value as our outer if, but is more 3D friendly since this may point out of the trangles plane
				simplex.erase(simplex.begin() + 1); // simplex = { simplex[0], simplex[2] };
			}
			else if (AB.dot(-A) > 0) // it is possible with a very wide angle we can be infront of AB
			{
				dir = lefttriplecross(AB, -A, AB);
				simplex.erase(simplex.begin()); // simplex = { simplex[1], simplex[2] };
			}
			else
			{
				dir = -A;
				simplex.erase(simplex.begin());
				simplex.erase(simplex.begin()); // simplex = { simplex[2] };
			}
		}
		// Similar to above, check if we're outside closest to AB
		else if (righttriplecross(AB, AB, AC).dot(-A) > 0)
		{
			if (AB.dot(-A) > 0)
			{
				dir = lefttriplecross(AB, -A, AB);
				simplex.erase(simplex.begin()); // simplex = { simplex[1], simplex[2] };
			}
			else
			{
				dir = -A;
				simplex.erase(simplex.begin());
				simplex.erase(simplex.begin()); // simplex = { simplex[2] };
			}
		}
		// we're inside, the origin is contained!
		else
		{
			dir = Vec2::Zero();
		}
	}
	else if (simplex.size() > 1)
	{
		FIZZIKS_LOG_CRITICAL("simplex of size {:d} invalid for GJK", simplex.size());
		FIZZIKS_ASSERT_AND_CRASH("invalid state reached in GJK");
	}
}

std::pair<bool, Simplex> getGJKSimplex(const ShapeType& s1, const Vec2& p1, const Mat2& r1,
									   const ShapeType& s2, const Vec2& p2, const Mat2& r2)
{
	Simplex simplex;
	simplex.reserve(maxIterationsGJK / 2); // we'll usually exit early

	Vec2 direction = p2 - p1; // doesn't really matter
	if (direction.squaredNorm() == 0) direction = Vec2(1, 0);

	auto point = getCSOSupport(s1, p1, r1, s2, p2, r2, direction);
	simplex.push_back(point);
	if (point.CSO.squaredNorm() == 0) return { true, std::move(simplex) };
	direction = -point.CSO;
	for (int i = 0; i < maxIterationsGJK; ++i)
	{
		point = getCSOSupport(s1, p1, r1, s2, p2, r2, direction);
		if (point.CSO.dot(direction) <= 0) return { false, std::move(simplex) }; // didn't pass origin -> it must be outside
		simplex.push_back(point); // need to insert at correct index
		enforceCCWWinding(simplex);
		reduceSimplex(simplex, direction);
		if (direction == Vec2::Zero()) return { true, std::move(simplex) };
	}

	FIZZIKS_LOG_DEBUG("Max GJK iterations surpassed");
	return { false, std::move(simplex) };
}

bool shapesOverlap(const InternalShape& s1, const Vec2& p1, val_t rot1,
				   const InternalShape& s2, const Vec2& p2, val_t rot2)
{
	Mat2 r1 = Mat2::Rotation(rot1), r2 = Mat2::Rotation(rot2);
	const auto [overlaps, _] = getGJKSimplex(s1.data, p1, r1, s2.data, p2, r2);
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
void blowupSimplex(Simplex& simplex,
				   const ShapeType& s1, const Vec2& p1, const Mat2& r1,
				   const ShapeType& s2, const Vec2& p2, const Mat2& r2)
{
	if (simplex.size() < 1 || simplex.size() > 2) return; // can only blow up a point or line

	switch(simplex.size())
	{
		case (1): // point
		{
			for (const Vec2& dir : dirs)
			{
				const SupportVertex point = getCSOSupport(s1, p1, r1, s2, p2, r2, dir);
				if ((point.CSO - simplex[0].CSO).squaredNorm() >= epsilon)
				{
					simplex.push_back(point);
					break;
				}
			}
			if (simplex.size() < 2) simplex.push_back(getCSOSupport(s1, p1, r1, s2, p2, r2, dirs[0]));
		[[fallthrough]];
		} // fall-through: point -> line -> triangle
		case (2): // line
		{
			const Vec2 line = simplex[1].CSO - simplex[0].CSO;
			Vec2 perp = line.perped(); // in 2D vs 3D, don't need to be careful about tangent vectors
			SupportVertex point = getCSOSupport(s1, p1, r1, s2, p2, r2, perp);
			if ((point.CSO - simplex[0].CSO).squaredNorm() < epsilon)
			{
				point = getCSOSupport(s1, p1, r1, s2, p2, r2, -perp);
			}
			simplex.push_back(point);
		}
	}

	// enforce CCW winding
	enforceCCWWinding(simplex);
}

// undefined cases for when point is outside of the simplex
Facet closestFacet(const Simplex& simplex, const Vec2& point)
{
	if (simplex.size() < 3) return { static_cast<size_t>(-1), static_cast<size_t>(-1), Vec2::Zero() }; // should never happen, will crash EPA

	Facet bestFacet;
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
			bestFacet.from = from;
			bestFacet.to = to;
		}
	}

	Vec2 A = simplex[bestFacet.from].CSO;
	Vec2 B = simplex[bestFacet.to].CSO;
	Vec2 edge = B - A;
	Vec2 dir = edge.perped();
	if (dir.dot(-A) > 0) dir = -dir;
	bestFacet.dir = dir;

	return bestFacet;
}

// This is only valid for c1.rx = c1.ry AND c2.rx = c2.ry
Contact getCircleCircleContact(const Ellipse& c1, const Vec2& p1, const Mat2& r1,
							   const Ellipse& c2, const Vec2& p2, const Mat2& r2)
{
	Contact contact;
	contact.overlaps = false;

	Vec2 d = p2 - p1;
	val_t dist2 = d.squaredNorm();
	val_t r = c1.rx + c2.rx;

	if (dist2 >= r * r) return contact;

	val_t dist = d.norm();
	Vec2 norm = dist > epsilon ? d / dist : Vec2(0, 1);

	contact.overlaps = true;
	contact.normal = norm;
	contact.penetration = r - dist;
	contact.tangent = { -norm.y, norm.x };

	contact.contactPointWorldA = p1 + norm * c1.rx;
	contact.contactPointWorldB = p2 - norm * c2.rx;

	contact.contactPointLocalA = r1.transposed() * (contact.contactPointWorldA - p1);
	contact.contactPointLocalB = r2.transposed() * (contact.contactPointWorldB - p2);

	contact.featureA = getFeature(c1, p1, contact.normal);
	contact.featureB = getFeature(c2, p2, -contact.normal);

	return contact;
}

Contact getShapeContact(const InternalShape& shape1, const Vec2& p1, val_t rot1,
						const InternalShape& shape2, const Vec2& p2, val_t rot2)
{
	const ShapeType& s1 = shape1.data; const ShapeType& s2 = shape2.data;
	Mat2 r1 = Mat2::Rotation(rot1), r2 = Mat2::Rotation(rot2);

	if (std::holds_alternative<Ellipse>(s1) && std::get<Ellipse>(s1).rx == std::get<Ellipse>(s1).ry &&
		std::holds_alternative<Ellipse>(s2) && std::get<Ellipse>(s2).rx == std::get<Ellipse>(s2).ry)
	{
		return getCircleCircleContact(std::get<Ellipse>(s1), p1, r1,
									  std::get<Ellipse>(s2), p2, r2);
	}

	Contact contact;
	auto [overlaps, simplex] = getGJKSimplex(s1, p1, r1, s2, p2, r2);
	contact.overlaps = overlaps;
	if (!overlaps) return contact;

	blowupSimplex(simplex, s1, p1, r1, s2, p2, r2);

	Facet facet = closestFacet(simplex, origin);
	SupportVertex support = getCSOSupport(s1, p1, r1, s2, p2, r2, facet.dir);
	SupportVertex lastSupport = { vec_max(), vec_max(), vec_max() };
	int iterations = 0;
	while (iterations++ < maxIterationsEPA && (support.CSO - lastSupport.CSO).squaredNorm() > epsilon)
	{
		// since s1 and s2 is convex, so is their minkowski difference,
		// so we don't need to remove any vertices
		size_t insert = (facet.from + 1) % simplex.size();
		simplex.insert(simplex.begin() + insert, support);
		facet = closestFacet(simplex, origin);
		lastSupport = support;
		support = getCSOSupport(s1, p1, r1, s2, p2, r2, facet.dir);
	}

	if (iterations == maxIterationsEPA + 1)
		FIZZIKS_LOG_DEBUG("Max EPA iterations surpassed");

	// closest edge of final simplex to the origin
	facet = closestFacet(simplex, origin);
	size_t i0 = facet.from, i1 = facet.to;
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
	contact.contactPointWorldA = r1 * vert.A + p1;
	contact.contactPointWorldB = r2 * vert.B + p2;
	contact.featureA = getFeature(s1, vert.A, contact.normal);
	contact.featureB = getFeature(s2, vert.B, -contact.normal);

	return contact;
}

#pragma endregion
}
