#pragma once
#include <Fizziks/Fizziks.h>
#include <Fizziks/Vec.h>
#include <Fizziks/Shape.h>

namespace Fizziks
{
struct Ray
{
	Vec2 pos;
	Vec2 dir;
	val_t maxDist = -1;
};

struct RaycastResult
{
	bool hit;
	uint32_t ID;
	Vec2 point, normal;
	val_t entryT, exitT;
};

FIZZIKS_API val_t raycast(const Ray& ray, const AABB& aabb);
}
