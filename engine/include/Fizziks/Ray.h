#pragma once
#include <Fizziks/Fizziks.h>
#include <Fizziks/Vec.h>
#include <Fizziks/Shape.h>

namespace Fizziks
{
struct FIZZIKS_API Ray
{
	Vec2 pos;
	Vec2 dir;
	val_t maxDist = -1;
};

struct FIZZIKS_API RaycastResult
{
	bool hit;
	uint32_t ID;
	Vec2 point, normal;
	val_t entryT, exitT;
};

val_t raycast(const Ray& ray, const AABB& aabb);
}
