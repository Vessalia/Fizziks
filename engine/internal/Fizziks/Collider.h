#pragma once
#include <Fizziks/FizzShape.h>

namespace Fizziks::internal
{
struct Collider
{
	// Physics-related part
	val_t mass;
	val_t MoI;
	
	val_t staticFrictionCoeff = val_t(0.2);
	val_t dynamicFrictionCoeff = val_t(0.1);

	// Geometry-related part
	val_t rotation;
	InternalShape shape;

	// World-related part
	Vec2 pos;
};

Collider buildCollider(const ColliderDef& def);
ColliderDef toColliderDef(const Collider& collider);
}