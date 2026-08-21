#pragma once
#include <Fizziks/Fizziks.h>
#include <Fizziks/Shape.h>
#include <Fizziks/Vec.h>

namespace Fizziks
{
struct ColliderDef
{
	// inferred from other values i.e., ignored when set by user
	uint32_t ID;
	val_t MoI;

	// Physics-related part
	val_t mass = 0;

	val_t staticFrictionCoeff = val_t(0.2);
	val_t dynamicFrictionCoeff = val_t(0.1);

	// Geometry-related part
	val_t rotation = 0;
	Shape shape = Rect { 1, 1 }; // this is the default shape

	// World-related part
	Vec2 position = { 0, 0 };
};

class ColliderDefBuilder
{
public:
	ColliderDef build() const
	{
		return def;
	}

	ColliderDefBuilder& setMass(val_t mass)
	{
		def.mass = mass;
		return *this;
	}

	ColliderDefBuilder& setStaticFrictionCoeff(val_t staticFrictionCoeff)
	{
		def.staticFrictionCoeff = staticFrictionCoeff;
		return *this;
	}

	ColliderDefBuilder& setDynamicFrictionCoeff(val_t dynamicFrictionCoeff)
	{
		def.dynamicFrictionCoeff = dynamicFrictionCoeff;
		return *this;
	}

	ColliderDefBuilder& setRotation(val_t rotation)
	{
		def.rotation = rotation;
		return *this;
	}

	ColliderDefBuilder& setPosition(const Vec2& position)
	{
		def.position = position;
		return *this;
	}

	ColliderDefBuilder& setShape(const Shape& shape)
	{
		def.shape = shape;
		return *this;
	}

private:
	ColliderDef def;
};
}
