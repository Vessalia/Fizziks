#pragma once

#include <Fizziks/Fizziks.h>
#include <Fizziks/ColliderDef.h>
#include <Fizziks/Vec.h>

#include <vector>

namespace Fizziks
{
enum class BodyType
{
	STATIC,
	DYNAMIC,
	KINEMATIC
};

struct BodyDef
{
	uint32_t layer = 0;

	Vec2 initPosition = { 0, 0 };
	Vec2 initVelocity = { 0, 0 };

	val_t initRotation = 0;
	val_t initAngularVelocity = 0;

	val_t gravityScale = 1;

	// need to move this over to colliders
	val_t restitution = val_t(0.2);
	val_t linearDamping = val_t(0.05);
	val_t angularDamping = val_t(0.05);

	BodyType bodyType = BodyType::DYNAMIC;

	std::vector<ColliderDef> colliderDefs = {};
};

class BodyDefBuilder
{
public:
	BodyDef build() const
	{
		return def;
	}

	BodyDefBuilder& setLayer(uint32_t layer)
	{
		def.layer = layer;
		return *this;
	}

	BodyDefBuilder& setInitPosition(const Vec2& initPosition)
	{
		def.initPosition = initPosition;
		return *this;
	}

	BodyDefBuilder& setInitVelocity(const Vec2& initVelocity)
	{
		def.initVelocity = initVelocity;
		return *this;
	}

	BodyDefBuilder& setInitRotation(val_t initRotation)
	{
		def.initRotation = initRotation;
		return *this;
	}

	BodyDefBuilder& setInitAngularVelocity(val_t initAngularVelocity)
	{
		def.initAngularVelocity = initAngularVelocity;
		return *this;
	}

	BodyDefBuilder& setGravityScale(val_t gravityScale)
	{
		def.gravityScale = gravityScale;
		return *this;
	}

	BodyDefBuilder& setRestitution(val_t restitution)
	{
		def.restitution = restitution;
		return *this;
	}

	BodyDefBuilder& setLinearDamping(val_t linearDamping)
	{
		def.linearDamping = linearDamping;
		return *this;
	}

	BodyDefBuilder& setAngularDamping(val_t angularDamping)
	{
		def.angularDamping = angularDamping;
		return *this;
	}

	BodyDefBuilder& setBodyType(BodyType bodyType)
	{
		def.bodyType = bodyType;
		return *this;
	}

	BodyDefBuilder& setColliderDefs(const std::vector<ColliderDef>& colliderDefs)
	{
		def.colliderDefs = colliderDefs;
		return *this;
	}

private:
	BodyDef def;
};
}
