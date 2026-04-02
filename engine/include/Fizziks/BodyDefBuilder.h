#pragma once
#include <Fizziks/RigidDef.h>

namespace Fizziks
{
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

	BodyDefBuilder& setColliderDefs(const std::vector<Collider>& colliderDefs) 
	{
		def.colliderDefs = colliderDefs;
		return *this;
	}

private:
	BodyDef def;
};
}
