#pragma once
#include <Fizziks/Fizziks.h>
#include <Fizziks/BodyDef.h>
#include <Fizziks/Vec.h>

#include <memory>
#include <functional>

namespace Fizziks::internal
{
class RigidBodyImpl;

struct FIZZIKS_API RigidBodyImplDeleter
{
	void operator()(RigidBodyImpl* p) const;
};
}

namespace Fizziks
{
class FIZZIKS_API RigidBody
{
public:
	RigidBody(const RigidBody& other);
	RigidBody& operator=(const RigidBody& other);

	RigidBody(RigidBody&&) noexcept = default;
	RigidBody& operator=(RigidBody&&) noexcept = default;

	void destroy();

	RigidBody& setBody(const BodyDef& def);

	Vec2 position() const;
	RigidBody& position(const Vec2& pos);

	val_t rotation() const;
	RigidBody& rotation(val_t rot);

	Vec2 centroidPosition() const;

	Vec2 velocity() const;
	RigidBody& velocity(const Vec2& vel);

	val_t angularVelocity() const;
	RigidBody& angularVelocity(val_t angVel);

	val_t mass() const;
	RigidBody& mass(val_t m);

	val_t gravityScale() const;
	RigidBody& gravityScale(val_t gs);

	BodyType bodyType() const;
	RigidBody& bodyType(const BodyType& type);

	uint32_t layer() const;
	RigidBody& layer(uint32_t layer);

	val_t linearDamping() const;
	RigidBody& linearDamping(val_t linDamp);

	val_t angularDamping() const;
	RigidBody& angularDamping(val_t angDamp);

	val_t restitution() const;
	RigidBody& restitution(val_t res);

	RigidBody& applyForce(const Vec2& force, const Vec2& at = { 0, 0 });

	RigidBody& addCollider(const ColliderDef& def);
	RigidBody& removeCollider(uint32_t ID);
	ColliderDef getCollider(uint32_t ID) const;
	RigidBody& setCollider(uint32_t ID, const ColliderDef& def);
	std::vector<ColliderDef> colliders() const;

	// todo
	// void collisionOnEnter();
	// void collisionOnStay();
	// void collisionOnExit();

	bool operator==(const RigidBody& other) const;

private:
	friend class FizzWorld;
	friend struct RigidBodyHash;

	RigidBody() : impl(nullptr, internal::RigidBodyImplDeleter{}) { }

	std::unique_ptr<internal::RigidBodyImpl, internal::RigidBodyImplDeleter> impl;
};

struct RigidBodyHash
{
	size_t operator()(const RigidBody& r) const;
};
}
