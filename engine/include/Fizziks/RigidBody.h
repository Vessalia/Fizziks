#pragma once
#include "Fizziks.h"
#include "RigidDef.h"
#include "Vec.h"

#include <memory>

namespace Fizziks::internal
{
class RigidBodyImpl;
}

namespace Fizziks
{
class FIZZIKS_API RigidBody
{
public:
    RigidBody(const RigidBody&) = delete;
    RigidBody& operator=(const RigidBody&) = delete;

    RigidBody(RigidBody&&) noexcept = default;
    RigidBody& operator=(RigidBody&&) noexcept = default;

    void destroy();

    RigidBody& setBody(const BodyDef& def);

    Vec2 position() const;
    RigidBody& position(const Vec2& pos);

    val_t rotation() const;
    RigidBody& rotation(const val_t rot);

    Vec2 centroidPosition() const;

    Vec2 velocity() const;
    RigidBody& velocity(const Vec2& vel);

    val_t angularVelocity() const;
    RigidBody& angularVelocity(const val_t angVel);

    val_t mass() const;
    RigidBody& mass(const val_t m);

    val_t gravityScale() const;
    RigidBody& gravityScale(const val_t gs);

    BodyType bodyType() const;
    RigidBody& bodyType(const BodyType& type);

    uint32_t layerMask() const;
    RigidBody& layerMask(const uint32_t mask);

    RigidBody& applyForce(const Vec2& force, const Vec2& at = { 0, 0 });
    RigidBody& addCollider(const Collider& collider, const Vec2& at = { 0, 0 });

    std::vector<std::pair<Collider, Vec2>> colliders() const;

private:
    friend class FizzWorld;

    RigidBody() : impl(nullptr) { }

    internal::RigidBodyImpl* impl;
};
}
