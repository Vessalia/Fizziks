#pragma once
#include <Dense.h>
#include <RigidDef.h>
#include <Handle.h>
#include <Shape.h>

namespace Fizziks
{
class RigidBody
{
private:
    friend class FizzWorld;

    Handle handle;
    FizzWorld* world;

    RigidBody(Handle handle, FizzWorld* world);
    
public:
    void destroy();

    RigidBody& setBody(const BodyDef& def);

    Vector2p position() const;
    RigidBody& position(const Vector2p& pos);

    Vector2p centroidPosition() const;

    Vector2p velocity() const;
    RigidBody& velocity(const Vector2p& vel);

    val_t angularVelocity() const;
    RigidBody& angularVelocity(val_t angVel);

    val_t mass() const;
    RigidBody& mass(val_t m);

    val_t gravityScale() const;
    RigidBody& gravityScale(val_t gs);

    BodyType bodyType() const;
    RigidBody& bodyType(BodyType type);

    RigidBody& applyForce(const Vector2p& force, const Vector2p& at = Vector2p::Zero());
    RigidBody& addCollider(const Collider& collider, const Vector2p& at = Vector2p::Zero());

    std::vector<std::pair<Collider, Vector2p>> colliders() const;
};
};
