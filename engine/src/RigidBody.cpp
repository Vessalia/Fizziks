#include "RigidBody.h"
#include "FizzWorld.h"

namespace Fizziks
{
RigidBody::RigidBody(Handle handle, FizzWorld* world) 
    : handle(handle)
    , world(world) { }

void RigidBody::destroy() 
{ 
    if (world)
    {
        world->destroyBody(*this);
        world = nullptr;
    }
}

RigidBody& RigidBody::setBody(const BodyDef& def)
{
    world->set_body(*this, def);
    return *this;
}

RigidBody& RigidBody::applyForce(const Vector2p& force, const Vector2p& at) 
{
    world->apply_force(*this, force, at); 
    return *this;
}

RigidBody& RigidBody::addCollider(const Collider& collider, const Vector2p& at)
{
    world->add_collider(*this, collider, at);
    return *this;
}

std::vector<std::pair<Collider, Vector2p>> RigidBody::colliders() const
{
    return world->body_colliders(*this);
}

Vector2p RigidBody::position() const 
{ 
    return world->body_position(*this);
}
RigidBody& RigidBody::position(const Vector2p& pos) 
{ 
    world->body_position(*this, pos);
    return *this; 
}

val_t RigidBody::rotation() const
{
    return world->body_rotation(*this);
}
RigidBody& RigidBody::rotation(const val_t rot)
{
    world->body_rotation(*this, rot);
    return *this;
}

Vector2p RigidBody::centroidPosition() const
{
    return world->body_centroidPosition(*this);
}

Vector2p RigidBody::velocity() const 
{ 
    return world->body_velocity(*this);
}
RigidBody& RigidBody::velocity(const Vector2p& vel) 
{ 
    world->body_velocity(*this, vel);
    return *this; 
}

val_t RigidBody::angularVelocity() const 
{ 
    return world->body_angularVelocity(*this);
}
RigidBody& RigidBody::angularVelocity(const val_t angVel) 
{ 
    world->body_angularVelocity(*this, angVel);
    return *this; 
}

val_t RigidBody::mass() const 
{ 
    return world->body_mass(*this);
}
RigidBody& RigidBody::mass(const val_t m) 
{ 
    world->body_mass(*this, m);
    return *this; 
}

val_t RigidBody::gravityScale() const
{
    return world->body_gravityScale(*this);
}
RigidBody& RigidBody::gravityScale(const val_t gs)
{
    world->body_gravityScale(*this, gs);
    return *this;
}

BodyType RigidBody::bodyType() const 
{ 
    return world->body_bodyType(*this);
}

RigidBody& RigidBody::bodyType(const BodyType& type)
{
    world->body_bodyType(*this, type);
    return *this;
}
};
