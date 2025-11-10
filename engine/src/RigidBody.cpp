#include "RigidBody.h"
#include "FizzWorld.h"

namespace Fizziks
{
RigidBody::RigidBody(Handle handle, FizzWorld* world) 
    : handle(handle)
    , world(world) { }

RigidBody::~RigidBody() { if (world) world->destroyBody(*this); }

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

Vector2p RigidBody::position() const 
{ 
    return world->body_position(*this);
}
RigidBody& RigidBody::position(const Vector2p& pos) 
{ 
    world->body_position(*this, pos);
    return *this; 
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
RigidBody& RigidBody::angularVelocity(val_t angVel) 
{ 
    world->body_angularVelocity(*this, angVel);
    return *this; 
}

val_t RigidBody::mass() const 
{ 
    return world->body_mass(*this);
}
RigidBody& RigidBody::mass(val_t m) 
{ 
    world->body_mass(*this, m);
    return *this; 
}

val_t RigidBody::gravityScale() const
{
    return world->body_gravityScale(*this);
}
RigidBody& RigidBody::gravityScale(val_t gs)
{
    world->body_gravityScale(*this, gs);
    return *this;
}

bool RigidBody::isStatic() const 
{ 
    return world->body_isStatic(*this);
}

RigidBody& RigidBody::isStatic(bool is)
{
    world->body_isStatic(*this, is);
    return *this;
}
};
