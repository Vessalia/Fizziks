#include "RigidBody.h"
#include "FizzWorld.h"

namespace Fizziks
{
RigidBody::RigidBody(Handle handle, FizzWorld* world) 
    : handle(handle)
    , world(world) { }

RigidBody& RigidBody::setBody(const BodyDef& def)
{
    world->set_body(*this, def);
    return *this;
}

RigidBody& RigidBody::applyForce(const Vector2p& force) 
{
    world->apply_force(*this, force); 
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

Vector2p RigidBody::angularVelocity() const 
{ 
    return world->body_angularVelocity(*this);
}
RigidBody& RigidBody::angularVelocity(const Vector2p& angVel) 
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

Shape RigidBody::shape() const 
{ 
    return world->body_shape(*this);
}
RigidBody& RigidBody::shape(Shape s) 
{ 
    world->body_shape(*this, s);
    return *this; 
}

bool RigidBody::isStatic() const 
{ 
    return world->body_mass(*this);
}

RigidBody& RigidBody::isStatic(bool is)
{
    world->body_isStatic(*this, is);
    return *this;
}
};
