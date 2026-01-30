#include <Fizziks/RigidBody.h>
#include <Fizziks/FizzWorld.h>
#include <Fizziks/RigidBodyImpl.h>
#include <Fizziks/FizzWorldImpl.h>

#define THIS (impl)
#define WORLD (impl->world->impl)

namespace Fizziks
{
void RigidBody::destroy() 
{ 
    if (!THIS || !WORLD) return;

    if (THIS->world)
    {
        THIS->world->destroyBody(*this);
        THIS->world = nullptr;
    }
}

RigidBody& RigidBody::setBody(const BodyDef& def)
{
    WORLD->set_body(*THIS, def);
    return *this;
}

RigidBody& RigidBody::applyForce(const Vec2& force, const Vec2& at) 
{
    WORLD->apply_force(*THIS, force, at); 
    return *this;
}

RigidBody& RigidBody::addCollider(const Collider& collider)
{
    WORLD->add_collider(*THIS, collider);
    return *this;
}

std::vector<Collider> RigidBody::colliders() const
{
    return WORLD->body_colliders(*THIS);
}

Vec2 RigidBody::position() const 
{ 
    return WORLD->body_position(*THIS);
}
RigidBody& RigidBody::position(const Vec2& pos) 
{ 
    WORLD->body_position(*THIS, pos);
    return *this; 
}

val_t RigidBody::rotation() const
{
    return WORLD->body_rotation(*THIS);
}
RigidBody& RigidBody::rotation(val_t rot)
{
    WORLD->body_rotation(*THIS, rot);
    return *this;
}

Vec2 RigidBody::centroidPosition() const
{
    return WORLD->body_centroidPosition(*THIS);
}

Vec2 RigidBody::velocity() const 
{ 
    return WORLD->body_velocity(*THIS);
}
RigidBody& RigidBody::velocity(const Vec2& vel) 
{ 
    WORLD->body_velocity(*THIS, vel);
    return *this; 
}

val_t RigidBody::angularVelocity() const 
{ 
    return WORLD->body_angularVelocity(*THIS);
}
RigidBody& RigidBody::angularVelocity(val_t angVel) 
{ 
    WORLD->body_angularVelocity(*THIS, angVel);
    return *this; 
}

val_t RigidBody::mass() const 
{ 
    return WORLD->body_mass(*THIS);
}
RigidBody& RigidBody::mass(val_t m) 
{ 
    WORLD->body_mass(*THIS, m);
    return *this; 
}

val_t RigidBody::gravityScale() const
{
    return WORLD->body_gravityScale(*THIS);
}
RigidBody& RigidBody::gravityScale(val_t gs)
{
    WORLD->body_gravityScale(*THIS, gs);
    return *this;
}

BodyType RigidBody::bodyType() const 
{ 
    return WORLD->body_bodyType(*THIS);
}

RigidBody& RigidBody::bodyType(const BodyType& type)
{
    WORLD->body_bodyType(*THIS, type);
    return *this;
}

uint32_t RigidBody::layer() const
{
    return WORLD->body_layer(*THIS);
}

RigidBody& RigidBody::layer(uint32_t mask)
{
    WORLD->body_layer(*THIS, mask);
    return *this;
}
}
