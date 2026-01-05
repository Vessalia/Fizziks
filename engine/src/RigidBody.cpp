#include <RigidBody.h>
#include <FizzWorld.h>
#include <RigidBodyImpl.h>
#include <FizzWorldImpl.h>

#define THIS (impl)
#define WORLD (impl->world->impl)

namespace Fizziks
{
RigidBody::RigidBody() : impl(nullptr) { } // impl constructed by FizzWorld
RigidBody::~RigidBody() { if (impl) { delete impl; } impl = nullptr; }

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

RigidBody& RigidBody::addCollider(const Collider& collider, const Vec2& at)
{
    WORLD->add_collider(*THIS, collider, at);
    return *this;
}

std::vector<std::pair<Collider, Vec2>> RigidBody::colliders() const
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
RigidBody& RigidBody::rotation(const val_t rot)
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
RigidBody& RigidBody::angularVelocity(const val_t angVel) 
{ 
    WORLD->body_angularVelocity(*THIS, angVel);
    return *this; 
}

val_t RigidBody::mass() const 
{ 
    return WORLD->body_mass(*THIS);
}
RigidBody& RigidBody::mass(const val_t m) 
{ 
    WORLD->body_mass(*THIS, m);
    return *this; 
}

val_t RigidBody::gravityScale() const
{
    return WORLD->body_gravityScale(*THIS);
}
RigidBody& RigidBody::gravityScale(const val_t gs)
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

uint32_t RigidBody::layerMask() const
{
    return WORLD->body_layerMask(*THIS);
}

RigidBody& RigidBody::layerMask(const uint32_t mask)
{
    WORLD->body_layerMask(*THIS, mask);
    return *this;
}
}
