#include "RigidBody.h"
#include "FizzWorld.h"

namespace Fizziks
{
RigidBody::RigidBody(Handle pool_handle, Handle world_handle, FizzWorld* world, const BodyDef& def) 
    : pool_handle(pool_handle)
    , world_handle(world_handle)
    , world(world) 
{
    setBody(def);
}

RigidBody& RigidBody::setBody(const BodyDef& def)
{
    body.position = def.initPosition;
    body.velocity = def.initVelocity;
    body.angularVelocity = def.initAngularVelocity;

    body.mass = def.mass;

    body.shape = def.shape;

    body.isStatic = def.isStatic;

    return *this;
}

RigidBody& RigidBody::applyForce(const Vector2p& force) 
{
    world->apply_force(world_handle, force); 
    return *this;
}

Vector2p RigidBody::position() const 
{ 
    if(world)
    {
        auto* worldBody = world->get_body(world_handle);
        if(worldBody) return worldBody->position;
    }
    
    return body.position; 
}
RigidBody& RigidBody::position(const Vector2p& pos) 
{ 
    if(world)
    {
        auto* body = world->get_body(world_handle);
        if(body) body->position = pos; 
    }

    return *this; 
}

Vector2p RigidBody::velocity() const 
{ 
    if(world)
    {
        auto* worldBody = world->get_body(world_handle);
        if(worldBody) return worldBody->velocity;
    }
    
    return body.velocity; 
}
RigidBody& RigidBody::velocity(const Vector2p& vel) 
{ 
    if(world)
    {
        auto* body = world->get_body(world_handle);
        if(body) body->velocity = vel; 
    }

    return *this; 
}

Vector2p RigidBody::angularVelocity() const 
{ 
    if(world)
    {
        auto* worldBody = world->get_body(world_handle);
        if(worldBody) return worldBody->angularVelocity;
    }
    
    return body.angularVelocity; 
}
RigidBody& RigidBody::angularVelocity(const Vector2p& angVel) 
{ 
    if(world)
    {
        auto* body = world->get_body(world_handle);
        if(body) body->angularVelocity = angVel; 
    }

    return *this; 
}

val_t RigidBody::mass() const 
{ 
    if(world)
    {
        auto* worldBody = world->get_body(world_handle);
        if(worldBody) return worldBody->mass;
    }
    
    return body.mass; 
}
RigidBody& RigidBody::mass(val_t m) 
{ 
    if(world)
    {
        auto* body = world->get_body(world_handle);
        if(body) body->mass = m; 
    }

    return *this; 
}

Shape RigidBody::shape() const 
{ 
    if(world)
    {
        auto* worldBody = world->get_body(world_handle);
        if(worldBody) return worldBody->shape;
    }
    
    return body.shape; 
}
RigidBody& RigidBody::shape(Shape s) 
{ 
    if(world)
    {
        auto* body = world->get_body(world_handle);
        if(body) body->shape = s; 
    }

    return *this; 
}

bool RigidBody::isStatic() const 
{ 
    if(world)
    {
        auto* worldBody = world->get_body(world_handle);
        if(worldBody) return worldBody->isStatic;
    }
    
    return body.isStatic; 
}
RigidBody& RigidBody::isStatic(bool is) 
{ 
    if(world)
    {
        auto* body = world->get_body(world_handle);
        if(body) body->isStatic = is; 
    }

    return *this; 
}
};
