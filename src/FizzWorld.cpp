#include "FizzWorld.h"

namespace Fizziks
{
struct FizzWorld::BodyData
{
    Vector2p position, velocity, angularVelocity, accumForce;
    val_t rotation, accumTorque;

    val_t mass;

    Shape shape;

    bool isStatic;
};

const FizzWorld::BodyData FizzWorld::null_body = 
{
    fizzmax<Vector2p>(),
    fizzmax<Vector2p>(),
    fizzmax<Vector2p>(),
    fizzmax<Vector2p>(),
    fizzmax<val_t>(),
    fizzmax<val_t>(),
    fizzmax<val_t>(),
    {},
    false
};

void FizzWorld::apply_force(const RigidBody& rb, const Vector2p& force)
{
    BodyData* body = get_body(rb);
    if(body) body->accumForce += force;
}

FizzWorld::BodyData* FizzWorld::get_body(const RigidBody& rb) const
{
    if(!rb.world_handle.isValid)
        return nullptr;

    return activeBodies[rb.world_handle.index];
}

void FizzWorld::set_body(const RigidBody& rb, const BodyDef& def)
{
    Handle bodyHandle = activeBodyHandles[rb.world_handle.index];
    BodyData* body = activeBodies[bodyHandle.index];
    
    body->position = def.initPosition;
    body->velocity = def.initVelocity;
    body->angularVelocity = def.initAngularVelocity;

    body->mass = def.mass;

    body->shape = def.shape;

    body->isStatic = def.isStatic;
}

Vector2p FizzWorld::body_position(const RigidBody& rb) const 
{
    BodyData* body = get_body(rb);
    if(body) return body->position;
    else     return null_body.position;
}
void FizzWorld::body_position(const RigidBody& rb, const Vector2p& pos) 
{ 
    BodyData* body = get_body(rb);
    if(body) body->position = pos;
}

Vector2p FizzWorld::body_velocity(const RigidBody& rb) const 
{ 
    BodyData* body = get_body(rb);
    if(body) return body->velocity;
    else     return null_body.velocity;
}
void FizzWorld::body_velocity(const RigidBody& rb, const Vector2p& vel) 
{ 
    BodyData* body = get_body(rb);
    if(body) body->velocity = vel;
}

Vector2p FizzWorld::body_angularVelocity(const RigidBody& rb) const 
{ 
    BodyData* body = get_body(rb);
    if(body) return body->angularVelocity;
    else     return null_body.angularVelocity;
}
void FizzWorld::body_angularVelocity(const RigidBody& rb, const Vector2p& angVel) 
{ 
    BodyData* body = get_body(rb);
    if(body) body->angularVelocity = angVel;
}

val_t FizzWorld::body_mass(const RigidBody& rb) const 
{ 
    BodyData* body = get_body(rb);
    if(body) return body->mass;
    else     return null_body.mass;
}
void FizzWorld::body_mass(const RigidBody& rb, val_t m) 
{ 
    BodyData* body = get_body(rb);
    if(body) body->mass = m;
}

Shape FizzWorld::body_shape(const RigidBody& rb) const 
{ 
    BodyData* body = get_body(rb);
    if(body) return body->shape;
    else     return null_body.shape;
}
void FizzWorld::body_shape(const RigidBody& rb, Shape s) 
{ 
    BodyData* body = get_body(rb);
    if(body) body->shape = s;
}

bool FizzWorld::body_isStatic(const RigidBody& rb) const 
{ 
    BodyData* body = get_body(rb);
    if(body) return body->isStatic;
    else     return null_body.isStatic;
}
void FizzWorld::body_isStatic(const RigidBody& rb, bool is) 
{ 
    BodyData* body = get_body(rb);
    if(body) body->isStatic = is;
}

RigidBody FizzWorld::createBody(const BodyDef& def)
{
    auto entity = rigidBodyPool.get();
    BodyData* b = entity.second;
    RigidBody rb(entity.first, {activeBodies.size(), 0, true}, *this);
    activeBodies.push_back(b);
    activeBodyHandles.push_back(rb.world_handle);
    set_body(rb, def);
    return rb;
}

void FizzWorld::destroyBody(RigidBody& rb)
{
    size_t index = rb.world_handle.index;
    if(index >= activeBodyHandles.size() || !rb.world_handle.isValid)
        return;

    // deferred destruction queue in future
    size_t last = activeBodies.size() - 1;
    if(index != last && last > 0)
    {
        activeBodies[last]->world_handle.index = index;
        std::swap(activeBodies[index], activeBodies[last]);
    }

    activeBodies.pop_back();
    rigidBodyPool.release(body.pool_handle);
    body.world_handle.isValid = false;
}

const Vector2p FizzWorld::Gravity = {0, -9.81};
void FizzWorld::tick(val_t dt)
{
    for (auto* body : activeBodies)
    {
        body->accumForce += Gravity;
        if (body->isStatic) body->accumForce.setZero();
        body->velocity += body->accumForce * dt;
        body->position += body->velocity * dt;
        body->accumForce.setZero();
    }
}
};
