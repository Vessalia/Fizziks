#include "FizzWorld.h"

namespace Fizziks
{
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
    if (rb.world_handle.index >= activeHandles.size()) return nullptr;
    Handle activeHandle = activeHandles[rb.world_handle.index];
    if(activeHandle.gen != rb.world_handle.gen) return nullptr;

    return activeBodies[activeHandle.index];
}

void FizzWorld::set_body(const RigidBody& rb, const BodyDef& def)
{
    BodyData* body = get_body(rb);
    if(body) set_body(body, def);
}

void FizzWorld::set_body(BodyData* body, const BodyDef& def)
{
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
    Handle handle{ activeHandles.size(), 0 };
    if(freeList.size() > 0)
    {
        uint32_t free_index = freeList.back(); freeList.pop_back();
        activeList.push_back(free_index);
        handle.index = free_index;
        handle.gen = activeHandles[free_index].gen;
        activeHandles[free_index].index = activeBodies.size();
    }
    else
    {
        activeList.push_back(activeHandles.size());
        activeHandles.push_back(handle);
    }

    BodyData* b = entity.second;
    RigidBody rb(entity.first, handle, this);
    activeBodies.push_back(b);
    set_body(rb, def);
    return rb;
}

void FizzWorld::destroyBody(RigidBody& rb)
{
    Handle* activeHandle = &activeHandles[rb.world_handle.index];
    if(activeHandle->gen != rb.world_handle.gen) return; // nothing to do here

    //swap remove
    uint32_t swapIndex = activeHandle->index;
    std::swap(activeBodies[swapIndex], activeBodies.back());
    std::swap(activeList[swapIndex], activeList.back());
    activeBodies.pop_back();
    activeList.pop_back();
    rigidBodyPool.release(rb.pool_handle);
    if(swapIndex < activeBodies.size())
    {
        uint32_t movedIndex = activeList[swapIndex];
        activeHandles[movedIndex].index = swapIndex;
    }
    activeHandle->gen++;
    freeList.push_back(rb.world_handle.index);
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
