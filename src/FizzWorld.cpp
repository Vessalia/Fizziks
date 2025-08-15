#include "FizzWorld.h"

namespace Fizziks
{
void FizzWorld::apply_force(Handle handle, Vector2p force)
{
    if(!handle.isValid)
        return;

    activeBodies[handle.index]->body.accumForce += force;
}

RigidBody::BodyData* FizzWorld::get_body(Handle handle) const
{
    if(!handle.isValid)
        return nullptr;

    return &activeBodies[handle.index]->body;
}

RigidBody FizzWorld::createBody(const BodyDef& def)
{
    auto entity = rigidBodyPool.get();
    RigidBody* rb = entity.second;
    rb->pool_handle = entity.first;
    rb->world_handle = {activeBodies.size(), 0, true};
    rb->world = this;
    rb->setBody(def);
    activeBodies.push_back(rb);
    return *rb;
}

void FizzWorld::destroyBody(RigidBody& body)
{
    size_t index = body.world_handle.index;
    if(index >= activeBodies.size() || !body.world_handle.isValid)
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
    for (auto& rb : activeBodies)
    {
        RigidBody::BodyData& body = rb->body;
        body.accumForce += Gravity;
        if (body.isStatic) body.accumForce.setZero();
        body.velocity += body.accumForce * dt;
        body.position += body.velocity * dt;
        body.accumForce.setZero();
    }
}
};
