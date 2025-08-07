#include "FizzWorld.h"

FizzWorld::FizzWorld(int units_x, int units_y) { }

void FizzWorld::apply_force(size_t worldIndex, Vector2p force)
{
    activeBodies[worldIndex].body.accumForce += force;
}

RigidBody& FizzWorld::createBody(const BodyDef& def)
{
    auto [handle, body] = rigidBodyPool.get();
    body.handle = handle;
    body.world = this;
    body.worldIndex = activeBodies.size();
    body.setBody(def);
    activeBodies.emplace_back(body);
    return body;
}

void FizzWorld::destroyBody(const RigidBody& body)
{
    size_t index = body.worldIndex;
    size_t last = activeBodies.size() - 1;
    if(index != last && last > 0)
    {
        activeBodies[last].worldIndex = index;
        std::swap(activeBodies[index], activeBodies[last]);
    }

    activeBodies.pop_back();
    rigidBodyPool.release(body.handle);
}

void FizzWorld::tick(val_t dt)
{
    for (auto& rb : activeBodies)
    {
        rb.body.velocity += rb.body.accumForce * dt;
        rb.body.position += rb.body.velocity * dt;
        rb.body.accumForce.setZero();
    }
}
