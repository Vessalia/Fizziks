#include "FizzWorld.h"
#include "RigidBody.h"

void FizzWorld::apply_force(size_t worldIndex, Vector2p force)
{
    activeBodies[worldIndex]->body.accumForce += force;
}

RigidBody FizzWorld::createBody(const BodyDef& def)
{
    auto entity = rigidBodyPool.get();
    RigidBody* body = entity.second;
    body->handle = entity.first;
    body->world = this;
    body->worldIndex = activeBodies.size();
    body->setBody(def);
    activeBodies.push_back(body);
    return *body;
}

void FizzWorld::destroyBody(RigidBody& body)
{
    size_t index = body.worldIndex;
    size_t last = activeBodies.size() - 1;
    if(index != last && last > 0)
    {
        activeBodies[last]->worldIndex = index;
        std::swap(activeBodies[index], activeBodies[last]);
    }

    activeBodies.pop_back();
    rigidBodyPool.release(body.handle);

    body.world = nullptr;
}

const Vector2p FizzWorld::Gravity = {0, -9.81};
void FizzWorld::tick(val_t dt)
{
    for (auto& rb : activeBodies)
    {
        rb->body.accumForce += Gravity;
        rb->body.velocity += rb->body.accumForce * dt;
        rb->body.position += rb->body.velocity * dt;
        rb->body.accumForce.setZero();
    }
}
