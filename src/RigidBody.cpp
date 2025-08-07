#include "RigidBody.h"

RigidBody::RigidBody(Handle handle, FizzWorld* world, size_t worldIndex, const BodyDef& def) 
    : handle(handle)
    , world(world) 
    , worldIndex(worldIndex)
{
    setBody(def);
}

void RigidBody::setBody(const BodyDef& def)
{
    body.position = def.initPosition;
    body.velocity = def.initVelocity;
    body.angularVelocity = def.initAngularVelocity;

    body.mass = def.mass;

    body.shape = def.shape;

    body.isStatic = def.isStatic;
}

void RigidBody::applyForce(const Vector2p& force) { world->apply_force(worldIndex, force); }
