#pragma once
#include "Dense.h"
#include "Pool.h"
#include "RigidBody.h"
#include "BodyDef.h"

namespace Fizziks
{
class FizzWorld
{
public:
    static const Vector2p Gravity;

    FizzWorld(int units_x, int units_y) { }
    FizzWorld() : FizzWorld(10, 10) { }

    RigidBody createBody(const BodyDef& def);
    void destroyBody(RigidBody& body);

    void tick(val_t dt);

private:
    friend class RigidBody;

    Pool<RigidBody> rigidBodyPool;
    std::vector<RigidBody*> activeBodies;

    RigidBody::BodyData* get_body(Handle handle) const;

    void apply_force(Handle handle, Vector2p force);
};
};
