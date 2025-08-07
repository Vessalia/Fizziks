#pragma once
#include "Dense.h"
#include "RigidBody.h"
#include "Pool.h"

class FizzWorld
{
public:
    FizzWorld(int units_x, int units_y);
    FizzWorld() : FizzWorld(10, 10) { }

    RigidBody& createBody(const BodyDef& def);
    void destroyBody(const RigidBody& body);

    void tick(val_t dt);

private:
    friend class RigidBody;
    Pool<RigidBody> rigidBodyPool;
    std::vector<RigidBody> activeBodies;

    void apply_force(size_t worldIndex, Vector2p force);
};
