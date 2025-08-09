#pragma once
#include "Dense.h"
#include "Pool.h"

// handle circular dependency
class RigidBody;
struct BodyDef;

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

    void apply_force(size_t worldIndex, Vector2p force);
};
