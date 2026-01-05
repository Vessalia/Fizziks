#pragma once
#include <Fizziks.h>
#include <RigidBody.h>
#include <RigidDef.h>
#include <Broadphase.h>
#include <VectorP.h>

#include <vector>
#include <queue>
#include <unordered_map>

namespace Fizziks
{
class FIZZIKS_API FizzWorld
{
public:
    Vec2 Gravity = {0, -9.81};

    FizzWorld(size_t unitsX, size_t unitsY, size_t worldScale, int collisionIterations, val_t timeStep);
    FizzWorld() : FizzWorld(20, 20, 2, 5, 1 / 200.f) { }
    ~FizzWorld() = default;

    RigidBody createBody(const BodyDef& def);
    void destroyBody(RigidBody& body);

    Vec2 worldScale() const;

    void tick(const val_t dt);

private:
    friend class RigidBody;

    class Impl;
    std::unique_ptr<Impl> impl;
};
};
