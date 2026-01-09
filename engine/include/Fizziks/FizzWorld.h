#pragma once
#include <Fizziks/Fizziks.h>
#include <Fizziks/RigidBody.h>
#include <Fizziks/RigidDef.h>
#include <Fizziks/Vec.h>

#include <vector>
#include <queue>
#include <unordered_map>

namespace Fizziks::internal
{
class FizzWorldImpl;
}

namespace Fizziks
{
class FIZZIKS_API FizzWorld
{
public:
    Vec2 Gravity = {0, -9.81};

    FizzWorld(size_t unitsX, size_t unitsY, size_t worldScale, int collisionIterations, val_t timeStep);
    FizzWorld() : FizzWorld(20, 20, 2, 5, 1 / 20.f) { }
    ~FizzWorld();

    RigidBody createBody(const BodyDef& def);
    void destroyBody(RigidBody& body);

    Vec2 worldScale() const;

    void tick(const val_t dt);

private:
    friend class RigidBody;

    internal::FizzWorldImpl* impl;
};
}
