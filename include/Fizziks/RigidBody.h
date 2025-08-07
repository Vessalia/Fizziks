#pragma once
#include "Dense.h"
#include "FizzWorld.h"
#include "Handle.h"
#include "Shape.h"

struct BodyDef
{
    Vector2p initPosition;
    Vector2p initVelocity;
    Vector2p initAngularVelocity;

    val_t mass;

    Shape shape;

    bool isStatic;
};

class RigidBody
{
public:
    RigidBody() : RigidBody({}, nullptr, -1, {}) { };

    void setBody(const BodyDef& def);

    void applyForce(const Vector2p& force);

private:
    friend class FizzWorld;

    struct BodyData
    {
        Vector2p position, velocity, angularVelocity, accumForce;
        val_t rotation, accumTorque;

        val_t mass;

        Shape shape;

        bool isStatic;
    };

    BodyData body;

    Handle handle;
    FizzWorld* world;

    size_t worldIndex;

    RigidBody(Handle handle, FizzWorld* world, size_t worldIndex, const BodyDef& def);
};
