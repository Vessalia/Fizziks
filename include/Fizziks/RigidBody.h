#pragma once
#include "Dense.h"
#include "BodyDef.h"
#include "Handle.h"
#include "Shape.h"

namespace Fizziks
{
class FizzWorld;

class RigidBody
{
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

    Handle pool_handle;
    Handle world_handle;
    FizzWorld* world;

    RigidBody(Handle pool_handle, Handle world_handle, FizzWorld* world, const BodyDef& def);
    
public:
    RigidBody() : RigidBody({}, {}, nullptr, {}) { };

    RigidBody& setBody(const BodyDef& def);

    Vector2p position() const;
    RigidBody& position(const Vector2p& pos);

    Vector2p velocity() const;
    RigidBody& velocity(const Vector2p& vel);

    Vector2p angularVelocity() const;
    RigidBody& angularVelocity(const Vector2p& angVel);

    val_t mass() const;
    RigidBody& mass(val_t m);

    Shape shape() const;
    RigidBody& shape(Shape s);

    bool isStatic() const;
    RigidBody& isStatic(bool is);

    RigidBody& applyForce(const Vector2p& force);
};
};
