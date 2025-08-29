#pragma once
#include "Dense.h"
#include "BodyDef.h"
#include "Handle.h"
#include "Shape.h"

namespace Fizziks
{
class RigidBody
{
private:
    friend class FizzWorld;

    Handle handle;
    FizzWorld* world;

    RigidBody(Handle handle, FizzWorld* world);
    
public:
    RigidBody(const RigidBody& other) 
    { 
        handle = other.handle;
        world = other.world; 
    }

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
