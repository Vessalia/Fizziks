#pragma once
#include <Fizziks/Handle.h>

namespace Fizziks::internal
{
class RigidBodyImpl 
{
    friend class RigidBody;
    friend class FizzWorld;
    friend class FizzWorldImpl;

private:
    Handle handle;
    FizzWorld* world;

    RigidBodyImpl() : handle({ 0, 0 }), world(nullptr) { }
    RigidBodyImpl(Handle handle, FizzWorld* world) : handle(handle), world(world) { }
};
}

