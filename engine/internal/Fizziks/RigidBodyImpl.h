#pragma once
#include <RigidBody.h>
#include <FizzWorld.h>
#include <FizzWorldImpl.h>
#include <Handle.h>

namespace Fizziks
{
class RigidBody::Impl 
{
    friend class RigidBody;
    friend class FizzWorld;
    friend class FizzWorld::Impl;

private:
    Handle handle;
    FizzWorld* world;

    RigidBody::Impl(Handle handle, FizzWorld* world);
};
}

