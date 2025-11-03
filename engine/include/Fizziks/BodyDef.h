#pragma once
#include <Dense.h>
#include <Shape.h>

namespace Fizziks
{
struct BodyDef
{
    Vector2p initPosition;
    Vector2p initVelocity;
    Vector2p initAngularVelocity;

    val_t initRotation;

    val_t mass;
    val_t gravityScale;

    Shape shape;

    bool isStatic;
};
};
