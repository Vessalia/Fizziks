#pragma once
#include <Dense.h>
#include <Shape.h>

namespace Fizziks
{
struct Collider
{
    // Physics-related part
    val_t mass;
    val_t MoI;

    // Geometry-related part
    Shape shape;
    val_t rotation;
};

struct BodyDef
{
    Vector2p initPosition;
    Vector2p initVelocity;

    val_t initRotation;
    val_t initAngularVelocity;

    val_t gravityScale;

    bool isStatic;

    std::vector<std::pair<Collider, Vector2p>> colliderDefs;
};
};
