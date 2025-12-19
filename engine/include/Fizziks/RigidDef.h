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
    val_t rotation;
    Shape shape;
};

struct BodyDef
{
    Vector2p initPosition;
    Vector2p initVelocity;

    val_t initRotation;
    val_t initAngularVelocity;

    val_t gravityScale;

    val_t restitution;
    val_t staticFrictionCoeff;
    val_t dynamicFrictionCoeff;
    val_t linearDamping;
    val_t angularDamping;

    bool isStatic;

    std::vector<std::pair<Collider, Vector2p>> colliderDefs;
};

BodyDef initBodyDef()
{
    return { 
        Vector2p::Zero(), Vector2p::Zero(), 
        0, 0, 
        1, 
        0.2, 0.2, 0.1, 0.05, 0.05,
        false, 
        {} 
    };
}

Collider createCollider(const Shape& shape, const val_t mass, const val_t rotation);
};
