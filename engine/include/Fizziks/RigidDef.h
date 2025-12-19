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

enum class BodyType
{
    STATIC,
    DYNAMIC,
    KINEMATIC
};

struct BodyDef
{
    Vector2p initPosition;
    Vector2p initVelocity;

    val_t initRotation;
    val_t initAngularVelocity;

    val_t gravityScale;

    // need to move this over to colliders
    val_t restitution;
    val_t staticFrictionCoeff;
    val_t dynamicFrictionCoeff;
    val_t linearDamping;
    val_t angularDamping;

    BodyType bodyType;

    std::vector<std::pair<Collider, Vector2p>> colliderDefs;
};

BodyDef initBodyDef();
Collider createCollider(const Shape& shape, const val_t mass, const val_t rotation);
};
