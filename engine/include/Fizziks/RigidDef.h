#pragma once
#include <Fizziks/Fizziks.h>
#include <Fizziks/Shape.h>

namespace Fizziks
{
struct FIZZIKS_API Collider
{
    // Physics-related part
    val_t mass;
    val_t MoI;

    // Geometry-related part
    val_t rotation;
    Shape shape;

    // World-related part
    Vec2 pos;
};

enum class FIZZIKS_API BodyType
{
    STATIC,
    DYNAMIC,
    KINEMATIC
};

struct FIZZIKS_API BodyDef
{
    uint32_t layer = 0;

    Vec2 initPosition = { 0, 0 };
    Vec2 initVelocity = { 0, 0 };

    val_t initRotation = 0;
    val_t initAngularVelocity = 0;

    val_t gravityScale = 1;

    // need to move this over to colliders
    val_t restitution = 0.2;
    val_t staticFrictionCoeff = 0.2;
    val_t dynamicFrictionCoeff = 0.1;
    val_t linearDamping = 0.05;
    val_t angularDamping = 0.05;

    BodyType bodyType = BodyType::DYNAMIC;

    std::vector<Collider> colliderDefs = {};
};

FIZZIKS_API Collider createCollider(const Shape& shape, val_t mass, val_t rotation, const Vec2& pos = Vec2::Zero());
}
