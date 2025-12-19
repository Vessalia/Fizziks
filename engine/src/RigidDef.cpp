#include <RigidDef.h>

namespace Fizziks 
{
BodyDef initBodyDef() 
{
    return { 
        Vector2p::Zero(), Vector2p::Zero(), 
        0, 0, 
        1, 
        0.2, 0.2, 0.1, 0.05, 0.05,
        BodyType::DYNAMIC, 
        {} 
    };
}
Collider createCollider(const Shape& shape, const val_t mass, const val_t rotation)
{
    Collider collider;
    collider.mass = mass;
    collider.shape = shape;
    collider.rotation = rotation;
    collider.MoI = getMoI(shape, mass);

    return collider;
}
}
