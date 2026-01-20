#include <Fizziks/RigidDef.h>

namespace Fizziks 
{
Collider createCollider(const Shape& shape, val_t mass, val_t rotation, const Vec2& pos)
{
    Collider collider;
    collider.mass = mass;
    collider.shape = shape;
    collider.rotation = rotation;
    collider.MoI = getMoI(shape, mass);
    collider.pos = pos;

    return collider;
}
}
