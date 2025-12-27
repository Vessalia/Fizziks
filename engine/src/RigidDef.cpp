#include <RigidDef.h>

namespace Fizziks 
{
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
