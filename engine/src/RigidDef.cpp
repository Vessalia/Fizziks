#include <Fizziks/RigidDef.h>

namespace Fizziks
{
ColliderDef createColliderDef(const Shape& shape, val_t mass, val_t rotation, const Vec2& pos)
{
	ColliderDef colliderDef;
	colliderDef.mass = mass;
	colliderDef.shape = shape;
	colliderDef.rotation = rotation;
	colliderDef.pos = pos;

	return colliderDef;
}
}
