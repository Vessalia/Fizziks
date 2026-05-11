#include <Fizziks/Collider.h>

namespace Fizziks::internal
{
Collider buildCollider(const ColliderDef& def)
{
	Collider collider;

	collider.mass = def.mass;
	collider.staticFrictionCoeff = def.staticFrictionCoeff;
	collider.dynamicFrictionCoeff = def.dynamicFrictionCoeff;
	collider.rotation = def.rotation;
	collider.shape = toInternal(def.shape);
	collider.pos = def.pos;
	
	collider.MoI = getMoI(collider.shape, collider.mass);

	return collider;
}

ColliderDef toColliderDef(uint32_t ID, const Collider& collider)
{
	ColliderDef colliderDef;
	
	colliderDef.ID = ID;
	colliderDef.mass = collider.mass;
	colliderDef.staticFrictionCoeff = collider.staticFrictionCoeff;
	colliderDef.dynamicFrictionCoeff = collider.dynamicFrictionCoeff;
	colliderDef.rotation = collider.rotation;
	colliderDef.shape = toExternal(collider.shape);
	colliderDef.pos = collider.pos;

	colliderDef.MoI = collider.MoI;

	return colliderDef;
}
}
