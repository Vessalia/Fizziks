#pragma once
#include <Fizziks/Handle.h>

#include <functional>

namespace Fizziks::internal
{
class RigidBodyImpl
{
	friend class RigidBody;
	friend struct RigidBodyImplDeleter;
	friend class FizzWorld;
	friend class FizzWorldImpl;
	friend struct std::hash<RigidBodyImpl>;

	bool operator==(const RigidBodyImpl& other) const
	{
		return handle == other.handle && world->worldID == other.world->worldID;
	}

private:
	Handle handle;
	FizzWorld* world;

	RigidBodyImpl() : handle({ 0, 0 }), world(nullptr) { }
	RigidBodyImpl(Handle handle, FizzWorld* world) : handle(handle), world(world) { }
};
}

namespace std
{
template <>
struct hash<Fizziks::internal::RigidBodyImpl>
{
	size_t operator()(const Fizziks::internal::RigidBodyImpl& impl) const
	{
		size_t h1 = std::hash<Fizziks::internal::Handle>{}(impl.handle);
		size_t h2 = std::hash<size_t>{}(impl.world->worldID);
		return h1 ^ (h2 << 1);
	}
};
}
