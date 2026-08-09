#pragma once
#include <Fizziks/Handle.h>

namespace Fizziks::internal
{
class RigidBodyImpl
{
	friend class RigidBody;
	friend struct RigidBodyImplDeleter;
	friend class FizzWorld;
	friend class FizzWorldImpl;
	friend struct std::hash<RigidBodyImpl>;

public:
	bool operator==(const RigidBodyImpl& other) const
	{
		return handle == other.handle && *world == *other.world;
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
	size_t operator()(const Fizziks::internal::RigidBodyImpl impl) const
	{
		return std::hash<Fizziks::internal::Handle>{}(impl.handle);
	}
};
}
