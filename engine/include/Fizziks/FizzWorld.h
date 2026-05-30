#pragma once
#include <Fizziks/Fizziks.h>
#include <Fizziks/RigidBody.h>
#include <Fizziks/RigidDef.h>
#include <Fizziks/Vec.h>

#include <vector>
#include <queue>
#include <unordered_map>
#include <memory>

namespace Fizziks::internal
{
class FizzWorldImpl;

struct FIZZIKS_API FizzWorldImplDeleter
{
	void operator()(FizzWorldImpl* p) const;
};
}

namespace Fizziks
{
class FIZZIKS_API FizzWorld
{
public:
	Vec2 Gravity = {0, val_t(-9.81)};

	enum class AccelStruct
	{
		SIMPLE, BVH
	};

	FizzWorld(size_t unitsX, size_t unitsY, int collisionIterations, val_t timeStep, AccelStruct accel = AccelStruct::BVH);
	FizzWorld() : FizzWorld(20, 20, 5, 1 / 20.f, AccelStruct::BVH) { }
	~FizzWorld() = default;

	FizzWorld(FizzWorld&&) noexcept = default;
	FizzWorld& operator=(FizzWorld&&) noexcept = default;

	FizzWorld(const FizzWorld&) = delete;
	FizzWorld& operator=(const FizzWorld&) = delete;

	RigidBody createBody(const BodyDef& def);
	void destroyBody(RigidBody& body);

	Vec2 worldScale() const;

	void tick(val_t dt);

	std::vector<AABB> getBroadphaseDebugInfo() const;

private:
	friend class RigidBody;

	std::unique_ptr<internal::FizzWorldImpl, internal::FizzWorldImplDeleter> impl;
};
}
