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
	size_t worldID;

	Vec2 Gravity = {0, val_t(-9.81)};
	val_t timescale = 1;

	enum class AccelStruct
	{
		SIMPLE, BVH
	};

	FizzWorld(size_t unitsX, size_t unitsY, int collisionIterations, val_t timeStep, AccelStruct accel = AccelStruct::BVH);
	FizzWorld() : FizzWorld(20, 20, 5, 1 / 20.f, AccelStruct::BVH) { }
	~FizzWorld() = default;

	FizzWorld(const FizzWorld&) = delete;
	FizzWorld& operator=(const FizzWorld&) = delete;

	FizzWorld(FizzWorld&&) noexcept = default;
	FizzWorld& operator=(FizzWorld&&) noexcept = default;

	RigidBody createBody(const BodyDef& def);
	void destroyBody(RigidBody& body);

	Vec2 worldScale() const;

	void tick(val_t dt);

	void broadphase(AccelStruct accel);

	std::vector<RigidBody> getActiveBodies() const;
	std::vector<AABB> getBroadphaseDebugInfo() const;

private:
	friend class RigidBody;

	inline static size_t nextWorldID = 0;

	std::unique_ptr<internal::FizzWorldImpl, internal::FizzWorldImplDeleter> impl;

	std::unordered_map<RigidBody, size_t, RigidBodyHash> bodyToIndex;
	std::vector<RigidBody> activeBodies;
};
}
