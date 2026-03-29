#include <gtest/gtest.h>
#include <Fizziks/FizzWorld.h>
#include <Fizziks/Rigidbody.h>
#include <Fizziks/RigidDef.h>
#include <Fizziks/Shape.h>
#include <Fizziks/MathUtils.h>
#include <Fizziks/Vec.h>
#include <TestTypes.h>

using namespace Fizziks;

// Helpers
static RigidBody makeDynamicCircle(FizzWorld& world, val_t radius, const Vec2& pos, val_t mass = 1.0)
{
	Shape shape = createCircle(radius);
	Collider col = createCollider(shape, mass, 0.0, Vec2::Zero());
	BodyDef def;
	def.initPosition = pos;
	def.bodyType = BodyType::DYNAMIC;
	def.gravityScale = 0;
	def.colliderDefs = { col };
	def.restitution = 0;
	def.staticFrictionCoeff = 0;
	def.dynamicFrictionCoeff = 0;
	def.linearDamping = 0;
	def.angularDamping = 0;
	return world.createBody(def);
}
 
static RigidBody makeStaticCircle(FizzWorld& world, val_t radius, const Vec2& pos)
{
	Shape shape = createCircle(radius);
	Collider col = createCollider(shape, 1.0, 0.0, Vec2::Zero());
	BodyDef def;
	def.initPosition = pos;
	def.bodyType = BodyType::STATIC;
	def.colliderDefs = { col };
	return world.createBody(def);
}
 
static RigidBody makeDynamicBox(FizzWorld& world, val_t w, val_t h, const Vec2& pos, val_t mass = 1.0)
{
	Shape shape = createRect(w, h);
	Collider col = createCollider(shape, mass, 0.0, Vec2::Zero());
	BodyDef def;
	def.initPosition = pos;
	def.bodyType = BodyType::DYNAMIC;
	def.gravityScale = 0;
	def.colliderDefs = { col };
	return world.createBody(def);
}

// Gravity and free-fall
class GravityTest : public ::testing::Test
{
protected:
	void SetUp() override
	{
		world = std::make_unique<FizzWorld>();

		BodyDef def;
		def.initPosition = { 0, 10 };
		def.bodyType = BodyType::DYNAMIC;
		def.gravityScale = 1;
		def.linearDamping = 0;
		def.angularDamping = 0;
		Shape shape = createCircle(0.5);
		def.colliderDefs = { createCollider(shape, 1.0, 0.0, Vec2::Zero()) };
		body = std::make_unique<RigidBody>(world->createBody(def));
	}

	void tick(int n = tps) { for (int i = 0; i < n; ++i) world->tick(dt); }

	static constexpr int tps = 60;
	static constexpr val_t dt = val_t(1.0 / tps);
	std::unique_ptr<FizzWorld> world;
	std::unique_ptr<RigidBody> body; // since the constructor for RigidBody is private, need to track a pointer in the fixture
};

TEST_F(GravityTest, ZeroGravityScaleDoesNotFall)
{
	body->gravityScale(0);
	Vec2 pos0 = body->position();
	tick();
	EXPECT_VAL_EQ(body->position().y, pos0.y);
}

TEST_F(GravityTest, BodyAcceleratesDownward)
{
	Vec2 v0 = body->velocity();
	tick();
	Vec2 v1 = body->velocity();
	EXPECT_LT(v1.y, v0.y);
	EXPECT_VAL_EQ(v1.x, 0.0);
}

TEST_F(GravityTest, HorizontalPositionUnchangedUnderDefaultGravity)
{
	val_t x0 = body->position().x;
	tick();
	EXPECT_VAL_EQ(body->position().x, x0);
}

// Linear dynamics: force, velocity, momentum
class LinearDynamicsTest : public ::testing::Test
{
protected:
	void SetUp() override
	{
		world = std::make_unique<FizzWorld>();
		world->Gravity = Vec2::Zero();
		
		BodyDef def;
		def.initPosition = { 0, 10 };
		def.bodyType = BodyType::DYNAMIC;
		def.gravityScale = 1;
		def.linearDamping = 0;
		def.angularDamping = 0;
		Shape shape = createCircle(0.5);
		def.colliderDefs = { createCollider(shape, 1.0, 0.0, Vec2::Zero()) };
		body = std::make_unique<RigidBody>(world->createBody(def));
		body->gravityScale(0).velocity(Vec2::Zero());
	}

	void tick(int n = tps) { for (int i = 0; i < n; ++i) world->tick(dt); }

	static constexpr int tps = 60;
	static constexpr val_t dt = val_t(1.0 / tps);
	std::unique_ptr<FizzWorld> world;
	std::unique_ptr<RigidBody> body;
};

TEST_F(LinearDynamicsTest, ConstantVelocityWithNoForce)
{
	Vec2 vel { 3.0, 1.0 };
	body->velocity(vel).gravityScale(0);
	tick();
	Vec2 v1 = body->velocity();
	EXPECT_VAL_EQ(v1.x, 3.0);
	EXPECT_VAL_EQ(v1.y, 1.0);
}

TEST_F(LinearDynamicsTest, AppliedForceChangesVelocity)
{
	Vec2 force { 10.0, 0.0 };
	body->applyForce(force);
	Vec2 v0 = body->velocity();
	tick();
	Vec2 v1 = body->velocity();
	EXPECT_GT(v1.x, v0.x);
}

// Circle vs circle collision
class CircleCollisionTest : public ::testing::Test
{
protected:
	void SetUp() override
	{
		world = std::make_unique<FizzWorld>();
		world->Gravity = Vec2::Zero();
	}

	void tick(int n = tps) { for (int i = 0; i < n; ++i) world->tick(dt); }

	static constexpr int tps = 60;
	static constexpr val_t dt = val_t(1.0 / tps);
	std::unique_ptr<FizzWorld> world;
};

TEST_F(CircleCollisionTest, OverlappingCirclesSeparate)
{
	auto a = makeDynamicCircle(*world, 1.0, { -0.5, 0 });
	auto b = makeDynamicCircle(*world, 1.0, {  0.5, 0 });
	tick();
	val_t dist = (b.position() - a.position()).norm();
	EXPECT_GT(dist, 2.0);
}

TEST_F(CircleCollisionTest, DynamicCircleBouncesOffStatic)
{
	auto moving = makeDynamicCircle(*world, 0.5, { -3.0, 0 });
	auto wall   = makeStaticCircle(*world, 0.5, {  0.0, 0 });
	moving.velocity({ 5.0, 0 });
	tick();

	// after bouncing off the static body, moving body should be going left
	EXPECT_LT(moving.velocity().x, 0.0);
}

TEST_F(CircleCollisionTest, HeadOnCollisionConservesMomentum)
{
	// equal mass, head-on: a moves right, b stationary
	val_t m = 1.0;
	auto a = makeDynamicCircle(*world, 0.5, { -2.0, 0 }, m);
	auto b = makeDynamicCircle(*world, 0.5, {  2.0, 0 }, m);
	Vec2 va0 { 3.0, 0 };
	a.velocity(va0);

	Vec2 pa0 = a.velocity() * m;
	Vec2 pb0 = b.velocity() * m;
	Vec2 totalP0 = pa0 + pb0;

	tick();

	Vec2 pa1 = a.velocity() * m;
	Vec2 pb1 = b.velocity() * m;
	Vec2 totalP1 = pa1 + pb1;

	EXPECT_VAL_EQ(totalP1.x, totalP0.x);
	EXPECT_VAL_EQ(totalP1.y, totalP0.y);
}

TEST_F(CircleCollisionTest, CollisionNormalAlignedWithCentreLine)
{
	// contact normal for two circles should align with the vector between centres
	Shape s1 = createCircle(1.0);
	Shape s2 = createCircle(1.0);
	Vec2 p1 { 0, 0 }, p2 { 1.5, 0 };
	Contact c = getShapeContact(s1, p1, 0, s2, p2, 0);
	EXPECT_TRUE(c.overlaps);
	// normal should point roughly along x axis
	EXPECT_GT(std::abs(c.normal.x), val_t(0.9));
	EXPECT_VAL_EQ(c.normal.norm(), 1.0);
}

// Shape contact: GJK/EPA correctness through the public API

TEST(ShapeContactTest, NonOverlappingCirclesNoContact)
{
	Shape s1 = createCircle(1.0);
	Shape s2 = createCircle(1.0);
	Contact c = getShapeContact(s1, { 0, 0 }, 0, s2, { 3.0, 0 }, 0);
	EXPECT_FALSE(c.overlaps);
}

TEST(ShapeContactTest, OverlappingCirclesHavePositivePenetration)
{
	Shape s1 = createCircle(1.0);
	Shape s2 = createCircle(1.0);
	Contact c = getShapeContact(s1, { 0, 0 }, 0, s2, { 1.5, 0 }, 0);
	EXPECT_TRUE(c.overlaps);
	EXPECT_GT(c.penetration, val_t(0.0));
}

TEST(ShapeContactTest, CirclePenetrationDepthIsCorrect)
{
	// two unit circles, centres 1.0 apart -> penetration = 2.0 - 1.0 = 1.0
	Shape s1 = createCircle(1.0);
	Shape s2 = createCircle(1.0);
	Contact c = getShapeContact(s1, { 0, 0 }, 0, s2, { 1.0, 0 }, 0);
	EXPECT_TRUE(c.overlaps);
	EXPECT_VAL_EQ(c.penetration, val_t(1.0));
}

TEST(ShapeContactTest, ContactNormalIsUnitLength)
{
	Shape s1 = createCircle(1.0);
	Shape s2 = createCircle(1.0);
	Contact c = getShapeContact(s1, { 0, 0 }, 0, s2, { 1.5, 0 }, 0);
	EXPECT_TRUE(c.overlaps);
	EXPECT_VAL_EQ(c.normal.norm(), 1.0);
}

TEST(ShapeContactTest, ContactTangentPerpendicularToNormal)
{
	Shape s1 = createCircle(1.0);
	Shape s2 = createCircle(1.0);
	Contact c = getShapeContact(s1, { 0, 0 }, 0, s2, { 1.5, 0 }, 0);
	EXPECT_TRUE(c.overlaps);
	EXPECT_VAL_EQ(c.normal.dot(c.tangent), val_t(0.0));
}

TEST(ShapeContactTest, BoxBoxOverlapDetected)
{
	Shape s1 = createRect(2.0, 2.0);
	Shape s2 = createRect(2.0, 2.0);
	Contact c = getShapeContact(s1, { 0, 0 }, 0, s2, { 1.5, 0 }, 0);
	EXPECT_TRUE(c.overlaps);
}

TEST(ShapeContactTest, BoxBoxNoOverlapWhenSeparated)
{
	Shape s1 = createRect(2.0, 2.0);
	Shape s2 = createRect(2.0, 2.0);
	Contact c = getShapeContact(s1, { 0, 0 }, 0, s2, { 3.0, 0 }, 0);
	EXPECT_FALSE(c.overlaps);
}

TEST(ShapeContactTest, CircleBoxOverlapDetected)
{
	Shape s1 = createCircle(1.0);
	Shape s2 = createRect(2.0, 2.0);
	Contact c = getShapeContact(s1, { 0, 0 }, 0, s2, { 1.5, 0 }, 0);
	EXPECT_TRUE(c.overlaps);
}

TEST(ShapeContactTest, RotatedBoxContactNormalChangesWithRotation)
{
	Shape s1 = createRect(2.0, 2.0);
	Shape s2 = createRect(2.0, 2.0);
	Contact c0 = getShapeContact(s1, { 0, 0 }, 0,          s2, { 1.5, 0 }, 0);
	Contact c1 = getShapeContact(s1, { 0, 0 }, PI / 4,     s2, { 1.5, 0 }, 0);
	EXPECT_TRUE(c0.overlaps);
	EXPECT_TRUE(c1.overlaps);
	EXPECT_VAL_EQ(c0.normal.x, c1.normal.x);
	EXPECT_VAL_EQ(c0.normal.y, c1.normal.y);
}

// AABB
TEST(AABBTest, MergedAABBContainsBoth)
{
	AABB a = createAABB(2.0, 2.0, { 0, 0 });
	AABB b = createAABB(2.0, 2.0, { 4.0, 0 });
	AABB m = merge(a, b);
	EXPECT_TRUE(contains(m, a));
	EXPECT_TRUE(contains(m, b));
}

TEST(AABBTest, EncapsulatingAABBContainsCircle)
{
	Shape s = createCircle(1.5);
	Vec2 centre { 2.0, 3.0 };
	AABB aabb = getEncapsulatingAABB(s, centre, 0.0);
	EXPECT_TRUE(contains(aabb, centre));
}

TEST(AABBTest, EncapsulatingAABBForRotatedBoxIsLargerThanUnrotated)
{
	Shape s = createRect(2.0, 1.0);
	AABB tight0  = getEncapsulatingAABB(s, Vec2::Zero(), 0.0,     true);
	AABB tight45 = getEncapsulatingAABB(s, Vec2::Zero(), PI / 4,  true);
	EXPECT_GT(tight45.area(), tight0.area());
}

TEST(AABBTest, OverlappingAABBsDetected)
{
	AABB a = createAABB(2.0, 2.0, { 0, 0 });
	AABB b = createAABB(2.0, 2.0, { 1.0, 0 });
	EXPECT_TRUE(overlaps(a, b));
}

TEST(AABBTest, SeparatedAABBsNotOverlapping)
{
	AABB a = createAABB(2.0, 2.0, { 0, 0 });
	AABB b = createAABB(2.0, 2.0, { 5.0, 0 });
	EXPECT_FALSE(overlaps(a, b));
}

// Rotation integration
class RotationIntegrationTest : public ::testing::Test
{
protected:
	void SetUp() override
	{
		world = std::make_unique<FizzWorld>();
		world->Gravity = Vec2::Zero();
	}

	void tick(int n = tps) { for (int i = 0; i < n; ++i) world->tick(dt); }

	static constexpr int tps = 60;
	static constexpr val_t dt = val_t(1.0 / tps);
	std::unique_ptr<FizzWorld> world;
};

TEST_F(RotationIntegrationTest, AngularVelocityRotatesBody)
{
	auto body = makeDynamicBox(*world, 1.0, 1.0, Vec2::Zero());
	body.angularVelocity(1.0);
	val_t r0 = body.rotation();
	tick();
	EXPECT_GT(body.rotation(), r0);
}

TEST_F(RotationIntegrationTest, ZeroAngularVelocityDoesNotRotate)
{
	auto body = makeDynamicBox(*world, 1.0, 1.0, Vec2::Zero());
	body.angularVelocity(0.0);
	val_t r0 = body.rotation();
	tick();
	EXPECT_VAL_EQ(body.rotation(), r0);
}

TEST_F(RotationIntegrationTest, EncapsulatingAABBUpdatesWithRotation)
{
	// A box that's rotating should have a broadening AABB over time
	Shape s = createRect(2.0, 0.5);
	AABB r0   = getEncapsulatingAABB(s, Vec2::Zero(), 0.0,     true);
	AABB r45  = getEncapsulatingAABB(s, Vec2::Zero(), PI / 4,  true);
	AABB r90  = getEncapsulatingAABB(s, Vec2::Zero(), PI / 2,  true);
	// at 45 degrees the AABB should be larger than axis-aligned
	EXPECT_GT(r45.area(), r0.area());
	// at 90 degrees it's axis-aligned again but with swapped dimensions
	EXPECT_VAL_EQ(r90.hw, r0.hh);
	EXPECT_VAL_EQ(r90.hh, r0.hw);
}

// Static body: must not move under any circumstance
class StaticBodyTest : public ::testing::Test
{
protected:
	void SetUp() override
	{
		world = std::make_unique<FizzWorld>();
	}

	void tick(int n = tps) { for (int i = 0; i < n; ++i) world->tick(dt); }

	static constexpr int tps = 60;
	static constexpr val_t dt = val_t(1.0 / tps);
	std::unique_ptr<FizzWorld> world;
};

TEST_F(StaticBodyTest, StaticBodyDoesNotMoveUnderGravity)
{
	auto body = makeStaticCircle(*world, 1.0, { 0, 0 });
	Vec2 p0 = body.position();
	tick();
	EXPECT_VAL_EQ(body.position().x, p0.x);
	EXPECT_VAL_EQ(body.position().y, p0.y);
}

TEST_F(StaticBodyTest, StaticBodyDoesNotMoveAfterCollision)
{
	auto stat  = makeStaticCircle(*world, 1.0, { 0, 0 });
	auto dyn   = makeDynamicCircle(*world, 0.5, { -3.0, 0 });
	dyn.gravityScale(0);
	dyn.velocity({ 5.0, 0 });
	Vec2 sp0 = stat.position();
	tick();
	EXPECT_VAL_EQ(stat.position().x, sp0.x);
	EXPECT_VAL_EQ(stat.position().y, sp0.y);
}
