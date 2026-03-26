#include <gtest/gtest.h>
#include <Fizziks/Vec.h>
#include <Fizziks/MathUtils.h>
#include <TestTypes.h>

using namespace Fizziks;

// Standalone tests
TEST(Vec2Construction, DefaultInitializesToZero)
{
	Vec2 v;
	EXPECT_VAL_EQ(v.x, 0.0);
	EXPECT_VAL_EQ(v.y, 0.0);
}

TEST(Vec2Construction, ValueConstructorSetsComponents)
{
	Vec2 v { 1.0, 2.0 };
	EXPECT_VAL_EQ(v.x, 1.0);
	EXPECT_VAL_EQ(v.y, 2.0);
}

// Fixture
class Vec2ArithmeticTest : public ::testing::Test
{
protected:
	void SetUp() override
	{
		a = { 1.0, 2.0 };
		b = { 3.0, 4.0 };
	}

	Vec2 a, b;
};

TEST_F(Vec2ArithmeticTest, AdditionIsComponentWise)
{
	Vec2 result = a + b;
	EXPECT_VAL_EQ(result.x, 4.0);
	EXPECT_VAL_EQ(result.y, 6.0);
}

TEST_F(Vec2ArithmeticTest, AdditionIsCommutative)
{
	Vec2 ab = a + b;
	Vec2 ba = b + a;
	EXPECT_VAL_EQ(ab.x, ba.x);
	EXPECT_VAL_EQ(ab.y, ba.y);
}

TEST_F(Vec2ArithmeticTest, SubtractionIsComponentWise)
{
	Vec2 result = b - a;
	EXPECT_VAL_EQ(result.x, 2.0);
	EXPECT_VAL_EQ(result.y, 2.0);
}

TEST_F(Vec2ArithmeticTest, ScalarMultiplication)
{
	Vec2 result = a * 2.0;
	EXPECT_VAL_EQ(result.x, 2.0);
	EXPECT_VAL_EQ(result.y, 4.0);
}

TEST_F(Vec2ArithmeticTest, ScalarDivision)
{
	Vec2 result = a / 2.0;
	EXPECT_VAL_EQ(result.x, 0.5);
	EXPECT_VAL_EQ(result.y, 1.0);
}

// Dot and cross product
TEST_F(Vec2ArithmeticTest, DotProductIsCommutative)
{
	EXPECT_VAL_EQ(a.dot(b), b.dot(a));
}

TEST_F(Vec2ArithmeticTest, DotProductKnownValue)
{
	// 1*3 + 2*4 = 11
	EXPECT_VAL_EQ(a.dot(b), 11.0);
}

TEST_F(Vec2ArithmeticTest, CrossProductIsAntiCommutative)
{
	val_t ab = a.cross(b);
	val_t ba = b.cross(a);
	EXPECT_VAL_EQ(ab, -ba);
}

TEST_F(Vec2ArithmeticTest, CrossProductOrthogonalToBothInputs)
{
	Vec2 c = a.cross(1.0);
	Vec2 d = b.cross(-2.0);
	EXPECT_VAL_EQ(a.dot(c), 0.0f);
	EXPECT_VAL_EQ(b.dot(d), 0.0f);
}

// Magnitude and normalization
class Vec2NormalizationTest : public ::testing::Test
{
protected:
	void SetUp() override
	{
		unit_x = { 1.0, 0.0 };
		diagonal = { 1.0, 1.0 };
	}

	Vec2 unit_x, diagonal;
};

TEST_F(Vec2NormalizationTest, LengthOfUnitXIsOne)
{
	EXPECT_VAL_EQ(unit_x.norm(), 1.0f);
}

TEST_F(Vec2NormalizationTest, LengthMatchesPythagorean)
{
	Vec2 v {3.0, 4.0};
	EXPECT_VAL_EQ(v.norm(), 5.0);
}

TEST_F(Vec2NormalizationTest, NormalizedVectorHasUnitLength)
{
	Vec2 n = diagonal.normalized();
	EXPECT_VAL_EQ(n.norm(), 1.0);
}

TEST_F(Vec2NormalizationTest, NormalizeResultsInUnitLengthVector)
{
	diagonal.normalize();
	EXPECT_VAL_EQ(diagonal.norm(), 1.0);
}

TEST_F(Vec2NormalizationTest, NormalizedVectorPreservesDirection)
{
	Vec2 n = diagonal.normalized();
	EXPECT_VAL_EQ(n.x, n.y);
}
