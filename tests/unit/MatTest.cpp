#include <gtest/gtest.h>
#include <Fizziks/Vec.h>
#include <Fizziks/MathUtils.h>
#include <TestTypes.h>

using namespace Fizziks;

// Standalone tests
TEST(Mat2Construction, DefaultInitializesToZero)
{
	Mat2 m;
	EXPECT_VAL_EQ(m.m00, 0.0);
	EXPECT_VAL_EQ(m.m10, 0.0);
	EXPECT_VAL_EQ(m.m01, 0.0);
	EXPECT_VAL_EQ(m.m11, 0.0);
}

TEST(Mat2Construction, ValueConstructorSetsComponents)
{
	Mat2 m { 1.0, 2.0, 3.0, 4.0 };
	EXPECT_VAL_EQ(m.m00, 1.0);
	EXPECT_VAL_EQ(m.m10, 2.0);
	EXPECT_VAL_EQ(m.m01, 3.0);
	EXPECT_VAL_EQ(m.m11, 4.0);
}

// Fixture
class Mat2ArithmeticTest : public ::testing::Test
{
protected:
	void SetUp() override
	{
		a = { 1.0, 2.0, 3.0, 4.0 };
		b = { 5.0, 6.0, 7.0, 8.0 };
	}

	Mat2 a, b;
};

TEST_F(Mat2ArithmeticTest, AdditionIsElementWise)
{
	Mat2 result = a + b;
	EXPECT_VAL_EQ(result.m00, 6.0);
	EXPECT_VAL_EQ(result.m10, 8.0);
	EXPECT_VAL_EQ(result.m01, 10.0);
	EXPECT_VAL_EQ(result.m11, 12.0);
}

TEST_F(Mat2ArithmeticTest, AdditionIsCommutative)
{
	Mat2 ab = a + b;
	Mat2 ba = b + a;
	EXPECT_VAL_EQ(ab.m00, ba.m00);
	EXPECT_VAL_EQ(ab.m10, ba.m10);
	EXPECT_VAL_EQ(ab.m01, ba.m01);
	EXPECT_VAL_EQ(ab.m11, ba.m11);
}
