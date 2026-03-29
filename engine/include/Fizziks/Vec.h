#pragma once
#include <Fizziks/Fizziks.h>

#include <cmath>

namespace Fizziks
{    
struct FIZZIKS_API Vec2
{
	val_t x = 0;
	val_t y = 0;

	static constexpr Vec2 Zero()
	{
		return Vec2{ 0, 0 };
	}

	static Vec2 FromAngle(val_t angle)
	{
		return Vec2{ std::cos(angle), std::sin(angle) };
	}

	Vec2 lerped(const Vec2& other, val_t t) const;
	Vec2& lerp(const Vec2& other, val_t t);

	val_t dot(const Vec2& other) const;
	val_t cross(const Vec2& other) const;
	Vec2 cross(val_t z) const;

	val_t norm() const;
	val_t squaredNorm() const;

	Vec2 normalized() const;
	Vec2 rotated(val_t angle) const;
	Vec2& normalize();
	Vec2& rotate(val_t angle);

	val_t angle() const;

	bool operator==(const Vec2&) const = default;
	bool operator!=(const Vec2&) const = default;

	Vec2 operator+(const Vec2& other) const;
	Vec2 operator-(const Vec2& other) const;
	Vec2 operator*(val_t scalar) const;
	Vec2 operator/(val_t scalar) const;

	Vec2& operator+=(const Vec2& other);
	Vec2& operator-=(const Vec2& other);
	Vec2& operator*=(val_t scalar);
	Vec2& operator/=(val_t scalar);
};

FIZZIKS_API Vec2 operator-(const Vec2& v);
FIZZIKS_API Vec2 operator*(val_t scalar, const Vec2& v);

struct FIZZIKS_API Mat2
{
	// column major
	val_t m00 = 0, m10 = 0;
	val_t m01 = 0, m11 = 0;

	static constexpr Mat2 Identity()
	{
		Mat2 mat;
		mat.m00 = 1; mat.m01 = 0;
		mat.m10 = 0; mat.m11 = 1;
		return mat;
	}

	static Mat2 Rotation(val_t angle)
	{
		Mat2 mat;
		val_t c = std::cos(angle);
		val_t s = std::sin(angle);
		mat.m00 = c;  mat.m01 = -s;
		mat.m10 = s;  mat.m11 =  c;
		return mat;
	}

	static constexpr Mat2 Scale(val_t sx, val_t sy)
	{
		Mat2 mat;
		mat.m00 = sx; mat.m01 = 0;
		mat.m10 = 0;  mat.m11 = sy;
		return mat;
	}

	Vec2 col(int i) const;
	Vec2 row(int i) const;

	bool operator==(const Mat2&) const = default;
	bool operator!=(const Mat2&) const = default;

	Mat2 operator+(const Mat2& other) const;
	Mat2 operator-(const Mat2& other) const;
	Vec2 operator*(const Vec2& vec) const;
	Mat2 operator*(val_t scalar) const;
	Mat2 operator*(const Mat2& mat) const;
	Mat2 operator/(val_t scalar) const;

	Mat2& operator+=(const Mat2& other);
	Mat2& operator-=(const Mat2& other);
	Mat2& operator*=(val_t scalar);
	Mat2& operator*=(const Mat2& mat);
	Mat2& operator/=(val_t scalar);
	
	Mat2 transposed() const;
	Mat2& transpose();

	val_t determinant() const;
	Mat2 inverse() const;
	Mat2& invert();
};

FIZZIKS_API Mat2 operator-(const Mat2& m);
FIZZIKS_API Mat2 operator*(val_t scalar, const Mat2& m);
}
