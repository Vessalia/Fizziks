#include <Fizziks/MathUtils.h>
#include <Fizziks/Vec.h>

namespace Fizziks
{
int mod(int a, int b)
{
	b = abs(b);
	while(a < 0) a += b;
	return a % b;
}

Vec2 lefttriplecross(const Vec2& a, const Vec2& b, const Vec2& c)
{
	return (b * a.dot(c)) - (a * b.dot(c));
}

Vec2 righttriplecross(const Vec2& a, const Vec2& b, const Vec2& c)
{
	return (b * a.dot(c)) - (c * a.dot(b));
}
}
