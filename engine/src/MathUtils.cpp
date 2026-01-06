#include "MathUtils.h"
#include "Vec.h"

namespace Fizziks
{
int mod(int a, int b)
{
    b = abs(b);
    while(a < 0) a += b;
    return a % b;
}

val_t crossproduct(const Vec2& a, const Vec2& b)
{
    return a.x * b.y - a.y * b.x;
}

Vec2 crossproduct(val_t w, const Vec2& r)
{
    return { -w * r.y, w * r.x };
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
