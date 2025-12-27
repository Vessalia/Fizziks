#include "Dense.h"

namespace Fizziks
{
int mod(int a, int b)
{
    b = abs(b);
    while(a < 0) a += b;
    return a % b;
}

val_t crossproduct(const Vector2p& a, const Vector2p& b)
{
    return a.x() * b.y() - a.y() * b.x();
}

Vector2p crossproduct(val_t w, const Vector2p& r)
{
    return Vector2p(-w * r.y(), w * r.x());
}

Vector2p lefttriplecross(const Vector2p& a, const Vector2p& b, const Vector2p& c)
{
    return (b * a.dot(c)) - (a * b.dot(c));
}
Vector2p righttriplecross(const Vector2p& a, const Vector2p& b, const Vector2p& c)
{
    return (b * a.dot(c)) - (c * a.dot(b));
}
};
