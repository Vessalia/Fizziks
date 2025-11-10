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
};
