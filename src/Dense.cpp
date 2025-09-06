#include "Dense.h"

namespace Fizziks
{
int mod(int a, int b)
{
    b = abs(b);
    while(a < 0) a += b;
    return a % b;
}

val_t getDist2(Vector2p v1, Vector2p v2)
{
    return (v1 - v2).squaredNorm();
}

val_t getDist2(Vector3p v1, Vector3p v2)
{
    return (v1 - v2).squaredNorm();
}
};
