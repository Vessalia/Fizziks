#include "Dense.h"

namespace Fizziks
{
int mod(int a, int b)
{
    b = abs(b);
    while(a < 0) a += b;
    return a % b;
}
};
