#pragma once
#include <vector>
#include <BitArray.h>

namespace Fizziks
{
class UniformGrid
{
private:
    size_t entityCount;
    std::vector<BitArray> columns;
    std::vector<BitArray> rows;
};
};
