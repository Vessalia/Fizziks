#pragma once

namespace Fizziks
{
struct Handle 
{
    size_t index;
    size_t gen;
    bool     isValid;

    bool operator==(const Handle& other) const { return index == other.index && gen == other.gen && isValid == other.isValid; }
};
};
