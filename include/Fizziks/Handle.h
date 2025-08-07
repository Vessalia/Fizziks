#pragma once

struct Handle 
{
    uint32_t index;
    uint32_t gen;

    bool operator==(const Handle& other) const { return index == other.index && gen == other.gen; }
};
