#pragma once

namespace Fizziks::internal
{
struct Handle 
{
    uint32_t index;
    uint32_t gen;

    bool operator==(const Handle&) const = default;
};
}
