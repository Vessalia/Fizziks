#pragma once
#include <Dense.h>
#include <RigidDef.h>

namespace Fizziks
{
struct Ray
{
    Vector2p pos;
    Vector2p dir;
};

struct RaycastResult
{
    bool hit;
    uint32_t ID;
    Vector2p point;
    Vector2p normal;
};
};
