#pragma once
#include <Fizziks.h>
#include <RigidDef.h>

namespace Fizziks
{
struct FIZZIKS_API Ray
{
    Vector2p pos;
    Vector2p dir;
};

struct FIZZIKS_API RaycastResult
{
    bool hit;
    uint32_t ID;
    Vector2p point;
    Vector2p normal;
};
};
