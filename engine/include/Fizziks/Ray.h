#pragma once
#include <Fizziks.h>
#include <Vec.h>

namespace Fizziks
{
struct FIZZIKS_API Ray
{
    Vec2 pos;
    Vec2 dir;
};

struct FIZZIKS_API RaycastResult
{
    bool hit;
    uint32_t ID;
    Vec2 point;
    Vec2 normal;
};
}
