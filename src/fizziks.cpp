#include <iostream>
#include <chrono>

#include "Dense.h"
#include "FizzWorld.h"
#include "RigidBody.h"
#include "BodyDef.h"
#include "Shape.h"
#include "BitArray.h"

using namespace Fizziks;

FizzWorld world;

void main() 
{
    world = FizzWorld();
    BodyDef def = {
        {0.5, 19.5},
        {0, 0},
        {0, 0},
        0,
        1,
        createAABB(1, 1),
        false
    };
    RigidBody body = world.createBody(def);

    val_t totalTime = 0;
    auto lastTick = std::chrono::system_clock::now();
    val_t dt = 0;
    while(true)
    {
        auto now = std::chrono::system_clock::now();
        dt = std::chrono::duration_cast<std::chrono::duration<val_t>>(now - lastTick).count();
        totalTime += dt;
        world.tick(dt);
        lastTick = now;
    }
}
