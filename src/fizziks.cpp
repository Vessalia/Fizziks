#include <iostream>
#include <chrono>

#include "Dense.h"
#include "FizzWorld.h"
#include "RigidBody.h"
#include "Shape.h"

FizzWorld world;

void main() 
{
    world = FizzWorld(10, 10);
    Shape shape;
    shape.circle.radius = 1;
    shape.type = ShapeType::CIRCLE;
    Vector2p zero = {0, 0};
    BodyDef def = {
        zero,
        zero,
        zero,
        1,
        shape,
        false
    };
    RigidBody body = world.createBody(def);

    auto lastTick = std::chrono::system_clock::now();
    val_t dt = 0;
    while(true)
    {
        auto now = std::chrono::system_clock::now();
        dt = (now - lastTick).count();
        world.tick(dt);
        lastTick = now;
    }
}
