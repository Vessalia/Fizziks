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
    BitArray t(9);
    BitArray r(257);
    t.set(10);
    t.set(10);
    r.flip(10);
    r.flip(10);
    t.clear(10).trim();
    std::cout << "t = " << t << "\nr = " << r << "\nt | r = " << (t | r) << "\nt & r = " << (t & r) << "\nt ^ r = " << (t ^ r) << std::endl;
    
    // world = FizzWorld(10, 10);
    // BodyDef def = {
    //     {0, 0},
    //     {0, 0},
    //     {0, 0},
    //     1,
    //     createCircle(1),
    //     false
    // };
    // RigidBody body = world.createBody(def);
    // RigidBody b2 = world.createBody(def);

    // val_t totalTime = 0;
    // auto lastTick = std::chrono::system_clock::now();
    // val_t dt = 0;
    // while(true)
    // {
    //     auto now = std::chrono::system_clock::now();
    //     dt = std::chrono::duration_cast<std::chrono::duration<val_t>>(now - lastTick).count();
    //     totalTime += dt;
    //     body.applyForce(-world.Gravity);
    //     body.velocity(Vector2p(1, 0));
    //     world.tick(dt);
    //     lastTick = now;
    //     world.destroyBody(body);
    //     body = world.createBody(def);
    // }
}
