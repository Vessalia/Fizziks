#include <iostream>
#include <chrono>

#include "SDL3/SDL.h"

#include "Dense.h"
#include "FizzWorld.h"
#include "RigidBody.h"
#include "BodyDef.h"
#include "Shape.h"
#include "BitArray.h"

using namespace Fizziks;

FizzWorld world;

const unsigned int SCREEN_WIDTH = 1280;
const unsigned int SCREEN_HEIGHT = 720;

static SDL_Window* gWindow = nullptr;

Uint32 lastUpdateTime = SDL_GetTicks();

void draw();
void close();

int main(int argc, char** argv) 
{
    gWindow = SDL_CreateWindow("Fizziks Test", SCREEN_WIDTH, SCREEN_HEIGHT, SDL_WINDOW_RESIZABLE);
    
    bool quit = false;
    while (!quit)
    {
        SDL_Event e;
        while (SDL_PollEvent(&e))
        {
            if (e.type == SDL_EVENT_QUIT)
            {
                quit = true;
            }
        }

        float deltaTime = (SDL_GetTicks() - lastUpdateTime) / 1000.f;
        lastUpdateTime = SDL_GetTicks();

        draw();
    }

    close();
    
    // world = FizzWorld();
    // BodyDef def = {
    //     {1, 19},
    //     {0, 0},
    //     {0, 0},
    //     0,
    //     1,
    //     createAABB(1, 1),
    //     false
    // };
    // RigidBody body = world.createBody(def);
    // def.initPosition = {3.5, 18};
    // RigidBody b2 = world.createBody(def);

    // val_t totalTime = 0;
    // auto lastTick = std::chrono::system_clock::now();
    // val_t dt = 0;
    // while(true)
    // {
    //     auto now = std::chrono::system_clock::now();
    //     dt = std::chrono::duration_cast<std::chrono::duration<val_t>>(now - lastTick).count();
    //     totalTime += dt;
    //     world.tick(dt);
    //     lastTick = now;
    // }
}

void draw()
{

}

void close()
{
    SDL_DestroyWindow(gWindow);
}
