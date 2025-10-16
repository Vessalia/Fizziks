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

const unsigned int SCREEN_WIDTH = 920;
const unsigned int SCREEN_HEIGHT = 920;

static SDL_Window* gWindow = nullptr;
static SDL_Renderer* gRenderer = nullptr;

static FizzWorld world = FizzWorld();

static std::vector<RigidBody> bodies;

Uint32 lt = SDL_GetTicks();

void draw();
Vector2p transformToScreenSpace(Vector2p pos);
void close();

int main(int argc, char** argv) 
{
    gWindow = SDL_CreateWindow("Fizziks Test", SCREEN_WIDTH, SCREEN_HEIGHT, 0);
    gRenderer = SDL_CreateRenderer(gWindow, NULL);

    BodyDef def = 
    {
        { 10, 10 },
        { 0, 0 },
        { 0, 0 },
        0,
        1,
        createAABB(1, 1),
        true
    };

    bodies.push_back(world.createBody(def));
    def.initPosition += Vector2p(0, 2);
    def.isStatic = false;
    bodies.push_back(world.createBody(def));
    
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

        float dt = (SDL_GetTicks() - lt) / 1000.f;
        lt = SDL_GetTicks();

        SDL_SetRenderDrawColor(gRenderer, 0, 0, 0, SDL_ALPHA_OPAQUE);
        SDL_RenderClear(gRenderer);

        world.tick(dt / 20);
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
    //     lastTick = now;
    // }
}

void draw()
{
    SDL_SetRenderDrawColor(gRenderer, 100, 60, 150, SDL_ALPHA_OPAQUE);
    for(int i = 0; i < bodies.size(); ++i)
    {
        auto body = bodies[i];
        Vector2p pos = transformToScreenSpace(body.position());
        if(body.shape().type == ShapeType::AABB)
        {
            Vector2p dim { body.shape().aabb.halfHeight, body.shape().aabb.halfWidth };
            Vector2p tdim = transformToScreenSpace(dim);
            SDL_FRect rect;
            rect.x = pos.x() - tdim.x();
            rect.y = SCREEN_HEIGHT - (pos.y() - tdim.y());
            rect.w = 2 * tdim.x();
            rect.h = 2 * tdim.y();
            SDL_RenderFillRect(gRenderer, &rect);
        }
        else if(body.shape().type == ShapeType::CIRCLE)
        {
            
        }
    }
    SDL_RenderPresent(gRenderer);
}

Vector2p transformToScreenSpace(Vector2p pos)
{
    Vector2p scale = world.worldScale();
    return { pos.x() * SCREEN_WIDTH / scale.x(), pos.y() * SCREEN_HEIGHT / scale.y() };
}

void close()
{
    SDL_DestroyRenderer(gRenderer);
    SDL_DestroyWindow(gWindow);
}
