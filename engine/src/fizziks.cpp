#include <iostream>
#include <chrono>

#include "SDL3/SDL.h"

#include "Dense.h"
#include "FizzWorld.h"
#include "RigidBody.h"
#include "RigidDef.h"
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
void close();

int main(int argc, char** argv) 
{
    gWindow = SDL_CreateWindow("Fizziks Test", SCREEN_WIDTH, SCREEN_HEIGHT, 0);
    gRenderer = SDL_CreateRenderer(gWindow, NULL);

    BodyDef def = initBodyDef();
    def.colliderDefs.push_back({ createCollider(createCircle(1), 1, 0), Vector2p::Zero() });
    bodies.push_back(world.createBody(def));
    def.initPosition.y() += 3; def.initVelocity.y() -= 0.5;
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

        world.tick(dt / 10);
        draw();
    }

    close();
}

void draw()
{
    SDL_RenderPresent(gRenderer);
}

void close()
{
    SDL_DestroyRenderer(gRenderer);
    SDL_DestroyWindow(gWindow);
}
