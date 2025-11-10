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

    Shape s1 = createPolygon({Vector2p(0, 0), Vector2p{1, 0}, Vector2p(1, 1)});
    Shape s2 = createRect(2, 2);
    Shape s3 = createCircle(1);
    bool t0 = shapesOverlap(s1, Vector2p(0, 0), 0, s2, Vector2p(-2, 0), 0); // no rotation, no overlap, no circle
    bool t1 = shapesOverlap(s2, Vector2p(0, 0), 0, s3, Vector2p(2, 0), 0); // no rotation, no overlap, circle
    bool t2 = shapesOverlap(s2, Vector2p(0, 2.1), deg2rad(45), s2, Vector2p(0, 0), 0); // rotation, overlap, cirlce
    
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

        // world.tick(dt / 10);
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
