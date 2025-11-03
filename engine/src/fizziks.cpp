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

    BodyDef def;
    def.initPosition = { 5, 10 };
    def.initVelocity = { 0, 0 };
    def.initAngularVelocity = { 0, 0 };
    def.initRotation = 0;
    def.mass = 1;
    def.gravityScale = 1;
    def.shape = createAABB(10, 1);
    def.isStatic = true;

    bodies.push_back(world.createBody(def));
    def.initPosition += Vector2p(-3, 2);
    def.isStatic = false;
    def.shape = createAABB(1, 1);
    bodies.push_back(world.createBody(def));
    def.initVelocity = Vector2p(-2, 0);
    def.initPosition += Vector2p(6, 0);
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
    for(int i = 0; i < bodies.size(); ++i)
    {
        auto body = bodies[i];
        if (body.isStatic())
        {
            SDL_SetRenderDrawColor(gRenderer, 160, 150, 30, SDL_ALPHA_OPAQUE);
        }
        else
        {
            SDL_SetRenderDrawColor(gRenderer, 100, 60, 150, SDL_ALPHA_OPAQUE);
        }

        Vector2p pos = transformToScreenSpace(body.position());
        if(body.shape().type == ShapeType::AABB)
        {
            Vector2p dim { body.shape().aabb.halfWidth, body.shape().aabb.halfHeight };
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
