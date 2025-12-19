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

Vector2p transformToScreenSpace(Vector2p pos);

int main(int argc, char** argv) 
{
    gWindow = SDL_CreateWindow("Fizziks Test", SCREEN_WIDTH, SCREEN_HEIGHT, 0);
    gRenderer = SDL_CreateRenderer(gWindow, NULL);

    BodyDef def = initBodyDef();
    def.initPosition = Vector2p(10, 10);
    def.colliderDefs.push_back({ createCollider(createCircle(1), 1, 0), Vector2p::Zero()});
    bodies.push_back(world.createBody(def));
    def.initPosition.y() += 3; def.initVelocity.y() -= 2;
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
        if (body.bodyType() == BodyType::STATIC)
        {
            SDL_SetRenderDrawColor(gRenderer, 160, 150, 30, SDL_ALPHA_OPAQUE);
        }
        else if (body.bodyType() == BodyType::DYNAMIC)
        {
            SDL_SetRenderDrawColor(gRenderer, 100, 60, 150, SDL_ALPHA_OPAQUE);
        }
        else
        {
            SDL_SetRenderDrawColor(gRenderer, 60, 150, 100, SDL_ALPHA_OPAQUE);
        }

        const auto colliders = body.colliders();
        Vector2p bodyPos = body.centroidPosition();
        for (auto [collider, colliderPos] : colliders)
        {
            Vector2p localPos = bodyPos + colliderPos;
            Vector2p pos = transformToScreenSpace(localPos);
            Shape shape = collider.shape;
            if (shape.type == ShapeType::CIRCLE)
            {
                val_t r = std::get<Circle>(shape.data).radius;
                int sr = (int)transformToScreenSpace(Vector2p{r, 0}).x();

                int x = (int)pos.x();
                int y = SCREEN_HEIGHT - (int)pos.y();

                for (int i = -sr; i <= sr; ++i)
                {
                    int dx = (int)std::sqrt(sr * sr - i * i);
                    SDL_RenderLine(gRenderer, x - dx, y + i, x + dx, y + i);
                }
            }
            else if (shape.type == ShapeType::POLYGON)
            {
                const auto& verts = std::get<Polygon>(shape.data).vertices;

                std::vector<SDL_Vertex> sdlVerts;
                sdlVerts.reserve(verts.size());

                for (const auto& v : verts)
                {
                    Vector2p screenV = transformToScreenSpace(localPos + v);

                    SDL_Vertex sdlVert;
                    sdlVert.position = SDL_FPoint{ screenV.x(), SCREEN_HEIGHT - screenV.y() };
                    sdlVert.color = SDL_FColor{ 255, 255, 255, 255 };
                    sdlVert.tex_coord = SDL_FPoint{ 0, 0 };
                    sdlVerts.push_back(sdlVert);
                }

                // Build triangle fan indices
                std::vector<int> indices;
                indices.reserve((verts.size() - 2) * 3);

                for (size_t i = 1; i + 1 < sdlVerts.size(); ++i)
                {
                    indices.push_back(0);
                    indices.push_back(static_cast<int>(i));
                    indices.push_back(static_cast<int>(i + 1));
                }

                // Single draw call
                SDL_RenderGeometry(
                    gRenderer,
                    nullptr,
                    sdlVerts.data(), static_cast<int>(sdlVerts.size()),
                    indices.data(), static_cast<int>(indices.size())
                );
            }
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
