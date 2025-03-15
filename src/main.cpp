#define SDL_MAIN_USE_CALLBACKS 1
#include <SDL3/SDL.h>
#include <SDL3_image/SDL_image.h>
#include <SDL3/SDL_main.h>
#include <cmath>
#include <random>
#include <chrono>

#include "Boid.h"
#include "Collider.h"

const int windowWidth = 800;
const int windowHeight = 800;

const int tickRate = 60;
const int initialBoidCount = 300;

const float boidSize = 15;
const float boidViewRange = 60.0f;
const float boidViewFOV = 200.0f;

const float boidMaxSpeed = 4.0f;
const float boidAcceleration = 0.2f;

const float boidSeparationStrength = 6.0f;
const float boidAlignmentStrength = 0.1f;
const float boidCohesionStrength = 0.2f;

static SDL_Window *window = NULL;
static SDL_Renderer *renderer = NULL;
static SDL_Texture *boidTexture = NULL;

using namespace std;

vector<Boid> Boids;
vector<Collider> Colliders;

void DrawBoids()
{
    for (Boid &boid : Boids)
    {
        SDL_FRect rect = { boid.position.x - boidSize / 2, boid.position.y - boidSize / 2, boidSize, boidSize };
        float angle = SDL_atan2f(boid.velocity.x, -boid.velocity.y) * (180.0f / M_PI);
        SDL_RenderTextureRotated(renderer, boidTexture, nullptr, &rect, angle, nullptr, SDL_FLIP_NONE);
    }
}

void DrawColliders()
{
    SDL_SetRenderDrawColorFloat(renderer, 1, 0, 0, SDL_ALPHA_OPAQUE_FLOAT);
    for (Collider &collider : Colliders)
    {
        for (int i = 0; i < collider.Points.size()-1; i++)
        {
            Vec2 p0 = collider.Points[i];
            Vec2 p1 = collider.Points[i+1];
            SDL_RenderLine(renderer, p0.x, p0.y, p1.x, p1.y);
        }

        if (collider.Loop)
        {
            Vec2 p0 = collider.Points.front();
            Vec2 p1 = collider.Points.back();
            SDL_RenderLine(renderer, p0.x, p0.y, p1.x, p1.y);
        }
    }
}

float randomFloat(float min, float max)
{
    static std::random_device rd;
    static std::mt19937 engine(rd());
    std::uniform_real_distribution<float> dist(min, max);
    return dist(engine);
}

void CreateRandomBoids(size_t count)
{
    for (size_t i = 0; i < count; i++)
    {
        Boid boid;
        boid.id = i;
        boid.position = Vec2(randomFloat(0, windowWidth), randomFloat(0, windowHeight));
        // Start with an initial velocity (you can also use a random unit vector)
        boid.velocity = Vec2(randomFloat(-1.f, 1.f), randomFloat(-1.f, 1.f));
        boid.velocity.Normalize();
        // Optionally initialize desiredDirection if still used elsewhere
        boid.desiredDirection = boid.velocity;
        Boids.push_back(boid);
    }
}

void UpdateBoid(Boid &boid)
{
    Vec2 separationForce;
    Vec2 alignmentForce;
    Vec2 cohesionForce;
    int neighborCount = 0;

    for (Boid &other : Boids)
    {
        if (boid == other)
            continue;

        Vec2 diff = boid.position - other.position;
        float distance = diff.Magnitude();

        if (distance > 0 && distance <= boidViewRange)
        {
            float angle = Vec2::AngleBetween(boid.velocity.Normalized(), diff.Normalized());
            if (angle <= boidViewFOV / 2.0f)
            {
                separationForce = separationForce + (diff.Normalized() / distance);
                alignmentForce = alignmentForce + other.velocity;
                cohesionForce = cohesionForce + other.position;
                neighborCount++;
            }
        }
    }

    if (neighborCount > 0)
    {
        separationForce = separationForce / static_cast<float>(neighborCount);
        alignmentForce = alignmentForce / static_cast<float>(neighborCount);
        cohesionForce = (cohesionForce / static_cast<float>(neighborCount)) - boid.position;

        separationForce = separationForce * boidSeparationStrength;

        if (alignmentForce.Magnitude() > 0)
        {
            alignmentForce.Normalize();
            alignmentForce = alignmentForce * boidAlignmentStrength;
        }
        if (cohesionForce.Magnitude() > 0)
        {
            cohesionForce.Normalize();
            cohesionForce = cohesionForce * boidCohesionStrength;
        }
    }

    Vec2 acceleration = separationForce + alignmentForce + cohesionForce;
    boid.velocity = boid.velocity + acceleration * boidAcceleration;

    if (boid.velocity.Magnitude() > boidMaxSpeed)
        boid.velocity.SetLength(boidMaxSpeed);

    boid.position = boid.position + boid.velocity;

    // Wrap around screen boundaries
    if (boid.position.x < 0) boid.position.x = windowWidth;
    else if (boid.position.x > windowWidth) boid.position.x = 0;

    if (boid.position.y < 0) boid.position.y = windowHeight;
    else if (boid.position.y > windowHeight) boid.position.y = 0;
}

void UpdateBoids()
{
    for (Boid &boid : Boids)
    {
        UpdateBoid(boid);
    }
}

SDL_AppResult SDL_AppInit(void **appstate, int argc, char *argv[])
{
    SDL_SetAppMetadata("Boids", "1.0", "boids");

    if (!SDL_Init(SDL_INIT_VIDEO))
    {
        SDL_Log("Couldn't initialize SDL: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }

    if (!SDL_CreateWindowAndRenderer("Boids", windowWidth, windowHeight, SDL_WINDOW_HIGH_PIXEL_DENSITY, &window, &renderer))
    {
        SDL_Log("Couldn't create window/renderer: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }
    SDL_SetRenderLogicalPresentation(renderer, windowWidth, windowHeight, SDL_LOGICAL_PRESENTATION_LETTERBOX);

    boidTexture = IMG_LoadTexture(renderer, "assets/cursor-pointing-up.svg");
    if (!boidTexture)
    {
        SDL_Log("Could not load SVG: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }

    SDL_GL_SetAttribute(SDL_GL_MULTISAMPLESAMPLES, 4);

    CreateRandomBoids(initialBoidCount);
    Colliders.push_back(Collider::Rectangle(300, 300, 50, 50));

    return SDL_APP_CONTINUE;
}

SDL_AppResult SDL_AppEvent(void *appstate, SDL_Event *event)
{
    if (event->type == SDL_EVENT_QUIT)
    {
        return SDL_APP_SUCCESS;
    }
    return SDL_APP_CONTINUE;
}

void FixedUpdate()
{
    SDL_SetRenderDrawColorFloat(renderer, 1, 1, 1, SDL_ALPHA_OPAQUE_FLOAT);
    SDL_RenderClear(renderer);

    UpdateBoids();
    DrawBoids();
    DrawColliders();

    SDL_RenderPresent(renderer);
}

SDL_AppResult SDL_AppIterate(void *appstate)
{
    auto t0 = chrono::high_resolution_clock::now();
    FixedUpdate();
    auto t1 = chrono::high_resolution_clock::now();
    chrono::duration<double> elapsedTime = t1 - t0;
    int remainingTime = (1000 / tickRate) - static_cast<int>(elapsedTime.count());
    SDL_Delay(remainingTime);
    return SDL_APP_CONTINUE;
}

void SDL_AppQuit(void *appstate, SDL_AppResult result)
{

}
