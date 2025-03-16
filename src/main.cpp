#define SDL_MAIN_USE_CALLBACKS 1
#include <SDL3/SDL.h>
#include <SDL3_image/SDL_image.h>
#include <SDL3/SDL_main.h>
#include <cmath>
#include <random>
#include <chrono>

#include "Vec2.h"
#include "Boid.h"
#include "Collider.h"
#include "Physics2D.h"

const int windowWidth = 800;
const int windowHeight = 800;

const int tickRate = 60;
const int initialBoidCount = 200;

const float boidSize = 15;
const float boidViewRange = 60.0f;
const float boidViewFOV = 270.0f;

const float boidMaxSpeed = 4.0f;
const float boidAcceleration = 0.2f;

const float boidSeparationStrength = 12.0f;
const float boidAlignmentStrength = 0.2f;
const float boidCohesionStrength = 0.4f;
const float boidObstacleAvoidStrength = 6.0f;

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
    SDL_SetRenderDrawColorFloat(renderer, 1, 0.3, 0, SDL_ALPHA_OPAQUE_FLOAT);
    for (Collider &collider : Colliders)
    {
        if (collider.IsInvisible) continue;

        for (size_t i = 0; i < collider.Points.size()-1; i++)
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

void DrawRay(RayHit& hitInfo)
{
    SDL_SetRenderDrawColorFloat(renderer, 1, 0, 0, SDL_ALPHA_OPAQUE_FLOAT);

    if (hitInfo.hit)
    {
        SDL_RenderLine(renderer, hitInfo.ray.origin.x, hitInfo.ray.origin.y, hitInfo.point.x, hitInfo.point.y);
    }
}

void DrawRays(vector<RayHit>& hitInfos)
{
    for (RayHit &hit : hitInfos)
    {
        DrawRay(hit);
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
    Vec2 obstacleForce;
    int neighborCount = 0;

    // Process neighbors for separation, alignment, and cohesion forces
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

        // For separation we keep the distance effect
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

    // Process obstacle avoidance via raycasting
    std::vector<RayHit> hits;
    // Using a maxDistance (e.g., 200) and a ray count (e.g., 8) for your FOV rays
    Physics2D::RaycastMulti(Colliders, Physics2D::CreateFOVRays(boid.position, boid.velocity, 180, 200, 8), hits);
    //DrawRays(hits);

    int hitCount = 0;
    for (RayHit &hit : hits)
    {
        if (!hit.hit)
            continue;

        // Determine how close the obstacle is relative to the ray's max distance.
        float t = hit.distance / hit.ray.maxDistance;  // 0 when very close, 1 when at max distance
        // Use a quadratic falloff so that the avoidance force increases more sharply as you get closer.
        float falloff = (1.0f - t) * (1.0f - t);

        // Calculate an avoidance direction that steers away from the obstacle.
        Vec2 avoidanceDir = boid.position - hit.point;
        if (avoidanceDir.Magnitude() > 1e-6f)
            avoidanceDir.Normalize();

        // Add the weighted avoidance direction.
        obstacleForce = obstacleForce + (avoidanceDir * falloff);
        hitCount++;
    }

    if (hitCount > 0)
    {
        obstacleForce = obstacleForce / static_cast<float>(hitCount);
        obstacleForce = obstacleForce * boidObstacleAvoidStrength;
    }

    // If the computed obstacle force is nearly zero, pick a random avoidance direction.
    // This helps when all rays return too-similar (or weak) data, so the boid can choose a direction.
    if (obstacleForce.Magnitude() < 1e-3f)
    {
        float randomAngle = static_cast<float>(rand()) / static_cast<float>(RAND_MAX) * 2.0f * M_PI;
        obstacleForce = Vec2(std::cos(randomAngle), std::sin(randomAngle)) * boidObstacleAvoidStrength;
    }

    // Compute total acceleration from all steering forces.
    Vec2 acceleration = separationForce + alignmentForce + cohesionForce + obstacleForce;

    // Add constant forward acceleration if below max speed.
    const float boidForwardAccel = 0.5f;  // Adjust as needed.
    float currentSpeed = boid.velocity.Magnitude();
    if (currentSpeed > 1e-6f && currentSpeed < boidMaxSpeed)
    {
        acceleration = acceleration + boid.velocity.Normalized() * boidForwardAccel;
    }

    // Update velocity and clamp to boidMaxSpeed.
    boid.velocity = boid.velocity + acceleration * boidAcceleration;
    if (boid.velocity.Magnitude() > boidMaxSpeed)
        boid.velocity.SetLength(boidMaxSpeed);

    boid.position = boid.position + boid.velocity;

    // Wrap around screen boundaries.
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

    Collider worldBorder = Collider::Rectangle(0, 0, windowWidth-1, windowHeight-1);
    worldBorder.IsHollow = true;
    worldBorder.IsInvisible = true;
    
    Colliders.push_back(worldBorder);
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

    /*
    Vec2 mPos;
    SDL_GetMouseState(&mPos.x, &mPos.y);
    Vec2 origin = Vec2(windowWidth/2, windowHeight/2);
    Vec2 direction = Vec2(mPos.x - origin.x, mPos.y - origin.y);

    vector<Ray> rays = Physics2D::CreateFOVRays(origin, direction, 90, 9999, 8);
    vector<RayHit> hits;
    Physics2D::RaycastMulti(Colliders, rays, hits);
    DrawRays(hits);
    */

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
