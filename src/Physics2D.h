#pragma once

#include <iostream>

#include "Vec2.h"
#include "Collider.h"

struct Ray
{
    Vec2 origin;
    Vec2 direction;
    float maxDistance;
} typedef;

struct RayHit
{
    bool hit;
    Ray ray;
    Vec2 point;
    float distance;
    const Collider* collider;

} typedef;

class Physics2D
{
    public:
        static std::vector<Ray> CreateFOVRays(Vec2 origin, Vec2 direction, float FOV, float maxDistance, int rayCount);
        static bool GetColliderIntersection(const Collider& collider, const Ray& ray, RayHit& hitInfo);
        static bool Raycast(const std::vector<Collider>& colliders, Ray ray, RayHit& hitInfo);
        static bool RaycastMulti(const std::vector<Collider>& colliders, const std::vector<Ray>& rays, std::vector<RayHit>& hitInfos);
};