#include "Physics2D.h"

#include <vector>
#include <cmath>

std::vector<Ray> Physics2D::CreateFOVRays(Vec2 origin, Vec2 direction, float FOV, float maxDistance, int rayCount)
{
    std::vector<Ray> rays;
    rays.reserve(rayCount);

    // Convert FOV from degrees to radians.
    float FOV_rad = FOV * (static_cast<float>(M_PI) / 180.0f);
    float halfFOV = FOV_rad / 2.0f;

    // If there's only one ray, just return the main direction.
    if (rayCount == 1)
    {
        Ray r;
        r.origin = origin;
        r.direction = direction.Normalized();
        r.maxDistance = 1000.0f; // Default max distance; adjust as needed.
        rays.push_back(r);
        return rays;
    }

    // Compute the angle increment between rays.
    float angleIncrement = FOV_rad / static_cast<float>(rayCount - 1);
    
    // Normalize the base direction.
    Vec2 baseDir = direction.Normalized();
    
    // Create each ray by rotating the base direction.
    for (int i = 0; i < rayCount; ++i)
    {
        // Compute the rotation offset.
        float angleOffset = -halfFOV + angleIncrement * i;
        
        // Rotate baseDir by angleOffset.
        float cosA = std::cos(angleOffset);
        float sinA = std::sin(angleOffset);
        Vec2 rotated;
        rotated.x = baseDir.x * cosA - baseDir.y * sinA;
        rotated.y = baseDir.x * sinA + baseDir.y * cosA;
        
        Ray ray;
        ray.origin = origin;
        ray.direction = rotated;
        ray.maxDistance = maxDistance;
        rays.push_back(ray);
    }
    
    return rays;
}

bool Physics2D::GetColliderIntersection(const Collider &collider, const Ray &ray, RayHit &hitInfo)
{
    bool hit = false;
    // Start with the maximum allowed distance.
    float closestT = ray.maxDistance;
    int count = static_cast<int>(collider.Points.size());
    
    // Need at least two points to form an edge.
    if (count < 2)
        return false;
    
    // If the collider loops, check all edges; otherwise, check count-1 segments.
    int segments = collider.Loop ? count : count - 1;
    
    // Iterate over each edge (line segment) in the collider.
    for (int i = 0; i < segments; i++)
    {
        // Endpoints of the segment:
        const Vec2 &p = collider.Points[i];
        const Vec2 &q = collider.Loop ? collider.Points[(i + 1) % count] : collider.Points[i + 1];
        
        // The segment vector:
        Vec2 s = q - p;
        // The ray's direction (assumed normalized):
        Vec2 rDir = ray.direction;
        
        // Compute cross product of ray direction and segment.
        float rxs = Vec2::Cross(rDir, s);
        // If rxs is near zero, the lines are parallel.
        if (std::fabs(rxs) < 1e-6f)
            continue;
        
        // Compute parameters for the ray and segment:
        Vec2 diff = p - ray.origin;
        float t = Vec2::Cross(diff, s) / rxs; // Parameter along the ray
        float u = Vec2::Cross(diff, rDir) / rxs; // Parameter along the segment
        
        // Valid intersection if:
        //   t is non-negative and less than the maxDistance, and
        //   u is between 0 and 1 (i.e. lies on the segment)
        if (t >= 0 && t <= ray.maxDistance && u >= 0 && u <= 1)
        {
            if (t < closestT)
            {
                closestT = t;
                hitInfo.hit = true;
                hitInfo.distance = t;
                hitInfo.point = ray.origin + rDir * t;
                hitInfo.collider = &collider;
                hitInfo.ray = ray;
                hit = true;
            }
        }
    }
    
    return hit;
}

bool Physics2D::Raycast(const std::vector<Collider> &colliders, Ray ray, RayHit &hitInfo)
{
    bool hitSomething = false;
    float closestDistance = ray.maxDistance;
    RayHit tempHit;
    
    // Initialize hitInfo as a miss.
    hitInfo.hit = false;
    
    // Loop through all colliders to find the closest hit.
    for (const Collider &collider : colliders)
    {
        if (GetColliderIntersection(collider, ray, tempHit))
        {
            if (tempHit.distance < closestDistance)
            {
                closestDistance = tempHit.distance;
                hitInfo = tempHit;
                hitSomething = true;
            }
        }
    }
    
    return hitSomething;
}

bool Physics2D::RaycastMulti(const std::vector<Collider> &colliders, const std::vector<Ray> &rays, std::vector<RayHit> &hitInfos)
{
    bool anyHit = false;
    // Resize the output vector so each ray has a corresponding RayHit.
    hitInfos.resize(rays.size());
    
    // Process each ray.
    for (size_t i = 0; i < rays.size(); ++i)
    {
        RayHit hit;
        hit.hit = false;
        if (Raycast(colliders, rays[i], hit))
        {
            hitInfos[i] = hit;
            anyHit = true;
        }
        else
        {
            // Optionally, assign the default miss result.
            hitInfos[i] = hit;
        }
    }
    return anyHit;
}
