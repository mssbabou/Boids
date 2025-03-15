#include "Collider.h"

Collider::Collider()
{

}

Collider::Collider(std::vector<Vec2> points)
{
    Points = points;
}

Collider Collider::Rectangle(float x, float y, float w, float h)
{
    std::vector<Vec2> points; 
    points.push_back(Vec2(x    , y));
    points.push_back(Vec2(x + w, y));
    points.push_back(Vec2(x + w, y + h));
    points.push_back(Vec2(x    , y + h));
    
    Collider collider = Collider(points);
    collider.Loop = true;
    return collider;
}