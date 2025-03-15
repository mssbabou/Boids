#include <iostream>
#include <list>
#include "Vec2.h"

class Collider
{
    public:
        std::vector<Vec2> Points;
        bool IsHollow = true;
        bool IsInvisible = false;
        bool Loop = true;

        Collider();
        Collider(std::vector<Vec2> points);

        static Collider Rectangle(float x, float y, float w, float h);
};