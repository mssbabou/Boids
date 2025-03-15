#include <iostream>

#include "Vec2.h"

class Boid
{
    public:
        int id;

        Vec2 position = Vec2();
        Vec2 velocity = Vec2();
        Vec2 desiredDirection = Vec2();

        Boid();

        bool operator==(const Boid& other) const;
};