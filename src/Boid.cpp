#include "Boid.h"

Boid::Boid()
{

}

bool Boid::operator==(const Boid& other) const 
{
    return id == other.id;
}