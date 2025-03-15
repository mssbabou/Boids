#pragma once

#include <iostream>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

class Vec2
{
    public:
        float x;
        float y;

        Vec2(float x = 0, float y = 0);
        float Magnitude();
        float SqrMagnitude();
        void Normalize();
        Vec2 Normalized();
        void SetLength(float l);
        
        static float Dot(Vec2 a, Vec2 b);
        static float Distance(Vec2 a, Vec2 b);
        static float SqrDistance(Vec2 a, Vec2 b);
        static float AngleBetween(Vec2 a, Vec2 b);
        
        Vec2 operator+(const Vec2& other) const;
        Vec2 operator-(const Vec2& other) const;
        Vec2 operator+=(const Vec2& other) const;
        Vec2 operator*(const Vec2& other) const;
        Vec2 operator*=(const Vec2& other) const;
        Vec2 operator/(const Vec2& other) const;
        Vec2 operator/=(const Vec2& other) const;
        
        Vec2 operator+(const float& other) const;
        Vec2 operator*(const float& other) const;
        Vec2 operator/(const float& other) const;
};