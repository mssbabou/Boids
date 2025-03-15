#pragma once

#include <iostream>

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