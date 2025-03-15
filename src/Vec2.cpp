#include "Vec2.h"

Vec2::Vec2(float x, float y)
{
    this->x = x;
    this->y = y;
}

float Vec2::Magnitude()
{
    return sqrt(x * x + y * y);
}

float Vec2::SqrMagnitude()
{
    return x * x + y * y;
}

void Vec2::Normalize()
{
    float length = sqrt(x * x + y * y);

    if (length <= 0.0f) return;

    x /= length;
    y /= length;
}

Vec2 Vec2::Normalized()
{
    float length = sqrt(x * x + y * y);

    if (length <= 0.0f) return Vec2();

    return Vec2(x / length, y / length);
}

float Vec2::Dot(Vec2 a, Vec2 b)
{
    return a.x * b.x + a.y * b.y;
}

float Vec2::Cross(Vec2 a, Vec2 b)
{
    return a.x * b.y - a.y * b.x;
}

float Vec2::Distance(Vec2 a, Vec2 b)
{
    Vec2 v = Vec2(b.x - a.x, b.y - a.y);

    return v.Magnitude();
}

float Vec2::SqrDistance(Vec2 a, Vec2 b)
{
    Vec2 v = Vec2(b.x - a.x, b.y - a.y);

    return v.SqrMagnitude();
}

float Vec2::AngleBetween(Vec2 a, Vec2 b)
{
    float dotProduct = Dot(a, b);
    float magProduct = a.Magnitude() * b.Magnitude();

    if (magProduct == 0.0f) return 0.0f;

    float cosTheta = dotProduct / magProduct;

    cosTheta = std::fmax(-1.0f, std::fmin(1.0f, cosTheta));

    return std::acos(cosTheta);
}

Vec2 Vec2::operator+(const Vec2& other) const {
    return Vec2(x + other.x, y + other.y);
}

Vec2 Vec2::operator-(const Vec2& other) const {
    return Vec2(x - other.x, y - other.y);
}

Vec2 Vec2::operator+=(const Vec2& other) const {
    return Vec2(x + other.x, y + other.y);
}

Vec2 Vec2::operator*(const Vec2& other) const {
    return Vec2(x + other.x, y + other.y);
}

Vec2 Vec2::operator*=(const Vec2& other) const {
    return Vec2(x + other.x, y + other.y);
}

Vec2 Vec2::operator/(const Vec2& other) const {
    return Vec2(x / other.x, y / other.y);
}

Vec2 Vec2::operator/=(const Vec2& other) const {
    return Vec2(x / other.x, y / other.y);
}

Vec2 Vec2::operator+(const float& other) const {
    return Vec2(x + other, y + other);
}

Vec2 Vec2::operator*(const float& other) const {
    return Vec2(x * other, y * other);
}

Vec2 Vec2::operator/(const float& other) const {
    return Vec2(x / other, y / other);
}

void Vec2::SetLength(float length)
{
    Normalize();
    x *= length;
    y *= length;
}
