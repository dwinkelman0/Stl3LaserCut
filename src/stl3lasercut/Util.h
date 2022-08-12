// Copyright 2022 by Daniel Winkelman. All rights reserved.

#pragma once

#include <iostream>
#include <tuple>
#include <vector>

namespace stl3lasercut {
using Vec2 = std::tuple<float, float>;
using Vec3 = std::tuple<float, float, float>;

namespace unit3 {
extern const Vec3 x, y, z;
}

Vec3 operator-(const Vec3 vec);
Vec2 operator+(const Vec2 vec1, const Vec2 vec2);
Vec2 operator-(const Vec2 vec1, const Vec2 vec2);
Vec3 operator-(const Vec3 vec1, const Vec3 vec2);
Vec2 operator/(const Vec2 vec, const float x);
Vec3 operator/(const Vec3 vec, const float x);
float dot(const Vec2 vec1, const Vec2 vec2);
float dot(const Vec3 vec1, const Vec3 vec2);
float cross(const Vec2 vec1, const Vec2 vec2);
Vec3 cross(const Vec3 vec1, const Vec3 vec2);
float abs(const Vec2 vec);
float abs(const Vec3 vec);
float angle(const Vec2 vec1, const Vec2 vec2);
float angle(const Vec3 vec1, const Vec3 vec2);
Vec2 rotate2D(const Vec2 vec, const float angle);
Vec3 rotateX(const Vec3 vec, const float angle);
Vec3 rotateY(const Vec3 vec, const float angle);
Vec3 rotateZ(const Vec3 vec, const float angle);

std::ostream &operator<<(std::ostream &os, const Vec2 &vec);
std::ostream &operator<<(std::ostream &os, const Vec3 &vec);

float getPolygonArea(const std::vector<Vec2> &points);
}  // namespace stl3lasercut
