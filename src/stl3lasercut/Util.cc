// Copyright 2022 by Daniel Winkelman. All rights reserved.

#include "Util.h"

#include <cmath>

namespace stl3lasercut {
namespace unit3 {
const Vec3 x(1, 0, 0);
const Vec3 y(0, 1, 0);
const Vec3 z(0, 0, 1);
}  // namespace unit3

Vec3 operator-(const Vec3 vec) {
  return {-std::get<0>(vec), -std::get<1>(vec), -std::get<2>(vec)};
}

Vec2 operator+(const Vec2 vec1, const Vec2 vec2) {
  return {std::get<0>(vec1) + std::get<0>(vec2),
          std::get<1>(vec1) + std::get<1>(vec2)};
}

Vec2 operator-(const Vec2 vec1, const Vec2 vec2) {
  return {std::get<0>(vec1) - std::get<0>(vec2),
          std::get<1>(vec1) - std::get<1>(vec2)};
}

Vec3 operator-(const Vec3 vec1, const Vec3 vec2) {
  return {std::get<0>(vec1) - std::get<0>(vec2),
          std::get<1>(vec1) - std::get<1>(vec2),
          std::get<2>(vec1) - std::get<2>(vec2)};
}

Vec2 operator/(const Vec2 vec, const float x) {
  return {std::get<0>(vec) / x, std::get<1>(vec) / x};
}

Vec3 operator/(const Vec3 vec, const float x) {
  return {std::get<0>(vec) / x, std::get<1>(vec) / x, std::get<2>(vec) / x};
}

float dot(const Vec2 vec1, const Vec2 vec2) {
  return std::get<0>(vec1) * std::get<0>(vec2) +
         std::get<1>(vec1) * std::get<1>(vec2);
}

float dot(const Vec3 vec1, const Vec3 vec2) {
  return std::get<0>(vec1) * std::get<0>(vec2) +
         std::get<1>(vec1) * std::get<1>(vec2) +
         std::get<2>(vec1) * std::get<2>(vec2);
}

float cross(const Vec2 vec1, const Vec2 vec2) {
  return std::get<0>(vec1) * std::get<1>(vec2) -
         std::get<0>(vec2) * std::get<1>(vec1);
}

Vec3 cross(const Vec3 vec1, const Vec3 vec2) {
  return {std::get<1>(vec1) * std::get<2>(vec2) -
              std::get<2>(vec1) * std::get<1>(vec2),
          std::get<2>(vec1) * std::get<0>(vec2) -
              std::get<0>(vec1) * std::get<2>(vec2),
          std::get<0>(vec1) * std::get<1>(vec2) -
              std::get<1>(vec1) * std::get<0>(vec2)};
}

float abs(const Vec2 vec) {
  return std::sqrt(std::get<0>(vec) * std::get<0>(vec) +
                   std::get<1>(vec) * std::get<1>(vec));
}

float abs(const Vec3 vec) {
  return std::sqrt(std::get<0>(vec) * std::get<0>(vec) +
                   std::get<1>(vec) * std::get<1>(vec) +
                   std::get<2>(vec) * std::get<2>(vec));
}

float angle(const Vec2 vec1, const Vec2 vec2) {
  float mag = abs(vec1) * abs(vec2);
  return mag > 0 ? std::acos(dot(vec1, vec2) / mag) : 0;
}

float angle(const Vec3 vec1, const Vec3 vec2) {
  float mag = abs(vec1) * abs(vec2);
  return mag > 0 ? std::acos(dot(vec1, vec2) / mag) : 0;
}

Vec2 rotate2D(const Vec2 vec, const float angle) {
  return {
      std::cos(angle) * std::get<0>(vec) - std::sin(angle) * std::get<1>(vec),
      std::sin(angle) * std::get<0>(vec) + std::cos(angle) * std::get<1>(vec)};
}

Vec3 rotateX(const Vec3 vec, const float angle) {
  return {
      std::get<0>(vec),
      std::cos(angle) * std::get<1>(vec) - std::sin(angle) * std::get<2>(vec),
      std::sin(angle) * std::get<1>(vec) + std::cos(angle) * std::get<2>(vec)};
}

Vec3 rotateY(const Vec3 vec, const float angle) {
  return {
      std::sin(angle) * std::get<2>(vec) + std::cos(angle) * std::get<0>(vec),
      std::get<1>(vec),
      std::cos(angle) * std::get<2>(vec) - std::sin(angle) * std::get<0>(vec)};
}

Vec3 rotateZ(const Vec3 vec, const float angle) {
  return {
      std::cos(angle) * std::get<0>(vec) - std::sin(angle) * std::get<1>(vec),
      std::sin(angle) * std::get<0>(vec) + std::cos(angle) * std::get<1>(vec),
      std::get<2>(vec)};
}

std::ostream &operator<<(std::ostream &os, const Vec2 &vec) {
  os << "(" << std::get<0>(vec) << ", " << std::get<1>(vec) << ")";
  return os;
}

std::ostream &operator<<(std::ostream &os, const Vec3 &vec) {
  os << "(" << std::get<0>(vec) << ", " << std::get<1>(vec) << ", "
     << std::get<2>(vec) << ")";
  return os;
}
}  // namespace stl3lasercut
