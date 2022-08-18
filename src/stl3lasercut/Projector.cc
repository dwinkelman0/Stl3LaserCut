// Copyright 2022 by Daniel Winkelman. All rights reserved.

#include "Projector.h"

#include <cmath>
#include <numbers>
#include <tuple>

namespace stl3lasercut {
DirectedLine xAxis = *DirectedLine::fromPoints({0, 0}, {1, 0});

Projector2D::Projector2D(const BoundedLine &reference)
    : center_(reference.getMidpoint()), angle_(xAxis.getAngle(reference)) {}

Vec2 Projector2D::normalize(const Vec2 &point) const {
  return rotate2D(point - center_, -angle_);
}

Vec2 Projector2D::restore(const Vec2 &point) const {
  return rotate2D(point, angle_) + center_;
}

Projector3D::Projector3D(const Vec3 &normal, const Vec3 &point)
    : angleZ_(angle(cross(normal, unit3::z), unit3::x) *
              (dot(normal, unit3::x) > 0 ? 1 : -1)),
      angleX_(angle(normal, unit3::z) *
              (dot(rotateZ(normal, angleZ_), unit3::y) > 0 ? 1 : -1)),
      offsetZ_(std::get<2>(rotateX(rotateZ(point, angleZ_), angleX_))) {
  const float threshold = 1e-6;
  float totalAngle = angle(rotateZ(normal, angleZ_), unit3::y) +
                     angle(rotateZ(normal, angleZ_), unit3::z);
  assert(totalAngle < threshold ||
         totalAngle - std::numbers::pi / 2 < threshold ||
         totalAngle - std::numbers::pi < threshold ||
         totalAngle - std::numbers::pi * 3 / 2 < threshold);
  assert(angle(rotateX(rotateZ(normal, angleZ_), angleX_), unit3::z) <
         threshold);
  assert(offsetZ_ - abs(point) < threshold);
}

Vec2 Projector3D::normalize(const Vec3 &point) const {
  Vec3 rotated = rotateZ(rotateX(rotateZ(point, angleZ_), angleX_), -angleZ_);
  return {std::get<0>(rotated), std::get<1>(rotated)};
}

Vec3 Projector3D::restore(const Vec2 &point) const {
  Vec3 rotated(std::get<0>(point), std::get<1>(point), offsetZ_);
  return rotateZ(rotateX(rotateZ(rotated, angleZ_), -angleX_), -angleZ_);
}

bool Projector3D::operator==(const Projector3D &other) const {
  const float threshold = 1e-4;
  return std::abs(angleZ_ - other.angleZ_) < threshold &&
         std::abs(angleX_ - other.angleX_) < threshold &&
         std::abs(offsetZ_ - other.offsetZ_) < threshold;
}

bool Projector3D::operator<(const Projector3D &other) const {
  if (*this == other) {
    return false;
  } else {
    return std::tie(angleZ_, angleX_, offsetZ_) <
           std::tie(other.angleZ_, other.angleX_, other.offsetZ_);
  }
}

std::ostream &operator<<(std::ostream &os, const Projector3D &projector) {
  os << "(angleZ: " << projector.angleZ_ << ", angleX: " << projector.angleX_
     << ", offset: " << projector.offsetZ_ << ")";
  return os;
}

Projector3D Projector3D::nullProjector(unit3::z, Vec3(0, 0, 0));
}  // namespace stl3lasercut
