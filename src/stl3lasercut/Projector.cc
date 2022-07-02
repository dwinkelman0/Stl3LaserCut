// Copyright 2022 by Daniel Winkelman. All rights reserved.

#include "Projector.h"

#include <cmath>

namespace stl3lasercut {
Projector2D::Projector2D(const BoundedLine &reference)
    : center_(reference.getMidpoint()),
      angle_(std::atan2(std::get<1>(reference.getMidpoint()),
                        std::get<0>(reference.getMidpoint()))) {}

Vec2 Projector2D::normalize(const Vec2 &point) const {
  return rotate2D(point - center_, -angle_);
}

Vec2 Projector2D::restore(const Vec2 &point) const {
  return rotate2D(point, angle_) + center_;
}
}  // namespace stl3lasercut
