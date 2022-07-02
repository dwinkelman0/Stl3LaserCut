// Copyright 2022 by Daniel Winkelman. All rights reserved.

#pragma once

#include <stl3lasercut/Line.h>
#include <stl3lasercut/Util.h>

namespace stl3lasercut {
class Projector2D {
 public:
  Projector2D(const BoundedLine &reference);

  Vec2 normalize(const Vec2 &point) const;
  Vec2 restore(const Vec2 &point) const;

 private:
  Vec2 center_;
  float angle_;
};
}  // namespace stl3lasercut
