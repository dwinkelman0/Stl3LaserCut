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

class Projector3D {
 public:
  Projector3D(const Vec3 &normal, const Vec3 &point);

  Vec2 normalize(const Vec3 &point) const;
  Vec3 restore(const Vec2 &point) const;

  bool operator==(const Projector3D &other) const;
  bool operator<(const Projector3D &other) const;

  friend std::ostream &operator<<(std::ostream &os,
                                  const Projector3D &projector);

 private:
  float angleZ_;
  float angleX_;
  float offsetZ_;
};
}  // namespace stl3lasercut
