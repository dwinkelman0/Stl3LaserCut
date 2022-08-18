// Copyright 2022 by Daniel Winkelman. All rights reserved.

#include <gtest/gtest.h>
#include <stl3lasercut/Projector.h>
#include <stl3lasercut/StlIo.h>

#include <cmath>
#include <numbers>

#include "Util.h"

namespace stl3lasercut {
TEST(Projector, Basic2D) {
  BoundedLine reference = *BoundedLine::fromPoints({0, 0}, {2, 2});
  Projector2D projector(reference);
  ASSERT_FLOAT_EQ(projector.angle_, std::numbers::pi / 4);
  testPoint(projector.normalize({0, 0}), {-std::sqrt(2), 0});
  testPoint(projector.normalize({2, 2}), {std::sqrt(2), 0});
  testPoint(projector.normalize({0, 2}), {0, std::sqrt(2)});
  testPoint(projector.restore({-std::sqrt(2), 0}), {0, 0});
  testPoint(projector.restore({std::sqrt(2), 0}), {2, 2});
  testPoint(projector.restore({0, std::sqrt(2)}), {0, 2});
}

TEST(Projector, Advanced2D) {
  BoundedLine reference = *BoundedLine::fromPoints({0, 1}, {-2, -1});
  Projector2D projector(reference);
  ASSERT_FLOAT_EQ(projector.angle_, -std::numbers::pi * 3 / 4);
  testPoint(projector.normalize({0, 1}), {-std::sqrt(2), 0});
  testPoint(projector.normalize({-2, -1}), {std::sqrt(2), 0});
  testPoint(projector.normalize({0, -1}), {0, std::sqrt(2)});
  testPoint(projector.restore({-std::sqrt(2), 0}), {0, 1});
  testPoint(projector.restore({std::sqrt(2), 0}), {-2, -1});
  testPoint(projector.restore({0, std::sqrt(2)}), {0, -1});
}

TEST(Projector, Projector3D) {
  StlTriangle t0({Vec3(1, 0, 0), Vec3(0, 1, 0), Vec3(0, 0, 1)});
  StlTriangle t1({Vec3(2, -1, 0), Vec3(1, 0, 0), Vec3(1, -1, 1)});
  StlTriangle t2({Vec3(2, 0, 0), Vec3(1, 1, 0), Vec3(1, 0, 1)});
  Projector3D p0 = t0.getProjector();
  ASSERT_EQ(t1.getProjector(), p0);
  ASSERT_LT(p0, t2.getProjector());
  testPoint(p0.restore(p0.normalize({1, 0, 0})), {1, 0, 0});
  testPoint(p0.restore(p0.normalize({0, 1, 0})), {0, 1, 0});
  testPoint(p0.restore(p0.normalize({0, 0, 1})), {0, 0, 1});
  // TODO: could use some more thorough testing
}
}  // namespace stl3lasercut
