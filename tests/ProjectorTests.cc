// Copyright 2022 by Daniel Winkelman. All rights reserved.

#include <gtest/gtest.h>
#include <stl3lasercut/Projector.h>
#include <stl3lasercut/StlIo.h>

#include <cmath>

namespace stl3lasercut {
void testPoint(const Vec2 &a, const Vec2 &b) {
  ASSERT_LT(std::abs(std::get<0>(a) - std::get<0>(b)), 1e-6);
  ASSERT_LT(std::abs(std::get<1>(a) - std::get<1>(b)), 1e-6);
}

void testPoint(const Vec3 &a, const Vec3 &b) {
  ASSERT_LT(std::abs(std::get<0>(a) - std::get<0>(b)), 1e-6);
  ASSERT_LT(std::abs(std::get<1>(a) - std::get<1>(b)), 1e-6);
  ASSERT_LT(std::abs(std::get<2>(a) - std::get<2>(b)), 1e-6);
}

TEST(Projector, Projector2D) {
  BoundedLine reference = *BoundedLine::fromPoints({0, 0}, {2, 2});
  Projector2D projector(reference);
  testPoint(projector.normalize({0, 0}), {-std::sqrt(2), 0});
  testPoint(projector.normalize({2, 2}), {std::sqrt(2), 0});
  testPoint(projector.normalize({0, 2}), {0, std::sqrt(2)});
  testPoint(projector.restore({-std::sqrt(2), 0}), {0, 0});
  testPoint(projector.restore({std::sqrt(2), 0}), {2, 2});
  testPoint(projector.restore({0, std::sqrt(2)}), {0, 2});
}

TEST(Projector, Projector3D) {
  StlTriangle t0({Vec3(1, 0, 0), Vec3(0, 1, 0), Vec3(0, 0, 1)});
  StlTriangle t1({Vec3(2, -1, 0), Vec3(1, 0, 0), Vec3(1, -1, 1)});
  StlTriangle t2({Vec3(2, 0, 0), Vec3(1, 1, 0), Vec3(1, 0, 1)});
  Projector3D p0 = t0.getProjector();
  ASSERT_EQ(t1.getProjector(), p0);
  ASSERT_LT(p0, t2.getProjector());
  // testPoint(p0.restore(p0.normalize({0, 0, 0})),);
  testPoint(p0.restore(p0.normalize({1, 0, 0})), {1, 0, 0});
  testPoint(p0.restore(p0.normalize({0, 1, 0})), {0, 1, 0});
  testPoint(p0.restore(p0.normalize({0, 0, 1})), {0, 0, 1});
  // TODO: could use some more thorough testing
}
}  // namespace stl3lasercut
