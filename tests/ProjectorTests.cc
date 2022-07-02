// Copyright 2022 by Daniel Winkelman. All rights reserved.

#include <gtest/gtest.h>
#include <stl3lasercut/Projector.h>

#include <cmath>

namespace stl3lasercut {
void testPoint(const Vec2 &a, const Vec2 &b) {
  ASSERT_LT(std::abs(std::get<0>(a) - std::get<0>(b)), 1e-6);
  ASSERT_LT(std::abs(std::get<1>(a) - std::get<1>(b)), 1e-6);
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
}  // namespace stl3lasercut
