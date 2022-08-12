// Copyright 2022 by Daniel Winkelman. All rights reserved.

#include <gtest/gtest.h>
#include <stl3lasercut/Util.h>

namespace stl3lasercut {
TEST(Util, GetPolygonArea) {
  std::vector<Vec2> p0 = {{0, 0}, {1, 0}, {0, 1}};
  std::vector<Vec2> p1 = {{0, 0}, {0, 1}, {1, 0}};
  std::vector<Vec2> p2 = {{5, 1}, {2, 2}, {1, 5}, {1, 1}};
  ASSERT_EQ(getPolygonArea(p0), 0.5);
  ASSERT_EQ(getPolygonArea(p1), -0.5);
  ASSERT_EQ(getPolygonArea(p2), 4);
}
}  // namespace stl3lasercut
