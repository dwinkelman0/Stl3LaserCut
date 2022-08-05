// Copyright 2022 by Daniel Winkelman. All rights reserved.

#include "Util.h"

#include <gtest/gtest.h>

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
}  // namespace stl3lasercut
