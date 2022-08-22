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

const uint32_t BASE_COLOR = 1;
const uint32_t INTERMEDIATE_COLOR = 2;
const uint32_t OFFSET_COLOR = 3;

namespace offset {
std::vector<InterferencePlane::OffsetCalculation> single(
    const InterferencePlane::OffsetFunction &function,
    const bool calculateInterference) {
  return {InterferencePlane::OffsetCalculation{
      .function = function,
      .baseColor = BASE_COLOR,
      .perpendicularColor = BASE_COLOR,
      .newColor = OFFSET_COLOR,
      .calculateInterference = calculateInterference}};
}

InterferencePlane::OffsetCalculation first(
    const InterferencePlane::OffsetFunction &function,
    const bool calculateInterference) {
  return InterferencePlane::OffsetCalculation{
      .function = function,
      .baseColor = BASE_COLOR,
      .perpendicularColor = BASE_COLOR,
      .newColor = INTERMEDIATE_COLOR,
      .calculateInterference = calculateInterference};
}

InterferencePlane::OffsetCalculation second(
    const InterferencePlane::OffsetFunction &function,
    const bool calculateInterference) {
  return InterferencePlane::OffsetCalculation{
      .function = function,
      .baseColor = INTERMEDIATE_COLOR,
      .perpendicularColor = INTERMEDIATE_COLOR,
      .newColor = OFFSET_COLOR,
      .calculateInterference = calculateInterference};
}

InterferencePlane::OffsetFunction constant(const float offset) {
  return [offset](const auto &a, const auto &b) { return offset; };
}

InterferencePlane::OffsetFunction ring(const RingVector<float> &ring) {
  auto counter = std::make_shared<uint32_t>(0);
  return [ring, counter](const auto &a, const auto &b) {
    return ring[(*counter)++];
  };
}
}  // namespace offset
}  // namespace stl3lasercut
