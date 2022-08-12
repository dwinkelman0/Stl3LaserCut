// Copyright 2022 by Daniel Winkelman. All rights reserved.

#include <gtest/gtest.h>
#include <stl3lasercut/AssemblyPlane.h>
#include <stl3lasercut/LoopPlane.h>

#include <algorithm>
#include <map>

#include "SampleGeometry.h"

namespace stl3lasercut {
std::vector<uint32_t> getCanonicalOrder(const std::vector<uint32_t> &vec);

TEST(LoopPlane, GetCanonicalOrder) {
  std::vector<uint32_t> e0 = {0, 1, 2, 3};
  std::vector<uint32_t> e1 = {2, 3, 0, 1};
  ASSERT_EQ(getCanonicalOrder(e0), e0);
  ASSERT_EQ(getCanonicalOrder(e1), e0);
  std::vector<uint32_t> e2 = {3, 6, 5, 5, 6, 3, 7};
  std::vector<uint32_t> e3 = {5, 6, 3, 7, 3, 6, 5};
  ASSERT_EQ(getCanonicalOrder(e2), e2);
  ASSERT_EQ(getCanonicalOrder(e3), e2);
}

struct LoopPlaneTestCase {
  std::string name;
  std::vector<std::vector<Vec2>> points;
  LoopPlane::Characteristic characteristic;

  friend std::ostream &operator<<(std::ostream &os,
                                  const LoopPlaneTestCase &tc);
};

std::ostream &operator<<(std::ostream &os, const LoopPlaneTestCase &tc) {
  os << tc.name << " (" << tc.points.size() << " loops)";
  return os;
}

#define SAMPLE(n) .name = #n, .points = samples::n

namespace ch {
LoopPlane::Characteristic make(
    const bool isPositive,
    const std::vector<std::pair<uint32_t, LoopPlane::Characteristic>> &sizes) {
  LoopPlane::Characteristic output;
  for (const auto &[size, child] : sizes) {
    output.addData({.isPositive = isPositive, .size = size}, child);
  }
  return output;
}
}  // namespace ch

class LoopPlaneTests : public testing::TestWithParam<LoopPlaneTestCase> {
 public:
  LoopPlaneTests()
      : assemblyPlane_(std::make_shared<AssemblyPlane>(
            nullptr, 1, Projector3D::nullProjector)),
        loopPlane_(nullptr) {}

  void SetUp() override {
    for (const std::vector<Vec2> &loop : GetParam().points) {
      assemblyPlane_->addLoop(loop);
    }
    loopPlane_ = std::make_shared<LoopPlane>(assemblyPlane_, 0);
  }

 protected:
  std::shared_ptr<AssemblyPlane> assemblyPlane_;
  std::shared_ptr<LoopPlane> loopPlane_;
};

TEST_P(LoopPlaneTests, Characteristic) {
  HierarchicalOrdering<LoopPlane::Loop> ordering = loopPlane_->getLoops();
  ASSERT_EQ(LoopPlane::getCharacteristic(ordering), GetParam().characteristic);
}

INSTANTIATE_TEST_SUITE_P(
    LoopPlane, LoopPlaneTests,
    testing::Values(
        LoopPlaneTestCase{SAMPLE(acuteTriangle),
                          .characteristic = ch::make(true, {{3, {}}})},
        LoopPlaneTestCase{SAMPLE(rightTriangle),
                          .characteristic = ch::make(true, {{3, {}}})},
        LoopPlaneTestCase{SAMPLE(obtuseTriangle),
                          .characteristic = ch::make(true, {{3, {}}})},
        LoopPlaneTestCase{SAMPLE(disjointTriangles),
                          .characteristic = ch::make(true, {{3, {}}, {3, {}}})},
        LoopPlaneTestCase{SAMPLE(twoTangentTriangles),
                          .characteristic = ch::make(true, {{3, {}}, {3, {}}})},
        LoopPlaneTestCase{
            SAMPLE(negativeTriangle),
            .characteristic = ch::make(true,
                                       {{3, ch::make(false, {{3, {}}})}})},
        LoopPlaneTestCase{
            .name = "negativeTriangleReversed",
            .points = {samples::negativeTriangle[1],
                       samples::negativeTriangle[0]},
            .characteristic = ch::make(true,
                                       {{3, ch::make(false, {{3, {}}})}})}));
}  // namespace stl3lasercut
