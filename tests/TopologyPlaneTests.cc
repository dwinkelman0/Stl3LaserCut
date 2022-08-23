// Copyright 2022 by Daniel Winkelman. All rights reserved.

#include <gtest/gtest.h>
#include <stl3lasercut/AssemblyPlane.h>
#include <stl3lasercut/Desmos.h>
#include <stl3lasercut/InterferencePlane.h>
#include <stl3lasercut/TopologyPlane.h>

#include "SampleGeometry.h"
#include "Util.h"

namespace stl3lasercut {
#define SAMPLE(n) .name = #n, .points = samples::n

class TopologyPlaneBaseTest {
 public:
  TopologyPlaneBaseTest(
      const std::vector<std::vector<Vec2>> &points, const std::string &name,
      const std::vector<InterferencePlane::OffsetCalculation> &calculations)
      : assemblyPlane_(std::make_shared<AssemblyPlane>(
            nullptr, 1, Projector3D::nullProjector)),
        loopPlane_(nullptr),
        interferencePlane_(assemblyPlane_),
        name_(name) {
    for (const std::vector<Vec2> &loop : points) {
      assemblyPlane_->addLoop(loop);
    }
    loopPlane_ = std::make_shared<LoopPlane>(assemblyPlane_, BASE_COLOR);
    interferencePlane_.addLoopPlane(loopPlane_);
    interferencePlane_.applyOffsetFunctions(calculations);
    interferencePlane_.finalize();
  }

  ~TopologyPlaneBaseTest() {
    std::ofstream outputFile("desmos_" + name_ + ".html");
    DesmosOutput desmos(outputFile, assemblyPlane_);
    desmos.outputInterferencePlane(interferencePlane_);
  }

 protected:
  std::shared_ptr<AssemblyPlane> assemblyPlane_;
  std::shared_ptr<LoopPlane> loopPlane_;
  InterferencePlane interferencePlane_;
  std::string name_;
};

struct TopologyPlaneTestCase {
  std::string name;
  std::vector<std::vector<Vec2>> points;
  std::vector<InterferencePlane::OffsetCalculation> calculations;

  friend std::ostream &operator<<(std::ostream &os,
                                  const TopologyPlaneTestCase &tc);
};

std::ostream &operator<<(std::ostream &os, const TopologyPlaneTestCase &tc) {
  os << tc.name;
  return os;
}

class TopologyPlaneTest : public TopologyPlaneBaseTest,
                          public testing::TestWithParam<TopologyPlaneTestCase> {
 public:
  TopologyPlaneTest()
      : TopologyPlaneBaseTest(GetParam().points, "topology_" + GetParam().name,
                              GetParam().calculations),
        topologyPlane_(interferencePlane_),
        root_(std::make_shared<TopologyPlane>(topologyPlane_)) {}

  void SetUp() override {
    // topologyPlane_.debug();
    // topologyPlane_.simplifyCycle();
    // topologyPlane_.debug();
  }

 protected:
  TopologyPlane topologyPlane_;
  std::shared_ptr<TopologyPlane> root_;
};

TEST_P(TopologyPlaneTest, Basic) {}

INSTANTIATE_TEST_SUITE_P(
    TopologyPlane, TopologyPlaneTest,
    testing::Values(
        TopologyPlaneTestCase{
            SAMPLE(acuteTriangle),
            .calculations = offset::single(offset::constant(-1), true)},
        TopologyPlaneTestCase{
            SAMPLE(obtuseTriangle),
            .calculations = offset::single(
                offset::ring(RingVector<float>({-0.4, -0.8, -1.2})), true)},
        TopologyPlaneTestCase{
            SAMPLE(valley),
            .calculations = offset::single(
                offset::ring(RingVector<float>({0.5, 0, 0.3, 1.3, 0})), true)},
        TopologyPlaneTestCase{
            SAMPLE(disjointTriangles),
            .calculations = offset::single(offset::constant(-0.5), true)},
        TopologyPlaneTestCase{
            .name = "disjointTriangles_overlap",
            .points = samples::disjointTriangles,
            .calculations = offset::single(offset::constant(-1), true)},
        TopologyPlaneTestCase{
            SAMPLE(bubbleNegativeTriangle),
            .calculations = offset::single(
                offset::ring(RingVector<float>({0, 0.4, 0, 0, 2, 0.6})),
                true)}),
    testing::PrintToStringParamName());
}  // namespace stl3lasercut
