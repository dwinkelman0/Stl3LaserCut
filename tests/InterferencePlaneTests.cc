// Copyright 2022 by Daniel Winkelman. All rights reserved.

#include <gtest/gtest.h>
#include <stl3lasercut/AssemblyPlane.h>
#include <stl3lasercut/InterferencePlane.h>
#include <stl3lasercut/LoopPlane.h>

#include "SampleGeometry.h"

namespace stl3lasercut {
struct InterferencePlaneTestCase {
  std::string name;
  std::vector<std::vector<Vec2>> points;
  uint32_t numEdgeGroups;

  friend std::ostream &operator<<(std::ostream &os,
                                  const InterferencePlaneTestCase &tc);
};

std::ostream &operator<<(std::ostream &os,
                         const InterferencePlaneTestCase &tc) {
  os << tc.name << " (" << tc.points.size() << " loops)";
  return os;
}

#define SAMPLE(n) .name = #n, .points = samples::n

class InterferenePlaneTests
    : public testing::TestWithParam<InterferencePlaneTestCase> {
 public:
  InterferenePlaneTests()
      : assemblyPlane_(std::make_shared<AssemblyPlane>(
            nullptr, 1, Projector3D::nullProjector)),
        loopPlane_(nullptr),
        interferencePlane_(assemblyPlane_) {}

  void SetUp() override {
    for (const std::vector<Vec2> &loop : GetParam().points) {
      assemblyPlane_->addLoop(loop);
    }
    loopPlane_ = std::make_shared<LoopPlane>(assemblyPlane_, DEFAULT_COLOR);
    interferencePlane_.addLoopPlane(loopPlane_);
  }

 protected:
  std::shared_ptr<AssemblyPlane> assemblyPlane_;
  std::shared_ptr<LoopPlane> loopPlane_;
  InterferencePlane interferencePlane_;

  const uint32_t DEFAULT_COLOR = 3;
};

TEST_P(InterferenePlaneTests, Initialization) {
  for (const InterferencePlane::Graph::ConstEdge &edge :
       interferencePlane_.graph_.getEdges()) {
    ASSERT_GE(edge.getValue()->edges.size(), 1);
    ASSERT_EQ(edge.getValue()->edges.begin()->color, 3);
    ASSERT_EQ(edge.getValue()->edges.begin()->orientation,
              InterferencePlane::Orientation::PARALLEL);
  }
  for (const auto &[coord, logicalEdge] : interferencePlane_.edges_) {
    ASSERT_EQ(logicalEdge.orientationClass,
              InterferencePlane::OrientationClass::PARALLEL);
    ASSERT_GE(logicalEdge.group->edges.size(), 1);
    ASSERT_GE(logicalEdge.group->points.size(), 2);
    auto it = interferencePlane_.parallelEdgeAdjacency_.find(coord.id);
    ASSERT_NE(it, interferencePlane_.parallelEdgeAdjacency_.end());
    ASSERT_LT(it->second.first, std::numeric_limits<uint32_t>::max());
    ASSERT_LT(it->second.second, std::numeric_limits<uint32_t>::max());
  }
  ASSERT_EQ(interferencePlane_.groupMap_.size(), GetParam().numEdgeGroups);
}

INSTANTIATE_TEST_SUITE_P(
    InterferenePlane, InterferenePlaneTests,
    testing::Values(
        InterferencePlaneTestCase{SAMPLE(acuteTriangle), .numEdgeGroups = 3},
        InterferencePlaneTestCase{SAMPLE(rightTriangle), .numEdgeGroups = 3},
        InterferencePlaneTestCase{SAMPLE(obtuseTriangle), .numEdgeGroups = 3},
        InterferencePlaneTestCase{SAMPLE(disjointTriangles),
                                  .numEdgeGroups = 6},
        InterferencePlaneTestCase{SAMPLE(twoTangentTriangles),
                                  .numEdgeGroups = 5},
        InterferencePlaneTestCase{SAMPLE(negativeTriangle), .numEdgeGroups = 6},
        InterferencePlaneTestCase{SAMPLE(bubbleNegativeTriangle),
                                  .numEdgeGroups = 6},
        InterferencePlaneTestCase{SAMPLE(twoTangentNegativeTriangles),
                                  .numEdgeGroups = 8},
        InterferencePlaneTestCase{SAMPLE(tangentPositiveAndNegativeTriangles),
                                  .numEdgeGroups = 8}));
}  // namespace stl3lasercut
