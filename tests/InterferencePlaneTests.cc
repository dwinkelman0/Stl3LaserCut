// Copyright 2022 by Daniel Winkelman. All rights reserved.

#include <gtest/gtest.h>
#include <stl3lasercut/AssemblyPlane.h>
#include <stl3lasercut/Desmos.h>
#include <stl3lasercut/InterferencePlane.h>
#include <stl3lasercut/LoopPlane.h>

#include "SampleGeometry.h"

namespace stl3lasercut {
#define SAMPLE(n) .name = #n, .points = samples::n

const uint32_t BASE_COLOR = 1;
const uint32_t INTERMEDIATE_COLOR = 2;
const uint32_t OFFSET_COLOR = 3;

class InterferencePlaneBaseTest {
 public:
  InterferencePlaneBaseTest(const std::vector<std::vector<Vec2>> &points,
                            const std::string &name)
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
  }

  ~InterferencePlaneBaseTest() {
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

struct InterferencePlaneSetupCase {
  std::string name;
  std::vector<std::vector<Vec2>> points;
  uint32_t numEdgeGroups;

  friend std::ostream &operator<<(std::ostream &os,
                                  const InterferencePlaneSetupCase &tc);
};

std::ostream &operator<<(std::ostream &os,
                         const InterferencePlaneSetupCase &tc) {
  os << tc.name;
  return os;
}

class InterferencePlaneSetup
    : public InterferencePlaneBaseTest,
      public testing::TestWithParam<InterferencePlaneSetupCase> {
 public:
  InterferencePlaneSetup()
      : InterferencePlaneBaseTest(GetParam().points, GetParam().name) {}
};

TEST_P(InterferencePlaneSetup, Setup) {
  std::map<std::shared_ptr<InterferencePlane::EdgeGroup>, uint32_t> groupCount;
  for (const InterferencePlane::Graph::ConstEdge &edge :
       interferencePlane_.graph_.getEdges()) {
    groupCount.emplace(edge.getValue(), 0).first->second++;
    ASSERT_EQ(edge.getValue()->edges.begin()->color, BASE_COLOR);
    ASSERT_EQ(edge.getValue()->edges.begin()->orientation,
              InterferencePlane::Orientation::PARALLEL);
  }
  for (const auto &[coord, group] : interferencePlane_.edges_) {
    auto it = interferencePlane_.edgeAdjacency_.find(coord.id);
    ASSERT_NE(it, interferencePlane_.edgeAdjacency_.end());
    ASSERT_LT(it->second.first, std::numeric_limits<uint32_t>::max());
    ASSERT_LT(it->second.second, std::numeric_limits<uint32_t>::max());
    ASSERT_EQ(groupCount.find(group)->second + 1, group->points.size());
  }
  ASSERT_EQ(interferencePlane_.groupMap_.size(), GetParam().numEdgeGroups);
}

INSTANTIATE_TEST_SUITE_P(
    InterferencePlane, InterferencePlaneSetup,
    testing::Values(
        InterferencePlaneSetupCase{SAMPLE(acuteTriangle), .numEdgeGroups = 3},
        InterferencePlaneSetupCase{SAMPLE(obtuseTriangle), .numEdgeGroups = 3},
        InterferencePlaneSetupCase{SAMPLE(straightAnglePolygon),
                                   .numEdgeGroups = 3},
        InterferencePlaneSetupCase{SAMPLE(obtuseConcavePolygon),
                                   .numEdgeGroups = 4},
        InterferencePlaneSetupCase{SAMPLE(disjointTriangles),
                                   .numEdgeGroups = 6},
        InterferencePlaneSetupCase{SAMPLE(twoTangentTriangles),
                                   .numEdgeGroups = 5},
        InterferencePlaneSetupCase{SAMPLE(negativeTriangle),
                                   .numEdgeGroups = 6},
        InterferencePlaneSetupCase{SAMPLE(bubbleNegativeTriangle),
                                   .numEdgeGroups = 6},
        InterferencePlaneSetupCase{SAMPLE(tangentPositiveAndNegativeTriangles),
                                   .numEdgeGroups = 8}),
    testing::PrintToStringParamName());

struct Characteristic {
  uint32_t vertices;
  uint32_t edges;
};

struct InterferencePlaneOffsetCase {
  std::string name;
  std::vector<std::vector<Vec2>> points;
  std::vector<InterferencePlane::OffsetCalculation> calculations;
  Characteristic characteristic;

  friend std::ostream &operator<<(std::ostream &os,
                                  const InterferencePlaneSetupCase &tc);
};

std::ostream &operator<<(std::ostream &os,
                         const InterferencePlaneOffsetCase &tc) {
  os << tc.name;
  return os;
}

class InterferencePlaneOffset
    : public InterferencePlaneBaseTest,
      public testing::TestWithParam<InterferencePlaneOffsetCase> {
 public:
  InterferencePlaneOffset()
      : InterferencePlaneBaseTest(GetParam().points, GetParam().name) {}

  void SetUp() override {
    interferencePlane_.applyOffsetFunctions(GetParam().calculations);
  }
};

TEST_P(InterferencePlaneOffset, Offset) {
  ASSERT_EQ(interferencePlane_.graph_.getVertices().getCount(),
            GetParam().characteristic.vertices);
  ASSERT_EQ(interferencePlane_.graph_.getEdges().getCount(),
            GetParam().characteristic.edges);
}

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

INSTANTIATE_TEST_SUITE_P(
    InterferencePlane, InterferencePlaneOffset,
    testing::Values(
        InterferencePlaneOffsetCase{
            .name = "acuteTriangle_noOffset",
            .points = samples::acuteTriangle,
            .calculations = offset::single(offset::constant(0), true),
            .characteristic = Characteristic{.vertices = 3, .edges = 3}},
        InterferencePlaneOffsetCase{
            .name = "acuteTriangle_noInterference",
            .points = samples::acuteTriangle,
            .calculations = offset::single(offset::constant(-1), false),
            .characteristic = Characteristic{.vertices = 6, .edges = 6}},
        InterferencePlaneOffsetCase{
            .name = "acuteTriangle_interference",
            .points = samples::acuteTriangle,
            .calculations = offset::single(offset::constant(-1), true),
            .characteristic = Characteristic{.vertices = 12, .edges = 18}},
        InterferencePlaneOffsetCase{
            .name = "acuteTriangle_middleInterference",
            .points = samples::acuteTriangle,
            .calculations = {offset::first(offset::constant(-1), false),
                             offset::second(offset::constant(0.5), true)},
            .characteristic = Characteristic{.vertices = 15, .edges = 21}},
        InterferencePlaneOffsetCase{
            .name = "acuteTriangle_doubleInterference",
            .points = samples::acuteTriangle,
            .calculations = {offset::first(offset::constant(-1), true),
                             offset::second(offset::constant(0.5), true)},
            .characteristic = Characteristic{.vertices = 21, .edges = 33}},
        InterferencePlaneOffsetCase{
            .name = "rightTriangle",
            .points = samples::rightTriangle,
            .calculations = offset::single(offset::constant(-0.5), true),
            .characteristic = Characteristic{.vertices = 12, .edges = 18}},
        InterferencePlaneOffsetCase{
            .name = "obtuseTriangle_noOffset",
            .points = samples::obtuseTriangle,
            .calculations = offset::single(offset::constant(0), true),
            .characteristic = Characteristic{.vertices = 3, .edges = 3}},
        InterferencePlaneOffsetCase{
            .name = "obtuseTriangle_positiveOffset",
            .points = samples::obtuseTriangle,
            .calculations = offset::single(offset::constant(0.2), true),
            .characteristic = Characteristic{.vertices = 18, .edges = 30}},
        InterferencePlaneOffsetCase{
            .name = "obtuseTriangle_negativeOffset",
            .points = samples::obtuseTriangle,
            .calculations = offset::single(offset::constant(-0.2), true),
            .characteristic = Characteristic{.vertices = 16, .edges = 26}},
        InterferencePlaneOffsetCase{
            .name = "obtuseTriangle_middleInterference",
            .points = samples::obtuseTriangle,
            .calculations = {offset::first(offset::constant(-1), false),
                             offset::second(offset::constant(0.5), true)},
            .characteristic = Characteristic{.vertices = 25, .edges = 41}},
        InterferencePlaneOffsetCase{
            .name = "obtuseConcavePolygon_positiveOffset",
            .points = samples::obtuseConcavePolygon,
            .calculations = offset::single(offset::constant(0.2), true),
            .characteristic = Characteristic{.vertices = 30, .edges = 52}},
        InterferencePlaneOffsetCase{
            .name = "obtuseConcavePolygon_negativeOffset",
            .points = samples::obtuseConcavePolygon,
            .calculations = offset::single(offset::constant(-0.2), true),
            .characteristic = Characteristic{.vertices = 28, .edges = 48}},
        InterferencePlaneOffsetCase{
            .name = "disjointTriangles_negativeOffset",
            .points = samples::disjointTriangles,
            .calculations = offset::single(offset::constant(-1), false),
            .characteristic = Characteristic{.vertices = 18, .edges = 24}},
        InterferencePlaneOffsetCase{
            .name = "obtuseTriangle_variableNegativeOffset",
            .points = samples::obtuseTriangle,
            .calculations = offset::single(
                offset::ring(RingVector<float>({-1, -0.5, -0.2})), true),
            .characteristic = Characteristic{.vertices = 17, .edges = 28}},
        InterferencePlaneOffsetCase{
            .name = "obtuseTriangle_partialZeroOffset",
            .points = samples::obtuseTriangle,
            .calculations = offset::single(
                offset::ring(RingVector<float>({-1, -0, 0})), true),
            .characteristic = Characteristic{.vertices = 7, .edges = 10}}),
    testing::PrintToStringParamName());
}  // namespace stl3lasercut
