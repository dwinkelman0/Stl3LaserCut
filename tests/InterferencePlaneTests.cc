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
      : InterferencePlaneBaseTest(GetParam().points,
                                  "setup_" + GetParam().name) {}
};

TEST_P(InterferencePlaneSetup, Setup) {
  std::map<std::shared_ptr<InterferencePlane::EdgeGroup>, uint32_t> groupCount;
  for (const InterferencePlane::Graph::ConstEdge &edge :
       interferencePlane_.graph_.getEdges()) {
    groupCount.emplace(edge.getValue(), 0).first->second++;
    ASSERT_EQ(edge.getValue()->edges.begin()->color, BASE_COLOR);
    ASSERT_EQ(edge.getValue()->edges.begin()->orientation,
              EdgeCoordinate::Orientation::PARALLEL);
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
      : InterferencePlaneBaseTest(GetParam().points,
                                  "offset_" + GetParam().name) {}

  void SetUp() override {
    interferencePlane_.applyOffsetFunctions(GetParam().calculations);
  }
};

TEST_P(InterferencePlaneOffset, Offset) {
  interferencePlane_.pruneVertices();
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
            .characteristic = Characteristic{.vertices = 20, .edges = 34}},
        InterferencePlaneOffsetCase{
            .name = "obtuseTriangle_negativeOffset",
            .points = samples::obtuseTriangle,
            .calculations = offset::single(offset::constant(-0.2), true),
            .characteristic = Characteristic{.vertices = 18, .edges = 30}},
        InterferencePlaneOffsetCase{
            .name = "obtuseTriangle_middleInterference",
            .points = samples::obtuseTriangle,
            .calculations = {offset::first(offset::constant(-1), false),
                             offset::second(offset::constant(0.5), true)},
            .characteristic = Characteristic{.vertices = 36, .edges = 63}},
        InterferencePlaneOffsetCase{
            .name = "obtuseConcavePolygon_positiveOffset",
            .points = samples::obtuseConcavePolygon,
            .calculations = offset::single(offset::constant(0.2), true),
            .characteristic = Characteristic{.vertices = 34, .edges = 60}},
        InterferencePlaneOffsetCase{
            .name = "obtuseConcavePolygon_negativeOffset",
            .points = samples::obtuseConcavePolygon,
            .calculations = offset::single(offset::constant(-0.2), true),
            .characteristic = Characteristic{.vertices = 34, .edges = 60}},
        InterferencePlaneOffsetCase{
            .name = "disjointTriangles_negativeOffset",
            .points = samples::disjointTriangles,
            .calculations = offset::single(offset::constant(-1), false),
            .characteristic = Characteristic{.vertices = 18, .edges = 24}},
        InterferencePlaneOffsetCase{
            .name = "straightAnglePolygon_negativeOffset",
            .points = samples::straightAnglePolygon,
            .calculations = offset::single(offset::constant(-1), true),
            .characteristic = Characteristic{.vertices = 14, .edges = 26}},
        InterferencePlaneOffsetCase{
            .name = "obtuseTriangle_variableNegativeOffset",
            .points = samples::obtuseTriangle,
            .calculations = offset::single(
                offset::ring(RingVector<float>({-1, -0.5, -0.2})), true),
            .characteristic = Characteristic{.vertices = 19, .edges = 32}},
        InterferencePlaneOffsetCase{
            .name = "obtuseTriangle_partialZeroOffset",
            .points = samples::obtuseTriangle,
            .calculations = offset::single(
                offset::ring(RingVector<float>({-1, -0, 0})), true),
            .characteristic = Characteristic{.vertices = 7, .edges = 10}}),
    testing::PrintToStringParamName());

struct InterferencePlaneEdgeBoundsCase {
  std::string name;
  std::vector<std::vector<Vec2>> points;
  std::vector<InterferencePlane::OffsetCalculation> calculations;
  std::map<EdgeCoordinate, std::pair<EdgeCoordinate, EdgeCoordinate>> bounds;

  friend std::ostream &operator<<(std::ostream &os,
                                  const InterferencePlaneSetupCase &tc);
};

std::ostream &operator<<(std::ostream &os,
                         const InterferencePlaneEdgeBoundsCase &tc) {
  os << tc.name;
  return os;
}

class InterferencePlaneEdgeBounds
    : public InterferencePlaneBaseTest,
      public testing::TestWithParam<InterferencePlaneEdgeBoundsCase> {
 public:
  InterferencePlaneEdgeBounds()
      : InterferencePlaneBaseTest(GetParam().points,
                                  "bounds_" + GetParam().name) {}

  void SetUp() override {
    interferencePlane_.applyOffsetFunctions(GetParam().calculations);
    interferencePlane_.finalize();
  }

  std::optional<uint32_t> getVertex(const EdgeCoordinate &a,
                                    const EdgeCoordinate &b) {
    auto aGroup = expectToFind(interferencePlane_.edges_, a)->second;
    auto bGroup = expectToFind(interferencePlane_.edges_, b)->second;
    if (aGroup != bGroup) {
      std::vector<uint32_t> intersection;
      std::set<uint32_t> aSet(aGroup->points.begin(), aGroup->points.end());
      std::set<uint32_t> bSet(bGroup->points.begin(), bGroup->points.end());
      std::set_intersection(aSet.begin(), aSet.end(), bSet.begin(), bSet.end(),
                            std::back_inserter(intersection));
      if (intersection.empty()) {
        return std::nullopt;
      } else {
        return intersection.front();
      }
    } else {
      return std::nullopt;
    }
  }
};

TEST_P(InterferencePlaneEdgeBounds, Bounds) {
  for (const auto &[coord, expected] : GetParam().bounds) {
    const auto &[lowerCoord, upperCoord] = expected;
    const auto range =
        expectToFind(interferencePlane_.edgeBounds_, coord)->second;
    ASSERT_EQ(range.getRanges().size(), 1);
    const auto actual = *range.getRanges().begin();
    ASSERT_EQ(actual.first, getVertex(coord, lowerCoord))
        << coord << " x " << lowerCoord;
    ASSERT_EQ(actual.second, getVertex(coord, upperCoord))
        << coord << " x " << upperCoord;
  }
}

namespace bounds {
EdgeCoordinate edge(const uint32_t id, const uint32_t color,
                    const EdgeCoordinate::Orientation orientation =
                        EdgeCoordinate::Orientation::PARALLEL) {
  return EdgeCoordinate(id, color, orientation);
}
}  // namespace bounds

INSTANTIATE_TEST_SUITE_P(
    InterferencePlane, InterferencePlaneEdgeBounds,
    testing::Values(
        InterferencePlaneEdgeBoundsCase{
            .name = "acuteConcavePolygon",
            .points = samples::acuteConcavePolygon,
            .calculations = offset::single(
                offset::ring(RingVector<float>({0, -0.1, -0.2, 0})), true),
            .bounds =
                {{bounds::edge(0, BASE_COLOR),
                  {bounds::edge(3, BASE_COLOR), bounds::edge(1, OFFSET_COLOR)}},
                 {bounds::edge(0, OFFSET_COLOR),
                  {bounds::edge(3, BASE_COLOR), bounds::edge(1, OFFSET_COLOR)}},
                 {bounds::edge(1, BASE_COLOR),
                  {bounds::edge(0, BASE_COLOR), bounds::edge(2, BASE_COLOR)}}}},
        InterferencePlaneEdgeBoundsCase{
            .name = "obtuseConcavePolygon",
            .points = samples::obtuseConcavePolygon,
            .calculations = offset::single(
                offset::ring(RingVector<float>({-0.3, 0, -0.1, -0.2})), true),
            .bounds =
                {{bounds::edge(0, BASE_COLOR),
                  {bounds::edge(3, BASE_COLOR), bounds::edge(1, BASE_COLOR)}},
                 {bounds::edge(0, OFFSET_COLOR),
                  {bounds::edge(3, BASE_COLOR), bounds::edge(1, BASE_COLOR)}},
                 {bounds::edge(3, BASE_COLOR),
                  {bounds::edge(2, OFFSET_COLOR), bounds::edge(0, BASE_COLOR)}},
                 {bounds::edge(
                      0, OFFSET_COLOR,
                      EdgeCoordinate::Orientation::OUTGOING_PERPENDICULAR),
                  {bounds::edge(0, OFFSET_COLOR), bounds::edge(0, BASE_COLOR)}},
                 {bounds::edge(
                      3, OFFSET_COLOR,
                      EdgeCoordinate::Orientation::INCOMING_PERPENDICULAR),
                  {bounds::edge(3, BASE_COLOR),
                   bounds::edge(0, OFFSET_COLOR)}}}},
        InterferencePlaneEdgeBoundsCase{
            .name = "straightAnglePolygon_noInterference",
            .points = samples::straightAnglePolygon,
            .calculations = offset::single(
                offset::ring(RingVector<float>({-0.3, -0.2, 0, 0})), false),
            .bounds =
                {{bounds::edge(
                      0, OFFSET_COLOR,
                      EdgeCoordinate::Orientation::INCOMING_PERPENDICULAR),
                  {bounds::edge(0, OFFSET_COLOR),
                   bounds::edge(1, OFFSET_COLOR)}}}}),
    testing::PrintToStringParamName());
}  // namespace stl3lasercut
