// Copyright 2022 by Daniel Winkelman. All rights reserved.

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <stl3lasercut/AssemblyPlane.h>
#include <stl3lasercut/VertexConnectivityGraph.h>

namespace stl3lasercut {
class VertexConnectivityGraphTest : public testing::Test {
 private:
  static std::shared_ptr<AssemblyPlane> makeAssemblyPlane() {
    auto assembly =
        std::make_shared<AssemblyPlane>(nullptr, 0, Projector3D::nullProjector);
    assembly->pointLookup_({2, 2});

    // Points are ordered counterclockwise so that the lines they generate are
    // increasing by DirectedLine::AngularComparator<false>
    assembly->pointLookup_({6, 1});
    assembly->pointLookup_({2, 0});
    assembly->pointLookup_({0, 0});
    assembly->pointLookup_({0, 2});
    assembly->pointLookup_({1, 3});
    assembly->pointLookup_({1, 4});
    assembly->pointLookup_({2, 3});
    assembly->pointLookup_({5, 4});
    assembly->pointLookup_({5, 3});
    assembly->pointLookup_({3, 2});

    assembly->pointLookup_({8, 6});
    assembly->pointLookup_({8, 4});
    assembly->pointLookup_({4, 2});
    return assembly;
  }

 public:
  VertexConnectivityGraphTest()
      : assembly_(makeAssemblyPlane()), graph_(assembly_, 0) {}

  void checkComponent(const uint32_t vertex, const uint32_t numVertices) {
    auto it = graph_.components_.find(vertex);
    ASSERT_NE(it, graph_.components_.end()) << "vertex " << vertex;
    ASSERT_EQ(it->second.size(), numVertices) << "vertex " << vertex;
  }

  void checkPointsReachable(
      const std::vector<std::pair<uint32_t, bool>> &points) {
    for (auto it = points.begin(), end = points.end(); it != end; ++it) {
      if (it->second) {
        std::vector<uint32_t> outgoing;
        for (auto jt = it; jt != end; ++jt) {
          if (!jt->second) {
            outgoing.push_back(jt->first);
          }
        }
        MultiVertexConnectivityGraph::ReachablePointSet reachableSet =
            graph_.getReachablePoints(it->first);
        std::vector<uint32_t> reachable(reachableSet.begin(),
                                        reachableSet.end());
        ASSERT_THAT(reachable, ::testing::ContainerEq(outgoing));
      }
    }
  }

  void checkUnconnected(const uint32_t numVertices) {
    ASSERT_EQ(graph_.unconnected_.size(), numVertices);
  }

 protected:
  std::shared_ptr<AssemblyPlane> assembly_;
  MultiVertexConnectivityGraph graph_;
};

TEST_F(VertexConnectivityGraphTest, ConnectBasic) {
  graph_.connect(1, 2);
  graph_.connect(3, 4);

  checkComponent(1, 2);
  checkComponent(3, 2);
  checkPointsReachable({{1, true}, {2, false}});
  checkPointsReachable({{3, true}, {4, false}});
  checkUnconnected(0);
}

TEST_F(VertexConnectivityGraphTest, ConnectEqual) {
  graph_.connect(7, 9);
  graph_.connect(7, 9);
  checkPointsReachable({{7, true}, {9, false}});
}

TEST_F(VertexConnectivityGraphTest, AddVertexThenConnect) {
  graph_.addVertex(2, false);
  graph_.addVertex(2, false);
  graph_.addVertex(2, false);
  graph_.addVertex(3, true);
  graph_.addVertex(6, false);
  graph_.addVertex(7, true);
  graph_.addVertex(9, true);
  checkUnconnected(5);

  graph_.connect(1, 2);
  graph_.connect(3, 4);
  graph_.connect(5, 7);

  checkComponent(1, 2);
  checkComponent(3, 2);
  checkComponent(5, 4);
  checkPointsReachable({{1, true}, {2, false}});
  checkPointsReachable({{3, true}, {4, false}});
  checkPointsReachable({{5, true}, {6, false}, {7, true}, {7, false}});
  checkUnconnected(1);
}

TEST_F(VertexConnectivityGraphTest, ConnectMergeOverlap) {
  graph_.connect(1, 3);
  graph_.connect(6, 8);
  graph_.connect(2, 4);
  graph_.connect(5, 7);

  checkComponent(1, 4);
  checkComponent(5, 4);
  checkPointsReachable({{1, true}, {2, true}, {3, false}, {4, false}});
  checkPointsReachable({{5, true}, {6, true}, {7, false}, {8, false}});
  checkUnconnected(0);
}

TEST_F(VertexConnectivityGraphTest, ConnectMergeTangent) {
  graph_.connect(1, 2);
  graph_.connect(5, 6);
  graph_.connect(7, 8);
  graph_.connect(1, 3);
  graph_.connect(4, 6);
  graph_.connect(8, 9);

  checkComponent(1, 3);
  checkComponent(4, 3);
  checkComponent(7, 4);
  checkPointsReachable({{1, true}, {2, false}, {3, false}});
  checkPointsReachable({{4, true}, {5, true}, {6, false}});
  checkPointsReachable({{7, true}, {8, true}, {8, false}, {9, false}});
}

TEST_F(VertexConnectivityGraphTest, ConnectMergeEngulf) {
  graph_.connect(1, 5);
  graph_.addVertex(3, true);
  graph_.connect(9, 6);

  checkComponent(9, 5);
  checkPointsReachable(
      {{9, true}, {1, true}, {3, true}, {5, false}, {6, false}});
}

TEST_F(VertexConnectivityGraphTest, ConnectMergeFullCircle) {
  graph_.connect(1, 6);
  graph_.connect(5, 2);

  checkComponent(1, 4);
  checkPointsReachable({{1, true}, {2, false}, {5, true}, {6, false}});
}

TEST_F(VertexConnectivityGraphTest, Rename) {
  graph_.addVertex(1, false);
  graph_.addVertex(2, true);
  graph_.connect(1, 3);
  checkPointsReachable({{1, true}, {1, false}, {2, true}, {3, false}});

  graph_.rename(1, 11);
  graph_.rename(2, 12);
  graph_.rename(3, 13);

  checkComponent(11, 4);
  checkPointsReachable({{11, true}, {11, false}, {12, true}, {13, false}});
  checkUnconnected(0);
}
}  // namespace stl3lasercut
