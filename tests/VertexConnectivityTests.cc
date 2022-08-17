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

  void checkComponent(const uint32_t vertex, const uint32_t numVertices) const {
    auto it = graph_.components_.find(vertex);
    ASSERT_NE(it, graph_.components_.end()) << "vertex " << vertex;
    ASSERT_EQ(it->second.size(), numVertices) << "vertex " << vertex;
  }

  template <bool IsForward>
  void checkPointsReachableGeneric(
      const std::vector<std::pair<uint32_t, bool>> &points) const {
    for (auto it = points.begin(), end = points.end(); it != end; ++it) {
      if (!IsForward ^ it->second) {
        std::vector<uint32_t> outgoing;
        for (auto jt = it; jt != end; ++jt) {
          if (IsForward ^ jt->second) {
            outgoing.push_back(jt->first);
          }
        }
        auto reachableSet = graph_.getReachablePoints<IsForward>(it->first);
        std::vector<uint32_t> reachable(reachableSet.begin(),
                                        reachableSet.end());
        ASSERT_THAT(reachable, ::testing::ContainerEq(outgoing))
            << "vertex " << it->first << (IsForward ? " forward" : " backward");
      }
    }
  }

  void checkPointsReachable(
      const std::vector<std::pair<uint32_t, bool>> &points) const {
    checkPointsReachableGeneric<true>(points);
    auto reversed(points);
    std::reverse(reversed.begin(), reversed.end());
    checkPointsReachableGeneric<false>(reversed);
  }

  template <bool IsForward>
  void checkPointsReachableFromEverywhereGeneric(
      const std::set<uint32_t> &input, const std::set<uint32_t> &output) const {
    for (const uint32_t a : input) {
      std::set<uint32_t,
               MultiVertexConnectivityGraph::AngularComparator<IsForward>>
          expected(MultiVertexConnectivityGraph::AngularComparator<IsForward>(
              assembly_, 0, a));
      expected.insert(output.begin(), output.end());
      auto reachable = graph_.getReachablePoints<IsForward>(a);
      ASSERT_THAT(reachable, ::testing::ContainerEq(expected));
    }
  }

  void checkPointsReachableFromEverywhere(const std::set<uint32_t> &incoming,
                                          const std::set<uint32_t> &outgoing) {
    checkPointsReachableFromEverywhereGeneric<true>(incoming, outgoing);
    checkPointsReachableFromEverywhereGeneric<false>(outgoing, incoming);
  }

  void checkUnconnected(const uint32_t numVertices) const {
    ASSERT_EQ(graph_.unconnected_.size(), numVertices);
  }

 protected:
  std::shared_ptr<AssemblyPlane> assembly_;
  MultiVertexConnectivityGraph graph_;
};

TEST_F(VertexConnectivityGraphTest, ConnectBasic) {
  ASSERT_TRUE(graph_.connect(1, 2));
  ASSERT_TRUE(graph_.connect(3, 4));

  checkComponent(1, 2);
  checkComponent(3, 2);
  checkPointsReachable({{1, true}, {2, false}});
  checkPointsReachable({{3, true}, {4, false}});
  checkUnconnected(0);
}

TEST_F(VertexConnectivityGraphTest, ConnectEqual) {
  ASSERT_TRUE(graph_.connect(7, 9));
  ASSERT_FALSE(graph_.connect(7, 9));
  checkPointsReachable({{7, true}, {9, false}});
}

TEST_F(VertexConnectivityGraphTest, AddVertexThenConnect) {
  ASSERT_TRUE(graph_.addVertex(2, false));
  ASSERT_FALSE(graph_.addVertex(2, false));
  ASSERT_TRUE(graph_.addVertex(3, true));
  graph_.addVertex(6, false);
  graph_.addVertex(7, true);
  graph_.addVertex(9, true);
  checkUnconnected(5);

  ASSERT_TRUE(graph_.connect(1, 2));
  ASSERT_TRUE(graph_.connect(3, 4));
  ASSERT_TRUE(graph_.connect(5, 7));

  checkComponent(1, 2);
  checkComponent(3, 2);
  checkComponent(5, 4);
  checkPointsReachable({{1, true}, {2, false}});
  checkPointsReachable({{3, true}, {4, false}});
  checkPointsReachable({{5, true}, {6, false}, {7, true}, {7, false}});
  checkUnconnected(1);
}

TEST_F(VertexConnectivityGraphTest, ConnectMergeOverlap) {
  ASSERT_TRUE(graph_.connect(1, 3));
  ASSERT_TRUE(graph_.connect(6, 8));
  ASSERT_TRUE(graph_.connect(2, 4));
  ASSERT_TRUE(graph_.connect(5, 7));

  checkComponent(1, 4);
  checkComponent(5, 4);
  checkPointsReachable({{1, true}, {2, true}, {3, false}, {4, false}});
  checkPointsReachable({{5, true}, {6, true}, {7, false}, {8, false}});
  checkUnconnected(0);
}

TEST_F(VertexConnectivityGraphTest, ConnectMergeTangent) {
  ASSERT_TRUE(graph_.connect(1, 2));
  ASSERT_TRUE(graph_.connect(5, 6));
  ASSERT_TRUE(graph_.connect(7, 8));
  ASSERT_TRUE(graph_.connect(1, 3));
  ASSERT_TRUE(graph_.connect(4, 6));
  ASSERT_TRUE(graph_.connect(8, 9));

  checkComponent(1, 3);
  checkComponent(4, 3);
  checkComponent(7, 4);
  checkPointsReachable({{1, true}, {2, false}, {3, false}});
  checkPointsReachable({{4, true}, {5, true}, {6, false}});
  checkPointsReachable({{7, true}, {8, true}, {8, false}, {9, false}});
}

TEST_F(VertexConnectivityGraphTest, ConnectMergeTangentSandwich) {
  ASSERT_TRUE(graph_.connect(2, 4));
  ASSERT_TRUE(graph_.connect(5, 8));
  ASSERT_TRUE(graph_.connect(4, 5));

  checkComponent(2, 6);
  checkPointsReachable(
      {{2, true}, {4, true}, {4, false}, {5, true}, {5, false}, {8, false}});
}

TEST_F(VertexConnectivityGraphTest, ConnectMergeEngulf) {
  ASSERT_TRUE(graph_.connect(1, 5));
  ASSERT_TRUE(graph_.addVertex(3, true));
  ASSERT_TRUE(graph_.connect(9, 6));

  checkComponent(9, 5);
  checkPointsReachable(
      {{9, true}, {1, true}, {3, true}, {5, false}, {6, false}});
}

TEST_F(VertexConnectivityGraphTest, ConnectMergeTangentFullCircle) {
  ASSERT_TRUE(graph_.connect(1, 5));
  ASSERT_TRUE(graph_.connect(5, 1));

  checkComponent(1, 4);
  checkPointsReachableFromEverywhere({1, 5}, {1, 5});
}

TEST_F(VertexConnectivityGraphTest, ConnectMergeOverlapFullCircle) {
  ASSERT_TRUE(graph_.connect(1, 6));
  ASSERT_TRUE(graph_.connect(5, 2));

  checkComponent(1, 4);
  checkPointsReachableFromEverywhere({1, 5}, {2, 6});
}

TEST_F(VertexConnectivityGraphTest, ConnectMergeStraightLineEdgeCase) {
  ASSERT_TRUE(graph_.connect(2, 7));
  ASSERT_TRUE(graph_.connect(10, 4));
  ASSERT_TRUE(graph_.connect(4, 10));

  checkComponent(10, 6);
  checkPointsReachableFromEverywhere({10, 2, 4}, {4, 10, 7});
}

TEST_F(VertexConnectivityGraphTest, Rename) {
  graph_.addVertex(1, false);
  graph_.addVertex(2, true);
  graph_.connect(1, 3);
  checkComponent(1, 3);
  checkPointsReachable({{1, true}, {2, true}, {3, false}});

  graph_.rename(1, 11);
  graph_.rename(2, 12);
  graph_.rename(3, 13);

  checkComponent(11, 3);
  checkPointsReachable({{11, true}, {12, true}, {13, false}});
  checkUnconnected(1);
}
}  // namespace stl3lasercut
