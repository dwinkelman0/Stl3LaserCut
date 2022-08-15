// Copyright 2022 by Daniel Winkelman. All rights reserved.

#pragma once

#include <algo/Graph.h>
#include <gtest/gtest.h>
#include <stl3lasercut/Line.h>

namespace stl3lasercut {
class AssemblyPlane;

class ChainedVertexConnectivityGraph {
  FRIEND_TEST(MeshTests, AssemblyPlane);

 private:
  using Graph = algo::DirectedGraph<algo::Unit, algo::Unit, algo::Unit>;

 public:
  void connect(const uint32_t v0, const uint32_t v1);
  uint32_t getFurthestConnection(const uint32_t v0) const;

  friend std::ostream &operator<<(std::ostream &os,
                                  const ChainedVertexConnectivityGraph &graph);

 private:
  Graph graph_;
};

class MultiVertexConnectivityGraph {
  friend class VertexConnectivityGraphTest;

 private:
  class AngularComparator {
   public:
    /** Must be constructed from an outgoing edge. */
    AngularComparator(std::shared_ptr<const AssemblyPlane> assemblyPlane,
                      const uint32_t centralVertex, const uint32_t basisVertex);
    bool operator()(const uint32_t a, const uint32_t b) const;
    bool operator()(const std::pair<uint32_t, bool> &a,
                    const std::pair<uint32_t, bool> &b) const;

   private:
    DirectedLine getLineFromPoint(const uint32_t vertex) const;

   private:
    std::shared_ptr<const AssemblyPlane> assembly_;
    Vec2 centralPoint_;
    DirectedLine::AngularComparator<false> comparator_;
  };

  using ComponentMap =
      std::map<uint32_t,
               std::set<std::pair<uint32_t, bool>, AngularComparator>>;

 public:
  using ReachablePointSet = std::set<uint32_t, AngularComparator>;

 public:
  MultiVertexConnectivityGraph(
      const std::shared_ptr<const AssemblyPlane> &assemblyPlane,
      const uint32_t centralVertex);

  bool connect(const uint32_t v0, const uint32_t v1);
  bool addVertex(const uint32_t v0, const bool isIncoming);
  void rename(const uint32_t v0, const uint32_t v1);
  ReachablePointSet getReachablePoints(const uint32_t v0);

  friend std::ostream &operator<<(std::ostream &os,
                                  const MultiVertexConnectivityGraph &graph);

 private:
  bool componentContainsPoint(const ComponentMap::const_iterator &it,
                              const uint32_t vertex) const;
  void rename(const uint32_t v0, const uint32_t v1, const bool isIncoming);

 private:
  std::shared_ptr<const AssemblyPlane> assembly_;
  uint32_t centralVertex_;
  std::set<std::pair<uint32_t, bool>> unconnected_;
  ComponentMap components_;
  bool fullCircle_;
};
}  // namespace stl3lasercut
