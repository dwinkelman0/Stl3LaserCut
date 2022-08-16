// Copyright 2022 by Daniel Winkelman. All rights reserved.

#pragma once

#include <algo/Graph.h>
#include <gtest/gtest.h>
#include <stl3lasercut/Line.h>
#include <stl3lasercut/LoopPlane.h>
#include <stl3lasercut/Util.h>
#include <stl3lasercut/VertexConnectivityGraph.h>

#include <map>
#include <set>

namespace stl3lasercut {
class AssemblyPlane;
class LoopPlane;

/** An InterferencePlane performs offsets and dynamic intersection creation. */
class InterferencePlane {
  FRIEND_TEST(InterferenePlaneTests, Initialization);
  FRIEND_TEST(InterferenePlaneTests, ConstantOffset);

 private:
  enum class Orientation { PARALLEL, RIGHT_PERPENDICULAR, LEFT_PERPENDICULAR };

  struct EdgeCoordinate {
    uint32_t id;
    uint32_t color;
    Orientation orientation;

    bool operator<(const EdgeCoordinate &other) const;

    friend std::ostream &operator<<(std::ostream &os,
                                    const EdgeCoordinate &coord);
  };
  friend std::ostream &operator<<(std::ostream &os,
                                  const EdgeCoordinate &coord);

  class EdgeGroup {
   private:
    struct Comparator {
     public:
      Comparator(const std::shared_ptr<AssemblyPlane> &assemblyPlane,
                 const DirectedLine &line);

      bool operator()(const uint32_t a, const uint32_t b) const;

     private:
      std::shared_ptr<AssemblyPlane> assembly_;
      DirectedLine::PointComparator comparator_;
    };

   public:
    EdgeGroup(const std::shared_ptr<AssemblyPlane> &assemblyPlane,
              const DirectedLine &line);

    std::set<EdgeCoordinate>
        edges; /** Logical edges associated with this group. */
    DirectedLine line;
    std::set<uint32_t, Comparator>
        points; /** Points associated with this group, actual points are stored
                   in the AssemblyPlane. */

    friend std::ostream &operator<<(std::ostream &os, const EdgeGroup &group);
  };
  friend std::ostream &operator<<(std::ostream &os, const EdgeGroup &group);

  using Graph = algo::DirectedGraph<algo::Unit, std::shared_ptr<EdgeGroup>,
                                    MultiVertexConnectivityGraph>;

 public:
  using OffsetFunction =
      std::function<float(const std::shared_ptr<AssemblyPlane> &,
                          const std::shared_ptr<AssemblyPlane> &)>;

 public:
  InterferencePlane(const std::shared_ptr<AssemblyPlane> &assembly);

  void addLoopPlane(const std::shared_ptr<LoopPlane> &loopPlane);
  void applyOffsetFunction(const OffsetFunction &func, const uint32_t baseColor,
                           const uint32_t perpendicularColor,
                           const uint32_t newColor);

 private:
  void addParallelEdgesFromLoop(const LoopPlane::Loop &loop,
                                const uint32_t color);
  bool addPoint(const uint32_t index);
  void addEdge(const uint32_t v0, const uint32_t v1, const uint32_t edgeId,
               const uint32_t color);
  void addAngle(const uint32_t v0, const uint32_t v1, const uint32_t v2,
                const uint32_t e0, const uint32_t e1);

  void addParallelEdgeFromOffset(const EdgeCoordinate &coord, uint32_t newColor,
                                 const float offset);
  void addPerpendicularEdgeThroughPoint(const EdgeCoordinate &coord,
                                        const uint32_t point,
                                        const Orientation orientation);

  void computeInterferenceWithAdjacentEdges(const EdgeCoordinate &coord);
  void computeInterferenceWithColor(const EdgeCoordinate &coord,
                                    const uint32_t color);

  void findAndInsertIntersection(const std::shared_ptr<EdgeGroup> &a,
                                 const std::shared_ptr<EdgeGroup> &b);
  void insertVertexInEdgeGroup(const std::shared_ptr<EdgeGroup> &group,
                               const uint32_t vertex);

  void fixVertexConnectivity();
  bool areEdgesContinuous(const std::shared_ptr<EdgeGroup> &incoming,
                          const std::shared_ptr<EdgeGroup> &outgoing) const;

 private:
  std::shared_ptr<AssemblyPlane> assembly_;
  Graph graph_;
  std::map<EdgeCoordinate, std::shared_ptr<EdgeGroup>> edges_;
  std::map<DirectedLine, std::shared_ptr<EdgeGroup>> groupMap_;
  std::map<uint32_t, std::pair<uint32_t, uint32_t>>
      parallelEdgeAdjacency_; /** Maps an edge ID to the (lower, upper) adjacent
                                 edge IDs. */
  std::map<uint32_t, uint32_t> colorAdjacency_; /** Maps a color to the color
                                                   from which it is derived. */
};
}  // namespace stl3lasercut
