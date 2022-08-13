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

 private:
  enum class Orientation { PARALLEL, RIGHT_PERPENDICULAR, LEFT_PERPENDICULAR };
  enum class OrientationClass { PARALLEL, PERPENDICULAR };

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
  };

  struct LogicalEdge {
    std::shared_ptr<EdgeGroup> group;
    OrientationClass orientationClass;
    uint32_t edgeId;
    uint32_t lower,
        upper; /** If orientationClass is PARALLEL, then these represent ids of
                  adjacent edges; if orientationClass is PERPENDICULAR, then
                  these represent colors of edges with same id. */

    friend std::ostream &operator<<(std::ostream &os, const LogicalEdge &edge);
  };
  friend std::ostream &operator<<(std::ostream &os, const LogicalEdge &edge);

  using Graph = algo::DirectedGraph<algo::Unit, std::shared_ptr<EdgeGroup>,
                                    VertexConnectivityGraph>;

 public:
  InterferencePlane(const std::shared_ptr<AssemblyPlane> &assembly);

  void addLoopPlane(const std::shared_ptr<LoopPlane> &loopPlane);

 private:
  void addParallelEdgesFromLoop(const LoopPlane::Loop &loop,
                                const uint32_t color);
  void addPoint(const uint32_t index);
  void addEdge(const uint32_t v0, const uint32_t v1, const uint32_t edgeId,
               const uint32_t color);
  void addAngle(const uint32_t v0, const uint32_t v1, const uint32_t v2,
                const uint32_t e0, const uint32_t e1, const uint32_t color);

 private:
  std::shared_ptr<AssemblyPlane> assembly_;
  Graph graph_;
  std::map<EdgeCoordinate, LogicalEdge> edges_;
  std::map<DirectedLine, std::shared_ptr<EdgeGroup>> groupMap_;
};
}  // namespace stl3lasercut