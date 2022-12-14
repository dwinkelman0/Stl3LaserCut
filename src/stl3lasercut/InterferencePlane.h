// Copyright 2022 by Daniel Winkelman. All rights reserved.

#pragma once

#include <algo/Graph.h>
#include <gtest/gtest.h>
#include <stl3lasercut/Line.h>
#include <stl3lasercut/LoopPlane.h>
#include <stl3lasercut/RangeUnion.h>
#include <stl3lasercut/Util.h>
#include <stl3lasercut/VertexConnectivityGraph.h>

#include <map>
#include <set>

namespace stl3lasercut {
class AssemblyPlane;
class LoopPlane;

class EdgeCoordinate {
 public:
  enum class Orientation {
    PARALLEL,
    INCOMING_PERPENDICULAR,
    OUTGOING_PERPENDICULAR
  };

 public:
  uint32_t id;
  uint32_t color;
  Orientation orientation;

  EdgeCoordinate(const uint32_t id, const uint32_t color,
                 const Orientation orientation)
      : id(id), color(color), orientation(orientation) {}

  bool operator<(const EdgeCoordinate &other) const;
  bool operator==(const EdgeCoordinate &other) const;

  friend std::ostream &operator<<(std::ostream &os,
                                  const EdgeCoordinate &coord);
};

/** An InterferencePlane performs offsets and dynamic intersection creation. */
class InterferencePlane {
  friend class TopologyPlane;
  friend class DesmosOutput;
  friend class InterferencePlaneEdgeBounds;
  FRIEND_TEST(InterferencePlaneSetup, Setup);
  FRIEND_TEST(InterferencePlaneOffset, Offset);
  FRIEND_TEST(InterferencePlaneEdgeBounds, Bounds);

 private:
  struct Comparator {
   public:
    Comparator(const std::shared_ptr<AssemblyPlane> &assemblyPlane,
               const DirectedLine &line);

    bool operator()(const uint32_t a, const uint32_t b) const;
    bool lessThanOrEqual(const uint32_t a, const uint32_t b) const;
    bool greaterThanOrEqual(const uint32_t a, const uint32_t b) const;

   private:
    std::shared_ptr<AssemblyPlane> assembly_;
    DirectedLine::PointComparator comparator_;
  };

  class EdgeGroup {
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

  struct KnownIntersectionsComparator {
   public:
    bool operator()(const std::pair<EdgeCoordinate, EdgeCoordinate> &a,
                    const std::pair<EdgeCoordinate, EdgeCoordinate> &b) const;

   private:
    static std::pair<EdgeCoordinate, EdgeCoordinate> getCanonicalOrder(
        const std::pair<EdgeCoordinate, EdgeCoordinate> &a);
  };

  using Range = RangeUnion<uint32_t, Comparator>;

  using Graph = algo::DirectedGraph<algo::Unit, std::shared_ptr<EdgeGroup>,
                                    MultiVertexConnectivityGraph>;

 public:
  using OffsetFunction =
      std::function<float(const std::shared_ptr<AssemblyPlane> &,
                          const std::shared_ptr<AssemblyPlane> &)>;

  struct OffsetCalculation {
    OffsetFunction function;
    uint32_t baseColor;
    uint32_t perpendicularColor;
    uint32_t newColor;
    bool calculateInterference;
  };

 public:
  InterferencePlane(const std::shared_ptr<AssemblyPlane> &assembly);

  void addLoopPlane(const std::shared_ptr<LoopPlane> &loopPlane);
  void applyOffsetFunctions(const std::vector<OffsetCalculation> &offsets);
  void finalize();

 private:
  void applyOffsetFunction(const OffsetFunction &func, const uint32_t baseColor,
                           const uint32_t perpendicularColor,
                           const uint32_t newColor,
                           const bool calculateInterference);
  void addParallelEdgesFromLoop(const LoopPlane::Loop &loop,
                                const uint32_t color);
  bool addPoint(const uint32_t index);
  void addEdge(const uint32_t v0, const uint32_t v1, const uint32_t edgeId,
               const uint32_t color);
  void addAngle(const uint32_t v0, const uint32_t v1, const uint32_t v2,
                const uint32_t e0, const uint32_t e1, const uint32_t color);

  void addParallelEdgeFromOffset(const EdgeCoordinate &coord, uint32_t newColor,
                                 const float offset,
                                 const bool calculateInterference);
  void addPerpendicularEdgesAtIntersection(const uint32_t incoming,
                                           const uint32_t outgoing,
                                           const uint32_t baseColor,
                                           const uint32_t perpendicularColor,
                                           const uint32_t newColor,
                                           const bool calculateInterference);
  void addPerpendicularEdgeThroughPoint(
      const uint32_t vertex, const bool isIncoming, const bool isNegativeAngle,
      const uint32_t id, const uint32_t baseColor, const uint32_t newColor,
      const bool calculateInterference);

  void computeInterferenceWithAdjacentEdges(const EdgeCoordinate &coord);
  void computeInterferenceWithColor(const EdgeCoordinate &coord,
                                    const uint32_t color);
  void findAndInsertGroupIntersection(const std::shared_ptr<EdgeGroup> &a,
                                      const std::shared_ptr<EdgeGroup> &b);
  void insertVertexInEdgeGroup(const std::shared_ptr<EdgeGroup> &group,
                               const uint32_t vertex);
  std::optional<uint32_t> findGroupIntersection(
      const std::shared_ptr<EdgeGroup> &a,
      const std::shared_ptr<EdgeGroup> &b) const;

  void fixVertexConnectivity();
  bool areEdgesContinuous(const std::shared_ptr<EdgeGroup> &incoming,
                          const std::shared_ptr<EdgeGroup> &outgoing) const;
  bool pruneVertices();

  void calculateEdgeBounds();
  bool restrictEdgeBounds();

 private:
  std::shared_ptr<AssemblyPlane> assembly_;
  Graph graph_;
  std::map<EdgeCoordinate, std::shared_ptr<EdgeGroup>> edges_;
  std::map<DirectedLine, std::shared_ptr<EdgeGroup>> groupMap_;
  std::map<uint32_t, std::pair<uint32_t, uint32_t>>
      edgeAdjacency_; /** Maps an edge ID to the (lower, upper) adjacent
                                 edge IDs. */
  std::map<uint32_t, std::pair<uint32_t, bool>>
      colorAdjacency_; /** Maps a color to the color from which it is derived */
  std::map<std::pair<EdgeCoordinate, EdgeCoordinate>, uint32_t,
           KnownIntersectionsComparator>
      knownIntersections_; /** Keep track of known intersections to help when
                              two adjacent edges belong to the same group. Not
                              meant to be exhaustive because intersections can
                              be inferred in other ways in most cases. */
  std::map<EdgeCoordinate, Range> edgeBounds_;
};
}  // namespace stl3lasercut
