// Copyright 2022 by Daniel Winkelman. All rights reserved.

#pragma once

#include <algo/Graph.h>
#include <gtest/gtest.h>
#include <stl3lasercut/HierarchicalOrdering.h>
#include <stl3lasercut/Line.h>

#include <compare>

namespace stl3lasercut {
class AssemblyPlane;

/** A LoopPlane is a set of loops which are guaranteed not to interfere, i.e.
 * overlap. */
class LoopPlane {
  FRIEND_TEST(MeshTests, LoopPlane);

 private:
  using Graph =
      algo::DirectedGraph<algo::Unit, uint32_t, std::map<uint32_t, uint32_t>>;

 public:
  class Loop {
   public:
    struct Characteristic {
      uint32_t isPositive;
      uint32_t size;

      bool operator<(const Characteristic &other) const;
      bool operator==(const Characteristic &other) const;
    };

   public:
    Loop(const std::set<uint32_t> &vertexSet,
         const std::vector<BoundedLine> &bounds,
         const std::vector<uint32_t> &edges);

    std::pair<std::partial_ordering, bool> operator<=>(const Loop &other) const;
    bool isPositive() const;
    Characteristic getCharacteristic() const;

   private:
    bool contains(const Loop &other) const;

   private:
    std::set<uint32_t> vertexSet_;
    std::vector<BoundedLine> bounds_;
    std::vector<uint32_t> edges_;
  };

 public:
  using Characteristic = HierarchicalOrdering<LoopPlane::Loop>::Output<
      LoopPlane::Loop::Characteristic>;

 public:
  LoopPlane(const std::shared_ptr<AssemblyPlane> &assembly,
            const uint32_t color);

  uint32_t getColor() const;
  void foreachEdge(
      const std::function<void(const Graph::ConstEdge &)> &func) const;
  void foreachEdgePair(
      const std::function<void(const Graph::ConstEdge &,
                               const Graph::ConstEdge &)> &func) const;

  HierarchicalOrdering<Loop> getLoops() const;

  static Characteristic getCharacteristic(
      const HierarchicalOrdering<Loop> &ordering);

 private:
  std::shared_ptr<AssemblyPlane> assembly_;
  Graph graph_;
  uint32_t color_;
};

std::ostream &operator<<(std::ostream &os, const LoopPlane::Characteristic &ch);
}  // namespace stl3lasercut
