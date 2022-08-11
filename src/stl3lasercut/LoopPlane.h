// Copyright 2022 by Daniel Winkelman. All rights reserved.

#pragma once

#include <algo/Graph.h>
#include <gtest/gtest.h>

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
  LoopPlane(const std::shared_ptr<const AssemblyPlane> &assembly,
            const uint32_t color);

 private:
  std::shared_ptr<const AssemblyPlane> assembly_;
  Graph graph_;
  uint32_t color_;
};
}  // namespace stl3lasercut
