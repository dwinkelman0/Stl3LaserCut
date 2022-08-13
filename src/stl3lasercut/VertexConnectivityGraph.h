// Copyright 2022 by Daniel Winkelman. All rights reserved.

#pragma once

#include <algo/Graph.h>
#include <gtest/gtest.h>

namespace stl3lasercut {
class AssemblyPlane;

class VertexConnectivityGraph {
  FRIEND_TEST(MeshTests, AssemblyPlane);

 private:
  using Graph = algo::DirectedGraph<algo::Unit, algo::Unit, algo::Unit>;

 public:
  VertexConnectivityGraph(const std::shared_ptr<AssemblyPlane> &assemblyPlane,
                          const uint32_t centralVertex,
                          const bool enforceChains);

  void connect(const uint32_t v0, const uint32_t v1);
  uint32_t getFurthestConnection(const uint32_t v0) const;

 private:
  std::shared_ptr<AssemblyPlane> assembly_;
  Graph graph_;
  uint32_t centralVertex_;
  bool enforceChains_;
};
}  // namespace stl3lasercut
