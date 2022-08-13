// Copyright 2022 by Daniel Winkelman. All rights reserved.

#include "VertexConnectivityGraph.h"

namespace stl3lasercut {
VertexConnectivityGraph::VertexConnectivityGraph(
    const std::shared_ptr<AssemblyPlane> &assemblyPlane,
    const uint32_t centralVertex, const bool enforceChains)
    : assembly_(assemblyPlane),
      centralVertex_(centralVertex),
      enforceChains_(enforceChains) {}

void VertexConnectivityGraph::connect(const uint32_t v0, const uint32_t v1) {
  graph_.emplaceVertex(v0);
  graph_.emplaceVertex(v1);
  graph_.emplaceEdge(v0, v1);
  if (enforceChains_ && (graph_.getEdgesFromVertex(v0).getCount() > 1 ||
                         graph_.getEdgesToVertex(v1).getCount() > 1)) {
    throw std::runtime_error("Chain invariant violated");
  }
}

uint32_t VertexConnectivityGraph::getFurthestConnection(
    const uint32_t v0) const {
  if (!enforceChains_) {
    throw std::runtime_error(
        "This method is not guaranteed to work without enforcing chains.");
  }
  if (graph_.getEdgesToVertex(v0).getCount() != 0) {
    throw std::runtime_error("This vertex is messed up.");
  }
  uint32_t terminalVertex = std::numeric_limits<uint32_t>::max();
  graph_.traverseDepthFirst(
      [this, &terminalVertex](const auto &vertex) {
        if (graph_.getEdgesFromVertex(vertex).getCount() > 1) {
          throw std::runtime_error("This vertex is messed up.");
        }
        terminalVertex = vertex.getIndex();
      },
      [](const auto &) {}, v0);
  return terminalVertex;
}
}  // namespace stl3lasercut
