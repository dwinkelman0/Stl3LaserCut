// Copyright 2022 by Daniel Winkelman. All rights reserved.

#include "LoopPlane.h"

#include <stl3lasercut/AssemblyPlane.h>

namespace stl3lasercut {
LoopPlane::LoopPlane(const std::shared_ptr<const AssemblyPlane> &assembly,
                     const uint32_t color)
    : assembly_(assembly), color_(color) {
  // Gather all unmatched edges from the AssemblyPlane
  for (const typename AssemblyPlane::Graph::ConstEdge &edge :
       assembly_->graph_.getEdges()) {
    if (!assembly_->graph_.getEdge(edge.getDest(), edge.getSource())) {
      // Place vertices
      graph_.emplaceVertex(edge.getSource());
      Graph::VertexIterator vertexIt =
          graph_.emplaceVertex(edge.getDest()).first;

      // Place edge
      graph_.emplaceEdge(edge.getSource(), edge.getDest()).first->getValue() =
          edge.getValue();

      // Complete vertex connectivity (distinguish positive and negative areas)
      const AssemblyPlane::VertexConnectivityGraph &connectivityGraph =
          AssemblyPlane::Graph::unwrap(
              assembly_->graph_.getVertex(edge.getDest()))
              .getValue();
      if (connectivityGraph.getEdgesToVertex(edge.getSource()).getCount() !=
              0 ||
          connectivityGraph.getEdgesFromVertex(edge.getSource()).getCount() !=
              1) {
        throw std::runtime_error("This vertex is messed up.");
      }
      uint32_t terminalVertex = std::numeric_limits<uint32_t>::max();
      connectivityGraph.traverseDepthFirst(
          [&terminalVertex](const auto &vertex) {
            terminalVertex = vertex.getIndex();
          },
          [](const auto &) {}, edge.getSource());

      vertexIt->getValue().emplace(edge.getSource(), terminalVertex);
    }
  }
}
}  // namespace stl3lasercut
