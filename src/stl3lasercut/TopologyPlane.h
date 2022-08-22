// Copyright 2022 by Daniel Winkelman. All rights reserved.

#pragma once

#include <algo/Graph.h>
#include <stl3lasercut/InterferencePlane.h>
#include <stl3lasercut/VertexConnectivityGraph.h>

#include <memory>

namespace stl3lasercut {
class TopologyPlane {
  friend class TopologyNode;

 private:
  struct Simplification {
    uint32_t source;
    uint32_t dest;
    uint32_t newVertex;
    std::set<EdgeCoordinate> edge;
  };

 private:
  using Graph = algo::DirectedGraph<algo::Unit, std::set<EdgeCoordinate>,
                                    MultiVertexConnectivityGraph::ExportSet>;

 public:
  TopologyPlane() {}
  TopologyPlane(const InterferencePlane &interferencePlane);
  void debug() const;

  std::map<uint32_t, Simplification> simplifyCycle();

  template <bool IsIncoming>
  std::set<uint32_t> getEdges(const uint32_t v0, const uint32_t v1,
                              const bool findIncoming) const;

  bool doVerticesOverlap(const uint32_t v0, const uint32_t v1) const;

  TopologyPlane::Simplification mergeVertices(const uint32_t v0,
                                              const uint32_t v1);

 private:
  std::shared_ptr<AssemblyPlane> assemblyPlane_;
  std::map<uint32_t, uint32_t> edgeAdjacency_;
  Graph graph_;
  std::map<uint32_t, Simplification> simplifications_;
  uint32_t vertexCounter_;
};

class TopologyNode : public std::enable_shared_from_this<TopologyNode> {
 public:
  TopologyNode(const TopologyPlane &topologyPlane);
  TopologyNode(const std::shared_ptr<TopologyNode> &parent,
               const TopologyPlane &topologyPlane);

 private:
  std::weak_ptr<TopologyNode> parent_;
  std::map<uint32_t, TopologyPlane::Simplification> reverseSimpMap_;
  std::map<uint32_t, uint32_t> forwardSimpMap_;
  std::set<uint32_t> simplifiedOriginalVertices_;

  std::optional<uint32_t> edgeId_;
  std::map<uint32_t, std::shared_ptr<TopologyNode>> children_;
};
}  // namespace stl3lasercut
