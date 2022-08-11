// Copyright 2022 by Daniel Winkelman. All rights reserved.

#include "Mesh.h"

#include <stl3lasercut/AssemblyPlane.h>

namespace stl3lasercut {
std::ostream &operator<<(std::ostream &os, const Mesh::Characteristic &ch) {
  os << ch.planes << " planes, " << ch.vertices << " vertices, " << ch.edges
     << " edges";
  return os;
}

Mesh::Mesh() : planeIdCounter_(0) {}

void Mesh::addTriangle(const StlTriangle &triangle) {
  // Check whether the associated projector is already present
  Projector3D projector = triangle.getProjector();
  const auto &[it, success] = planes_.emplace(
      projector,
      std::make_shared<AssemblyPlane>(*this, ++planeIdCounter_, projector));
  projector = it->first;
  std::shared_ptr<AssemblyPlane> plane = it->second;

  // Add vertices to (or retrieve vertices from) this graph
  uint32_t v0 = addVertex(std::get<0>(triangle.getVertices()));
  uint32_t v1 = addVertex(std::get<1>(triangle.getVertices()));
  uint32_t v2 = addVertex(std::get<2>(triangle.getVertices()));

  // Connect vertices
  addEdge(plane, v0, v1);
  addEdge(plane, v1, v2);
  addEdge(plane, v2, v0);

  // Update underlying plane
  plane->addTriangle(
      plane->registerPoint(v0, std::get<0>(triangle.getVertices())),
      plane->registerPoint(v1, std::get<1>(triangle.getVertices())),
      plane->registerPoint(v2, std::get<2>(triangle.getVertices())));
}

Mesh::Characteristic Mesh::getCharacteristic() const {
  if (isValid()) {
    uint32_t numEdges = 0;
    for (const Graph::ConstEdge &edge : graph_.getEdges()) {
      if (Graph::unwrap(graph_.getEdge(edge.getDest(), edge.getSource()))
              .getValue() != edge.getValue()) {
        ++numEdges;
      }
    }
    return Characteristic{.planes = static_cast<uint32_t>(planes_.size()),
                          .vertices = graph_.getVertices().getCount(),
                          .edges = numEdges / 2};
  } else {
    throw std::runtime_error("Cannot get characteristic of invalid mesh.");
  }
}

bool Mesh::isValid() const {
  return std::all_of(graph_.getVertices().begin(), graph_.getVertices().end(),
                     [this](const Graph::ConstVertex &vertex) {
                       // Vertices are balanced and have sufficient edges
                       // to/from
                       uint32_t numIncomingEdges =
                           graph_.getEdgesToVertex(vertex).getCount();
                       uint32_t numOutgoingEdges =
                           graph_.getEdgesFromVertex(vertex).getCount();
                       return numIncomingEdges >= 3 &&
                              numIncomingEdges == numOutgoingEdges;
                     }) &&
         std::all_of(graph_.getEdges().begin(), graph_.getEdges().end(),
                     [this](const Graph::ConstEdge &edge) {
                       // Edges are paired and have an associated plane
                       return graph_.getEdge(edge.getDest(),
                                             edge.getSource()) &&
                              edge.getValue();
                     });
}

uint32_t Mesh::addVertex(const Vec3 &point) {
  uint32_t index = vertexLookup_(point);
  graph_.emplaceVertex(index);
  return index;
}

void Mesh::addEdge(const std::shared_ptr<AssemblyPlane> &plane,
                   const uint32_t v0, const uint32_t v1) {
  auto [it, success] = graph_.emplaceEdge(v0, v1);
  if (!success) {
    throw std::runtime_error("This edge already exists.");
  }
  it->getValue() = plane;
}
}  // namespace stl3lasercut
