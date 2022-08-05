// Copyright 2022 by Daniel Winkelman. All rights reserved.

#include "Mesh.h"

namespace stl3lasercut {
Plane::Plane(const Vec3 &normal) : normal_(normal) {}

bool Plane::addEdge(const Projector3D &projector, const Vec3 &point,
                    const uint32_t v0, const uint32_t v1, const uint32_t v2,
                    const std::shared_ptr<Plane> &adjacentPlane) {
  graph_.emplaceVertex(v0);
  Graph::VertexIterator vertexIt = graph_.emplaceVertex(v1).first;
  vertexIt->getValue().mappedPoint = projector.normalize(point);
  vertexIt->getValue().vertexConnectivity.emplaceVertex(v0);
  vertexIt->getValue().vertexConnectivity.emplaceVertex(v2);
  vertexIt->getValue().vertexConnectivity.emplaceEdge(v0, v2);
  auto existingEdge = graph_.getEdge(v1, v0);
  if (existingEdge) {
    graph_.eraseEdge(*existingEdge);
    return true;
  } else {
    Graph::EdgeIterator edgeIt = graph_.emplaceEdge(v0, v1).first;
    if (adjacentPlane) {
      edgeIt->getValue().otherPlane = adjacentPlane;
      std::optional<Graph::EdgeIterator> otherEdgeIt =
          adjacentPlane->graph_.getEdge(v1, v0);
      assert(otherEdgeIt);
      (*otherEdgeIt)->getValue().otherPlane = shared_from_this();
    } else {
      edgeIt->getValue().otherPlane = nullptr;
    }
    return false;
  }
}

std::pair<uint32_t, uint32_t> Plane::getCharacteristic() const {
  return {graph_.getVertices().getCount(), graph_.getEdges().getCount()};
}

Mesh &Mesh::operator<<(const StlTriangle &triangle) {
  // Check whether the associated projector is already present
  Projector3D projector = triangle.getProjector();
  const auto &[it, success] =
      planes_.emplace(projector, std::make_shared<Plane>(triangle.getNormal()));
  projector = it->first;
  std::shared_ptr<Plane> plane = it->second;

  // Gather vertices
  uint32_t v0 = vertices_(std::get<0>(triangle.getVertices()));
  uint32_t v1 = vertices_(std::get<1>(triangle.getVertices()));
  uint32_t v2 = vertices_(std::get<2>(triangle.getVertices()));

  // Add each edge
  addEdge(plane, projector, std::get<1>(triangle.getVertices()), v0, v1, v2);
  addEdge(plane, projector, std::get<2>(triangle.getVertices()), v1, v2, v0);
  addEdge(plane, projector, std::get<0>(triangle.getVertices()), v2, v0, v1);

  return *this;
}

std::pair<uint32_t, uint32_t> Mesh::getCharacteristic() const {
  return {mesh_.getVertices().getCount(), mesh_.getEdges().getCount()};
}

const std::map<Projector3D, std::shared_ptr<Plane>> &Mesh::getPlanes() const {
  return planes_;
}

Vec3 Mesh::getVertexVector(const uint32_t index) const {
  return vertices_(index);
}

void Mesh::addEdge(const std::shared_ptr<Plane> &plane,
                   const Projector3D &projector, const Vec3 &point,
                   const uint32_t v0, const uint32_t v1, const uint32_t v2) {
  mesh_.emplaceVertex(v0);
  mesh_.emplaceVertex(v1);
  mesh_.emplaceEdge(v0, v1).first->getValue() = plane;
  std::optional<MeshGraph::EdgeIterator> adjacentEdge = mesh_.getEdge(v1, v0);
  if (plane->addEdge(projector, point, v0, v1, v2,
                     adjacentEdge ? (*adjacentEdge)->getValue() : nullptr)) {
    mesh_.eraseEdge(std::pair<uint32_t, uint32_t>(v0, v1));
    mesh_.eraseEdge(std::pair<uint32_t, uint32_t>(v1, v0));
  }
}
}  // namespace stl3lasercut
