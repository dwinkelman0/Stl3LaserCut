// Copyright 2022 by Daniel Winkelman. All rights reserved.

#include "Mesh.h"

namespace stl3lasercut {
Mesh &Mesh::operator<<(const StlTriangle &triangle) {
  // Check whether the associated projector is already present
  Projector3D projector = triangle.getProjector();
  const auto &[it, success] =
      planes_.emplace(projector, std::make_shared<PlaneGraph>());
  projector = it->first;
  auto plane = it->second;

  // Gather vertices
  uint32_t v0 = vertices_(std::get<0>(triangle.getVertices()));
  uint32_t v1 = vertices_(std::get<1>(triangle.getVertices()));
  uint32_t v2 = vertices_(std::get<2>(triangle.getVertices()));

  // Add each edge
  addEdge(plane, v0, v1, v2);
  addEdge(plane, v1, v2, v0);
  addEdge(plane, v2, v0, v1);

  return *this;
}

std::pair<uint32_t, uint32_t> Mesh::getCharacteristic() const {
  return {mesh_.getVertices().getCount(), mesh_.getEdges().getCount()};
}

const std::map<Projector3D, std::shared_ptr<Mesh::PlaneGraph>>
    &Mesh::getPlanes() const {
  return planes_;
}

Vec3 Mesh::getVertexVector(const uint32_t index) const {
  return vertices_(index);
}

void Mesh::debug() {
  for (const MeshGraph::Vertex &vertex : mesh_.getVertices()) {
    std::cout << vertex.getIndex() << ": ";
    for (const MeshGraph::Edge &edge : mesh_.getEdgesFromVertex(vertex)) {
      std::cout << edge.getDest() << ", ";
    }
    std::cout << std::endl;
  }
  std::map<uint32_t, uint32_t> planeSizeDist;
  for (const auto &[projector, plane] : planes_) {
    planeSizeDist.emplace(plane->getEdges().getCount(), 0).first->second++;
  }
  for (const auto &[size, count] : planeSizeDist) {
    std::cout << size << " vertices: " << count << " planes" << std::endl;
  }
  std::cout << mesh_.getVertices().getCount() << " total vertices" << std::endl;
  std::cout << mesh_.getEdges().getCount() << " total edges" << std::endl;
}

void Mesh::addEdge(const std::shared_ptr<PlaneGraph> &plane, const uint32_t v0,
                   const uint32_t v1, const uint32_t v2) {
  mesh_.emplaceVertex(v0);
  mesh_.emplaceVertex(v1);
  mesh_.emplaceEdge(v0, v1).first->getValue() = plane;
  plane->emplaceVertex(v0);
  PlaneGraph::VertexIterator vertexIt = plane->emplaceVertex(v1).first;
  vertexIt->getValue().emplaceVertex(v0);
  vertexIt->getValue().emplaceVertex(v2);
  vertexIt->getValue().emplaceEdge(v0, v2);
  auto existingEdge = plane->getEdge(v1, v0);
  if (existingEdge) {
    plane->eraseEdge(*existingEdge);
    mesh_.eraseEdge(std::pair<uint32_t, uint32_t>(v0, v1));
    mesh_.eraseEdge(std::pair<uint32_t, uint32_t>(v1, v0));
  } else {
    plane->emplaceEdge(v0, v1);
  }
}
}  // namespace stl3lasercut
