// Copyright 2022 by Daniel Winkelman. All rights reserved.

#include "AssemblyPlane.h"

namespace stl3lasercut {
AssemblyPlane::AssemblyPlane(const Mesh &mesh, const uint32_t id,
                             const Projector3D &projector)
    : mesh_(mesh), id_(id), projector_(projector) {}

uint32_t AssemblyPlane::registerPoint(const uint32_t meshIndex,
                                      const Vec3 &point) {
  Vec2 mappedPoint = projector_.normalize(point);
  uint32_t internalIndex = pointLookup_(mappedPoint);
  {
    const auto &[it, success] =
        externalToInternalIndexMap_.emplace(meshIndex, internalIndex);
    if (!success && it->second != internalIndex) {
      throw std::runtime_error(
          "This mesh index was already used with a different point.");
    }
  }
  {
    const auto &[it, success] =
        internalToExternalIndexMap_.emplace(internalIndex, meshIndex);
    if (!success && it->second != meshIndex) {
      throw std::runtime_error(
          "This point was already used with a different mesh index.");
    }
  }
  return internalIndex;
}

void AssemblyPlane::addTriangle(const uint32_t v0, const uint32_t v1,
                                const uint32_t v2) {
  addAngle(v0, v1, v2);
  addAngle(v1, v2, v0);
  addAngle(v2, v0, v1);
}

void AssemblyPlane::addAngle(const uint32_t v0, const uint32_t v1,
                             const uint32_t v2) {
  graph_.emplaceVertex(v0);
  Graph::VertexIterator vertexIt = graph_.emplaceVertex(v1).first;
  graph_.emplaceVertex(v2);
  graph_.emplaceEdge(v0, v1);
  graph_.emplaceEdge(v1, v2);
  VertexConnectivityGraph::VertexIterator it0 =
      vertexIt->getValue().emplaceVertex(v0).first;
  VertexConnectivityGraph::VertexIterator it2 =
      vertexIt->getValue().emplaceVertex(v2).first;
  if (vertexIt->getValue().getEdgesFromVertex(it0).getCount() > 0 ||
      vertexIt->getValue().getEdgesToVertex(it2).getCount() > 0) {
    throw std::runtime_error(
        "There were connectivity edges that should not have been there.");
  }
  vertexIt->getValue().emplaceEdge(v0, v2);
}
}  // namespace stl3lasercut
