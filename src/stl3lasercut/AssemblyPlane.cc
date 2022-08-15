// Copyright 2022 by Daniel Winkelman. All rights reserved.

#include "AssemblyPlane.h"

#include <stl3lasercut/RingVector.h>

namespace stl3lasercut {
AssemblyPlane::AssemblyPlane(const std::shared_ptr<Mesh> &mesh,
                             const uint32_t id, const Projector3D &projector)
    : mesh_(mesh), id_(id), projector_(projector), edgeIdCounter_(0) {}

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

void AssemblyPlane::addLoop(const std::vector<Vec2> &points) {
  RingVector<Vec2> loop(points);
  loop.foreach (
      [this](const std::vector<Vec2> &segment) {
        addAngle(pointLookup_(segment[0]), pointLookup_(segment[1]),
                 pointLookup_(segment[2]));
      },
      3);
}

Vec2 AssemblyPlane::getPoint(const uint32_t index) const {
  return pointLookup_(index);
}

void AssemblyPlane::addAngle(const uint32_t v0, const uint32_t v1,
                             const uint32_t v2) {
  graph_.emplaceVertex(v0);
  Graph::VertexIterator vertexIt = graph_.emplaceVertex(v1).first;
  graph_.emplaceVertex(v2);
  addEdge(v0, v1);
  addEdge(v1, v2);
  vertexIt->getValue().connect(v0, v2);
}

void AssemblyPlane::addEdge(const uint32_t v0, const uint32_t v1) {
  auto [it, success] = graph_.emplaceEdge(v0, v1);
  if (success) {
    it->getValue() = edgeIdCounter_++;
  }
}
}  // namespace stl3lasercut
