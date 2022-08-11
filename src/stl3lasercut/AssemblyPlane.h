// Copyright 2022 by Daniel Winkelman. All rights reserved.

#pragma once

#include <algo/Graph.h>
#include <algo/Lookup.h>
#include <gtest/gtest.h>
#include <stl3lasercut/Projector.h>

#include <memory>

namespace stl3lasercut {
class Mesh;

/** An AssemblyPlane is the interface between a Mesh and a LoopPlane. It
 * accepts triangles in 3D space, which are themselves right-handed loops, and
 * uses a graph to assemble them into a set of closed loops, i.e. a graph whose
 * edges form a topology of right- and left-handed loops. These can be exported
 * to a LoopPlane. */
class AssemblyPlane : public std::enable_shared_from_this<AssemblyPlane> {
  friend class LoopPlane;
  FRIEND_TEST(MeshTests, AssemblyPlane);

 private:
  using VertexConnectivityGraph =
      algo::DirectedGraph<algo::Unit, algo::Unit, algo::Unit>;
  using Graph =
      algo::DirectedGraph<algo::Unit, uint32_t, VertexConnectivityGraph>;

 public:
  AssemblyPlane(const std::shared_ptr<const Mesh> &mesh, const uint32_t id,
                const Projector3D &projector);

  uint32_t registerPoint(const uint32_t meshIndex, const Vec3 &point);
  void addTriangle(const uint32_t v0, const uint32_t v1, const uint32_t v2);

 private:
  void addAngle(const uint32_t v0, const uint32_t v1, const uint32_t v2);
  void addEdge(const uint32_t v0, const uint32_t v1);

 private:
  std::shared_ptr<const Mesh> mesh_;
  uint32_t id_;
  Projector3D projector_;
  Graph graph_;
  uint32_t edgeIdCounter_;
  algo::Lookup<Vec2> pointLookup_; /** Map internal indices to points. */
  std::map<uint32_t, uint32_t> externalToInternalIndexMap_;
  std::map<uint32_t, uint32_t> internalToExternalIndexMap_;
};
}  // namespace stl3lasercut
