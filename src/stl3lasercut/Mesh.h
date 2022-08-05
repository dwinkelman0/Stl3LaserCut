// Copyright 2022 by Daniel Winkelman. All rights reserved.

#pragma once

#include <algo/Graph.h>
#include <algo/Lookup.h>
#include <gtest/gtest.h>
#include <stl3lasercut/StlIo.h>

#include <map>
#include <memory>

namespace stl3lasercut {

class Plane : public std::enable_shared_from_this<Plane> {
  FRIEND_TEST(MeshTests, Internals);
  FRIEND_TEST(MeshTests, Projectors);

  using VertexConnectivityGraph =
      algo::DirectedGraph<algo::Unit, algo::Unit, algo::Unit>;

  struct EdgeData {
    std::shared_ptr<Plane> otherPlane;
  };

  struct VertexData {
    Vec2 mappedPoint;
    VertexConnectivityGraph vertexConnectivity;
  };

  using Graph = algo::DirectedGraph<algo::Unit, EdgeData, VertexData>;

 public:
  Plane(const Vec3 &normal);
  bool addEdge(const Projector3D &projector, const Vec3 &point,
               const uint32_t v0, const uint32_t v1, const uint32_t v2,
               const std::shared_ptr<Plane> &adjacentPlane);

  std::pair<uint32_t, uint32_t> getCharacteristic() const;

 private:
  Vec3 normal_;
  Graph graph_;
};

class Mesh {
  using MeshGraph =
      algo::DirectedGraph<algo::Unit, std::shared_ptr<Plane>, algo::Unit>;

 public:
  Mesh &operator<<(const StlTriangle &triangle);
  std::pair<uint32_t, uint32_t> getCharacteristic() const;
  const std::map<Projector3D, std::shared_ptr<Plane>> &getPlanes() const;
  Vec3 getVertexVector(const uint32_t index) const;

 protected:
  void addEdge(const std::shared_ptr<Plane> &plane,
               const Projector3D &projector, const Vec3 &point,
               const uint32_t v0, const uint32_t v1, const uint32_t v2);

 private:
  algo::Lookup<Vec3> vertices_;
  MeshGraph mesh_; /** Maintains connectivity of 3D structure. */
  std::map<Projector3D, std::shared_ptr<Plane>>
      planes_; /** Maintains connectivity (with a concept positive/negative
                  area) of 2D structure, and sorting vertices into planes. */
};
}  // namespace stl3lasercut
