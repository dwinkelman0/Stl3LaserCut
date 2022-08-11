// Copyright 2022 by Daniel Winkelman. All rights reserved.

#pragma once

#include <algo/Graph.h>
#include <algo/Lookup.h>
#include <gtest/gtest.h>
#include <stl3lasercut/Line.h>
#include <stl3lasercut/StlIo.h>

#include <map>
#include <memory>

namespace stl3lasercut {

class Plane : public std::enable_shared_from_this<Plane> {
  FRIEND_TEST(MeshTests, Internals);
  FRIEND_TEST(MeshTests, Projectors);
  FRIEND_TEST(PlaneBaseTests, Initialization);

  using VertexConnectivityGraph =
      algo::DirectedGraph<algo::Unit, algo::Unit, algo::Unit>;
  using OffsetFunction = std::function<float(const std::shared_ptr<Plane> &,
                                             const std::shared_ptr<Plane> &)>;

  enum class Orientation { PARALLEL, RIGHT_PERPENDICULAR, LEFT_PERPENDICULAR };

  struct EdgeData {
    std::shared_ptr<Plane> otherPlane;
    uint32_t id;
    std::set<uint32_t> colorIds;
    Orientation orientation;
    BoundedLine line;
  };

  struct VertexData {
    Vec2 mappedPoint;
    VertexConnectivityGraph vertexConnectivity;
  };

  using Graph = algo::DirectedGraph<algo::Unit, EdgeData, VertexData>;

 public:
  Plane(const uint32_t id, const Vec3 &normal);
  bool addEdge(const std::pair<uint32_t, Vec2> &p0,
               const std::pair<uint32_t, Vec2> &p1,
               const std::pair<uint32_t, Vec2> &p2,
               const std::shared_ptr<Plane> &adjacentPlane);
  bool addEdge(const Projector3D &projector,
               const std::pair<uint32_t, Vec3> &p0,
               const std::pair<uint32_t, Vec3> &p1,
               const std::pair<uint32_t, Vec3> &p2,
               const std::shared_ptr<Plane> &adjacentPlane);
  void finalizeBase();
  uint32_t addOffsetLayer(const OffsetFunction &offsetFunction,
                          const uint32_t baseColor);

  uint32_t getId() const;
  std::pair<uint32_t, uint32_t> getCharacteristic() const;
  std::optional<Graph::EdgeIterator> getCorrespondingEdge(const uint32_t v0,
                                                          const uint32_t v1);
  std::optional<uint32_t> getExternalVertexId(const uint32_t internalId) const;

 private:
  Graph::VertexIterator makeNewVertex(const Vec2 &point);
  Graph::VertexIterator makeNewVertex(const Vec2 &point,
                                      const uint32_t externalId);

 private:
  uint32_t id_;
  Vec3 normal_;
  Graph graph_;
  uint32_t colorIdCounter_;
  uint32_t edgeIdCounter_;
  std::map<uint32_t, std::set<uint32_t>> colorVertices_;
  std::map<uint32_t, uint32_t>
      externalVertexLookup_; /** Maps external id to internal id. */
  std::map<uint32_t, uint32_t>
      internalVertexLookup_;        /** Maps internal id to external id. */
  algo::Lookup<Vec2> vertexLookup_; /** Maps internal id to Vec2 point. */
};

class Mesh {
  using MeshGraph =
      algo::DirectedGraph<algo::Unit, std::shared_ptr<Plane>, algo::Unit>;

 public:
  Mesh();

  Mesh &operator<<(const StlTriangle &triangle);
  std::pair<uint32_t, uint32_t> getCharacteristic() const;
  const std::map<Projector3D, std::shared_ptr<Plane>> &getPlanes() const;
  Vec3 getVertexVector(const uint32_t index) const;

 protected:
  void addEdge(const std::shared_ptr<Plane> &plane,
               const Projector3D &projector,
               const std::pair<uint32_t, Vec3> &p0,
               const std::pair<uint32_t, Vec3> &p1,
               const std::pair<uint32_t, Vec3> &p2);

 private:
  algo::Lookup<Vec3> vertices_;
  MeshGraph mesh_; /** Maintains connectivity of 3D structure. */
  std::map<Projector3D, std::shared_ptr<Plane>>
      planes_; /** Maintains connectivity (with a concept positive/negative
                  area) of 2D structure, and sorting vertices into planes. */
  uint32_t planeIdCounter_;
};
}  // namespace stl3lasercut
