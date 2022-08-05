// Copyright 2022 by Daniel Winkelman. All rights reserved.

#pragma once

#include <algo/Graph.h>
#include <algo/Lookup.h>
#include <stl3lasercut/StlIo.h>

#include <map>

namespace stl3lasercut {

class Mesh {
  using VertexConnectivityGraph =
      algo::DirectedGraph<algo::Unit, algo::Unit, algo::Unit>;
  using PlaneGraph =
      algo::DirectedGraph<algo::Unit, algo::Unit, VertexConnectivityGraph>;
  using MeshGraph =
      algo::DirectedGraph<algo::Unit, std::shared_ptr<PlaneGraph>, algo::Unit>;

 public:
  Mesh &operator<<(const StlTriangle &triangle);
  std::pair<uint32_t, uint32_t> getCharacteristic() const;
  const std::map<Projector3D, std::shared_ptr<PlaneGraph>> &getPlanes() const;
  Vec3 getVertexVector(const uint32_t index) const;

  void debug();

 protected:
  void addEdge(const std::shared_ptr<PlaneGraph> &plane, const uint32_t v0,
               const uint32_t v1, const uint32_t v2);

 private:
  algo::Lookup<Vec3> vertices_;
  MeshGraph mesh_; /** Maintains connectivity of 3D structure. */
  std::map<Projector3D, std::shared_ptr<PlaneGraph>>
      planes_; /** Maintains connectivity (with a concept positive/negative
                  area) of 2D structure, and sorting vertices into planes.
                */
};
}  // namespace stl3lasercut
