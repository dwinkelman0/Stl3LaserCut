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
class AssemblyPlane;

class Mesh : public std::enable_shared_from_this<Mesh> {
  FRIEND_TEST(MeshTests, AssemblyPlane);
  FRIEND_TEST(MeshTests, LoopPlane);

 private:
  using Graph = algo::DirectedGraph<algo::Unit, std::shared_ptr<AssemblyPlane>,
                                    algo::Unit>;

 public:
  struct Characteristic {
    uint32_t planes;
    uint32_t vertices;
    uint32_t edges;

    friend std::ostream& operator<<(std::ostream& os, const Characteristic& ch);
  };

 public:
  Mesh();

  void addTriangle(const StlTriangle& triangle);

  Characteristic getCharacteristic() const;

  /** Does a very cursory check for obvious errors, not an exhaustive method. */
  bool isValid() const;

 private:
  uint32_t addVertex(const Vec3& point);
  void addEdge(const std::shared_ptr<AssemblyPlane>& plane, const uint32_t v0,
               const uint32_t v1);

 private:
  algo::Lookup<Vec3> vertexLookup_;
  Graph graph_; /** Maintains connectivity of 3D structure. */
  std::map<Projector3D, std::shared_ptr<AssemblyPlane>>
      planes_; /** Maintains connectivity (with a concept positive/negative
                  area) of 2D structure, and sorting vertices into planes. */
  uint32_t planeIdCounter_;
};
}  // namespace stl3lasercut
