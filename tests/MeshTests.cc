// Copyright 2022 by Daniel Winkelman. All rights reserved.

#include <gtest/gtest.h>
#include <stl3lasercut/AssemblyPlane.h>
#include <stl3lasercut/Mesh.h>

#include <map>

#include "SampleGeometry.h"
#include "Util.h"

namespace stl3lasercut {
struct MeshTestCase {
 public:
  std::string name;
  std::vector<StlTriangle> triangles;
  Mesh::Characteristic characteristic;

  friend std::ostream &operator<<(std::ostream &os, const MeshTestCase &mesh);
};

std::ostream &operator<<(std::ostream &os, const MeshTestCase &mesh) {
  os << mesh.name << " (" << mesh.triangles.size() << " triangles)";
  return os;
}

#define SAMPLE(n) .name = #n, .triangles = samples::n

class MeshTests : public testing::TestWithParam<MeshTestCase> {
 public:
  void SetUp() override {
    std::ofstream outputFile(GetParam().name + ".stl");
    StlOutput output(outputFile);
    for (const StlTriangle &triangle : GetParam().triangles) {
      mesh_.addTriangle(triangle);
      output << triangle;
    }
  }

 protected:
  Mesh mesh_;
};

TEST_P(MeshTests, Characteristic) {
  ASSERT_TRUE(mesh_.isValid());
  Mesh::Characteristic characteristic = mesh_.getCharacteristic();
  ASSERT_EQ(characteristic.planes, GetParam().characteristic.planes);
  ASSERT_EQ(characteristic.vertices, GetParam().characteristic.vertices);
  ASSERT_EQ(characteristic.edges, GetParam().characteristic.edges);
}

TEST_P(MeshTests, AssemblyPlane) {
  for (const auto &[projector, plane] : mesh_.planes_) {
    for (const AssemblyPlane::Graph::ConstVertex &vertex :
         plane->graph_.getVertices()) {
      for (const AssemblyPlane::VertexConnectivityGraph &graph :
           vertex.getValue().getConnectedComponents()) {
        ASSERT_EQ(graph.getVertices().getCount(),
                  graph.getEdges().getCount() + 1);
      }
    }
  }
}

INSTANTIATE_TEST_SUITE_P(
    Mesh, MeshTests,
    testing::Values(
        MeshTestCase{
            SAMPLE(tetrahedron),
            .characteristic = {.planes = 4, .vertices = 4, .edges = 6}},
        MeshTestCase{
            SAMPLE(octahedron),
            .characteristic = {.planes = 8, .vertices = 6, .edges = 12}},
        MeshTestCase{
            SAMPLE(bowtie),
            .characteristic = {.planes = 7, .vertices = 7, .edges = 13}}));
}  // namespace stl3lasercut
