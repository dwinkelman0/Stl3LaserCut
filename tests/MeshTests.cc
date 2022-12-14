// Copyright 2022 by Daniel Winkelman. All rights reserved.

#include <gtest/gtest.h>
#include <stl3lasercut/AssemblyPlane.h>
#include <stl3lasercut/LoopPlane.h>
#include <stl3lasercut/Mesh.h>

#include <algorithm>
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
  os << mesh.name;
  return os;
}

#define SAMPLE(n) .name = #n, .triangles = samples::n

class MeshTests : public testing::TestWithParam<MeshTestCase> {
 public:
  MeshTests() : mesh_(std::make_shared<Mesh>()) {}

  void SetUp() override {
    std::ofstream outputFile(GetParam().name + ".stl");
    StlOutput output(outputFile);
    for (const StlTriangle &triangle : GetParam().triangles) {
      mesh_->addTriangle(triangle);
      output << triangle;
    }
  }

 protected:
  std::shared_ptr<Mesh> mesh_;
};

TEST_P(MeshTests, Characteristic) {
  ASSERT_TRUE(mesh_->isValid());
  Mesh::Characteristic characteristic = mesh_->getCharacteristic();
  ASSERT_EQ(characteristic.planes, GetParam().characteristic.planes);
  ASSERT_EQ(characteristic.vertices, GetParam().characteristic.vertices);
  ASSERT_EQ(characteristic.edges, GetParam().characteristic.edges);
}

TEST_P(MeshTests, AssemblyPlane) {
  for (const auto &[projector, plane] : mesh_->planes_) {
    for (const AssemblyPlane::Graph::ConstVertex &vertex :
         plane->graph_.getVertices()) {
      ASSERT_GT(plane->graph_.getEdgesFromVertex(vertex).getCount(), 0);
      ASSERT_EQ(plane->graph_.getEdgesToVertex(vertex).getCount(),
                plane->graph_.getEdgesFromVertex(vertex).getCount());
      for (const ChainedVertexConnectivityGraph::Graph &graph :
           vertex.getValue().graph_.getConnectedComponents()) {
        ASSERT_EQ(graph.getVertices().getCount(),
                  graph.getEdges().getCount() + 1);
      }
    }
    std::set<uint32_t> edgeIds;
    for (const AssemblyPlane::Graph::ConstEdge &edge :
         plane->graph_.getEdges()) {
      edgeIds.insert(edge.getValue());
    }
    uint32_t numEdges = plane->graph_.getEdges().getCount();
    ASSERT_EQ(edgeIds.size(), numEdges);
    ASSERT_LT(*std::max_element(edgeIds.begin(), edgeIds.end()), numEdges);
  }
}

TEST_P(MeshTests, LoopPlane) {
  for (const auto &[projector, plane] : mesh_->planes_) {
    auto loopPlane = std::make_shared<LoopPlane>(plane, 0);
    for (const LoopPlane::Graph::ConstVertex &vertex :
         loopPlane->graph_.getVertices()) {
      ASSERT_EQ(loopPlane->graph_.getEdgesToVertex(vertex).getCount(),
                loopPlane->graph_.getEdgesFromVertex(vertex).getCount());
      ASSERT_EQ(vertex.getValue().size(),
                loopPlane->graph_.getEdgesToVertex(vertex).getCount());
    }
    loopPlane->getLoops();
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
            .characteristic = {.planes = 7, .vertices = 7, .edges = 13}},
        MeshTestCase{
            SAMPLE(thingy),
            .characteristic = {.planes = 7, .vertices = 6, .edges = 11}},
        MeshTestCase{
            SAMPLE(bowl),
            .characteristic = {.planes = 6, .vertices = 5, .edges = 9}},
        MeshTestCase{
            SAMPLE(dimpleFace),
            .characteristic = {.planes = 7, .vertices = 7, .edges = 12}}),
    testing::PrintToStringParamName());
}  // namespace stl3lasercut
