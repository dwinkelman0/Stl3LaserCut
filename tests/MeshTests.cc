// Copyright 2022 by Daniel Winkelman. All rights reserved.

#include <gtest/gtest.h>
#include <stl3lasercut/Mesh.h>

#include <map>

#include "SampleGeometry.h"
#include "Util.h"

namespace stl3lasercut {
struct MeshTestCase {
 public:
  std::string name;
  std::vector<StlTriangle> triangles;
  std::pair<uint32_t, uint32_t> characteristic;
  std::map<std::pair<uint32_t, uint32_t>, uint32_t> faces;

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
      mesh_ << triangle;
      output << triangle;
    }
    for (const auto &[projector, plane] : mesh_.getPlanes()) {
      plane->finalizeBase();
    }
  }

 protected:
  Mesh mesh_;
};

TEST_P(MeshTests, VertexEdgeCount) {
  ASSERT_EQ(mesh_.getCharacteristic().first, GetParam().characteristic.first);
  ASSERT_EQ(mesh_.getCharacteristic().second,
            GetParam().characteristic.second * 2);
  std::map<std::pair<uint32_t, uint32_t>, uint32_t> counts;
  for (const auto &[projector, plane] : mesh_.getPlanes()) {
    counts.emplace(plane->getCharacteristic(), 0).first->second++;
  }
  for (const auto &[pair, count] : counts) {
    auto it = GetParam().faces.find(pair);
    ASSERT_NE(it, GetParam().faces.end());
    ASSERT_EQ(count, it->second)
        << pair.first << " vertices, " << pair.second << " edges";
  }
  ASSERT_EQ(counts.size(), GetParam().faces.size());
}

TEST_P(MeshTests, Internals) {
  for (const auto &[projector, plane] : mesh_.getPlanes()) {
    ASSERT_GT(plane->getId(), 0);
    // ASSERT_EQ(plane->vertexMap_.size(),
    // plane->graph_.getVertices().getCount());
    ASSERT_EQ(plane->colorVertices_.find(0)->second.size(),
              plane->graph_.getVertices().getCount());
    for (const auto vertex : plane->graph_.getVertices()) {
      ASSERT_EQ(plane->graph_.getEdgesFromVertex(vertex).getCount(),
                plane->graph_.getEdgesToVertex(vertex).getCount());
      ASSERT_GE(vertex.getValue().vertexConnectivity.getVertices().getCount(),
                plane->graph_.getEdgesFromVertex(vertex).getCount() * 2);
      for (const Plane::VertexConnectivityGraph &connectivityGraph :
           vertex.getValue().vertexConnectivity.getConnectedComponents()) {
        ASSERT_EQ(connectivityGraph.getEdges().getCount(), 1);
        ASSERT_EQ(connectivityGraph.getVertices().getCount(), 2);
      }
    }
    for (const auto &edge : plane->graph_.getEdges()) {
      ASSERT_TRUE(edge.getValue().otherPlane);
      ASSERT_EQ(edge.getValue().colorIds.size(), 1);
      ASSERT_EQ(edge.getValue().orientation, Plane::Orientation::PARALLEL);
    }
  }
}

TEST_P(MeshTests, Projectors) {
  for (const auto &[projector, plane] : mesh_.getPlanes()) {
    for (const auto vertex : plane->graph_.getVertices()) {
      Vec3 point =
          mesh_.getVertexVector(*plane->getExternalVertexId(vertex.getIndex()));
      testPoint(projector.restore(projector.normalize(point)), point);
      testPoint(projector.restore(vertex.getValue().mappedPoint), point);
    }
  }
}

static float dummyOffsetFunction(const std::shared_ptr<Plane> &a,
                                 const std::shared_ptr<Plane> &b) {
  return -0.2;
}

TEST_P(MeshTests, PlaneOffsetFunction) {
  for (const auto &[projector, plane] : mesh_.getPlanes()) {
    std::cout << "new plane" << std::endl;
    plane->addOffsetLayer(dummyOffsetFunction, 0);
  }
}

INSTANTIATE_TEST_SUITE_P(
    Mesh, MeshTests,
    testing::Values(MeshTestCase{SAMPLE(tetrahedron), .characteristic = {4, 6},
                                 .faces = {{{3, 3}, 4}}},
                    MeshTestCase{SAMPLE(octahedron), .characteristic = {6, 12},
                                 .faces = {{{3, 3}, 8}}},
                    MeshTestCase{
                        SAMPLE(bowtie), .characteristic = {7, 13},
                        .faces = {{{3, 3}, 4}, {{4, 4}, 2}, {{5, 6}, 1}}}));
}  // namespace stl3lasercut
