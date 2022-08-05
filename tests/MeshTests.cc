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

class Tests : public testing::TestWithParam<MeshTestCase> {
 public:
  void SetUp() override {
    std::ofstream outputFile(GetParam().name + ".stl");
    StlOutput output(outputFile);
    for (const StlTriangle &triangle : GetParam().triangles) {
      mesh_ << triangle;
      output << triangle;
    }
  }

 protected:
  Mesh mesh_;
};

TEST_P(Tests, VertexEdgeCount) {
  ASSERT_EQ(mesh_.getCharacteristic().first, GetParam().characteristic.first);
  ASSERT_EQ(mesh_.getCharacteristic().second,
            GetParam().characteristic.second * 2);
  std::map<std::pair<uint32_t, uint32_t>, uint32_t> counts;
  for (const auto &[projector, plane] : mesh_.getPlanes()) {
    counts
        .emplace(std::pair<uint32_t, uint32_t>(plane->getVertices().getCount(),
                                               plane->getEdges().getCount()),
                 0)
        .first->second++;
  }
  for (const auto &[pair, count] : counts) {
    auto it = GetParam().faces.find(pair);
    ASSERT_NE(it, GetParam().faces.end());
    ASSERT_EQ(count, it->second)
        << pair.first << " vertices, " << pair.second << " edges";
  }
  ASSERT_EQ(counts.size(), GetParam().faces.size());
}

TEST_P(Tests, Internals) {
  for (const auto &[projector, plane] : mesh_.getPlanes()) {
    for (const auto vertex : plane->getVertices()) {
      ASSERT_EQ(plane->getEdgesFromVertex(vertex).getCount(),
                plane->getEdgesToVertex(vertex).getCount());
      ASSERT_GE(vertex.getValue().getVertices().getCount(),
                plane->getEdgesFromVertex(vertex).getCount() * 2);
    }
  }
}

TEST_P(Tests, Projectors) {
  for (const auto &[projector, plane] : mesh_.getPlanes()) {
    for (const auto vertex : plane->getVertices()) {
      Vec3 point = mesh_.getVertexVector(vertex.getIndex());
      testPoint(projector.restore(projector.normalize(point)), point);
    }
  }
}

INSTANTIATE_TEST_SUITE_P(
    Mesh, Tests,
    testing::Values(MeshTestCase{.name = "tetrahedron",
                                 .triangles = samples::tetrahedron,
                                 .characteristic = {4, 6},
                                 .faces = {{{3, 3}, 4}}},
                    MeshTestCase{.name = "octahedron",
                                 .triangles = samples::octahedron,
                                 .characteristic = {6, 12},
                                 .faces = {{{3, 3}, 8}}},
                    MeshTestCase{
                        .name = "bowtie",
                        .triangles = samples::bowtie,
                        .characteristic = {7, 13},
                        .faces = {{{3, 3}, 4}, {{4, 4}, 2}, {{5, 6}, 1}}}));
}  // namespace stl3lasercut
