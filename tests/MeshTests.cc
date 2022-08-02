// Copyright 2022 by Daniel Winkelman. All rights reserved.

#include <gtest/gtest.h>
#include <stl3lasercut/Mesh.h>

#include <map>

#include "SampleGeometry.h"

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

class MeshTest : public testing::TestWithParam<MeshTestCase> {
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

TEST_P(MeshTest, VertexEdgeCount) {
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

INSTANTIATE_TEST_SUITE_P(
    Mesh, MeshTest,
    testing::Values(
        MeshTestCase{
            "tetrahedron", samples::tetrahedron, {4, 6}, {{{3, 3}, 4}}},
        MeshTestCase{"octahedron", samples::octahedron, {6, 12}, {{{3, 3}, 8}}},
        MeshTestCase{"bowtie",
                     samples::bowtie,
                     {7, 13},
                     {{{3, 3}, 4}, {{4, 4}, 2}, {{5, 6}, 1}}}));
}  // namespace stl3lasercut
