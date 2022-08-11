// Copyright 2022 by Daniel Winkelman. All rights reserved.

#include <gtest/gtest.h>
#include <stl3lasercut/Mesh.h>
#include <stl3lasercut/RingVector.h>

#include "SampleGeometry.h"

namespace stl3lasercut {
class PlaneSetup {
 public:
  PlaneSetup(const std::vector<std::vector<Vec2>> &points)
      : plane_(std::make_shared<Plane>(1, unit3::z)) {
    uint32_t counter = 0;
    for (const std::vector<Vec2> &loop : points) {
      RingVector<std::pair<uint32_t, Vec2>> ring =
          RingVector<Vec2>(loop).foreach<std::pair<uint32_t, Vec2>>(
              [&counter](const Vec2 &point) {
                return std::pair<uint32_t, Vec2>(counter++, point);
              });
      ring.foreach (
          [this](const std::vector<std::pair<uint32_t, Vec2>> &points) {
            plane_->addEdge(points[0], points[1], points[2], nullptr);
          },
          3);
    }
    plane_->finalizeBase();
  }

 protected:
  std::shared_ptr<Plane> plane_;
};

struct Characteristic {
  bool isPositive;
  uint32_t numVertices;
  std::vector<Characteristic> enclosed;

  friend std::ostream &operator<<(std::ostream &os,
                                  const Characteristic &characteristic);
  friend std::ostream &operator<<(
      std::ostream &os, const std::vector<Characteristic> &characteristic);
};

std::ostream &operator<<(std::ostream &os,
                         const Characteristic &characteristic) {
  os << (characteristic.isPositive ? '+' : '-') << characteristic.numVertices;
  if (!characteristic.enclosed.empty()) {
    os << characteristic.enclosed;
  }
  return os;
}

std::ostream &operator<<(std::ostream &os,
                         const std::vector<Characteristic> &characteristic) {
  os << "(";
  for (const Characteristic &c : characteristic) {
    os << c << ", ";
  }
  os << ")";
  return os;
}

namespace characteristic {
std::vector<Characteristic> flat(const std::vector<uint32_t> positives) {
  std::vector<Characteristic> output;
  for (const uint32_t n : positives) {
    output.push_back({.isPositive = true, .numVertices = n, .enclosed = {}});
  }
  return output;
}
}  // namespace characteristic

struct PlaneConstantOffsetTestCase {
  std::string name;
  std::vector<std::vector<Vec2>> points;
  float offset;
  std::vector<Characteristic> characteristic;

  friend std::ostream &operator<<(std::ostream &os,
                                  const PlaneConstantOffsetTestCase &test);
};

std::ostream &operator<<(std::ostream &os,
                         const PlaneConstantOffsetTestCase &test) {
  os << test.name << ": offset = " << test.offset
     << ", characteristic = " << test.characteristic;
  return os;
}

#define SAMPLE(n) .name = #n, .points = samples::n

class PlaneConstantOffsetTests
    : public testing::TestWithParam<PlaneConstantOffsetTestCase>,
      public PlaneSetup {
 public:
  PlaneConstantOffsetTests() : PlaneSetup(GetParam().points) {}

  void SetUp() {
    plane_->addOffsetLayer(
        [this](const std::shared_ptr<Plane> &a,
               const std::shared_ptr<Plane> &b) { return GetParam().offset; },
        0);
  }
};

TEST_P(PlaneConstantOffsetTests, Initialization) {
  // ASSERT_EQ(plane_->getCharacteristic().first,
  //           GetParam().characteristic.front().numVertices * 2);
  // ASSERT_EQ(plane_->getCharacteristic().second,
  //           GetParam().characteristic.front().numVertices * 2);
  const auto [vertices, edges] = plane_->getCharacteristic();
  std::cout << vertices << " vertices, " << edges << " edges" << std::endl;
}

INSTANTIATE_TEST_SUITE_P(Plane, PlaneConstantOffsetTests,
                         testing::Values(
                             PlaneConstantOffsetTestCase{
                                 SAMPLE(acuteTriangle), .offset = -0.5,
                                 .characteristic = characteristic::flat({3})},
                             PlaneConstantOffsetTestCase{
                                 SAMPLE(rightTriangle), .offset = -0.5,
                                 .characteristic = characteristic::flat({3})},
                             PlaneConstantOffsetTestCase{
                                 SAMPLE(straightAnglePolygon), .offset = -0.5,
                                 .characteristic = characteristic::flat({4})}));
}  // namespace stl3lasercut
