// Copyright 2022 by Daniel Winkelman. All rights reserved.

#pragma once

#include <stl3lasercut/Projector.h>

#include <fstream>

namespace stl3lasercut {

class StlTriangle {
 public:
  StlTriangle(const std::tuple<Vec3, Vec3, Vec3> &vertices);
  StlTriangle(const Vec3 &normal, const std::tuple<Vec3, Vec3, Vec3> &vertices);

  Projector3D getProjector() const;
  Vec3 getNormal() const;
  std::tuple<Vec3, Vec3, Vec3> getVertices() const;

  friend std::ostream &operator<<(std::ostream &os,
                                  const StlTriangle &triangle);

 private:
  Vec3 normal_;
  std::tuple<Vec3, Vec3, Vec3> vertices_;
};

class StlInput {
 public:
  StlInput(std::ifstream &inputFile);

  std::optional<StlTriangle> readTriangle();

 private:
  std::ifstream inputFile_;
};

class StlOutput {
 public:
  StlOutput(std::ofstream &outputFile);
  ~StlOutput();

  StlOutput &operator<<(const StlTriangle &triangle);

 private:
  std::ofstream outputFile_;
};
}  // namespace stl3lasercut
