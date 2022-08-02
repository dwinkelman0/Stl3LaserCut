// Copyright 2022 by Daniel Winkelman. All rights reserved.

#include "StlIo.h"

#include <sstream>
#include <string>

namespace stl3lasercut {
static Vec3 computeNormal(const std::tuple<Vec3, Vec3, Vec3> &vertices) {
  Vec3 normal = cross(std::get<1>(vertices) - std::get<0>(vertices),
                      std::get<2>(vertices) - std::get<0>(vertices));
  return normal / abs(normal);
}

static Vec3 readVectorFromStream(std::istream &inputStream) {
  Vec3 normal;
  inputStream >> std::get<0>(normal) >> std::get<1>(normal) >>
      std::get<2>(normal);
  return normal;
}

StlTriangle::StlTriangle(const std::tuple<Vec3, Vec3, Vec3> &vertices)
    : normal_(computeNormal(vertices)), vertices_(vertices) {}

StlTriangle::StlTriangle(const Vec3 &normal,
                         const std::tuple<Vec3, Vec3, Vec3> &vertices)
    : normal_(normal / abs(normal)), vertices_(vertices) {}

Projector3D StlTriangle::getProjector() const {
  return Projector3D(normal_, std::get<0>(vertices_));
}

Vec3 StlTriangle::getNormal() const { return normal_; }

std::tuple<Vec3, Vec3, Vec3> StlTriangle::getVertices() const {
  return vertices_;
}

std::ostream &operator<<(std::ostream &os, const StlTriangle &triangle) {
  os << "(" << std::get<0>(triangle.getVertices()) << ", "
     << std::get<1>(triangle.getVertices()) << ", "
     << std::get<2>(triangle.getVertices()) << ")";
  return os;
}

StlInput::StlInput(std::ifstream &inputFile)
    : inputFile_(std::move(inputFile)) {}

std::optional<StlTriangle> StlInput::readTriangle() {
  std::string inputText;
  std::getline(inputFile_, inputText);
  inputFile_ >> inputText;
  if (inputText == "endsolid") {
    return std::nullopt;
  }
  assert(inputText == "facet");
  inputFile_ >> inputText;
  assert(inputText == "normal");
  Vec3 normal = readVectorFromStream(inputFile_);
  normal = normal / abs(normal);
  inputFile_ >> inputText >> inputText >> inputText;
  assert(inputText == "vertex");
  Vec3 v0 = readVectorFromStream(inputFile_);
  inputFile_ >> inputText;
  assert(inputText == "vertex");
  Vec3 v1 = readVectorFromStream(inputFile_);
  inputFile_ >> inputText;
  assert(inputText == "vertex");
  Vec3 v2 = readVectorFromStream(inputFile_);
  inputFile_ >> inputText >> inputText;
  assert(inputText == "endfacet");
  return StlTriangle(normal, {v0, v1, v2});
}

StlOutput::StlOutput(std::ofstream &outputFile)
    : outputFile_(std::move(outputFile)) {
  outputFile_ << "solid" << std::endl;
}

StlOutput::~StlOutput() { outputFile_ << "endsolid" << std::endl; }

static std::string formatVec3(const Vec3 &vector) {
  std::stringstream output;
  output << std::get<0>(vector) << " " << std::get<1>(vector) << " "
         << std::get<2>(vector);
  return output.str();
}

StlOutput &StlOutput::operator<<(const StlTriangle &triangle) {
  outputFile_ << "facet normal " << formatVec3(triangle.getNormal()) << "\n";
  outputFile_ << "  outer loop\n";
  outputFile_ << "    vertex "
              << formatVec3(std::get<0>(triangle.getVertices())) << "\n";
  outputFile_ << "    vertex "
              << formatVec3(std::get<1>(triangle.getVertices())) << "\n";
  outputFile_ << "    vertex "
              << formatVec3(std::get<2>(triangle.getVertices())) << "\n";
  outputFile_ << "  endloop\n";
  outputFile_ << "endfacet\n";
  return *this;
}
}  // namespace stl3lasercut
