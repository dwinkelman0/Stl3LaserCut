// Copyright 2022 by Daniel Winkelman. All rights reserved.

#include <stl3lasercut/Mesh.h>
#include <stl3lasercut/StlIo.h>

int main() {
  std::ifstream inputFile("/Users/dwinkelman/Downloads/thinker_ascii.stl");
  stl3lasercut::StlInput input(inputFile);
  stl3lasercut::Mesh mesh;
  std::optional<stl3lasercut::StlTriangle> triangle = input.readTriangle();
  while (triangle) {
    mesh.addTriangle(*triangle);
    triangle = input.readTriangle();
  }
  if (mesh.isValid()) {
    std::cout << mesh.getCharacteristic() << std::endl;
  } else {
    std::cout << "Mesh is invalid" << std::endl;
  }
  return 0;
}
