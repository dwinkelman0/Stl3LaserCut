// Copyright 2022 by Daniel Winkelman. All rights reserved.

#include <stl3lasercut/Mesh.h>
#include <stl3lasercut/StlIo.h>

int main() {
  std::ifstream inputFile("/Users/dwinkelman/Downloads/thinker_ascii.stl");
  stl3lasercut::StlInput input(inputFile);
  stl3lasercut::Mesh mesh;
  std::optional<stl3lasercut::StlTriangle> triangle = input.readTriangle();
  while (triangle) {
    mesh << *triangle;
    triangle = input.readTriangle();
  }
  for (const auto &[projector, plane] : mesh.getPlanes()) {
    plane->finalizeBase();
  }
  auto [vertices, edges] = mesh.getCharacteristic();
  std::cout << vertices << " vertices, " << edges / 2 << " edges, "
            << mesh.getPlanes().size() << " planes" << std::endl;
  return 0;
}
