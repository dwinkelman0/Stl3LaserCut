// Copyright 2022 by Daniel Winkelman. All rights reserved.

#pragma once

#include <stl3lasercut/StlIo.h>

#include <vector>

namespace stl3lasercut {
namespace samples {
// 3D geometry
extern std::vector<StlTriangle> tetrahedron;
extern std::vector<StlTriangle> octahedron;
extern std::vector<StlTriangle> bowtie;
extern std::vector<StlTriangle> thingy;
extern std::vector<StlTriangle> bowl;
extern std::vector<StlTriangle> dimpleFace;

// 2D non-overlapping geometry
extern std::vector<std::vector<Vec2>> acuteTriangle;
extern std::vector<std::vector<Vec2>> rightTriangle;
extern std::vector<std::vector<Vec2>> obtuseTriangle;
extern std::vector<std::vector<Vec2>> convexPolygon;
extern std::vector<std::vector<Vec2>> acuteConcavePolygon;
extern std::vector<std::vector<Vec2>> rightConcavePolygon;
extern std::vector<std::vector<Vec2>> obtuseConcavePolygon;
extern std::vector<std::vector<Vec2>> straightAnglePolygon;
extern std::vector<std::vector<Vec2>> disjointTriangles;
extern std::vector<std::vector<Vec2>> twoTangentTriangles;
extern std::vector<std::vector<Vec2>> threeTangentTriangles;
extern std::vector<std::vector<Vec2>> doubleTangentPolygons;
extern std::vector<std::vector<Vec2>> negativeTriangle;
extern std::vector<std::vector<Vec2>> bubbleNegativeTriangle;
extern std::vector<std::vector<Vec2>> twoTangentNegativeTriangles;
extern std::vector<std::vector<Vec2>> tangentPositiveAndNegativeTriangles;
extern std::vector<std::vector<Vec2>> sandwichedTriangles;
extern std::vector<std::vector<Vec2>> doubleSandwichedTriangles;
extern std::vector<std::vector<Vec2>> bubblePositiveAndNegativeTriangles;
}  // namespace samples
}  // namespace stl3lasercut
