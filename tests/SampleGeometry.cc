// Copyright 2022 by Daniel Winkelman. All rights reserved.

#include "SampleGeometry.h"

namespace stl3lasercut {
namespace samples {
namespace tetrahedron_ {
Vec3 v0(1, 1, 1);
Vec3 v1(1, 0, 0);
Vec3 v2(0, 1, 0);
Vec3 v3(0, 0, 1);
std::vector<StlTriangle> geometry = {
    StlTriangle({v0, v1, v2}), StlTriangle({v0, v2, v3}),
    StlTriangle({v0, v3, v1}), StlTriangle({v1, v3, v2})};
}  // namespace tetrahedron_
std::vector<StlTriangle> tetrahedron(tetrahedron_::geometry);

namespace octahedron_ {
Vec3 v0(1, 0, 0);
Vec3 v1(0, 1, 0);
Vec3 v2(-1, 0, 0);
Vec3 v3(0, -1, 0);
Vec3 v4(0, 0, 1);
Vec3 v5(0, 0, -1);
std::vector<StlTriangle> geometry = {
    StlTriangle({v0, v1, v4}), StlTriangle({v5, v1, v0}),
    StlTriangle({v1, v2, v4}), StlTriangle({v5, v2, v1}),
    StlTriangle({v2, v3, v4}), StlTriangle({v5, v3, v2}),
    StlTriangle({v3, v0, v4}), StlTriangle({v5, v0, v3})};
}  // namespace octahedron_
std::vector<StlTriangle> octahedron(octahedron_::geometry);

namespace bowtie_ {
Vec3 v0(0, 0, 0);
Vec3 v1(2, 1, 0);
Vec3 v2(2, -1, 0);
Vec3 v3(-2, -1, 0);
Vec3 v4(-2, 1, 0);
Vec3 v5(0, 1, 2);
Vec3 v6(0, -1, 2);
std::vector<StlTriangle> geometry = {
    StlTriangle({v0, v1, v2}), StlTriangle({v0, v3, v4}),
    StlTriangle({v0, v5, v1}), StlTriangle({v0, v2, v6}),
    StlTriangle({v0, v6, v3}), StlTriangle({v0, v4, v5}),
    StlTriangle({v1, v5, v6}), StlTriangle({v6, v2, v1}),
    StlTriangle({v3, v6, v5}), StlTriangle({v5, v4, v3})};
}  // namespace bowtie_
std::vector<StlTriangle> bowtie(bowtie_::geometry);

std::vector<std::vector<Vec2>> acuteTriangle = {{{0, 0}, {2, 0}, {1, 2}}};

std::vector<std::vector<Vec2>> rightTriangle = {{{0, 0}, {2, 0}, {0, 2}}};

std::vector<std::vector<Vec2>> obtuseTriangle = {{{0, 0}, {2, 1}, {-2, 1}}};

std::vector<std::vector<Vec2>> convexPolygon = {
    {{0, 0}, {2, 0}, {4, 1}, {6, 3}, {5, 5}, {2, 3}}};

std::vector<std::vector<Vec2>> acuteConcavePolygon = {
    {{0, 0}, {4, -1}, {2, 0}, {4, 1}}};

std::vector<std::vector<Vec2>> rightConcavePolygon = {
    {{0, 0}, {4, -1}, {3, 0}, {4, 1}}};

std::vector<std::vector<Vec2>> obtuseConcavePolygon = {
    {{0, 0}, {3, 0}, {1, 1}, {0, 3}}};

std::vector<std::vector<Vec2>> straightAnglePolygon = {
    {{0, 0}, {2, 0}, {4, 0}, {2, 3}}};

std::vector<std::vector<Vec2>> disjointTriangles = {{{0, 0}, {2, 0}, {1, 2}},
                                                    {{4, 4}, {2, 4}, {3, 2}}};

std::vector<std::vector<Vec2>> twoTangentTriangles = {
    {{0, 0}, {2, 0}, {1, 2}}, {{0, 0}, {-1, 2}, {-2, 0}}};

std::vector<std::vector<Vec2>> threeTangentTriangles = {
    {{0, 0}, {2, 0}, {1, 2}},
    {{0, 0}, {-1, 2}, {-2, 0}},
    {{0, 0}, {-1, -2}, {1, -2}}};

std::vector<std::vector<Vec2>> doubleTangentPolygons = {
    {{0, 0}, {2, 0}, {1, 1}, {2, 2}, {0, 2}},
    {{2, 0}, {4, 0}, {4, 2}, {2, 2}, {3, 1}}};

std::vector<std::vector<Vec2>> negativeTriangle = {{{0, 0}, {6, 0}, {3, 6}},
                                                   {{2, 2}, {3, 4}, {4, 2}}};

std::vector<std::vector<Vec2>> bubbleNegativeTriangle = {
    {{0, 0}, {6, 0}, {3, 6}}, {{0, 0}, {3, 4}, {4, 1}}};

std::vector<std::vector<Vec2>> twoTangentNegativeTriangles = {
    {{0, 0}, {8, 0}, {4, 8}},
    {{2, 1}, {3, 3}, {4, 1}},
    {{4, 1}, {5, 3}, {6, 1}}};

std::vector<std::vector<Vec2>> tangentPositiveAndNegativeTriangles = {
    {{0, 0}, {6, 0}, {3, 6}},
    {{0, 0}, {3, 4}, {4, 1}},
    {{0, 0}, {-1, 2}, {-2, 0}}};

std::vector<std::vector<Vec2>> sandwichedTriangles = {
    {{0, 0}, {12, 0}, {6, 12}},
    {{2, 1}, {6, 10}, {10, 1}},
    {{4, 2}, {8, 2}, {6, 8}}};

std::vector<std::vector<Vec2>> doubleSandwichedTriangles = {
    {{0, 0}, {12, 0}, {6, 12}},
    {{2, 1}, {6, 10}, {10, 1}},
    {{4, 2}, {8, 2}, {6, 8}},
    {{5, 3}, {6, 6}, {7, 3}}};

std::vector<std::vector<Vec2>> bubblePositiveAndNegativeTriangles = {
    {{0, 0}, {8, 0}, {4, 8}},
    {{0, 0}, {4, 6}, {6, 2}},
    {{0, 0}, {4, 4}, {4, 2}}};
}  // namespace samples
}  // namespace stl3lasercut
