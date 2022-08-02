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
}  // namespace samples
}  // namespace stl3lasercut
