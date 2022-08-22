// Copyright 2022 by Daniel Winkelman. All rights reserved.

#pragma once

#include <stl3lasercut/InterferencePlane.h>
#include <stl3lasercut/RingVector.h>
#include <stl3lasercut/Util.h>

namespace stl3lasercut {
void testPoint(const Vec2 &a, const Vec2 &b);
void testPoint(const Vec3 &a, const Vec3 &b);

extern const uint32_t BASE_COLOR;
extern const uint32_t INTERMEDIATE_COLOR;
extern const uint32_t OFFSET_COLOR;

namespace offset {
std::vector<InterferencePlane::OffsetCalculation> single(
    const InterferencePlane::OffsetFunction &function,
    const bool calculateInterference);

InterferencePlane::OffsetCalculation first(
    const InterferencePlane::OffsetFunction &function,
    const bool calculateInterference);

InterferencePlane::OffsetCalculation second(
    const InterferencePlane::OffsetFunction &function,
    const bool calculateInterference);

InterferencePlane::OffsetFunction constant(const float offset);

InterferencePlane::OffsetFunction ring(const RingVector<float> &ring);
}  // namespace offset
}  // namespace stl3lasercut
