// Copyright 2022 by Daniel Winkelman. All rights reserved.

#include <gtest/gtest.h>
#include <stl3lasercut/Line.h>

#include <cmath>

namespace stl3lasercut {
static void testIntersection(const Line &a, const Line &b, const Vec2 &value) {
  std::optional<Vec2> intersection = a.getIntersection(b);
  ASSERT_TRUE(intersection);
  ASSERT_FLOAT_EQ(std::get<0>(*intersection), std::get<0>(value));
  ASSERT_FLOAT_EQ(std::get<1>(*intersection), std::get<1>(value));
}

TEST(Line, Creation) {
  ASSERT_TRUE(Line::fromPoints({0, 0}, {1, 1}));
  ASSERT_FALSE(Line::fromPoints({1, 1}, {1, 1}));
  ASSERT_EQ(Line::fromPoints({0, 0}, {1, 1})->getDirectionVector(),
            Vec2(-1, 1) / std::sqrt(2));
}

TEST(Line, Intersection) {
  Line l0 = *Line::fromPoints({0, 1}, {1, 0});
  Line l1 = *Line::fromPoints({0, 0}, {1, 1});
  Line l2 = *Line::fromPoints({1, 2}, {2, 1});
  Line l3 = *Line::fromPoints({1, 0}, {1, 1});
  Line l4 = *Line::fromPoints({0, 2}, {1, 2});
  testIntersection(l0, l1, {0.5, 0.5});
  testIntersection(l1, l0, {0.5, 0.5});
  testIntersection(l1, l3, {1, 1});
  testIntersection(l1, l4, {2, 2});
  ASSERT_FALSE(l0.getIntersection(l2));
  ASSERT_FALSE(l0.getIntersection(l0));
}

TEST(Line, PerpendicularThroughPoint) {
  Line l0 = *Line::fromPoints({0, 0}, {2, 1});
  Line l1 = l0.getPerpendicularLineThroughPoint({0, 5});
  testIntersection(l0, l1, {2, 1});
}

TEST(Line, PointComparator) {
  DirectedLine l0 = *DirectedLine::fromPoints({0, 0}, {1, 1});
  DirectedLine l1 = *DirectedLine::fromPoints({1, 1}, {0, 0});
  DirectedLine::PointComparator c0(l0);
  DirectedLine::PointComparator c1(l1);
  ASSERT_TRUE(c0({0, 0}, {1, 1}));
  ASSERT_TRUE(c0({0, 0}, {5, -4}));
  ASSERT_FALSE(c1({0, 0}, {1, 1}));
  ASSERT_FALSE(c1({0, 0}, {5, -4}));
}

TEST(Line, ParallelFromOffset) {
  DirectedLine baseLine = *DirectedLine::fromPoints({0, 0}, {2, 1});
  DirectedLine l0 = baseLine.getParallelLineWithOffset(std::sqrt(5));
  DirectedLine l1 = baseLine.getParallelLineWithOffset(-std::sqrt(5));
  DirectedLine testLine = *DirectedLine::fromPoints({-1, 0}, {-1, 2});
  testIntersection(l0, testLine, {-1, 2});
  testIntersection(l1, testLine, {-1, -3});
}

TEST(Line, BoundedCreation) {
  BoundedLine l0 = *BoundedLine::fromPoints({0, 0}, {4, 2});
  BoundedLine l1 = *BoundedLine::fromUndirectedLine(
      *Line::fromPoints({0, 0}, {2, 1}), Vec2(-2, -1), Vec2(4, 2));
  BoundedLine l2 = *BoundedLine::fromUndirectedLine(
      *Line::fromPoints({0, 0}, {2, 1}), Vec2(4, 2), Vec2(-2, -1));
  BoundedLine l3 = *BoundedLine::fromUndirectedLine(
      *Line::fromPoints({0, 0}, {2, 1}), *Line::fromPoints({0, -1}, {1, -1}),
      *Line::fromPoints({5, 6}, {4, 2}));
  BoundedLine l4 = *BoundedLine::fromDirectedLine(
      *DirectedLine::fromPoints({0, 0}, {2, 1}), Vec2(0, 0), Vec2(4, 2));
  BoundedLine l5 = *BoundedLine::fromDirectedLine(
      *DirectedLine::fromPoints({0, 0}, {2, 1}), Vec2(4, 2), Vec2(0, 0));
  ASSERT_FALSE(l0.isInverted());
  ASSERT_FALSE(l1.isInverted());
  ASSERT_FALSE(l2.isInverted());
  ASSERT_FALSE(l3.isInverted());
  ASSERT_FALSE(l4.isInverted());
  ASSERT_TRUE(l5.isInverted());
  ASSERT_FALSE(BoundedLine::fromUndirectedLine(
      *Line::fromPoints({0, 0}, {2, 1}), *Line::fromPoints({1, 1}, {3, 2}),
      Vec2(0, 0)));
  ASSERT_FALSE(BoundedLine::fromUndirectedLine(
      *Line::fromPoints({0, 0}, {2, 1}), Vec2(4, 2), Vec2(4, 2)));
  ASSERT_FALSE(BoundedLine::fromDirectedLine(
      *DirectedLine::fromPoints({0, 0}, {2, 1}),
      *Line::fromPoints({1, 1}, {3, 2}), Vec2(0, 0)));
  ASSERT_TRUE(BoundedLine::fromDirectedLine(
      *DirectedLine::fromPoints({0, 0}, {2, 1}), Vec2(4, 2), Vec2(4, 2)));
}

TEST(Line, InBounds) {
  BoundedLine l0 = *BoundedLine::fromDirectedLine(
      *DirectedLine::fromPoints({0, 0}, {2, 1}), Vec2(0, 0), Vec2(4, 2));
  BoundedLine l1 = *BoundedLine::fromDirectedLine(
      *DirectedLine::fromPoints({0, 0}, {2, 1}), Vec2(4, 2), Vec2(0, 0));
  ASSERT_FALSE(l0.isInverted());
  ASSERT_FALSE(l0.isInBounds({-2, -1}));
  ASSERT_TRUE(l0.isInBounds({0, 0}));
  ASSERT_TRUE(l0.isInBounds({1, 1}));
  ASSERT_TRUE(l0.isInBounds({4, 2}));
  ASSERT_FALSE(l0.isInBounds({6, 3}));
  ASSERT_TRUE(l1.isInverted());
  ASSERT_TRUE(l1.isInBounds({-2, -1}));
  ASSERT_TRUE(l1.isInBounds({0, 0}));
  ASSERT_FALSE(l1.isInBounds({1, 1}));
  ASSERT_TRUE(l1.isInBounds({4, 2}));
  ASSERT_TRUE(l1.isInBounds({6, 3}));
}

TEST(Line, BoundedIntersection) {
  BoundedLine l0 = *BoundedLine::fromPoints({0, 0}, {4, 2});
  BoundedLine l1 = *BoundedLine::fromPoints({0, -1}, {1, 0});
  BoundedLine l2 = *BoundedLine::fromPoints({-1, -1}, {3, 1});
  BoundedLine l3 = *BoundedLine::fromPoints({0, 0}, {5, 6});
  ASSERT_FALSE(l0.isInverted());
  ASSERT_FALSE(l1.isInverted());
  std::optional<Vec2> i0 = l0.getIntersection(l1);
  ASSERT_TRUE(i0);
  ASSERT_FLOAT_EQ(std::get<0>(*i0), 2);
  ASSERT_FLOAT_EQ(std::get<1>(*i0), 1);
  ASSERT_FALSE(l1.getIntersection(l0));
  ASSERT_FALSE(l0.getIntersection(l2));
  ASSERT_FALSE(l0.getBoundedIntersection(l1));
  ASSERT_FALSE(l1.getBoundedIntersection(l0));
  ASSERT_FALSE(l0.getBoundedIntersection(l2));
  ASSERT_TRUE(l1.getBoundedIntersection(l2));
  ASSERT_TRUE(l0.getBoundedIntersection(l3));
}
}  // namespace stl3lasercut
