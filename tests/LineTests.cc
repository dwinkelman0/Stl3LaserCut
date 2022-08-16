// Copyright 2022 by Daniel Winkelman. All rights reserved.

#include <gtest/gtest.h>
#include <stl3lasercut/Line.h>
#include <stl3lasercut/RingVector.h>

#include <cmath>
#include <numbers>

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

TEST(Line, AngularComparator) {
  DirectedLine l0 = *DirectedLine::fromPoints({0, 0}, {2, 1});
  DirectedLine l1 = *DirectedLine::fromPoints({0, 0}, {1, 5});
  DirectedLine l2 = *DirectedLine::fromPoints({0, 0}, {-1, 6});
  DirectedLine l3 = *DirectedLine::fromPoints({0, 0}, {-1, 2});
  DirectedLine l4 = *DirectedLine::fromPoints({0, 0}, {-1, 1});
  DirectedLine l5 = *DirectedLine::fromPoints({0, 0}, {-2, -1});
  DirectedLine l6 = *DirectedLine::fromPoints({0, 0}, {1, -2});
  DirectedLine l7 = *DirectedLine::fromPoints({0, 0}, {3, -1});
  DirectedLine l8 = *DirectedLine::fromPoints({0, 0}, {7, 1});
  DirectedLine::AngularComparator<true> comparator(l0);
  std::vector<DirectedLine> lines = {l0, l1, l2, l3, l4, l5, l6, l7, l8};
  for (auto it = lines.begin(); it != lines.end(); ++it) {
    for (auto jt = lines.begin(); jt != lines.end(); ++jt) {
      if (it < jt) {
        ASSERT_TRUE(comparator(*it, *jt));
        ASSERT_FALSE(comparator(*jt, *it));
      } else if (it > jt) {
        ASSERT_FALSE(comparator(*it, *jt));
        ASSERT_TRUE(comparator(*jt, *it));
      } else {
        ASSERT_FALSE(comparator(*it, *jt));
        ASSERT_FALSE(comparator(*jt, *it));
      }
    }
  }
}

TEST(Line, ParallelComparator) {
  DirectedLine l0 = *DirectedLine::fromPoints({0, 0}, {4, 5});
  DirectedLine l1 = *DirectedLine::fromPoints({0, 3}, {4, 8});
  DirectedLine l2 = *DirectedLine::fromPoints({0, 0}, {4, 6});
  DirectedLine::ParallelComparator comparator;
  ASSERT_TRUE(comparator(l0, l1));
  ASSERT_FALSE(comparator(l1, l0));
  ASSERT_THROW({ comparator(l0, l2); }, std::runtime_error);
}

TEST(Line, ParallelFromOffset) {
  DirectedLine baseLine = *DirectedLine::fromPoints({0, 0}, {2, 1});
  DirectedLine l0 = baseLine.getParallelLineWithOffset(std::sqrt(5));
  DirectedLine l1 = baseLine.getParallelLineWithOffset(-std::sqrt(5));
  DirectedLine testLine = *DirectedLine::fromPoints({-1, 0}, {-1, 2});
  testIntersection(l0, testLine, {-1, 2});
  testIntersection(l1, testLine, {-1, -3});
}

TEST(Line, ParallelThroughPoint) {
  DirectedLine baseLine = *DirectedLine::fromPoints({0, 0}, {2, 1});
  DirectedLine l0 = baseLine.getParallelLineThroughPoint({4, 3});
  DirectedLine testLine = *DirectedLine::fromPoints({2, 0}, {2, 5});
  testIntersection(l0, testLine, {2, 2});
}

TEST(Line, PerpendicularThroughPoint) {
  DirectedLine l0 = *DirectedLine::fromPoints({0, 0}, {2, 1});
  DirectedLine l1 = l0.getPerpendicularLineThroughPoint({0, 5}, true);
  ASSERT_GT(cross(l0.getDirectionVector(), l1.getDirectionVector()), 0);
  testIntersection(l0, l1, {2, 1});
  DirectedLine l2 = l0.getPerpendicularLineThroughPoint({0, 5}, false);
  ASSERT_LT(cross(l0.getDirectionVector(), l2.getDirectionVector()), 0);
  testIntersection(l0, l2, {2, 1});
}

TEST(Line, Angle) {
  DirectedLine l0 = *DirectedLine::fromPoints({0, 0}, {0, 1});
  DirectedLine l1 = *DirectedLine::fromPoints({0, 1}, {-1, 2});
  DirectedLine l2 = *DirectedLine::fromPoints({0, 1}, {1, 2});
  DirectedLine l3 = *DirectedLine::fromPoints({0, 1}, {1, 0});
  ASSERT_FLOAT_EQ(l0.getAngle(l1), std::numbers::pi / 4);
  ASSERT_FLOAT_EQ(l0.getAngle(l2), -std::numbers::pi / 4);
  ASSERT_FLOAT_EQ(l0.getAngle(l3), -std::numbers::pi * 3 / 4);
  ASSERT_FLOAT_EQ(l1.getAngle(l0), -std::numbers::pi / 4);
}

TEST(Line, BoundedCreation) {
  BoundedLine l0 = *BoundedLine::fromPoints({0, 0}, {4, 2});
  BoundedLine l4 = *BoundedLine::fromDirectedLine(
      *DirectedLine::fromPoints({0, 0}, {2, 1}), Vec2(0, 0), Vec2(4, 2));
  BoundedLine l5 = *BoundedLine::fromDirectedLine(
      *DirectedLine::fromPoints({0, 0}, {2, 1}), Vec2(4, 2), Vec2(0, 0));
  ASSERT_FALSE(l0.isInverted());
  ASSERT_FALSE(l4.isInverted());
  ASSERT_TRUE(l5.isInverted());
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
  std::optional<Vec2> i0 = l0.getPartiallyBoundedIntersection(l1);
  ASSERT_TRUE(i0);
  ASSERT_FLOAT_EQ(std::get<0>(*i0), 2);
  ASSERT_FLOAT_EQ(std::get<1>(*i0), 1);
  ASSERT_FALSE(l1.getPartiallyBoundedIntersection(l0));
  ASSERT_FALSE(l0.getPartiallyBoundedIntersection(l2));
  ASSERT_FALSE(l0.getBoundedIntersection(l1));
  ASSERT_FALSE(l1.getBoundedIntersection(l0));
  ASSERT_FALSE(l0.getBoundedIntersection(l2));
  ASSERT_TRUE(l1.getBoundedIntersection(l2));
  ASSERT_TRUE(l0.getBoundedIntersection(l3));
}

TEST(Line, IsPointContainedInBounds) {
  std::vector<Vec2> points = {{0, 0}, {4, 0}, {4, 3}, {2, 1}, {0, 3}};
  std::vector<BoundedLine> bounds;
  RingVector<Vec2>(points).foreachPair([&bounds](const Vec2 &a, const Vec2 &b) {
    std::optional<BoundedLine> line = BoundedLine::fromPoints(a, b);
    ASSERT_TRUE(line);
    bounds.push_back(*line);
  });
  ASSERT_TRUE(isPointContainedInBounds(bounds, {1, 0.5}));
  ASSERT_TRUE(isPointContainedInBounds(bounds, {1, 1}));
  ASSERT_TRUE(isPointContainedInBounds(bounds, {1, 1.5}));
  ASSERT_TRUE(isPointContainedInBounds(bounds, {1.9, 1}));
  ASSERT_FALSE(isPointContainedInBounds(bounds, {-1, 0}));
  ASSERT_FALSE(isPointContainedInBounds(bounds, {-1, 1}));
  ASSERT_FALSE(isPointContainedInBounds(bounds, {-1, 2}));
}
}  // namespace stl3lasercut
