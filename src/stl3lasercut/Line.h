// Copyright 2022 by Daniel Winkelman. All rights reserved.

#pragma once

#include <stl3lasercut/Util.h>

#include <memory>
#include <optional>
#include <variant>
#include <vector>

namespace stl3lasercut {
class Line {
 public:
  Line();

  static std::optional<Line> fromPoints(const Vec2 &b1, const Vec2 &b2);

  std::optional<Vec2> getIntersection(const Line &other) const;
  Vec2 getDirectionVector() const;

  bool operator<(const Line &other) const;
  friend std::ostream &operator<<(std::ostream &os, const Line &line);

 protected:
  Line(const float a, const float b, const float c);

 private:
  Line(const Vec2 &b1, const Vec2 &b2);

 protected:
  float a_, b_, c_; /** (a, b) vector is unit, c is positive. ax + by = c. */
};

class DirectedLine : public Line {
 public:
  class PointComparator {
   public:
    PointComparator(const DirectedLine &line);
    bool operator()(const Vec2 &a, const Vec2 &b) const;

   private:
    Vec2 direction_;
  };

  /** Compare directed lines based on their angle within [0, 2pi). 0 is
   * established by passing a reference line in the constructor. If RightHanded
   * is true, then a > b if the CCW angle of a relative to the reference is
   * greater than the CCW angle of b relative to the reference. */
  template <bool RightHanded>
  class AngularComparator {
   public:
    AngularComparator(const DirectedLine &line);
    bool operator()(const DirectedLine &a, const DirectedLine &b) const;
    bool isZero(const DirectedLine &a) const;

   private:
    Vec2 direction_;
  };

  class ParallelComparator {
   public:
    bool operator()(const DirectedLine &a, const DirectedLine &b) const;
  };

  DirectedLine();

  static std::optional<DirectedLine> fromPoints(const Vec2 &b1, const Vec2 &b2);

  DirectedLine getParallelLineWithOffset(const float offset) const;
  DirectedLine getParallelLineThroughPoint(const Vec2 &point) const;
  DirectedLine getPerpendicularLineThroughPoint(const Vec2 &point,
                                                const bool isRightHanded) const;
  float getAngle(const DirectedLine &other) const;

 private:
  DirectedLine(const Line &line);
  DirectedLine(const float a, const float b, const float c);
};

class BoundedLine : public DirectedLine {
 public:
  using Bound = std::variant<Vec2, DirectedLine>;

  BoundedLine();

  static std::optional<BoundedLine> fromPoints(const Vec2 &lower,
                                               const Vec2 &upper);
  static std::optional<BoundedLine> fromDirectedLine(const DirectedLine &line,
                                                     const Bound &lower,
                                                     const Bound &upper);

  Vec2 getLowerBound() const;
  Vec2 getUpperBound() const;
  Vec2 getMidpoint() const;

  bool isInverted() const;
  bool isInBounds(const Vec2 &point) const;
  std::optional<Vec2> getBoundedIntersection(const BoundedLine &line) const;
  std::optional<Vec2> getPartiallyBoundedIntersection(const Line &line) const;

  friend std::ostream &operator<<(std::ostream &os, const BoundedLine &line);

 private:
  BoundedLine(const DirectedLine &directedLine, const Vec2 &lower,
              const Vec2 &upper);
  Vec2 lower_, upper_;
};

bool isPointContainedInBounds(const std::vector<BoundedLine> &bounds,
                              const Vec2 &point);
}  // namespace stl3lasercut
