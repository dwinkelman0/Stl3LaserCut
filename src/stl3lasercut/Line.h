// Copyright 2022 by Daniel Winkelman. All rights reserved.

#pragma once

#include <stl3lasercut/Util.h>

#include <memory>
#include <optional>
#include <variant>

namespace stl3lasercut {
class Line {
 public:
  Line();

  static std::optional<Line> fromPoints(const Vec2 &b1, const Vec2 &b2);

  std::optional<Vec2> getIntersection(const Line &other) const;
  Line getPerpendicularLineThroughPoint(const Vec2 &point) const;
  Vec2 getDirectionVector() const;

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

  DirectedLine();

  static std::optional<DirectedLine> fromPoints(const Vec2 &b1, const Vec2 &b2);

  DirectedLine getParallelLineWithOffset(const float offset) const;
  DirectedLine getParallelLineThroughPoint(const Vec2 &point) const;
  float getAngle(const DirectedLine &other) const;

 private:
  DirectedLine(const Line &line);
  DirectedLine(const float a, const float b, const float c);
};

class BoundedLine : public DirectedLine {
 public:
  using Bound = std::variant<Vec2, Line>;

  BoundedLine();

  static std::optional<BoundedLine> fromPoints(const Vec2 &lower,
                                               const Vec2 &upper);
  static std::optional<BoundedLine> fromUndirectedLine(const Line &line,
                                                       const Bound &lower,
                                                       const Bound &upper);
  static std::optional<BoundedLine> fromDirectedLine(const DirectedLine &line,
                                                     const Bound &lower,
                                                     const Bound &upper);

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
}  // namespace stl3lasercut
