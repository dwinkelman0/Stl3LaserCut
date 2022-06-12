// Copyright 2022 by Daniel Winkelman. All rights reserved.

#include "Line.h"

namespace stl3lasercut {
std::optional<Line> Line::fromPoints(const Vec2 &b1, const Vec2 &b2) {
  return b1 == b2 ? std::nullopt : std::optional<Line>(Line(b1, b2));
}

std::optional<Vec2> Line::getIntersection(const Line &other) const {
  if (std::abs(cross(getDirectionVector(), other.getDirectionVector())) <
      1e-6) {
    return std::nullopt;
  } else if (a_ != 0) {
    float ratio = other.a_ / a_;
    float y = (other.c_ - ratio * c_) / (other.b_ - ratio * b_);
    float x = (c_ - b_ * y) / a_;
    return std::optional<Vec2>({x, y});
  } else {
    return other.getIntersection(*this);
  }
}

Line Line::getPerpendicularLineThroughPoint(const Vec2 &point) const {
  return Line(b_, -a_, b_ * std::get<0>(point) - a_ * std::get<1>(point));
}

Vec2 Line::getDirectionVector() const { return Vec2(a_, b_); }

Line::Line(const float a, const float b, const float c) : a_(a), b_(b), c_(c) {}

Line::Line(const Vec2 &b1, const Vec2 &b2)
    : a_(std::get<1>(b1) - std::get<1>(b2)),
      b_(std::get<0>(b2) - std::get<0>(b1)),
      c_(a_ * std::get<0>(b1) + b_ * std::get<1>(b1)) {
  float magnitude = abs(Vec2(a_, b_));
  a_ /= magnitude;
  b_ /= magnitude;
  c_ /= magnitude;
}

DirectedLine::PointComparator::PointComparator(const DirectedLine &line)
    : direction_(line.getDirectionVector()) {}

bool DirectedLine::PointComparator::operator()(const Vec2 &a,
                                               const Vec2 &b) const {
  return cross(b - a, direction_) > 0;
}

std::optional<DirectedLine> DirectedLine::fromPoints(const Vec2 &b1,
                                                     const Vec2 &b2) {
  std::optional<Line> output = Line::fromPoints(b1, b2);
  return output ? std::optional<DirectedLine>(DirectedLine(*output))
                : std::nullopt;
}

DirectedLine DirectedLine::getParallelLineWithOffset(const float offset) const {
  return DirectedLine(a_, b_, c_ + offset * abs(Vec2(a_, b_)));
}

DirectedLine::DirectedLine(const Line &line) : Line(line) {}

DirectedLine::DirectedLine(const float a, const float b, const float c)
    : Line(a, b, c) {}

std::optional<BoundedLine> BoundedLine::fromPoints(const Vec2 &lower,
                                                   const Vec2 &upper) {
  std::optional<DirectedLine> output = DirectedLine::fromPoints(lower, upper);
  return output ? std::optional<BoundedLine>(BoundedLine(*output, lower, upper))
                : std::nullopt;
}

namespace {
struct BoundVisitor {
 public:
  BoundVisitor(const Line &line) : line_(line) {}

  std::optional<Vec2> operator()(const Vec2 &point) const {
    return line_.getPerpendicularLineThroughPoint(point).getIntersection(line_);
  }

  std::optional<Vec2> operator()(const Line &line) const {
    return line_.getIntersection(line);
  }

 private:
  Line line_;
};
}  // namespace

std::optional<BoundedLine> BoundedLine::fromUndirectedLine(const Line &line,
                                                           const Bound &lower,
                                                           const Bound &upper) {
  BoundVisitor visitor(line);
  std::optional<Vec2> lowerBound = std::visit(visitor, lower);
  std::optional<Vec2> upperBound = std::visit(visitor, upper);
  if (lowerBound && upperBound) {
    std::optional<DirectedLine> directedLine =
        DirectedLine::fromPoints(*lowerBound, *upperBound);
    return directedLine ? std::optional<BoundedLine>(BoundedLine(
                              *directedLine, *lowerBound, *upperBound))
                        : std::nullopt;
  } else {
    return std::nullopt;
  }
}

std::optional<BoundedLine> BoundedLine::fromDirectedLine(
    const DirectedLine &line, const Bound &lower, const Bound &upper) {
  BoundVisitor visitor(line);
  std::optional<Vec2> lowerBound = std::visit(visitor, lower);
  std::optional<Vec2> upperBound = std::visit(visitor, upper);
  return lowerBound && upperBound ? std::optional<BoundedLine>(BoundedLine(
                                        line, *lowerBound, *upperBound))
                                  : std::nullopt;
}

bool BoundedLine::isInverted() const {
  return PointComparator(*this)(upper_, lower_);
}

bool BoundedLine::isInBounds(const Vec2 &point) const {
  PointComparator comparator(*this);
  return isInverted()
             ? !(comparator(point, lower_) && comparator(upper_, point))
             : !(comparator(point, lower_) || comparator(upper_, point));
}

std::optional<Vec2> BoundedLine::getIntersection(const Line &line) const {
  std::optional<Vec2> intersection = Line::getIntersection(line);
  return intersection && isInBounds(*intersection) ? intersection
                                                   : std::nullopt;
}

std::optional<Vec2> BoundedLine::getBoundedIntersection(
    const BoundedLine &line) const {
  std::optional<Vec2> intersection = Line::getIntersection(line);
  return intersection && isInBounds(*intersection) &&
                 line.isInBounds(*intersection)
             ? intersection
             : std::nullopt;
}

BoundedLine::BoundedLine(const DirectedLine &directedLine, const Vec2 &lower,
                         const Vec2 &upper)
    : DirectedLine(directedLine), lower_(lower), upper_(upper) {}
}  // namespace stl3lasercut
