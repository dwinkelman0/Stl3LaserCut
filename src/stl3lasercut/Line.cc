// Copyright 2022 by Daniel Winkelman. All rights reserved.

#include "Line.h"

#include <cmath>

namespace stl3lasercut {
Line::Line() : a_(0), b_(0), c_(0) {}

std::optional<Line> Line::fromPoints(const Vec2 &b1, const Vec2 &b2) {
  return b1 == b2 ? std::nullopt : std::optional<Line>(Line(b1, b2));
}

std::optional<Vec2> Line::getIntersection(const Line &other) const {
  if (std::abs(cross(getDirectionVector(), other.getDirectionVector())) <
      1e-6) {
    return std::nullopt;
  } else if (std::abs(a_) > 1e-6) {
    float ratio = other.a_ / a_;
    float y = (other.c_ - ratio * c_) / (other.b_ - ratio * b_);
    float x = (c_ - b_ * y) / a_;
    return std::optional<Vec2>({x, y});
  } else if (std::abs(other.a_) > 1e-6) {
    return other.getIntersection(*this);
  } else {
    return std::nullopt;
  }
}

Vec2 Line::getDirectionVector() const { return Vec2(a_, b_); }

std::ostream &operator<<(std::ostream &os, const Line &line) {
  if (line.a_ != 0 && line.b_ != 0) {
    os << line.a_ << "x + " << line.b_ << "y = " << line.c_;
  } else if (line.a_ != 0) {
    os << line.a_ << "x = " << line.c_;
  } else if (line.b_ != 0) {
    os << line.b_ << "y = " << line.c_;
  } else {
    os << "null line";
  }
  return os;
}

bool Line::operator<(const Line &other) const {
  return std::tie(a_, b_, c_) < std::tie(other.a_, other.b_, other.c_);
}

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

DirectedLine::DirectedLine() : Line() {}

DirectedLine::PointComparator::PointComparator(const DirectedLine &line)
    : direction_(line.getDirectionVector()) {}

bool DirectedLine::PointComparator::operator()(const Vec2 &a,
                                               const Vec2 &b) const {
  return cross(b - a, direction_) > 0;
}

Vec2 reflect(const Vec2 &vec) { return {std::get<0>(vec), -std::get<1>(vec)}; }

template <bool RightHanded>
DirectedLine::AngularComparator<RightHanded>::AngularComparator(
    const DirectedLine &line)
    : direction_(RightHanded ? line.getDirectionVector()
                             : reflect(line.getDirectionVector())) {}

template <bool RightHanded>
bool DirectedLine::AngularComparator<RightHanded>::operator()(
    const DirectedLine &a, const DirectedLine &b) const {
  Vec2 aVec =
      RightHanded ? a.getDirectionVector() : reflect(a.getDirectionVector());
  Vec2 bVec =
      RightHanded ? b.getDirectionVector() : reflect(b.getDirectionVector());
  if (cross(direction_, aVec) >= 0 && cross(direction_, bVec) >= 0) {
    return dot(direction_, bVec) < dot(direction_, aVec);
  } else if (cross(direction_, aVec) < 0 && cross(direction_, bVec) < 0) {
    return dot(direction_, aVec) < dot(direction_, bVec);
  } else if (cross(direction_, aVec) == 0 && dot(direction_, aVec) > 0) {
    return true;
  } else if (cross(direction_, bVec) == 0 && dot(direction_, bVec) > 0) {
    return false;
  } else {
    return cross(direction_, bVec) < cross(direction_, aVec);
  }
}

template <bool RightHanded>
bool DirectedLine::AngularComparator<RightHanded>::isZero(
    const DirectedLine &a) const {
  if (RightHanded) {
    return a.getDirectionVector() == direction_;
  } else {
    return reflect(a.getDirectionVector()) == direction_;
  }
}

template class DirectedLine::AngularComparator<true>;
template class DirectedLine::AngularComparator<false>;

bool DirectedLine::ParallelComparator::operator()(const DirectedLine &a,
                                                  const DirectedLine &b) const {
  if (a.getDirectionVector() != b.getDirectionVector()) {
    throw std::runtime_error("Lines are not parallel.");
  }
  return a.c_ < b.c_;
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

DirectedLine DirectedLine::getParallelLineThroughPoint(
    const Vec2 &point) const {
  return DirectedLine(a_, b_, dot(getDirectionVector(), point));
}

DirectedLine DirectedLine::getPerpendicularLineThroughPoint(
    const Vec2 &point, const bool isRightHanded) const {
  Vec2 vec = isRightHanded ? Vec2(-b_, a_) : Vec2(b_, -a_);
  return DirectedLine(std::get<0>(vec), std::get<1>(vec), dot(vec, point));
}

float DirectedLine::getAngle(const DirectedLine &other) const {
  float crossProduct = cross({a_, b_}, {other.a_, other.b_});
  return angle({a_, b_}, {other.a_, other.b_}) * (crossProduct > 0 ? 1 : -1);
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

BoundedLine::BoundedLine() : DirectedLine(), lower_(0, 0), upper_(0, 0) {}

namespace {
struct BoundVisitor {
 public:
  BoundVisitor(const DirectedLine &line) : line_(line) {}

  std::optional<Vec2> operator()(const Vec2 &point) const {
    return line_.getPerpendicularLineThroughPoint(point, true)
        .getIntersection(line_);
  }

  std::optional<Vec2> operator()(const DirectedLine &line) const {
    return line_.getIntersection(line);
  }

 private:
  DirectedLine line_;
};
}  // namespace

std::optional<BoundedLine> BoundedLine::fromDirectedLine(
    const DirectedLine &line, const Bound &lower, const Bound &upper) {
  BoundVisitor visitor(line);
  std::optional<Vec2> lowerBound = std::visit(visitor, lower);
  std::optional<Vec2> upperBound = std::visit(visitor, upper);
  return lowerBound && upperBound ? std::optional<BoundedLine>(BoundedLine(
                                        line, *lowerBound, *upperBound))
                                  : std::nullopt;
}

Vec2 BoundedLine::getLowerBound() const { return lower_; }

Vec2 BoundedLine::getUpperBound() const { return upper_; }

Vec2 BoundedLine::getMidpoint() const { return (upper_ + lower_) / 2; }

bool BoundedLine::isInverted() const {
  return PointComparator(*this)(upper_, lower_);
}

bool BoundedLine::isInBounds(const Vec2 &point) const {
  PointComparator comparator(*this);
  return isInverted()
             ? !(comparator(point, lower_) && comparator(upper_, point))
             : !(comparator(point, lower_) || comparator(upper_, point));
}

std::optional<Vec2> BoundedLine::getBoundedIntersection(
    const BoundedLine &line) const {
  std::optional<Vec2> intersection = Line::getIntersection(line);
  return intersection && isInBounds(*intersection) &&
                 line.isInBounds(*intersection)
             ? intersection
             : std::nullopt;
}

std::optional<Vec2> BoundedLine::getPartiallyBoundedIntersection(
    const Line &line) const {
  std::optional<Vec2> intersection = Line::getIntersection(line);
  return intersection && isInBounds(*intersection) ? intersection
                                                   : std::nullopt;
}

std::ostream &operator<<(std::ostream &os, const BoundedLine &line) {
  os << dynamic_cast<const Line &>(line) << ", " << line.lower_
     << " <= (x, y) <= " << line.upper_;
  return os;
}

BoundedLine::BoundedLine(const DirectedLine &directedLine, const Vec2 &lower,
                         const Vec2 &upper)
    : DirectedLine(directedLine), lower_(lower), upper_(upper) {}

bool isPointContainedInBounds(const std::vector<BoundedLine> &bounds,
                              const Vec2 &point) {
  Vec2 unitVector(1, 0);
  for (uint32_t i = 0; i < 100; ++i) {
    // Make quasi-random ray
    unitVector = rotate2D(unitVector, std::sqrt(i));
    DirectedLine line = *DirectedLine::fromPoints(point, point + unitVector);
    DirectedLine::PointComparator comparator(line);
    bool isValid = true;
    uint32_t numIntersections = 0;
    for (const BoundedLine &bound : bounds) {
      std::optional<Vec2> intersection = line.getIntersection(bound);
      if (!intersection) {
        isValid = false;
        break;
      } else {
        if (comparator(point, *intersection) &&
            bound.isInBounds(*intersection)) {
          ++numIntersections;
        }
      }
    }
    if (isValid) {
      return numIntersections % 2;
    }
  }
  throw std::runtime_error("Too many iterations, something is broken");
}
}  // namespace stl3lasercut
