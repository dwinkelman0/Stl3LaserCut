// Copyright 2022 by Daniel Winkelman. All rights reserved.

#include "LoopPlane.h"

#include <stl3lasercut/AssemblyPlane.h>

#include <algorithm>

namespace stl3lasercut {
bool LoopPlane::Loop::Characteristic::operator<(
    const Characteristic &other) const {
  return std::tie(isPositive, size) < std::tie(other.isPositive, other.size);
}

bool LoopPlane::Loop::Characteristic::operator==(
    const Characteristic &other) const {
  return std::tie(isPositive, size) == std::tie(other.isPositive, other.size);
}

std::vector<uint32_t> removeAdjacentDuplicates(
    const std::vector<uint32_t> &vec) {
  assert(vec.size() > 0);
  std::vector<uint32_t> output;
  output.push_back(vec.front());
  for (const uint32_t item : vec) {
    if (item != output.back()) {
      output.push_back(item);
    }
  }
  if (output.back() == output.front()) {
    output.erase(output.end() - 1);
  }
  return output;
}

std::strong_ordering compare(
    const std::vector<uint32_t>::const_iterator &begin,
    const std::vector<uint32_t>::const_iterator &end,
    const std::vector<uint32_t>::const_iterator &other) {
  for (auto a = begin, b = other; a < end; ++a, ++b) {
    if (*a < *b) {
      return std::strong_ordering::less;
    } else if (*a > *b) {
      return std::strong_ordering::greater;
    }
  }
  return std::strong_ordering::equal;
}

std::vector<uint32_t> getCanonicalOrder(const std::vector<uint32_t> &vec) {
  std::vector<uint32_t> output(vec);
  for (uint32_t i = 0; i < vec.size(); ++i) {
    std::strong_ordering res =
        compare(vec.begin() + i, vec.end(), output.begin());
    if (res == std::strong_ordering::less ||
        res == std::strong_ordering::equal &&
            compare(vec.begin(), vec.begin() + i,
                    output.begin() + vec.size() - i) ==
                std::strong_ordering::less) {
      output.clear();
      output.insert(output.end(), vec.begin() + i, vec.end());
      output.insert(output.end(), vec.begin(), vec.begin() + i);
    }
  }
  return output;
}

LoopPlane::Loop::Loop(const std::shared_ptr<const LoopPlane> &loopPlane,
                      const std::vector<BoundedLine> &bounds,
                      const std::vector<uint32_t> &vertices,
                      const std::vector<uint32_t> &edges)
    : loopPlane_(loopPlane),
      vertexSet_(vertices.begin(), vertices.end()),
      bounds_(bounds),
      vertices_(vertices),
      edges_(getCanonicalOrder(removeAdjacentDuplicates(edges))) {}

std::pair<std::partial_ordering, bool> LoopPlane::Loop::operator<=>(
    const Loop &other) const {
  bool edgeComparison =
      std::pair<uint32_t, std::vector<uint32_t>>(edges_.size(), edges_) <
      std::pair<uint32_t, std::vector<uint32_t>>(other.edges_.size(),
                                                 other.edges_);
  std::vector<uint32_t> intersection;
  std::set_intersection(vertexSet_.begin(), vertexSet_.end(),
                        other.vertexSet_.begin(), other.vertexSet_.end(),
                        std::back_inserter(intersection));
  if (!intersection.empty()) {
    // If the loops have common vertices, then they are in the same partition
    return {std::partial_ordering::equivalent, edgeComparison};
  } else {
    // Check whether a point from this is contained in other
    return {contains(other)         ? std::partial_ordering::greater
            : other.contains(*this) ? std::partial_ordering::less
                                    : std::partial_ordering::unordered,
            edgeComparison};
  }
}

bool LoopPlane::Loop::isPositive() const {
  std::vector<Vec2> points;
  for (const BoundedLine &line : bounds_) {
    points.push_back(line.getLowerBound());
  }
  return getPolygonArea(points) > 0;
}

LoopPlane::Loop::Characteristic LoopPlane::Loop::getCharacteristic() const {
  return Characteristic{.isPositive = isPositive(),
                        .size = static_cast<uint32_t>(edges_.size())};
}

std::optional<uint32_t> LoopPlane::Loop::getEdgeId(const uint32_t v0,
                                                   const uint32_t v1) const {
  std::optional<Graph::EdgeConstIterator> edgeIt =
      loopPlane_->graph_.getEdge(v0, v1);
  return edgeIt ? std::optional<uint32_t>(Graph::unwrap(edgeIt).getValue())
                : std::nullopt;
}

bool LoopPlane::Loop::contains(const Loop &other) const {
  Vec2 testPoint = other.bounds_.front().getMidpoint();
  return isPointContainedInBounds(bounds_, testPoint);
}

LoopPlane::LoopPlane(const std::shared_ptr<AssemblyPlane> &assembly,
                     const uint32_t color)
    : assembly_(assembly), color_(color) {
  // Gather all unmatched edges from the AssemblyPlane
  for (const typename AssemblyPlane::Graph::ConstEdge &edge :
       assembly_->graph_.getEdges()) {
    if (!assembly_->graph_.getEdge(edge.getDest(), edge.getSource())) {
      // Place vertices
      graph_.emplaceVertex(edge.getSource());
      Graph::VertexIterator vertexIt =
          graph_.emplaceVertex(edge.getDest()).first;

      // Place edge
      graph_.emplaceEdge(edge.getSource(), edge.getDest()).first->getValue() =
          edge.getValue();

      vertexIt->getValue().emplace(
          edge.getSource(), AssemblyPlane::Graph::unwrap(
                                assembly_->graph_.getVertex(edge.getDest()))
                                .getValue()
                                .getFurthestConnection(edge.getSource()));
    }
  }
}

uint32_t LoopPlane::getColor() const { return color_; }

void LoopPlane::foreachEdge(
    const std::function<void(const Graph::ConstEdge &)> &func) const {
  for (const Graph::ConstEdge &edge : graph_.getEdges()) {
    func(edge);
  }
}

void LoopPlane::foreachEdgePair(
    const std::function<void(const Graph::ConstEdge &,
                             const Graph::ConstEdge &)> &func) const {
  foreachEdge([this, &func](const auto &edge) {
    Graph::ConstVertex centralVertex =
        Graph::unwrap(graph_.getVertex(edge.getDest()));
    Graph::ConstEdge nextEdge = Graph::unwrap(graph_.getEdge(
        edge.getDest(),
        centralVertex.getValue().find(edge.getSource())->second));
    func(edge, nextEdge);
  });
}

HierarchicalOrdering<LoopPlane::Loop> LoopPlane::getLoops() const {
  HierarchicalOrdering<Loop> output;
  std::set<std::pair<uint32_t, uint32_t>> visitedEdges;
  for (const Graph::ConstEdge &edge : graph_.getEdges()) {
    if (visitedEdges.find(std::pair<uint32_t, uint32_t>(
            edge.getSource(), edge.getDest())) == visitedEdges.end()) {
      std::vector<BoundedLine> bounds;
      std::vector<uint32_t> vertices;
      std::vector<uint32_t> edges;
      uint32_t source = edge.getSource();
      uint32_t dest = edge.getDest();
      std::pair<uint32_t, uint32_t> first(source, dest);
      do {
        visitedEdges.emplace(source, dest);
        std::optional<BoundedLine> line = BoundedLine::fromPoints(
            assembly_->pointLookup_(source), assembly_->pointLookup_(dest));
        assert(line);
        bounds.push_back(*line);
        vertices.push_back(source);
        uint32_t edgeIndex =
            Graph::unwrap(graph_.getEdge(source, dest)).getValue();
        edges.push_back(edgeIndex);
        auto it = Graph::unwrap(graph_.getVertex(dest)).getValue().find(source);
        assert(it != Graph::unwrap(graph_.getVertex(dest)).getValue().end());
        source = dest;
        dest = it->second;
      } while (std::pair<uint32_t, uint32_t>(source, dest) != first);
      Loop loop(shared_from_this(), bounds, vertices, edges);
      output.insert(loop);
    }
  }
  return output;
}

LoopPlane::Characteristic LoopPlane::getCharacteristic(
    const HierarchicalOrdering<Loop> &ordering) {
  return ordering.map<LoopPlane::Loop::Characteristic>(
      &LoopPlane::Loop::getCharacteristic);
}

std::ostream &operator<<(std::ostream &os,
                         const LoopPlane::Characteristic &ch) {
  os << "[";
  for (const auto &[key, value] : ch.getData()) {
    os << (key.isPositive ? '+' : '-') << key.size;
    if (value.getData().size() > 0) {
      os << ": " << value;
    }
    os << ", ";
  }
  os << "]";
  return os;
}
}  // namespace stl3lasercut
