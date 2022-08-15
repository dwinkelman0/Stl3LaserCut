// Copyright 2022 by Daniel Winkelman. All rights reserved.

#include "InterferencePlane.h"

#include <stl3lasercut/AssemblyPlane.h>
#include <stl3lasercut/RingVector.h>

namespace stl3lasercut {
bool InterferencePlane::EdgeCoordinate::operator<(
    const EdgeCoordinate &other) const {
  return std::tie(id, color, orientation) <
         std::tie(other.id, other.color, other.orientation);
}

std::ostream &operator<<(std::ostream &os,
                         const InterferencePlane::EdgeCoordinate &coord) {
  os << "(" << coord.id << ", "
     << (coord.orientation == InterferencePlane::Orientation::PARALLEL ? "par"
         : coord.orientation ==
                 InterferencePlane::Orientation::RIGHT_PERPENDICULAR
             ? "rperp"
             : "lperp")
     << ")[" << coord.color << "]";
  return os;
}

InterferencePlane::EdgeGroup::Comparator::Comparator(
    const std::shared_ptr<AssemblyPlane> &assemblyPlane,
    const DirectedLine &line)
    : assembly_(assemblyPlane), comparator_(line) {}

bool InterferencePlane::EdgeGroup::Comparator::operator()(
    const uint32_t a, const uint32_t b) const {
  return comparator_(assembly_->pointLookup_(a), assembly_->pointLookup_(b));
}

InterferencePlane::EdgeGroup::EdgeGroup(
    const std::shared_ptr<AssemblyPlane> &assemblyPlane,
    const DirectedLine &line)
    : points(Comparator(assemblyPlane, line)), line(line) {}

InterferencePlane::InterferencePlane(
    const std::shared_ptr<AssemblyPlane> &assemblyPlane)
    : assembly_(assemblyPlane) {}

void InterferencePlane::addLoopPlane(
    const std::shared_ptr<LoopPlane> &loopPlane) {
  loopPlane->getLoops().map([this, loopPlane](const LoopPlane::Loop &loop) {
    addParallelEdgesFromLoop(loop, loopPlane->getColor());
  });
}

void InterferencePlane::addParallelEdgesFromLoop(const LoopPlane::Loop &loop,
                                                 const uint32_t color) {
  RingVector<uint32_t> ring(loop.vertices_);
  ring.foreach ([this](const uint32_t vertex) { addPoint(vertex); });
  ring.foreachPair([this, &loop, color](const uint32_t v0, const uint32_t v1) {
    std::optional<uint32_t> edgeId = loop.getEdgeId(v0, v1);
    assert(edgeId);
    addEdge(v0, v1, *edgeId, color);
  });
  ring.foreach (
      [this, &loop, color](const std::vector<uint32_t> &vertices) {
        uint32_t v0 = vertices[0], v1 = vertices[1], v2 = vertices[2];
        auto e0 = loop.getEdgeId(v0, v1), e1 = loop.getEdgeId(v1, v2);
        assert(e0 && e1);
        addAngle(v0, v1, v2, *e0, *e1);
      },
      3);
}

bool InterferencePlane::addPoint(const uint32_t index) {
  return graph_
      .emplaceVertex(index, MultiVertexConnectivityGraph(assembly_, index))
      .second;
}

void InterferencePlane::addEdge(const uint32_t v0, const uint32_t v1,
                                const uint32_t edgeId, const uint32_t color) {
  EdgeCoordinate coord{
      .id = edgeId, .color = color, .orientation = Orientation::PARALLEL};
  std::optional<Graph::EdgeIterator> existingEdge = graph_.getEdge(v0, v1);
  std::shared_ptr<EdgeGroup> group(nullptr);
  if (existingEdge) {
    group = Graph::unwrap(existingEdge).getValue();
    group->edges.insert(coord);
    auto pointSet = group->points;
    assert(pointSet.find(v0) != pointSet.end() &&
           pointSet.find(v1) != pointSet.end());
  } else {
    std::optional<BoundedLine> line = BoundedLine::fromPoints(
        assembly_->pointLookup_(v0), assembly_->pointLookup_(v1));
    assert(line);
    auto it = groupMap_.find(*line);
    group =
        it != groupMap_.end()
            ? it->second
            : groupMap_
                  .emplace(*line, std::make_shared<EdgeGroup>(assembly_, *line))
                  .first->second;
    group->edges.insert(coord);
    group->points.insert(v0);
    group->points.insert(v1);
    graph_.emplaceEdge(v0, v1).first->getValue() = group;
  }
  edges_.emplace(coord,
                 LogicalEdge{.group = group,
                             .orientationClass = OrientationClass::PARALLEL});
}

void InterferencePlane::addAngle(const uint32_t v0, const uint32_t v1,
                                 const uint32_t v2, const uint32_t e0,
                                 const uint32_t e1) {
  Graph::unwrap(graph_.getVertex(v1)).getValue().connect(v0, v2);
  parallelEdgeAdjacency_
      .emplace(e0, std::pair<uint32_t, uint32_t>(
                       std::numeric_limits<uint32_t>::max(),
                       std::numeric_limits<uint32_t>::max()))
      .first->second.second = e1;
  parallelEdgeAdjacency_
      .emplace(e1, std::pair<uint32_t, uint32_t>(
                       std::numeric_limits<uint32_t>::max(),
                       std::numeric_limits<uint32_t>::max()))
      .first->second.first = e0;
}

void InterferencePlane::computeInterference(const EdgeCoordinate &coord,
                                            const uint32_t color) {
  auto it = edges_.find(coord);
  assert(it != edges_.end());
  for (const auto &[otherCoord, logicalEdge] : edges_) {
    if (otherCoord.color == color) {
      findAndInsertIntersection(it->second.group, logicalEdge.group);
    }
  }
}

void InterferencePlane::findAndInsertIntersection(
    const std::shared_ptr<EdgeGroup> &a, const std::shared_ptr<EdgeGroup> &b) {
  std::optional<Vec2> intersection = a->line.getIntersection(b->line);
  if (intersection) {
    // Create a new vertex in the assembly
    uint32_t newVertex = assembly_->pointLookup_(*intersection);
    bool vertexIsNew = addPoint(newVertex);

    // Splice the new vertex in the edge groups
    const auto [aLower, aUpper] = insertVertexInEdgeGroup(a, newVertex);
    const auto [bLower, bUpper] = insertVertexInEdgeGroup(b, newVertex);
  }
}

std::pair<std::optional<uint32_t>, std::optional<uint32_t>>
InterferencePlane::insertVertexInEdgeGroup(
    const std::shared_ptr<EdgeGroup> &group, const uint32_t vertex) {
  const auto [it, success] = group->points.emplace(vertex);
  if (success) {
    // New vertex inserted, get bounding vertices
    auto upper = std::next(it, 1);
    if (it == group->points.begin()) {
      // The vertex was inserted at the front
      if (upper != group->points.end()) {
        graph_.emplaceEdge(*it, *upper).first->getValue() = group;
        Graph::unwrap(graph_.getVertex(vertex))
            .getValue()
            .addVertex(*upper, false);
        return {std::nullopt, *upper};
      } else {
        return {std::nullopt, std::nullopt};
      }
    } else {
      auto lower = std::next(it, -1);
      if (upper == group->points.end()) {
        // The vertex was inserted at the back
        graph_.emplaceEdge(*lower, *it);
        Graph::unwrap(graph_.getVertex(vertex))
            .getValue()
            .addVertex(*lower, true);
        return {*lower, std::nullopt};
      } else {
        // The vertex was inserted between two vertices
        assert(lower != group->points.begin() && upper != group->points.end());
        graph_.eraseEdge(Graph::unwrap(graph_.getEdge(*lower, *upper)));
        graph_.emplaceEdge(*lower, *it);
        graph_.emplaceEdge(*it, *upper);
        Graph::unwrap(graph_.getVertex(vertex))
            .getValue()
            .connect(*lower, *upper);

        // Fix vertex connectivity graph of adjacent vertices
        Graph::unwrap(graph_.getVertex(*lower))
            .getValue()
            .rename(*upper, vertex);
        Graph::unwrap(graph_.getVertex(*upper))
            .getValue()
            .rename(*lower, vertex);
        Graph::unwrap(graph_.getVertex(vertex))
            .getValue()
            .connect(*lower, *upper);

        return {*lower, *upper};
      }
    }
  } else {
    auto upper = std::next(it, 1);
    if (it == group->points.begin()) {
      return {std::nullopt, upper == group->points.end()
                                ? std::nullopt
                                : std::optional<uint32_t>(*upper)};
    } else {
      auto lower = std::next(it, -1);
      return {*lower, upper == group->points.end()
                          ? std::nullopt
                          : std::optional<uint32_t>(*upper)};
    }
  }
}
}  // namespace stl3lasercut
