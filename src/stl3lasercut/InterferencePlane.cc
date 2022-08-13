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

std::ostream &operator<<(std::ostream &os,
                         const InterferencePlane::LogicalEdge &edge) {
  os << "(" << edge.edgeId << ", "
     << (edge.orientationClass == InterferencePlane::OrientationClass::PARALLEL
             ? "par"
             : "perp")
     << "): lower = " << edge.lower << ", upper = " << edge.upper << ", "
     << edge.group->line;
  return os;
}

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
    auto edgeId = loop.getEdgeId(v0, v1);
    assert(edgeId);
    addEdge(v0, v1, *edgeId, color);
  });
  ring.foreach (
      [this, &loop, color](const std::vector<uint32_t> &vertices) {
        uint32_t v0 = vertices[0], v1 = vertices[1], v2 = vertices[2];
        auto e0 = loop.getEdgeId(v0, v1), e1 = loop.getEdgeId(v1, v2);
        assert(e0 && e1);
        addAngle(v0, v1, v2, *e0, *e1, color);
      },
      3);
}

void InterferencePlane::addPoint(const uint32_t index) {
  graph_.emplaceVertex(index, VertexConnectivityGraph(assembly_, index, false));
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
                             .orientationClass = OrientationClass::PARALLEL,
                             .edgeId = edgeId,
                             .lower = std::numeric_limits<uint32_t>::max(),
                             .upper = std::numeric_limits<uint32_t>::max()});
}

void InterferencePlane::addAngle(const uint32_t v0, const uint32_t v1,
                                 const uint32_t v2, const uint32_t e0,
                                 const uint32_t e1, const uint32_t color) {
  Graph::unwrap(graph_.getVertex(v1)).getValue().connect(v0, v2);
  auto it0 = edges_.find(EdgeCoordinate{
      .id = e0, .color = color, .orientation = Orientation::PARALLEL});
  assert(it0 != edges_.end());
  it0->second.upper = e1;
  auto it1 = edges_.find(EdgeCoordinate{
      .id = e1, .color = color, .orientation = Orientation::PARALLEL});
  assert(it0 != edges_.end());
  it1->second.lower = e0;
}
}  // namespace stl3lasercut
