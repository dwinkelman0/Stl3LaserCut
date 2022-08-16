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
                         const InterferencePlane::EdgeGroup &group) {
  std::copy(group.edges.begin(), group.edges.end(),
            std::ostream_iterator<InterferencePlane::EdgeCoordinate>(os, ", "));
  os << ": ";
  std::copy(group.points.begin(), group.points.end(),
            std::ostream_iterator<uint32_t>(os, " -> "));
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

void InterferencePlane::applyOffsetFunction(const OffsetFunction &func,
                                            const uint32_t baseColor,
                                            const uint32_t perpendicularColor,
                                            const uint32_t newColor) {
  for (const auto &[coord, group] : edges_) {
    if (coord.color == baseColor &&
        coord.orientation == Orientation::PARALLEL) {
      // Fix this to get assembly plane of corresponding edge
      addParallelEdgeFromOffset(coord, newColor, func(assembly_, assembly_));
    }
  }
  fixVertexConnectivity();
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
    std::optional<DirectedLine> line = DirectedLine::fromPoints(
        assembly_->pointLookup_(v0), assembly_->pointLookup_(v1));
    assert(line);
    auto it = groupMap_.find(*line);
    group =
        groupMap_.emplace(*line, std::make_shared<EdgeGroup>(assembly_, *line))
            .first->second;
    group->edges.insert(coord);
    group->points.insert(v0);
    group->points.insert(v1);
    graph_.emplaceEdge(v0, v1).first->getValue() = group;
  }
  edges_.emplace(coord, group);
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

void InterferencePlane::addParallelEdgeFromOffset(const EdgeCoordinate &coord,
                                                  const uint32_t newColor,
                                                  const float offset) {
  auto it = edges_.find(coord);
  if (it != edges_.end()) {
    EdgeCoordinate newCoord = {.id = coord.id,
                               .color = newColor,
                               .orientation = Orientation::PARALLEL};
    DirectedLine offsetLine =
        it->second->line.getParallelLineWithOffset(offset);
    std::shared_ptr<EdgeGroup> group =
        groupMap_
            .emplace(offsetLine,
                     std::make_shared<EdgeGroup>(assembly_, offsetLine))
            .first->second;
    group->edges.insert(newCoord);
    edges_.emplace(newCoord, group);
    computeInterferenceWithAdjacentEdges(newCoord);
    computeInterferenceWithColor(newCoord, newColor);
    computeInterferenceWithColor(newCoord, coord.color);
  }
}

void InterferencePlane::computeInterferenceWithAdjacentEdges(
    const EdgeCoordinate &coord) {
  auto adjacencyIt = parallelEdgeAdjacency_.find(coord.id);
  assert(adjacencyIt != parallelEdgeAdjacency_.end());
  const auto &[lower, upper] = adjacencyIt->second;
  std::set<std::shared_ptr<EdgeGroup>> groups;
  for (const auto &[other, group] : edges_) {
    if (other.id == lower || other.id == upper) {
      groups.insert(group);
    }
  }
  auto edgeIt = edges_.find(coord);
  assert(edgeIt != edges_.end());
  for (const std::shared_ptr<EdgeGroup> &group : groups) {
    findAndInsertIntersection(edgeIt->second, group);
  }
}

void InterferencePlane::computeInterferenceWithColor(
    const EdgeCoordinate &coord, const uint32_t color) {
  std::set<std::shared_ptr<EdgeGroup>> groups;
  for (const auto &[other, group] : edges_) {
    if (other.color == color) {
      groups.insert(group);
    }
  }
  auto edgeIt = edges_.find(coord);
  assert(edgeIt != edges_.end());
  for (const std::shared_ptr<EdgeGroup> &group : groups) {
    findAndInsertIntersection(edgeIt->second, group);
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
    insertVertexInEdgeGroup(a, newVertex);
    insertVertexInEdgeGroup(b, newVertex);
  }
}

void InterferencePlane::insertVertexInEdgeGroup(
    const std::shared_ptr<EdgeGroup> &group, const uint32_t vertex) {
  const auto [it, success] = group->points.emplace(vertex);
  if (success) {
    // New vertex inserted, get bounding vertices
    auto upper = std::next(it, 1);
    if (it == group->points.begin()) {
      // The vertex was inserted at the front
      if (upper != group->points.end()) {
        graph_.emplaceEdge(*it, *upper).first->getValue() = group;
      }
    } else {
      auto lower = std::next(it, -1);
      if (upper == group->points.end()) {
        // The vertex was inserted at the back
        graph_.emplaceEdge(*lower, *it).first->getValue() = group;
      } else {
        // The vertex was inserted between two vertices
        graph_.eraseEdge(Graph::unwrap(graph_.getEdge(*lower, *upper)));
        graph_.emplaceEdge(*lower, *it).first->getValue() = group;
        graph_.emplaceEdge(*it, *upper).first->getValue() = group;
      }
    }
  }
}

void InterferencePlane::fixVertexConnectivity() {
  // Fix up connectivity graph using continuity and adjacency rules
  for (Graph::Vertex &vertex : graph_.getVertices()) {
    for (const auto &incomingEdge : graph_.getEdgesToVertex(vertex)) {
      for (const auto &outgoingEdge : graph_.getEdgesFromVertex(vertex)) {
        if (areEdgesContinuous(incomingEdge.getValue(),
                               outgoingEdge.getValue())) {
          vertex.getValue().connect(incomingEdge.getSource(),
                                    outgoingEdge.getDest());
        }
      }
      vertex.getValue().addVertex(incomingEdge.getSource(), true);
    }
    for (const auto &outgoingEdge : graph_.getEdgesFromVertex(vertex)) {
      vertex.getValue().addVertex(outgoingEdge.getDest(), false);
    }
  }
}

bool InterferencePlane::areEdgesContinuous(
    const std::shared_ptr<EdgeGroup> &incoming,
    const std::shared_ptr<EdgeGroup> &outgoing) const {
  std::set<uint32_t> validEdges;
  for (const EdgeCoordinate coord : incoming->edges) {
    validEdges.insert(coord.id);
    auto it = parallelEdgeAdjacency_.find(coord.id);
    assert(it != parallelEdgeAdjacency_.end());
    validEdges.insert(it->second.second);
  }
  std::set<uint32_t> successorEdges;
  for (const EdgeCoordinate coord : outgoing->edges) {
    successorEdges.insert(coord.id);
  }
  return !areSetsDisjoint(validEdges, successorEdges);
}
}  // namespace stl3lasercut
