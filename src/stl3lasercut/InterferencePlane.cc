// Copyright 2022 by Daniel Winkelman. All rights reserved.

#include "InterferencePlane.h"

#include <stl3lasercut/AssemblyPlane.h>
#include <stl3lasercut/RingVector.h>

#include <numbers>

namespace stl3lasercut {
bool InterferencePlane::EdgeCoordinate::operator<(
    const EdgeCoordinate &other) const {
  return std::tie(id, color, orientation) <
         std::tie(other.id, other.color, other.orientation);
}

std::ostream &operator<<(std::ostream &os,
                         const InterferencePlane::EdgeCoordinate &coord) {
  os << coord.id << "."
     << (coord.orientation == InterferencePlane::Orientation::PARALLEL ? "par"
         : coord.orientation ==
                 InterferencePlane::Orientation::INCOMING_PERPENDICULAR
             ? "inperp"
             : "outperp")
     << "." << coord.color;
  return os;
}

InterferencePlane::Comparator::Comparator(
    const std::shared_ptr<AssemblyPlane> &assemblyPlane,
    const DirectedLine &line)
    : assembly_(assemblyPlane), comparator_(line) {}

bool InterferencePlane::Comparator::operator()(const uint32_t a,
                                               const uint32_t b) const {
  return comparator_(assembly_->pointLookup_(a), assembly_->pointLookup_(b));
}

bool InterferencePlane::Comparator::lessThanOrEqual(const uint32_t a,
                                                    const uint32_t b) const {
  return this->operator()(a, b) ||
         !this->operator()(a, b) && !this->operator()(b, a);
}

bool InterferencePlane::Comparator::greaterThanOrEqual(const uint32_t a,
                                                       const uint32_t b) const {
  return this->operator()(b, a) ||
         !this->operator()(b, a) && !this->operator()(a, b);
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

bool InterferencePlane::KnownIntersectionsComparator::operator()(
    const std::pair<EdgeCoordinate, EdgeCoordinate> &a,
    const std::pair<EdgeCoordinate, EdgeCoordinate> &b) const {
  return getCanonicalOrder(a) < getCanonicalOrder(b);
}

std::pair<InterferencePlane::EdgeCoordinate, InterferencePlane::EdgeCoordinate>
InterferencePlane::KnownIntersectionsComparator::getCanonicalOrder(
    const std::pair<EdgeCoordinate, EdgeCoordinate> &a) {
  return a.first < a.second
             ? a
             : std::pair<EdgeCoordinate, EdgeCoordinate>(a.second, a.first);
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

void InterferencePlane::applyOffsetFunctions(
    const std::vector<OffsetCalculation> &offsets) {
  for (const OffsetCalculation &setting : offsets) {
    applyOffsetFunction(setting.function, setting.baseColor,
                        setting.perpendicularColor, setting.newColor,
                        setting.calculateInterference);
  }
  pruneVertices();
  fixVertexConnectivity();
}

void InterferencePlane::applyOffsetFunction(const OffsetFunction &func,
                                            const uint32_t baseColor,
                                            const uint32_t perpendicularColor,
                                            const uint32_t newColor,
                                            const bool calculateInterference) {
  colorAdjacency_.emplace(newColor, baseColor);
  for (const auto &[coord, group] : edges_) {
    if (coord.color == baseColor &&
        coord.orientation == Orientation::PARALLEL) {
      // Fix this to get assembly plane of corresponding edge
      float offset = func(assembly_, assembly_);
      addParallelEdgeFromOffset(coord, newColor, offset, calculateInterference);
    }
  }
  for (const auto &[coord, group] : edges_) {
    if (coord.color == perpendicularColor &&
        coord.orientation == Orientation::PARALLEL) {
      auto adjacencyIt = expectToFind(edgeAdjacency_, coord.id);
      addPerpendicularEdgesAtIntersection(coord.id, adjacencyIt->second.second,
                                          baseColor, perpendicularColor,
                                          newColor, calculateInterference);
    }
  }
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
        addAngle(v0, v1, v2, *e0, *e1, color);
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
  EdgeCoordinate coord(edgeId, color, Orientation::PARALLEL);
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
                                 const uint32_t e1, const uint32_t color) {
  Graph::unwrap(graph_.getVertex(v1)).getValue().connect(v0, v2);
  edgeAdjacency_
      .emplace(e0, std::pair<uint32_t, uint32_t>(
                       std::numeric_limits<uint32_t>::max(),
                       std::numeric_limits<uint32_t>::max()))
      .first->second.second = e1;
  edgeAdjacency_
      .emplace(e1, std::pair<uint32_t, uint32_t>(
                       std::numeric_limits<uint32_t>::max(),
                       std::numeric_limits<uint32_t>::max()))
      .first->second.first = e0;
  knownIntersections_.emplace(
      std::pair<EdgeCoordinate, EdgeCoordinate>(
          EdgeCoordinate(e0, color, Orientation::PARALLEL),
          EdgeCoordinate(e1, color, Orientation::PARALLEL)),
      v1);
}

void InterferencePlane::addParallelEdgeFromOffset(
    const EdgeCoordinate &coord, const uint32_t newColor, const float offset,
    const bool calculateInterference) {
  auto it = expectToFind(edges_, coord);
  EdgeCoordinate newCoord(coord.id, newColor, Orientation::PARALLEL);
  DirectedLine offsetLine = it->second->line.getParallelLineWithOffset(offset);
  std::shared_ptr<EdgeGroup> group =
      groupMap_
          .emplace(offsetLine,
                   std::make_shared<EdgeGroup>(assembly_, offsetLine))
          .first->second;
  group->edges.insert(newCoord);
  edges_.emplace(newCoord, group);
  computeInterferenceWithColor(newCoord, newColor);
  if (calculateInterference) {
    computeInterferenceWithColor(newCoord, coord.color);
  }
}

void InterferencePlane::addPerpendicularEdgesAtIntersection(
    const uint32_t incoming, const uint32_t outgoing, const uint32_t baseColor,
    const uint32_t perpendicularColor, const uint32_t newColor,
    const bool calculateInterference) {
  auto incomingIt = expectToFind(
      edges_,
      EdgeCoordinate(incoming, perpendicularColor, Orientation::PARALLEL));
  auto outgoingIt = expectToFind(
      edges_,
      EdgeCoordinate(outgoing, perpendicularColor, Orientation::PARALLEL));
  if (std::abs(incomingIt->second->line.getAngle(outgoingIt->second->line)) <
      std::numbers::pi / 2) {
    std::optional<uint32_t> vertex =
        findGroupIntersection(incomingIt->second, outgoingIt->second);
    if (!vertex) {
      auto intersectionIt =
          knownIntersections_.find({incomingIt->first, outgoingIt->first});
      if (intersectionIt != knownIntersections_.end()) {
        vertex = intersectionIt->second;
      }
    }
    if (vertex) {
      addPerpendicularEdgeThroughPoint(*vertex, true, incoming, baseColor,
                                       newColor, calculateInterference);
      addPerpendicularEdgeThroughPoint(*vertex, false, outgoing, baseColor,
                                       newColor, calculateInterference);
    }
  }
}

void InterferencePlane::addPerpendicularEdgeThroughPoint(
    const uint32_t vertex, bool isIncoming, const uint32_t id,
    const uint32_t baseColor, const uint32_t newColor,
    const bool calculateInterference) {
  auto baseIt = expectToFind(
      edges_, EdgeCoordinate(id, baseColor, Orientation::PARALLEL));
  auto offsetIt =
      expectToFind(edges_, EdgeCoordinate(id, newColor, Orientation::PARALLEL));
  DirectedLine::ParallelComparator comparator;
  if (comparator(baseIt->second->line, offsetIt->second->line) ||
      comparator(offsetIt->second->line, baseIt->second->line)) {
    // Determine RIGHT or LEFT by relative orderings of parallel lines
    // The convention is derived from:
    //   cross(PARALLEL, PERPENDICULAR) > 0 ? RIGHT : LEFT
    bool isRightHanded =
        isIncoming ^ comparator(baseIt->second->line, offsetIt->second->line);
    EdgeCoordinate newCoord(id, newColor,
                            isIncoming ? Orientation::INCOMING_PERPENDICULAR
                                       : Orientation::OUTGOING_PERPENDICULAR);
    DirectedLine perpendicularLine =
        baseIt->second->line.getPerpendicularLineThroughPoint(
            assembly_->getPoint(vertex), isRightHanded);
    std::shared_ptr<EdgeGroup> group =
        groupMap_
            .emplace(perpendicularLine,
                     std::make_shared<EdgeGroup>(assembly_, perpendicularLine))
            .first->second;
    group->edges.insert(newCoord);
    edges_.emplace(newCoord, group);
    computeInterferenceWithColor(newCoord, newColor);
    if (calculateInterference) {
      computeInterferenceWithColor(newCoord, baseColor);
    }
  }
}

void InterferencePlane::computeInterferenceWithAdjacentEdges(
    const EdgeCoordinate &coord) {
  auto adjacencyIt = edgeAdjacency_.find(coord.id);
  assert(adjacencyIt != edgeAdjacency_.end());
  const auto &[lower, upper] = adjacencyIt->second;
  std::set<std::shared_ptr<EdgeGroup>> groups;
  for (const auto &[other, group] : edges_) {
    if (other.id == lower || other.id == upper) {
      groups.insert(group);
    }
  }
  auto edgeIt = expectToFind(edges_, coord);
  for (const std::shared_ptr<EdgeGroup> &group : groups) {
    findAndInsertGroupIntersection(edgeIt->second, group);
  }
}

void InterferencePlane::computeInterferenceWithColor(
    const EdgeCoordinate &coord, const uint32_t color) {
  std::set<std::shared_ptr<EdgeGroup>> groups;
  for (const auto &[other, group] : edges_) {
    if (other.color == color &&
        (coord.orientation == Orientation::PARALLEL &&
             other.orientation == Orientation::PARALLEL ||
         other.color == coord.color)) {
      groups.insert(group);
    }
  }
  auto edgeIt = expectToFind(edges_, coord);
  for (const std::shared_ptr<EdgeGroup> &group : groups) {
    findAndInsertGroupIntersection(edgeIt->second, group);
  }
}

void InterferencePlane::findAndInsertGroupIntersection(
    const std::shared_ptr<EdgeGroup> &a, const std::shared_ptr<EdgeGroup> &b) {
  // Check for intersection
  if (areSetsDisjoint(a->points, b->points)) {
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

std::optional<uint32_t> InterferencePlane::findGroupIntersection(
    const std::shared_ptr<EdgeGroup> &a,
    const std::shared_ptr<EdgeGroup> &b) const {
  if (a != b) {
    // Get rid of the custom comparator
    std::vector<uint32_t> output;
    std::set<uint32_t> aPoints(a->points.begin(), a->points.end());
    std::set<uint32_t> bPoints(b->points.begin(), b->points.end());
    std::set_intersection(aPoints.begin(), aPoints.end(), bPoints.begin(),
                          bPoints.end(), std::back_inserter(output));
    assert(output.size() <= 1);
    return output.empty() ? std::nullopt
                          : std::optional<uint32_t>(output.front());
  } else {
    return std::nullopt;
  }
}

void InterferencePlane::fixVertexConnectivity() {
  // Fix up connectivity graph using continuity and adjacency rules
  for (Graph::Vertex &vertex : graph_.getVertices()) {
    vertex.getValue().reset();
  }
  for (Graph::Vertex &vertex : graph_.getVertices()) {
    for (const auto &incomingEdge : graph_.getEdgesToVertex(vertex)) {
      for (const auto &outgoingEdge : graph_.getEdgesFromVertex(vertex)) {
        if (incomingEdge.getSource() != outgoingEdge.getDest() &&
            areEdgesContinuous(incomingEdge.getValue(),
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
    auto it = expectToFind(edgeAdjacency_, coord.id);
    validEdges.insert(it->second.second);
  }
  std::set<uint32_t> successorEdges;
  for (const EdgeCoordinate coord : outgoing->edges) {
    successorEdges.insert(coord.id);
  }
  return !areSetsDisjoint(validEdges, successorEdges);
}

bool InterferencePlane::pruneVertices() {
  std::set<uint32_t> pruned;
  bool progress = true;
  while (progress) {
    progress = false;
    auto it = graph_.getVertices().begin();
    auto end = graph_.getVertices().end();
    for (; it != end;) {
      auto oldIt = it;
      ++it;
      if (graph_.getEdgesFromVertex(oldIt).getCount() == 0 ||
          graph_.getEdgesToVertex(oldIt).getCount() == 0) {
        pruned.emplace(oldIt->getIndex());
        graph_.eraseVertex(oldIt);
        progress = true;
      }
    }
  }
  for (const auto &[line, group] : groupMap_) {
    std::set<uint32_t, Comparator> newSet(Comparator(assembly_, line));
    std::set<uint32_t> originalSet(group->points.begin(), group->points.end());
    std::set_difference(originalSet.begin(), originalSet.end(), pruned.begin(),
                        pruned.end(), std::inserter(newSet, newSet.begin()));
    std::swap(newSet, group->points);
  }
  return !pruned.empty();
}

void InterferencePlane::calculateEdgeBounds() {
  while (restrictEdgeBounds()) {
  }
}

bool InterferencePlane::restrictEdgeBounds() { return false; }
}  // namespace stl3lasercut
