// Copyright 2022 by Daniel Winkelman. All rights reserved.

#include "InterferencePlane.h"

#include <stl3lasercut/AssemblyPlane.h>
#include <stl3lasercut/RingVector.h>

#include <numbers>

namespace stl3lasercut {
bool EdgeCoordinate::operator<(const EdgeCoordinate &other) const {
  return std::tie(id, color, orientation) <
         std::tie(other.id, other.color, other.orientation);
}

std::ostream &operator<<(std::ostream &os, const EdgeCoordinate &coord) {
  os << coord.id << "."
     << (coord.orientation == EdgeCoordinate::Orientation::PARALLEL ? "par"
         : coord.orientation ==
                 EdgeCoordinate::Orientation::INCOMING_PERPENDICULAR
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
            std::ostream_iterator<EdgeCoordinate>(os, ", "));
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

std::pair<EdgeCoordinate, EdgeCoordinate>
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
}

void InterferencePlane::finalize() {
  calculateEdgeBounds();
  fixVertexConnectivity();
}

void InterferencePlane::applyOffsetFunction(const OffsetFunction &func,
                                            const uint32_t baseColor,
                                            const uint32_t perpendicularColor,
                                            const uint32_t newColor,
                                            const bool calculateInterference) {
  colorAdjacency_.emplace(
      newColor, std::pair<uint32_t, bool>(baseColor, calculateInterference));
  for (const auto &[coord, group] : edges_) {
    if (coord.color == baseColor &&
        coord.orientation == EdgeCoordinate::Orientation::PARALLEL) {
      // Fix this to get assembly plane of corresponding edge
      float offset = func(assembly_, assembly_);
      addParallelEdgeFromOffset(coord, newColor, offset, calculateInterference);
    }
  }
  for (const auto &[coord, group] : edges_) {
    if (coord.color == perpendicularColor &&
        coord.orientation == EdgeCoordinate::Orientation::PARALLEL) {
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
  ring.foreachPair([this, &loop, color](const uint32_t v0, const uint32_t v1) {
    std::optional<uint32_t> edgeId = loop.getEdgeId(v0, v1);
    assert(edgeId);
    computeInterferenceWithColor(
        EdgeCoordinate(*edgeId, color, EdgeCoordinate::Orientation::PARALLEL),
        color);
  });
}

bool InterferencePlane::addPoint(const uint32_t index) {
  return graph_
      .emplaceVertex(index, MultiVertexConnectivityGraph(assembly_, index))
      .second;
}

void InterferencePlane::addEdge(const uint32_t v0, const uint32_t v1,
                                const uint32_t edgeId, const uint32_t color) {
  EdgeCoordinate coord(edgeId, color, EdgeCoordinate::Orientation::PARALLEL);
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
          EdgeCoordinate(e0, color, EdgeCoordinate::Orientation::PARALLEL),
          EdgeCoordinate(e1, color, EdgeCoordinate::Orientation::PARALLEL)),
      v1);
}

void InterferencePlane::addParallelEdgeFromOffset(
    const EdgeCoordinate &coord, const uint32_t newColor, const float offset,
    const bool calculateInterference) {
  auto it = expectToFind(edges_, coord);
  EdgeCoordinate newCoord(coord.id, newColor,
                          EdgeCoordinate::Orientation::PARALLEL);
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
      edges_, EdgeCoordinate(incoming, perpendicularColor,
                             EdgeCoordinate::Orientation::PARALLEL));
  auto outgoingIt = expectToFind(
      edges_, EdgeCoordinate(outgoing, perpendicularColor,
                             EdgeCoordinate::Orientation::PARALLEL));
  float angle = incomingIt->second->line.getAngle(outgoingIt->second->line);
  if (std::abs(angle) < std::numbers::pi / 2) {
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
      addPerpendicularEdgeThroughPoint(*vertex, true, angle < 0, incoming,
                                       baseColor, newColor,
                                       calculateInterference);
      addPerpendicularEdgeThroughPoint(*vertex, false, angle < 0, outgoing,
                                       baseColor, newColor,
                                       calculateInterference);
    }
  }
}

void InterferencePlane::addPerpendicularEdgeThroughPoint(
    const uint32_t vertex, const bool isIncoming, const bool isNegativeAngle,
    const uint32_t id, const uint32_t baseColor, const uint32_t newColor,
    const bool calculateInterference) {
  auto baseIt = expectToFind(
      edges_,
      EdgeCoordinate(id, baseColor, EdgeCoordinate::Orientation::PARALLEL));
  auto offsetIt = expectToFind(
      edges_,
      EdgeCoordinate(id, newColor, EdgeCoordinate::Orientation::PARALLEL));
  DirectedLine::ParallelComparator comparator;
  if (comparator(baseIt->second->line, offsetIt->second->line) ||
      comparator(offsetIt->second->line, baseIt->second->line)) {
    // Determine RIGHT or LEFT by relative orderings of parallel lines
    // The convention is derived from:
    //   cross(PARALLEL, PERPENDICULAR) > 0 ? RIGHT : LEFT
    bool isRightHanded = isIncoming ^ (comparator(baseIt->second->line,
                                                  offsetIt->second->line) ||
                                       isNegativeAngle);
    EdgeCoordinate newCoord(
        id, newColor,
        isIncoming ? EdgeCoordinate::Orientation::INCOMING_PERPENDICULAR
                   : EdgeCoordinate::Orientation::OUTGOING_PERPENDICULAR);
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
    if (other.color == color) {
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
  if (areSetsDisjoint(std::set<uint32_t>(a->points.begin(), a->points.end()),
                      std::set<uint32_t>(b->points.begin(), b->points.end()))) {
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
      uint32_t incomingCount = graph_.getEdgesToVertex(oldIt).getCount();
      uint32_t outgoingCount = graph_.getEdgesFromVertex(oldIt).getCount();
      if (incomingCount == 0 || outgoingCount == 0) {
        pruned.emplace(oldIt->getIndex());
        graph_.eraseVertex(oldIt);
        progress = true;
      } else if (incomingCount == 1 && outgoingCount == 1) {
        // Get rid of redundant vertex
        auto incomingEdge = *graph_.getEdgesToVertex(oldIt).begin();
        auto outgoingEdge = *graph_.getEdgesFromVertex(oldIt).begin();
        if (incomingEdge.getValue() == outgoingEdge.getValue()) {
          pruned.emplace(oldIt->getIndex());
          graph_.emplaceEdge(incomingEdge.getSource(), outgoingEdge.getDest())
              .first->getValue() = incomingEdge.getValue();
          graph_.eraseEdge(incomingEdge);
          graph_.eraseEdge(outgoingEdge);
          graph_.eraseVertex(oldIt);
          progress = true;
        }
      }
    }
  }
  for (const auto &[line, group] : groupMap_) {
    std::set<uint32_t, Comparator> newSet(group->points.key_comp());
    std::set<uint32_t> originalSet(group->points.begin(), group->points.end());
    std::set_difference(originalSet.begin(), originalSet.end(), pruned.begin(),
                        pruned.end(), std::inserter(newSet, newSet.begin()));
    std::swap(newSet, group->points);
  }
  return !pruned.empty();
}

void InterferencePlane::calculateEdgeBounds() {
  while (restrictEdgeBounds()) {
    pruneVertices();
  }
}

bool InterferencePlane::restrictEdgeBounds() {
  auto makeParallelPredicate = [this](const EdgeCoordinate &coord,
                                      const bool isIncoming) {
    auto adjacency = expectToFind(edgeAdjacency_, coord.id)->second;
    uint32_t nextEdgeId = isIncoming ? adjacency.first : adjacency.second;
    return [coord, nextEdgeId](const EdgeCoordinate &other) {
      return (other.id == coord.id &&
              (other.color != coord.color ||
               other.orientation != EdgeCoordinate::Orientation::PARALLEL)) ||
             other.id == nextEdgeId;
    };
  };

  auto makePerpendicularPredicate = [this](const EdgeCoordinate &coord,
                                           const bool isIncoming) {
    auto edgeAdjacency = expectToFind(edgeAdjacency_, coord.id)->second;
    uint32_t nextEdgeId =
        isIncoming ? edgeAdjacency.first : edgeAdjacency.second;
    auto colorAdjacency = expectToFind(colorAdjacency_, coord.color)->second;
    uint32_t nextColor = colorAdjacency.first;
    bool isDerived = colorAdjacency.second;
    return
        [coord, nextEdgeId, nextColor, isDerived](const EdgeCoordinate &other) {
          return (other.id == coord.id || other.id == nextEdgeId) &&
                 (other.color == coord.color ||
                  other.color == nextColor && isDerived) &&
                 other.orientation == EdgeCoordinate::Orientation::PARALLEL;
        };
  };

  auto checkVertex = [this](
                         const std::function<bool(EdgeCoordinate)> &predicate) {
    return [this, predicate](const uint32_t vertex) {
      auto incoming = graph_.getEdgesToVertex(vertex);
      auto outgoing = graph_.getEdgesFromVertex(vertex);
      return std::any_of(incoming.begin(), incoming.end(),
                         [predicate](const Graph::ConstEdge &edge) {
                           return std::any_of(edge.getValue()->edges.begin(),
                                              edge.getValue()->edges.end(),
                                              predicate);
                         }) ||
             std::any_of(outgoing.begin(), outgoing.end(),
                         [predicate](const Graph::ConstEdge &edge) {
                           return std::any_of(edge.getValue()->edges.begin(),
                                              edge.getValue()->edges.end(),
                                              predicate);
                         });
    };
  };

  bool anyModified = false;
  for (const auto &[line, group] : groupMap_) {
    Range groupRange(Comparator(assembly_, line));
    groupRange.insert(*group->points.begin(), *group->points.rbegin());
    for (const EdgeCoordinate &coord : group->edges) {
      auto lowerIt = group->points.begin();
      auto upperIt = group->points.rbegin();
      if (coord.orientation == EdgeCoordinate::Orientation::PARALLEL) {
        lowerIt = std::find_if(group->points.begin(), group->points.end(),
                               checkVertex(makeParallelPredicate(coord, true)));
        upperIt =
            std::find_if(group->points.rbegin(), group->points.rend(),
                         checkVertex(makeParallelPredicate(coord, false)));
      } else {
        auto predicate = checkVertex(makePerpendicularPredicate(
            coord, coord.orientation !=
                       EdgeCoordinate::Orientation::INCOMING_PERPENDICULAR));
        lowerIt =
            std::find_if(group->points.begin(), group->points.end(), predicate);
        upperIt = std::find_if(group->points.rbegin(), group->points.rend(),
                               predicate);
      }
      Range range(Comparator(assembly_, line));
      range.insert(*lowerIt, *upperIt);
      auto [it, success] = edgeBounds_.emplace(coord, range);
      if (success) {
        anyModified = true;
      } else {
        Range intersection = range & it->second;
        if (intersection != it->second) {
          it->second = intersection;
          anyModified = true;
        }
      }
      groupRange = groupRange & it->second;
    }
    std::vector<std::set<uint32_t, Comparator>::iterator> toErase;
    for (auto it = group->points.begin(); it != group->points.end(); ++it) {
      if (!groupRange.contains(*it)) {
        toErase.push_back(it);
        if (it != group->points.begin()) {
          graph_.eraseEdge(
              std::pair<uint32_t, uint32_t>(*std::next(it, -1), *it));
        }
        auto nextIt = std::next(it);
        if (nextIt != group->points.end()) {
          graph_.eraseEdge(std::pair<uint32_t, uint32_t>(*it, *nextIt));
        }
      }
    }
    for (auto it : toErase) {
      group->points.erase(it);
    }
  }
  return anyModified;
}
}  // namespace stl3lasercut
