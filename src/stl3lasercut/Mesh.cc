// Copyright 2022 by Daniel Winkelman. All rights reserved.

#include "Mesh.h"

#include <stl3lasercut/RingVector.h>

namespace stl3lasercut {

bool Plane::EdgeCoordinate::operator<(const EdgeCoordinate &other) const {
  return std::tie(id, offsetId) < std::tie(other.id, other.offsetId);
}

Plane::Plane(const uint32_t id, const Vec3 &normal)
    : id_(id),
      normal_(normal),
      nullOffsetFunction_(offsetFunctions_(nullOffsetFunctionPtr_)),
      edgeIdCounter_(0) {}

bool Plane::addEdge(const Vec2 &point, const uint32_t v0, const uint32_t v1,
                    const uint32_t v2,
                    const std::shared_ptr<Plane> &adjacentPlane) {
  graph_.emplaceVertex(v0);
  vertexMap_.emplace(point, v1);
  offsetFunctionVertices_.emplace(nullOffsetFunction_, std::set<uint32_t>())
      .first->second.insert(v1);
  Graph::VertexIterator vertexIt = graph_.emplaceVertex(v1).first;
  vertexIt->getValue().mappedPoint = point;
  vertexIt->getValue().vertexConnectivity.emplaceVertex(v0);
  vertexIt->getValue().vertexConnectivity.emplaceVertex(v2);
  vertexIt->getValue().vertexConnectivity.emplaceEdge(v0, v2);
  auto existingEdge = graph_.getEdge(v1, v0);
  if (existingEdge) {
    graph_.eraseEdge(*existingEdge);
    return true;
  } else {
    Graph::EdgeIterator edgeIt = graph_.emplaceEdge(v0, v1).first;
    edgeIt->getValue().edgeCoords.insert(EdgeCoordinate{
        .id = edgeIdCounter_++, .offsetId = nullOffsetFunction_});
    if (adjacentPlane) {
      edgeIt->getValue().otherPlane = adjacentPlane;
      std::optional<Graph::EdgeIterator> otherEdgeIt =
          adjacentPlane->graph_.getEdge(v1, v0);
      assert(otherEdgeIt);
      (*otherEdgeIt)->getValue().otherPlane = shared_from_this();
    } else {
      edgeIt->getValue().otherPlane = nullptr;
    }
    return false;
  }
}

bool Plane::addEdge(const Projector3D &projector, const Vec3 &point,
                    const uint32_t v0, const uint32_t v1, const uint32_t v2,
                    const std::shared_ptr<Plane> &adjacentPlane) {
  return addEdge(projector.normalize(point), v0, v1, v2, adjacentPlane);
}

void Plane::finalizeBase() {
  for (Graph::Vertex &graphVertex : graph_.getVertices()) {
    std::vector<uint32_t> toErase;
    VertexConnectivityGraph &graph = graphVertex.getValue().vertexConnectivity;
    for (VertexConnectivityGraph::Vertex &vertex : graph.getVertices()) {
      if (graph.getEdgesToVertex(vertex).getCount() == 1 &&
          graph.getEdgesFromVertex(vertex).getCount() == 1) {
        toErase.emplace_back(vertex.getIndex());
      }
    }
    for (uint32_t index : toErase) {
      graph.emplaceEdge(graph.getEdgesToVertex(index).begin()->getSource(),
                        graph.getEdgesFromVertex(index).begin()->getDest());
      graph.eraseVertex(index);
    }
  }
  for (Graph::Edge &edge : graph_.getEdges()) {
    std::optional<BoundedLine> line = BoundedLine::fromPoints(
        (*graph_.getVertex(edge.getSource()))->getValue().mappedPoint,
        (*graph_.getVertex(edge.getDest()))->getValue().mappedPoint);
    assert(line);
    edge.getValue().line = *line;
  }
}

void Plane::addOffsetLayer(
    const std::shared_ptr<OffsetFunction> &offsetFunction,
    const std::shared_ptr<OffsetFunction> &baseOffsetFunction) {
  // Add offset function to lookup table
  uint32_t offsetFunctionIndex = offsetFunctions_(offsetFunction);
  uint32_t baseOffsetFunctionIndex = offsetFunctions_(baseOffsetFunction);

  const std::set<uint32_t> &validVertices =
      offsetFunctionVertices_.find(baseOffsetFunctionIndex)->second;
  std::set<uint32_t> remainingVertices(validVertices);
  while (!remainingVertices.empty()) {
    std::map<uint32_t, std::pair<uint32_t, uint32_t>> newVertexMap;
    std::vector<Graph::Edge> sequence;
    auto currentIt = graph_.getVertex(*remainingVertices.begin());
    assert(currentIt);
    Graph::Vertex &current = **currentIt;
    std::optional<uint32_t> previous(std::nullopt);
    uint32_t firstIndex = current.getIndex();
    do {
      std::cout << current.getIndex() << " -> ";
      remainingVertices.erase(current.getIndex());
      uint32_t next = current.getIndex();
      if (previous) {
        // There are two vertices in this loop already
        current.getValue().vertexConnectivity.traverseDepthFirst(
            [&validVertices,
             &next](const VertexConnectivityGraph::ConstVertex &vertex) {
              if (validVertices.find(vertex.getIndex()) !=
                  validVertices.end()) {
                next = vertex.getIndex();
              }
            },
            [](const VertexConnectivityGraph::ConstEdge &edge) {}, *previous);
        assert(next != current.getIndex());
      } else {
        // Find an arbitrary, unvisited vertex in this offset group
        auto outgoingEdges = graph_.getEdgesFromVertex(current);
        auto nextEdgeIt =
            std::find_if(outgoingEdges.begin(), outgoingEdges.end(),
                         [&remainingVertices](const Graph::ConstEdge &edge) {
                           return remainingVertices.find(edge.getDest()) !=
                                  remainingVertices.end();
                         });
        assert(nextEdgeIt != outgoingEdges.end());
        next = nextEdgeIt->getDest();
        assert(next != current.getIndex());
      }
      std::optional<Graph::VertexIterator> nextIt = graph_.getVertex(next);
      assert(nextIt);

      previous = current.getIndex();
      current = **nextIt;
      std::optional<Graph::EdgeIterator> edgeIt =
          graph_.getEdge(*previous, current);
      assert(edgeIt);
      sequence.push_back(**edgeIt);
    } while (current.getIndex() != firstIndex);
    std::cout << current.getIndex() << std::endl;

    RingVector<Graph::Edge> edgeRing(sequence);
    edgeRing.foreachPair([this, offsetFunction, offsetFunctionIndex,
                          &newVertexMap](const Graph::Edge &a,
                                         const Graph::Edge &b) {
      assert(a.getDest() == b.getSource());
      uint32_t index = a.getDest();
      std::optional<Graph::VertexIterator> vertexIt = graph_.getVertex(index);
      assert(vertexIt);
      float aOffset =
          a.getValue().otherPlane
              ? (*offsetFunction)(shared_from_this(), a.getValue().otherPlane)
              : 0;
      float bOffset =
          b.getValue().otherPlane
              ? (*offsetFunction)(shared_from_this(), a.getValue().otherPlane)
              : 0;
      if (aOffset != 0 || bOffset != 0) {
        // Code for creating new vertices associated with a point
        auto makeNewVertexFromPoint = [this](const Vec2 &point) {
          auto [vertexMapIt, success] = vertexMap_.emplace(point, 0);
          return success ? vertexMapIt->second : makeNewVertex()->getIndex();
        };

        // Calculate intersection
        std::optional<Vec2> intersection =
            a.getValue()
                .line.getParallelLineWithOffset(aOffset)
                .getIntersection(
                    b.getValue().line.getParallelLineWithOffset(bOffset));
        if (!intersection) {
          // The lines are parallel
          Line perpendicular =
              a.getValue().line.getPerpendicularLineThroughPoint(
                  (*vertexIt)->getValue().mappedPoint);
          std::optional<Vec2> aIntersection =
              a.getValue()
                  .line.getParallelLineWithOffset(aOffset)
                  .getIntersection(perpendicular);
          assert(aIntersection);
          if (aOffset == bOffset) {
            // The offsets are the same, fall through to normal code
            intersection = aIntersection;
          } else {
            // The offsets are different, generate two points
            std::optional<Vec2> bIntersection =
                b.getValue()
                    .line.getParallelLineWithOffset(bOffset)
                    .getIntersection(perpendicular);
            assert(bIntersection);
            newVertexMap.emplace(index,
                                 std::pair<uint32_t, uint32_t>(
                                     makeNewVertexFromPoint(*aIntersection),
                                     makeNewVertexFromPoint(*bIntersection)));
          }
        }

        // Add points to vertex map
        if (intersection) {
          std::cout << (*vertexIt)->getValue().mappedPoint << " => "
                    << *intersection << std::endl;
          uint32_t newIndex = makeNewVertexFromPoint(*intersection);
          newVertexMap.emplace(
              index, std::pair<uint32_t, uint32_t>(newIndex, newIndex));
        }
      }
    });

    // Perform a second pass to connect vertices
    std::cout << newVertexMap.size() << " new vertices" << std::endl;
  }
}

bool Plane::edgeMatchesOffset(const Graph::ConstEdge &edge,
                              const uint32_t offset) {
  return std::any_of(edge.getValue().edgeCoords.begin(),
                     edge.getValue().edgeCoords.end(),
                     [offset](const EdgeCoordinate &coord) {
                       return coord.offsetId == offset;
                     });
}

bool Plane::anyEdgeMatchesOffset(
    const Graph::Range<Graph::EdgeConstIterator> &range,
    const uint32_t offset) {
  return std::any_of(range.begin(), range.end(),
                     [offset](const Graph::ConstEdge &edge) {
                       return edgeMatchesOffset(edge, offset);
                     });
}

bool Plane::vertexMatchesOffset(const Graph::ConstVertex &vertex,
                                const uint32_t offset) const {
  return anyEdgeMatchesOffset(graph_.getEdgesToVertex(vertex), offset) &&
         anyEdgeMatchesOffset(graph_.getEdgesFromVertex(vertex), offset);
}

uint32_t Plane::getOutermostConnectedVertex(
    const Graph::ConstVertex &origin, const uint32_t start,
    const uint32_t offsetFunctionIndex) const {
  uint32_t output = start;
  origin.getValue().vertexConnectivity.traverseDepthFirst(
      [this, origin, offsetFunctionIndex,
       &output](const VertexConnectivityGraph::ConstVertex &vertex) {
        std::optional<Graph::EdgeConstIterator> edgeIt =
            graph_.getEdge(origin, vertex.getIndex());
        if (edgeIt) {
          Graph::ConstEdge &edge = **edgeIt;
          if (edgeMatchesOffset(edge, offsetFunctionIndex)) {
            output = vertex.getIndex();
          }
        }
      },
      [](const VertexConnectivityGraph::ConstEdge &edge) {}, start);
  return output;
}

Plane::Graph::VertexIterator Plane::makeNewVertex() {
  while (true) {
    const auto &[it, success] = graph_.emplaceVertex(vertexIdCounter_++);
    if (success) {
      return it;
    }
  }
}

uint32_t Plane::getId() const { return id_; }

std::pair<uint32_t, uint32_t> Plane::getCharacteristic() const {
  return {graph_.getVertices().getCount(), graph_.getEdges().getCount()};
}

float Plane::nullOffsetFunction(const std::shared_ptr<Plane> &a,
                                const std::shared_ptr<Plane> &b) {
  return 0;
}

std::shared_ptr<Plane::OffsetFunction> Plane::nullOffsetFunctionPtr_ =
    std::make_shared<OffsetFunction>(Plane::nullOffsetFunction);

Mesh::Mesh() : planeIdCounter_(0) {}

Mesh &Mesh::operator<<(const StlTriangle &triangle) {
  // Check whether the associated projector is already present
  Projector3D projector = triangle.getProjector();
  const auto &[it, success] = planes_.emplace(
      projector,
      std::make_shared<Plane>(++planeIdCounter_, triangle.getNormal()));
  projector = it->first;
  std::shared_ptr<Plane> plane = it->second;

  // Gather vertices
  uint32_t v0 = vertices_(std::get<0>(triangle.getVertices()));
  uint32_t v1 = vertices_(std::get<1>(triangle.getVertices()));
  uint32_t v2 = vertices_(std::get<2>(triangle.getVertices()));

  // Add each edge
  addEdge(plane, projector, std::get<1>(triangle.getVertices()), v0, v1, v2);
  addEdge(plane, projector, std::get<2>(triangle.getVertices()), v1, v2, v0);
  addEdge(plane, projector, std::get<0>(triangle.getVertices()), v2, v0, v1);

  return *this;
}

std::pair<uint32_t, uint32_t> Mesh::getCharacteristic() const {
  return {mesh_.getVertices().getCount(), mesh_.getEdges().getCount()};
}

const std::map<Projector3D, std::shared_ptr<Plane>> &Mesh::getPlanes() const {
  return planes_;
}

Vec3 Mesh::getVertexVector(const uint32_t index) const {
  return vertices_(index);
}

void Mesh::addEdge(const std::shared_ptr<Plane> &plane,
                   const Projector3D &projector, const Vec3 &point,
                   const uint32_t v0, const uint32_t v1, const uint32_t v2) {
  mesh_.emplaceVertex(v0);
  mesh_.emplaceVertex(v1);
  mesh_.emplaceEdge(v0, v1).first->getValue() = plane;
  std::optional<MeshGraph::EdgeIterator> adjacentEdge = mesh_.getEdge(v1, v0);
  if (plane->addEdge(projector, point, v0, v1, v2,
                     adjacentEdge ? (*adjacentEdge)->getValue() : nullptr)) {
    mesh_.eraseEdge(std::pair<uint32_t, uint32_t>(v0, v1));
    mesh_.eraseEdge(std::pair<uint32_t, uint32_t>(v1, v0));
  }
}
}  // namespace stl3lasercut
