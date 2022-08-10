// Copyright 2022 by Daniel Winkelman. All rights reserved.

#include "Mesh.h"

#include <stl3lasercut/RingVector.h>

#include <numbers>

namespace stl3lasercut {
Plane::Plane(const uint32_t id, const Vec3 &normal)
    : id_(id),
      normal_(normal),
      colorIdCounter_(1),
      edgeIdCounter_(0),
      vertexIdCounter_(0) {}

bool Plane::addEdge(const Vec2 &point, const uint32_t v0, const uint32_t v1,
                    const uint32_t v2,
                    const std::shared_ptr<Plane> &adjacentPlane) {
  graph_.emplaceVertex(v0);
  vertexMap_.emplace(point, v1);
  colorVertices_.emplace(0, std::set<uint32_t>()).first->second.insert(v1);
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
    edgeIt->getValue().id = edgeIdCounter_++;
    edgeIt->getValue().colorIds.insert(0);
    if (adjacentPlane) {
      edgeIt->getValue().otherPlane = adjacentPlane;
      Graph::unwrap(adjacentPlane->graph_.getEdge(v1, v0))
          .getValue()
          .otherPlane = shared_from_this();
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
        Graph::unwrap(graph_.getVertex(edge.getSource()))
            .getValue()
            .mappedPoint,
        Graph::unwrap(graph_.getVertex(edge.getDest())).getValue().mappedPoint);
    assert(line);
    edge.getValue().line = *line;
  }
}

uint32_t Plane::addOffsetLayer(const OffsetFunction &offsetFunction,
                               const uint32_t baseColor) {
  // Assign a new color
  uint32_t offsetColor = colorIdCounter_++;

  const std::set<uint32_t> &validVertices =
      colorVertices_.find(baseColor)->second;
  std::set<uint32_t> remainingVertices(validVertices);
  while (!remainingVertices.empty()) {
    std::map<uint32_t, std::pair<uint32_t, uint32_t>> newVertexMap;
    std::vector<Graph::Edge> sequence;
    Graph::Vertex current =
        Graph::unwrap(graph_.getVertex(*remainingVertices.begin()));
    std::optional<uint32_t> previous(std::nullopt);
    uint32_t firstIndex = current.getIndex();
    do {
      // Identify an initial loop
      std::cout << current.getIndex() << " -> ";
      remainingVertices.erase(current.getIndex());
      uint32_t nextIndex = current.getIndex();
      if (previous) {
        // There are two vertices in this loop already
        current.getValue().vertexConnectivity.traverseDepthFirst(
            [&validVertices,
             &nextIndex](const VertexConnectivityGraph::ConstVertex &vertex) {
              if (validVertices.find(vertex.getIndex()) !=
                  validVertices.end()) {
                nextIndex = vertex.getIndex();
              }
            },
            [](const VertexConnectivityGraph::ConstEdge &edge) {}, *previous);
        assert(nextIndex != current.getIndex());
      } else {
        // Find an arbitrary, unvisited vertex in this color
        auto outgoingEdges = graph_.getEdgesFromVertex(current);
        auto nextEdgeIt =
            std::find_if(outgoingEdges.begin(), outgoingEdges.end(),
                         [&remainingVertices](const Graph::ConstEdge &edge) {
                           return remainingVertices.find(edge.getDest()) !=
                                  remainingVertices.end();
                         });
        assert(nextEdgeIt != outgoingEdges.end());
        nextIndex = nextEdgeIt->getDest();
        assert(nextIndex != current.getIndex());
      }
      Graph::Vertex next = Graph::unwrap(graph_.getVertex(nextIndex));

      previous = current.getIndex();
      current = next;
      sequence.push_back(Graph::unwrap(graph_.getEdge(*previous, current)));
    } while (current.getIndex() != firstIndex);
    RingVector<Graph::Edge> edgeRing(sequence);
    std::cout << current.getIndex() << " (" << edgeRing.getSize()
              << " vertices in ring)" << std::endl;

    // Construct anchor points for the offset network
    edgeRing.foreachPair([this, offsetFunction, offsetColor, &newVertexMap](
                             const Graph::Edge &a, const Graph::Edge &b) {
      assert(a.getDest() == b.getSource());
      uint32_t index = a.getDest();
      Graph::Vertex vertex = Graph::unwrap(graph_.getVertex(index));
      float aOffset =
          offsetFunction(shared_from_this(), a.getValue().otherPlane);
      float bOffset =
          offsetFunction(shared_from_this(), b.getValue().otherPlane);
      if (aOffset != 0 || bOffset != 0) {
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
                  vertex.getValue().mappedPoint);
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
            newVertexMap.emplace(
                index, std::pair<uint32_t, uint32_t>(
                           makeNewVertex(*aIntersection)->getIndex(),
                           makeNewVertex(*bIntersection)->getIndex()));
          }
        }

        // Add points to vertex map
        if (intersection) {
          std::cout << vertex.getValue().mappedPoint << " ["
                    << vertex.getIndex() << "] => " << *intersection
                    << std::endl;
          uint32_t newIndex = makeNewVertex(*intersection)->getIndex();
          newVertexMap.emplace(
              index, std::pair<uint32_t, uint32_t>(newIndex, newIndex));
        }
      }
    });

    // Perform a second pass to connect anchor vertices
    RingVector<std::pair<Graph::Edge, Graph::Edge>> offsetEdges =
        edgeRing.foreach<std::pair<Graph::Edge, Graph::Edge>>(
            [this, offsetColor, offsetFunction,
             &newVertexMap](const Graph::Edge &edge) {
              auto sourceIt = newVertexMap.find(edge.getSource());
              auto destIt = newVertexMap.find(edge.getDest());
              uint32_t source = sourceIt != newVertexMap.end()
                                    ? sourceIt->second.second
                                    : edge.getSource();
              uint32_t dest = destIt != newVertexMap.end()
                                  ? destIt->second.first
                                  : edge.getDest();
              Graph::EdgeIterator newEdge =
                  graph_.emplaceEdge(source, dest).first;

              // Fill in basic information
              newEdge->getValue().otherPlane = edge.getValue().otherPlane;
              newEdge->getValue().id = edge.getValue().id;
              newEdge->getValue().colorIds.insert(offsetColor);

              // Calculate bounded line
              Graph::Vertex sourceVertex =
                  Graph::unwrap(graph_.getVertex(source));
              Graph::Vertex destVertex = Graph::unwrap(graph_.getVertex(dest));
              std::optional<BoundedLine> line = BoundedLine::fromDirectedLine(
                  edge.getValue().line.getParallelLineWithOffset(offsetFunction(
                      shared_from_this(), edge.getValue().otherPlane)),
                  sourceVertex.getValue().mappedPoint,
                  destVertex.getValue().mappedPoint);
              assert(line);
              newEdge->getValue().line = *line;

              return std::pair<Graph::Edge, Graph::Edge>(edge, *newEdge);
            });

    // Perform a pass to connect adjacent edges
    std::vector<Graph::Edge> edgesToRemove;
    offsetEdges.foreachPair([this, offsetColor, &edgesToRemove](
                                const std::pair<Graph::Edge, Graph::Edge> &a,
                                const std::pair<Graph::Edge, Graph::Edge> &b) {
      const Graph::Edge &a0 = a.first;
      const Graph::Edge &a1 = a.second;
      const Graph::Edge &b0 = b.first;
      const Graph::Edge &b1 = b.second;
      assert(a0.getDest() == b0.getSource());
      uint32_t v0 = a0.getDest();
      if (a0.getDest() == a1.getDest() && b0.getSource() == b1.getSource()) {
        // If neither edge represents a real offset, then do nothing
        return;
      }

      auto connectVertices = [this, offsetColor](const uint32_t edgeId,
                                                 const uint32_t source,
                                                 const uint32_t dest) {
        auto [it, success] = graph_.emplaceEdge(source, dest);
        assert(success);
        it->getValue().otherPlane = nullptr;
        it->getValue().id = edgeId;
        it->getValue().colorIds.insert(offsetColor);
        std::optional<BoundedLine> line = BoundedLine::fromPoints(
            Graph::unwrap(graph_.getVertex(source)).getValue().mappedPoint,
            Graph::unwrap(graph_.getVertex(dest)).getValue().mappedPoint);
        assert(line);
        it->getValue().line = *line;
      };

      if (a1.getDest() != b1.getSource()) {
        // The edges are parallel and have different offsets
        if (a0.getDest() != a1.getDest() && b0.getSource() != b1.getSource()) {
          DirectedLine::PointComparator comparator(
              Graph::unwrap(graph_.getEdge(a1.getDest(), v0)).getValue().line);
          if (comparator(Graph::unwrap(graph_.getVertex(a1.getDest()))
                             .getValue()
                             .mappedPoint,
                         Graph::unwrap(graph_.getVertex(b1.getSource()))
                             .getValue()
                             .mappedPoint)) {
            // a1 is further than b1
            connectVertices(a0.getValue().id, a1.getDest(), b1.getSource());
            connectVertices(a0.getValue().id, b1.getSource(), v0);
            connectVertices(b0.getValue().id, v0, b1.getSource());
          } else {
            // a1 is closer than b1
            connectVertices(a0.getValue().id, a1.getDest(), v0);
            connectVertices(b0.getValue().id, v0, a1.getDest());
            connectVertices(b0.getValue().id, a1.getDest(), b1.getSource());
          }
        } else if (a0.getDest() != a1.getDest()) {
          connectVertices(a0.getValue().id, a1.getDest(), v0);
        } else if (b0.getSource() != b1.getSource()) {
          connectVertices(b0.getValue().id, v0, b1.getSource());
        }
      } else {
        // The edges have an intersection, proceed based on angle between edges
        uint32_t v1 = a1.getDest();
        float angle = a0.getValue().line.getAngle(b0.getValue().line);
        if (std::abs(angle) < 1e-4) {
          // Close enough to straight
          connectVertices(a0.getValue().id, v1, v0);
          connectVertices(b0.getValue().id, v0, v1);
        } else if (std::abs(angle) < std::numbers::pi / 2) {
          // Requires perpendicular lines
        } else {
          // Simple intersections
          if (a0.getDest() != a1.getDest() &&
              b0.getSource() != b1.getSource()) {
            // Generate two more intersections and make a diamond
            std::optional<Vec2> p2 =
                a1.getValue().line.getIntersection(b0.getValue().line);
            std::optional<Vec2> p3 =
                a0.getValue().line.getIntersection(b1.getValue().line);
            Graph::VertexIterator v2 = makeNewVertex(*p2);
            Graph::VertexIterator v3 = makeNewVertex(*p3);
            assert(p2 && p3);
            connectVertices(a0.getValue().id, a1.getSource(), v2->getIndex());
            connectVertices(a0.getValue().id, v2->getIndex(), v1);
            connectVertices(b0.getValue().id, v1, v3->getIndex());
            connectVertices(b0.getValue().id, v3->getIndex(), b1.getDest());
            connectVertices(b0.getValue().id, v2->getIndex(), v0);
            connectVertices(a0.getValue().id, v0, v3->getIndex());
          } else if (a0.getDest() != a1.getDest()) {
            connectVertices(b0.getValue().id, v1, v0);
          } else if (b0.getSource() != b1.getSource()) {
            connectVertices(a0.getValue().id, v0, v1);
          }
        }
      }
    });
  }
  return offsetColor;
}

uint32_t Plane::getId() const { return id_; }

std::pair<uint32_t, uint32_t> Plane::getCharacteristic() const {
  return {graph_.getVertices().getCount(), graph_.getEdges().getCount()};
}

Plane::Graph::VertexIterator Plane::makeNewVertex(const Vec2 &point) {
  auto vertexMapIt = vertexMap_.find(point);
  if (vertexMapIt != vertexMap_.end()) {
    std::optional<Graph::VertexIterator> outputIt =
        graph_.getVertex(vertexMapIt->second);
    assert(outputIt);
    return *outputIt;
  }
  while (true) {
    auto [it, success] = graph_.emplaceVertex(vertexIdCounter_++);
    if (success) {
      it->getValue().mappedPoint = point;
      bool emplaceSuccess = vertexMap_.emplace(point, it->getIndex()).second;
      assert(emplaceSuccess);
      return it;
    }
  }
}

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
