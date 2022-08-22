// Copyright 2022 by Daniel Winkelman. All rights reserved.

#include "TopologyPlane.h"

namespace stl3lasercut {

TopologyPlane::TopologyPlane(const InterferencePlane &interferencePlane)
    : assemblyPlane_(interferencePlane.assembly_), vertexCounter_(0) {
  for (const auto &vertex : interferencePlane.graph_.getVertices()) {
    graph_.emplaceVertex(vertex.getIndex()).first->getValue() =
        vertex.getValue().exportPoints();
    vertexCounter_ = std::max(vertexCounter_, vertex.getIndex());
  }
  for (const auto &edge : interferencePlane.graph_.getEdges()) {
    graph_.emplaceEdge(edge.getSource(), edge.getDest()).first->getValue() =
        edge.getValue()->edges;
  }
  for (const auto &[vertex, adjacency] : interferencePlane.edgeAdjacency_) {
    edgeAdjacency_.emplace(vertex, adjacency.second);
  }
}

void TopologyPlane::debug() const {
  for (const auto &vertex : graph_.getVertices()) {
    std::cout << vertex.getIndex() << ": ";
    for (const auto &component : vertex.getValue()) {
      std::cout << "[";
      for (const auto &[index, isIncoming] : component) {
        std::cout << index << "." << (isIncoming ? "in" : "out") << ", ";
      }
      std::cout << "], ";
    }
    std::cout << std::endl;
  }
  for (const auto &edge : graph_.getEdges()) {
    std::cout << edge.getSource() << " -> " << edge.getDest() << ": ";
    std::copy(edge.getValue().begin(), edge.getValue().end(),
              std::ostream_iterator<EdgeCoordinate>(std::cout, ", "));
    std::cout << std::endl;
  }
}

template <bool IsIncoming>
std::set<EdgeCoordinate> getEdges() {}

bool componentContainsVertex(
    const std::vector<std::pair<uint32_t, bool>> &component,
    const uint32_t vertex, const bool isIncoming) {
  return std::find_if(
             component.begin(), component.end(),
             [vertex, isIncoming](const std::pair<uint32_t, bool> &edge) {
               return edge.first == vertex && edge.second == isIncoming;
             }) != component.end();
}

std::map<uint32_t, TopologyPlane::Simplification>
TopologyPlane::simplifyCycle() {
  std::map<uint32_t, TopologyPlane::Simplification> output;
  bool progress = true;
  while (progress) {
    progress = false;
    auto edges = graph_.getEdges();
    for (auto it = edges.begin(); it != edges.end(); ++it) {
      auto source = it->getSource();
      auto dest = it->getDest();
      if (!doVerticesOverlap(source, dest)) {
        auto sourceIn = getEdges<true>(source, dest, true);
        auto sourceOut = getEdges<true>(source, dest, false);
        auto destIn = getEdges<false>(dest, source, true);
        auto destOut = getEdges<false>(dest, source, false);
        if (std::includes(sourceIn.begin(), sourceIn.end(), destIn.begin(),
                          destIn.end()) &&
            std::includes(destOut.begin(), destOut.end(), sourceOut.begin(),
                          sourceOut.end())) {
          Simplification simplification = mergeVertices(source, dest);
          output.emplace(simplification.newVertex, simplification);
          progress = true;
          break;
        }
      }
    }
  }
  return output;
}

template <bool IsIncoming>
std::set<uint32_t> TopologyPlane::getEdges(const uint32_t v0, const uint32_t v1,
                                           const bool findIncoming) const {
  Graph::ConstVertex vertex = Graph::unwrap(graph_.getVertex(v0));
  auto componentIt = std::find_if(
      vertex.getValue().begin(), vertex.getValue().end(),
      [v1](const std::vector<std::pair<uint32_t, bool>> &component) {
        return std::find(component.begin(), component.end(),
                         std::pair<uint32_t, bool>(v1, !IsIncoming)) !=
               component.end();
      });
  std::set<uint32_t> output;
  if (componentIt != vertex.getValue().end()) {
    for (const auto &[vertex, isIncoming] : *componentIt) {
      if (isIncoming == findIncoming) {
        if (!(findIncoming != IsIncoming && vertex == v1)) {
          auto edgeSet =
              Graph::unwrap(graph_.getEdge(findIncoming ? vertex : v0,
                                           findIncoming ? v0 : vertex))
                  .getValue();
          for (const EdgeCoordinate &edge : edgeSet) {
            output.insert(edge.id);
          }
        }
      }
    }
  }
  return output;
}

bool TopologyPlane::doVerticesOverlap(const uint32_t v0,
                                      const uint32_t v1) const {
  std::set<uint32_t> incomingEdges, outgoingEdges;
  for (const Graph::ConstEdge &edge : graph_.getEdgesToVertex(v0)) {
    const std::set<EdgeCoordinate> &coords = edge.getValue();
    std::transform(coords.begin(), coords.end(),
                   std::inserter(incomingEdges, incomingEdges.begin()),
                   [](const EdgeCoordinate &coord) { return coord.id; });
  }
  for (const Graph::ConstEdge &edge : graph_.getEdgesFromVertex(v1)) {
    const std::set<EdgeCoordinate> &coords = edge.getValue();
    std::transform(coords.begin(), coords.end(),
                   std::inserter(outgoingEdges, outgoingEdges.begin()),
                   [this](const EdgeCoordinate &coord) {
                     return expectToFind(edgeAdjacency_, coord.id)->second;
                   });
  }
  return !areSetsDisjoint(incomingEdges, outgoingEdges);
}

template <typename T>
void makeCircularlyUnique(std::vector<T> &vec) {
  auto last = std::unique(vec.begin(), vec.end());
  vec.erase(last, vec.end());
  if (vec.back() == vec.front()) {
    vec.pop_back();
  }
}

TopologyPlane::Simplification TopologyPlane::mergeVertices(const uint32_t v0,
                                                           const uint32_t v1) {
  using Component = std::vector<std::pair<uint32_t, bool>>;
  using ComponentIt = Component::iterator;

  const auto findComponentAndVertex =
      [this](const uint32_t v0, const uint32_t v1, const bool isIncoming) {
        auto &set = Graph::unwrap(graph_.getVertex(v0)).getValue();
        for (auto componentIt = set.begin(); componentIt != set.end();
             ++componentIt) {
          auto output = std::find(componentIt->begin(), componentIt->end(),
                                  std::pair<uint32_t, bool>(v1, isIncoming));
          if (output != componentIt->end()) {
            return std::pair(componentIt, output);
          }
        }
        debug();
        throw std::runtime_error("This should not be reached.");
      };

  const auto relinkVertices = [this, findComponentAndVertex](
                                  const uint32_t oldVertex,
                                  const uint32_t ignoreVertex,
                                  const uint32_t newVertex,
                                  const Component &component) {
    for (const auto &[otherVertex, isIncoming] : component) {
      if (otherVertex != ignoreVertex) {
        auto [otherComponent, otherIt] =
            findComponentAndVertex(otherVertex, oldVertex, !isIncoming);
        otherIt->first = newVertex;
        makeCircularlyUnique(*otherComponent);
        auto [newEdge, success] =
            isIncoming ? graph_.emplaceEdge(otherVertex, newVertex)
                       : graph_.emplaceEdge(newVertex, otherVertex);
        auto oldEdge =
            Graph::unwrap(isIncoming ? graph_.getEdge(otherVertex, oldVertex)
                                     : graph_.getEdge(oldVertex, otherVertex));
        if (success) {
          std::swap(newEdge->getValue(), oldEdge.getValue());
        } else {
          newEdge->getValue().insert(oldEdge.getValue().begin(),
                                     oldEdge.getValue().end());
        }
        graph_.eraseEdge(oldEdge);
      }
    }
  };

  // Create new vertex and gather output
  auto [newVertex, newVertexSuccess] = graph_.emplaceVertex(++vertexCounter_);
  TopologyPlane::Simplification output = {
      .source = v0,
      .dest = v1,
      .newVertex = newVertex->getIndex(),
      .edge = Graph::unwrap(graph_.getEdge(v0, v1)).getValue()};

  // For each adjacent vertex, apply the merger
  auto [component0, it0] = findComponentAndVertex(v0, v1, false);
  auto [component1, it1] = findComponentAndVertex(v1, v0, true);
  relinkVertices(v0, v1, newVertex->getIndex(), *component0);
  relinkVertices(v1, v0, newVertex->getIndex(), *component1);

  // // Merge vertex data and create new vertex
  std::vector<std::pair<uint32_t, bool>> newComponent;
  newComponent.insert(newComponent.end(), component0->begin(), it0);
  newComponent.insert(newComponent.end(), std::next(it1), component1->end());
  newComponent.insert(newComponent.end(), component1->begin(), it1);
  newComponent.insert(newComponent.end(), std::next(it0), component0->end());
  makeCircularlyUnique(newComponent);
  assert(newVertexSuccess);
  newVertex->getValue().push_back(newComponent);

  // Erase old vertices
  graph_.eraseVertex(v0);
  graph_.eraseVertex(v1);

  return output;
}

TopologyNode::TopologyNode(const TopologyPlane &topologyPlane) {
  // simplify

  // if a loop can be made, then make the loop

  // otherwise, choose a simplification and recurse
}
}  // namespace stl3lasercut
