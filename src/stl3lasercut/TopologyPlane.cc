// Copyright 2022 by Daniel Winkelman. All rights reserved.

#include "TopologyPlane.h"

namespace stl3lasercut {
std::shared_ptr<TopologyPlane::Graph::Component>
TopologyPlane::Graph::Component::create(
    const Graph_ &graph, const std::shared_ptr<uint32_t> &vertexCounter) {
  return std::shared_ptr<Component>(new Component(graph, vertexCounter));
}

void TopologyPlane::Graph::Component::merge() {
  bool progress = true;
  while (progress) {
    progress = false;
    for (auto it = graph_.getEdges().begin(); it != graph_.getEdges().end();
         ++it) {
      if (canMerge(it->getSource(), it->getDest())) {
        Merge_ res = merge(it->getSource(), it->getDest());
        std::cout << "merge " << it->getSource() << " and " << it->getDest()
                  << " into " << res.newVertex << std::endl;
        progress = true;
        break;
      }
    }
  }
}

void TopologyPlane::Graph::Component::debug() const {
  for (const auto &vertex : graph_.getVertices()) {
    std::cout << vertex.getIndex() << ": ";
    if (vertex.getValue().getIsFullCircle()) {
      std::cout << "(full circle) ";
    }
    for (const auto &[other, isIncoming] : vertex.getValue().getOrder()) {
      std::cout << other << (isIncoming ? ".in" : ".out") << ", ";
    }
    std::cout << std::endl;
  }
  for (const auto &edge : graph_.getEdges()) {
    if (canMerge(edge.getSource(), edge.getDest())) {
      std::cout << edge.getSource() << " -> " << edge.getDest() << std::endl;
    }
  }
}

TopologyPlane::Graph::Component::Component(
    const Graph_ &graph, const std::shared_ptr<uint32_t> &vertexCounter)
    : graph_(graph), vertexCounter_(vertexCounter) {
  debug();
  merge();
}

bool TopologyPlane::Graph::Component::canMerge(const uint32_t v0,
                                               const uint32_t v1) const {
  if (graph_.getEdge(v1, v0)) {
    return false;
  }

  auto vc0 = Graph_::unwrap(graph_.getVertex(v0)).getValue();
  auto vc1 = Graph_::unwrap(graph_.getVertex(v1)).getValue();
  auto it0 = vc0.find(v1, false);
  auto it1 = vc1.find(v0, true);
  {
    auto prev0 = vc0.getPrevious(it0);
    auto next1 = vc1.getNext(it1);
    if (prev0 != vc0.getOrder().end() && next1 != vc1.getOrder().end() &&
        prev0->second == next1->second) {
      auto edges0 = getEdges(v0, *prev0);
      auto edges1 = getEdges(v1, *next1);
      if (edges0 == edges1) {
        return true;
      }
    }
  }
  {
    auto next0 = vc0.getNext(it0);
    auto prev1 = vc1.getPrevious(it1);
    if (next0 != vc0.getOrder().end() && prev1 != vc1.getOrder().end() &&
        next0->second == prev1->second) {
      auto edges0 = getEdges(v0, *next0);
      auto edges1 = getEdges(v1, *prev1);
      if (edges0 == edges1) {
        return true;
      }
    }
  }
  return false;
}

TopologyPlane::Graph::Component::Merge_ TopologyPlane::Graph::Component::merge(
    const uint32_t v0, const uint32_t v1) {
  auto relinkVertices = [this](const uint32_t oldVertex,
                               const uint32_t newVertex,
                               const uint32_t excludeVertex,
                               const VertexConnectivity &connectivity) {
    for (const auto &[otherVertex, isIncoming] : connectivity.getOrder()) {
      if (otherVertex != excludeVertex) {
        Graph_::unwrap(graph_.getVertex(otherVertex))
            .getValue()
            .rename(oldVertex, newVertex);
        Graph_::Edge oldEdge =
            Graph_::unwrap(isIncoming ? graph_.getEdge(otherVertex, oldVertex)
                                      : graph_.getEdge(oldVertex, otherVertex));
        Graph_::Edge &newEdge =
            *(isIncoming ? graph_.emplaceEdge(otherVertex, newVertex)
                         : graph_.emplaceEdge(newVertex, otherVertex))
                 .first;
        newEdge.getValue().insert(oldEdge.getValue().begin(),
                                  oldEdge.getValue().end());
        graph_.eraseEdge(oldEdge);
      }
    }
  };

  auto vc0 = Graph_::unwrap(graph_.getVertex(v0)).getValue();
  auto vc1 = Graph_::unwrap(graph_.getVertex(v1)).getValue();
  auto it0 = vc0.find(v1, false);
  auto it1 = vc1.find(v0, true);

  std::vector<std::pair<uint32_t, bool>> newOrder;
  newOrder.insert(newOrder.end(), vc0.getOrder().begin(), it0);
  newOrder.insert(newOrder.end(), std::next(it1), vc1.getOrder().end());
  newOrder.insert(newOrder.end(), vc1.getOrder().begin(), it1);
  newOrder.insert(newOrder.end(), std::next(it0), vc0.getOrder().end());
  auto last = std::unique(newOrder.begin(), newOrder.end());
  newOrder.erase(last, newOrder.end());
  if (newOrder.back() == newOrder.front()) {
    newOrder.pop_back();
  }
  Graph_::Vertex &newVertex =
      *graph_
           .emplaceVertex(
               ++(*vertexCounter_),
               VertexConnectivity(
                   newOrder, vc0.getIsFullCircle() || vc1.getIsFullCircle()))
           .first;
  relinkVertices(v0, newVertex.getIndex(), v1, vc0);
  relinkVertices(v1, newVertex.getIndex(), v0, vc1);
  Merge_ output = {.source = v0,
                   .dest = v1,
                   .newVertex = newVertex.getIndex(),
                   .edges = Graph_::unwrap(graph_.getEdge(v0, v1)).getValue()};
  graph_.eraseVertex(v0);
  graph_.eraseVertex(v1);
  return output;
}

std::set<uint32_t> TopologyPlane::Graph::Component::getEdges(
    const uint32_t v0, const std::pair<uint32_t, bool> &otherVertex) const {
  std::set<uint32_t> output;
  const auto &[v1, isIncoming] = otherVertex;
  for (const auto &coord : Graph_::unwrap(isIncoming ? graph_.getEdge(v1, v0)
                                                     : graph_.getEdge(v0, v1))
                               .getValue()) {
    output.emplace(coord.id);
  }
  return output;
}
}  // namespace stl3lasercut
