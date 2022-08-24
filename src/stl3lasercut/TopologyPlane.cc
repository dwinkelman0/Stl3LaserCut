// Copyright 2022 by Daniel Winkelman. All rights reserved.

#include "TopologyPlane.h"

namespace stl3lasercut {
std::shared_ptr<TopologyPlane::Graph::Component>
TopologyPlane::Graph::Component::create(
    const Graph_ &graph, const std::shared_ptr<uint32_t> &vertexCounter) {
  return std::shared_ptr<Component>(new Component(graph, vertexCounter));
}

std::shared_ptr<TopologyPlane::Graph::Component>
TopologyPlane::Graph::Component::merge() {
  auto output = std::make_shared<Component>(*this);
  bool progress = true;
  while (progress) {
    progress = false;
    for (auto it = output->graph_.getEdges().begin();
         it != output->graph_.getEdges().end(); ++it) {
      if (output->canMerge(it->getSource(), it->getDest())) {
        Merge_ res = output->merge(it->getSource(), it->getDest());
        std::cout << "merge " << it->getSource() << " and " << it->getDest()
                  << " into " << res.newVertex << std::endl;
        progress = true;
        break;
      }
    }
  }
  return output;
}

std::optional<std::pair<
    uint32_t,
    std::map<uint32_t, std::shared_ptr<TopologyPlane::Graph::Component>>>>
TopologyPlane::Graph::Component::fixEdgeId() {
  std::optional<EdgeFixProposal_> proposal = chooseEdgeToFix();
  if (!proposal) {
    return std::nullopt;
  }
  std::pair<
      uint32_t,
      std::map<uint32_t, std::shared_ptr<TopologyPlane::Graph::Component>>>
      output;
  output.first = proposal->edgeId;
  for (const uint32_t color : proposal->colors) {
    auto newComponent =
        output.second.emplace(color, std::make_shared<Component>(*this))
            .first->second;
    newComponent->fixEdgeId(proposal->edgeId, color);
  }
  return output;
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
}

TopologyPlane::Graph::Component::Component(
    const Graph_ &graph, const std::shared_ptr<uint32_t> &vertexCounter)
    : graph_(graph), vertexCounter_(vertexCounter) {}

bool TopologyPlane::Graph::Component::canMerge(const uint32_t v0,
                                               const uint32_t v1) const {
  if (graph_.getEdge(v1, v0)) {
    return false;
  }

  auto vc0 = Graph_::unwrap(graph_.getVertex(v0)).getValue();
  auto vc1 = Graph_::unwrap(graph_.getVertex(v1)).getValue();
  auto it0 = vc0.find(v1, false);
  auto it1 = vc1.find(v0, true);
  bool firstSideValid = false;
  bool secondSideValid = false;
  {
    auto prev0 = vc0.getPrevious(it0);
    auto next1 = vc1.getNext(it1);
    if (prev0 != vc0.getOrder().end() && next1 != vc1.getOrder().end()) {
      if (prev0->second == next1->second) {
        auto edges0 = getEdgeIdsForEdge(v0, *prev0);
        auto edges1 = getEdgeIdsForEdge(v1, *next1);
        if (edges0 == edges1) {
          firstSideValid = true;
        }
      }
    } else {
      firstSideValid = true;
    }
  }
  {
    auto next0 = vc0.getNext(it0);
    auto prev1 = vc1.getPrevious(it1);
    if (next0 != vc0.getOrder().end() && prev1 != vc1.getOrder().end()) {
      if (next0->second == prev1->second) {
        auto edges0 = getEdgeIdsForEdge(v0, *next0);
        auto edges1 = getEdgeIdsForEdge(v1, *prev1);
        if (edges0 == edges1) {
          secondSideValid = true;
        }
      }
    } else {
      secondSideValid = true;
    }
  }
  return firstSideValid && secondSideValid;
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

std::set<std::pair<uint32_t, bool>>
TopologyPlane::Graph::Component::getEdgeIdsForEdge(
    const uint32_t v0, const std::pair<uint32_t, bool> &otherVertex) const {
  std::set<std::pair<uint32_t, bool>> output;
  const auto &[v1, isIncoming] = otherVertex;
  for (const auto &coord : Graph_::unwrap(isIncoming ? graph_.getEdge(v1, v0)
                                                     : graph_.getEdge(v0, v1))
                               .getValue()) {
    output.emplace(coord.id,
                   coord.orientation == EdgeCoordinate::Orientation::PARALLEL);
  }
  return output;
}

std::optional<TopologyPlane::Graph::Component::EdgeFixProposal_>
TopologyPlane::Graph::Component::chooseEdgeToFix() const {
  // Generate proposals
  std::map<uint32_t, EdgeFixProposal_> proposals;
  for (const Graph_::ConstEdge &edge : graph_.getEdges()) {
    std::map<uint32_t, std::set<uint32_t>> edgeIdPopulation;
    for (const EdgeCoordinate &coord : edge.getValue()) {
      edgeIdPopulation.emplace(coord.id, std::set<uint32_t>())
          .first->second.insert(coord.color);
    }
    for (const auto &[edgeId, colorSet] : edgeIdPopulation) {
      EdgeFixProposal_ &proposal =
          proposals
              .emplace(edgeId, EdgeFixProposal_{.edgeId = edgeId,
                                                .colors = std::set<uint32_t>(),
                                                .numDecidableEdges = 0})
              .first->second;
      proposal.colors.insert(colorSet.begin(), colorSet.end());
      if (colorSet.size() == 1) {
        ++proposal.numDecidableEdges;
      }
    }
  }

  // Filter proposals
  for (auto it = proposals.begin(); it != proposals.end();) {
    it->second.colors.size() < 2 || it->second.numDecidableEdges == 0
        ? proposals.erase(it++)
        : ++it;
  }

  // Sort proposals by number of decidable edges
  return proposals.empty()
             ? std::nullopt
             : std::optional<EdgeFixProposal_>(
                   std::max_element(
                       proposals.begin(), proposals.end(),
                       [](const std::pair<uint32_t, EdgeFixProposal_> &a,
                          const std::pair<uint32_t, EdgeFixProposal_> &b) {
                         return a.second.numDecidableEdges <
                                b.second.numDecidableEdges;
                       })
                       ->second);
}

void TopologyPlane::Graph::Component::fixEdgeId(const uint32_t edgeId,
                                                const uint32_t color) {
  std::cout << "FIX " << edgeId << " to " << color << std::endl;
  for (auto edgeIt = graph_.getEdges().begin();
       edgeIt != graph_.getEdges().end();) {
    std::erase_if(edgeIt->getValue(),
                  [edgeId, color](const EdgeCoordinate &coord) {
                    return coord.id == edgeId && coord.color != color;
                  });
    uint32_t source = edgeIt->getSource();
    uint32_t dest = edgeIt->getDest();
    bool shouldErase = edgeIt->getValue().empty();
    ++edgeIt;
    if (shouldErase) {
      std::cout << "erase " << source << " -> " << dest << std::endl;
      eraseEdge(source, dest);
    }
  }
  prune();
}

void TopologyPlane::Graph::Component::prune() {
  bool progress = true;
  while (progress) {
    progress = false;
    for (auto vertexIt = graph_.getVertices().begin();
         vertexIt != graph_.getVertices().end();) {
      auto oldVertexIt = vertexIt;
      ++vertexIt;
      auto incomingEdges = graph_.getEdgesToVertex(oldVertexIt);
      auto outgoingEdges = graph_.getEdgesFromVertex(oldVertexIt);
      if (outgoingEdges.getCount() == 0 || incomingEdges.getCount() == 0) {
        for (auto edgeIt = incomingEdges.begin();
             edgeIt != incomingEdges.end();) {
          auto oldIt = edgeIt;
          ++edgeIt;
          eraseEdge(oldIt->getSource(), oldIt->getDest());
        }
        for (auto edgeIt = outgoingEdges.begin();
             edgeIt != outgoingEdges.end();) {
          auto oldIt = edgeIt;
          ++edgeIt;
          eraseEdge(oldIt->getSource(), oldIt->getDest());
        }
        graph_.eraseVertex(oldVertexIt);
        progress = true;
      }
    }
  }
}

void TopologyPlane::Graph::Component::eraseEdge(const uint32_t source,
                                                const uint32_t dest) {
  Graph_::unwrap(graph_.getVertex(source)).getValue().erase(dest, false);
  Graph_::unwrap(graph_.getVertex(dest)).getValue().erase(source, true);
  graph_.eraseEdge(std::pair<uint32_t, uint32_t>(source, dest));
}
}  // namespace stl3lasercut
