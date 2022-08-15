// Copyright 2022 by Daniel Winkelman. All rights reserved.

#include "VertexConnectivityGraph.h"

#include <stl3lasercut/AssemblyPlane.h>

namespace stl3lasercut {
void ChainedVertexConnectivityGraph::connect(const uint32_t v0,
                                             const uint32_t v1) {
  graph_.emplaceVertex(v0);
  graph_.emplaceVertex(v1);
  graph_.emplaceEdge(v0, v1);
  if (graph_.getEdgesFromVertex(v0).getCount() > 1 ||
      graph_.getEdgesToVertex(v1).getCount() > 1) {
    throw std::runtime_error("Chain invariant violated");
  }
}

uint32_t ChainedVertexConnectivityGraph::getFurthestConnection(
    const uint32_t v0) const {
  if (graph_.getEdgesToVertex(v0).getCount() != 0) {
    throw std::runtime_error("This vertex is messed up.");
  }
  uint32_t terminalVertex = std::numeric_limits<uint32_t>::max();
  graph_.traverseDepthFirst(
      [this, &terminalVertex](const auto &vertex) {
        if (graph_.getEdgesFromVertex(vertex).getCount() > 1) {
          throw std::runtime_error("This vertex is messed up.");
        }
        terminalVertex = vertex.getIndex();
      },
      [](const auto &) {}, v0);
  return terminalVertex;
}

std::ostream &operator<<(std::ostream &os,
                         const ChainedVertexConnectivityGraph &graph) {
  os << "(";
  for (const auto subgraph : graph.graph_.getConnectedComponents()) {
    for (const auto vertex : subgraph.getVertices()) {
      if (subgraph.getEdgesToVertex(vertex).getCount() == 0) {
        subgraph.traverseDepthFirst(
            [&os](const ChainedVertexConnectivityGraph::Graph::ConstVertex &v) {
              os << v.getIndex() << " -> ";
            },
            [](const auto &) {}, vertex);
        break;
      }
    }
    os << ", ";
  }
  os << ")";
  return os;
}

MultiVertexConnectivityGraph::AngularComparator::AngularComparator(
    std::shared_ptr<const AssemblyPlane> assemblyPlane,
    const uint32_t centralVertex, const uint32_t basisVertex)
    : assembly_(assemblyPlane),
      centralPoint_(assembly_->getPoint(centralVertex)),
      comparator_(*DirectedLine::fromPoints(assembly_->getPoint(basisVertex),
                                            centralPoint_)) {}

bool MultiVertexConnectivityGraph::AngularComparator::operator()(
    const uint32_t a, const uint32_t b) const {
  bool res = comparator_(getLineFromPoint(a), getLineFromPoint(b));
  return res;
}

bool MultiVertexConnectivityGraph::AngularComparator::operator()(
    const std::pair<uint32_t, bool> &a,
    const std::pair<uint32_t, bool> &b) const {
  if (!this->operator()(a.first, b.first)) {
    if (!this->operator()(b.first, a.first)) {
      return b.second < a.second;
    } else {
      return false;
    }
  } else {
    return true;
  }
}

DirectedLine MultiVertexConnectivityGraph::AngularComparator::getLineFromPoint(
    const uint32_t vertex) const {
  std::optional<DirectedLine> line =
      DirectedLine::fromPoints(assembly_->getPoint(vertex), centralPoint_);
  assert(line);
  return *line;
}

MultiVertexConnectivityGraph::MultiVertexConnectivityGraph(
    const std::shared_ptr<const AssemblyPlane> &assemblyPlane,
    const uint32_t centralVertex)
    : assembly_(assemblyPlane), centralVertex_(centralVertex) {}

void MultiVertexConnectivityGraph::connect(const uint32_t v0,
                                           const uint32_t v1) {
  // Get rid of unconnected vertices
  unconnected_.erase({v0, true});
  unconnected_.erase({v1, false});

  // Find components this intersects with or contains
  std::vector<ComponentMap::iterator> intersectingComponents;
  std::optional<ComponentMap::iterator> existingRoot;
  for (auto it = components_.begin(), end = components_.end(); it != end;
       ++it) {
    if (componentContainsPoint(it, v0) || componentContainsPoint(it, v1) ||
        it->second.key_comp()({v1, false}, {v0, true})) {
      intersectingComponents.push_back(it);
      if (componentContainsPoint(it, v0)) {
        // Incoming edge is contained in the component
        assert(!existingRoot);
        existingRoot = it;
      }
    }
  }

  // Merge everything
  ComponentMap::iterator root =
      existingRoot
          ? *existingRoot
          : components_
                .emplace(v0,
                         std::set<std::pair<uint32_t, bool>, AngularComparator>(
                             AngularComparator(assembly_, centralVertex_, v0)))
                .first;
  for (const auto it : intersectingComponents) {
    if (!existingRoot || existingRoot && it != *existingRoot) {
      root->second.insert(it->second.begin(), it->second.end());
      components_.erase(it);
    }
  }
  root->second.emplace(v0, true);
  root->second.emplace(v1, false);

  // Look for unconnected vertices to merge
  for (auto it = unconnected_.begin(), end = unconnected_.end(); it != end;) {
    auto oldIt = it++;
    if (componentContainsPoint(root, oldIt->first)) {
      root->second.insert(*oldIt);
      unconnected_.erase(oldIt);
    }
  }
}

void MultiVertexConnectivityGraph::addVertex(const uint32_t v0,
                                             const bool isIncoming) {
  // Check whether it intersects any components
  for (auto it = components_.begin(), end = components_.end(); it != end;
       ++it) {
    if (componentContainsPoint(it, v0)) {
      it->second.emplace(v0, isIncoming);
      return;
    }
  }
  unconnected_.emplace(v0, isIncoming);
}

void MultiVertexConnectivityGraph::rename(const uint32_t v0,
                                          const uint32_t v1) {
  rename(v0, v1, false);
  rename(v0, v1, true);
}

MultiVertexConnectivityGraph::ReachablePointSet
MultiVertexConnectivityGraph::getReachablePoints(const uint32_t v0) {
  ReachablePointSet output(AngularComparator(assembly_, centralVertex_, v0));
  for (const auto [index, vertices] : components_) {
    auto searchIt = vertices.find({v0, true});
    if (searchIt != vertices.end()) {
      for (auto it = searchIt, end = vertices.end(); it != end; ++it) {
        if (!it->second) {
          output.insert(it->first);
        }
      }
      return output;
    }
  }
  return output;
}

std::ostream &operator<<(std::ostream &os,
                         const MultiVertexConnectivityGraph &graph) {
  os << "(central = " << graph.centralVertex_ << ", ";
  for (const auto &[index, vertices] : graph.components_) {
    for (const auto &[vertex, isIncoming] : vertices) {
      os << vertex << "." << (isIncoming ? "in" : "out") << " -> ";
    }
    os << ", ";
  }
  os << ")";
  return os;
}

bool MultiVertexConnectivityGraph::componentContainsPoint(
    const ComponentMap::const_iterator &it, const uint32_t vertex) const {
  if (it->second.empty()) {
    return false;
  } else {
    bool lessThan = it->second.key_comp()(vertex, it->second.rbegin()->first);
    bool greaterThan =
        it->second.key_comp()(it->second.rbegin()->first, vertex);
    return lessThan || !lessThan && !greaterThan;
  }
}

void MultiVertexConnectivityGraph::rename(const uint32_t v0, const uint32_t v1,
                                          const bool isIncoming) {
  auto unconnectedIt = unconnected_.find({v0, isIncoming});
  if (unconnectedIt != unconnected_.end()) {
    unconnected_.erase(unconnectedIt);
    unconnected_.emplace(v1, isIncoming);
  }
  for (auto it = components_.begin(), end = components_.end(); it != end;) {
    auto connectedIt = it->second.find({v0, isIncoming});
    if (connectedIt != it->second.end()) {
      it->second.erase(connectedIt);
      it->second.emplace(v1, isIncoming);
    }
    auto oldIt = it++;
    if (oldIt->first == v0) {
      components_.emplace(v1, oldIt->second);
      components_.erase(oldIt);
    }
  }
}
}  // namespace stl3lasercut
