// Copyright 2022 by Daniel Winkelman. All rights reserved.

#pragma once

#include <algo/Graph.h>
#include <stl3lasercut/InterferencePlane.h>

#include <memory>

namespace stl3lasercut {
class InterferencePlane;

class TopologyPlane {
 public:
  class Graph : public std::enable_shared_from_this<Graph> {
   public:
    class VertexConnectivity {
     public:
      using Order_ = std::vector<std::pair<uint32_t, bool>>;
      using OrderIt_ = std::vector<std::pair<uint32_t, bool>>::iterator;
      using OrderConstIt_ =
          std::vector<std::pair<uint32_t, bool>>::const_iterator;

     public:
      VertexConnectivity(const Order_ &order, const bool fullCircle)
          : order_(order), fullCircle_(fullCircle) {}

      void rename(const uint32_t oldVertex, const uint32_t newVertex) {
        bool anyRenamed = false;
        for (auto &[vertex, isIncoming] : order_) {
          if (vertex == oldVertex) {
            vertex = newVertex;
            anyRenamed = true;
          }
        }
        if (anyRenamed) {
          auto last = std::unique(order_.begin(), order_.end());
          order_.erase(last, order_.end());
          if (order_.back() == order_.front()) {
            order_.pop_back();
          }
        }
      }

      const Order_ &getOrder() const { return order_; }

      bool getIsFullCircle() const { return fullCircle_; }

      OrderConstIt_ find(const uint32_t vertex, const bool isIncoming) const {
        auto output = std::find(order_.begin(), order_.end(),
                                std::pair<uint32_t, bool>(vertex, isIncoming));
        assert(output != order_.end());
        return output;
      }

      OrderConstIt_ getNext(OrderConstIt_ &it) const {
        auto nextIt = std::next(it);
        if (nextIt == order_.end()) {
          return fullCircle_ ? order_.begin() : order_.end();
        } else {
          return nextIt;
        }
      }

      OrderConstIt_ getPrevious(OrderConstIt_ &it) const {
        if (it == order_.begin()) {
          return fullCircle_ ? std::next(order_.end(), -1) : order_.end();
        } else {
          return std::next(it, -1);
        }
      }

     private:
      Order_ order_;
      bool fullCircle_;
    };

    using Graph_ = algo::DirectedGraph<algo::Unit, std::set<EdgeCoordinate>,
                                       VertexConnectivity>;

    class Component : public std::enable_shared_from_this<Component> {
     public:
      struct Merge_ {
        uint32_t source, dest, newVertex;
        std::set<EdgeCoordinate> edges;
      };

      static std::shared_ptr<Component> create(
          const Graph_ &graph, const std::shared_ptr<uint32_t> &vertexCounter);

      void merge();

      void debug() const;

     protected:
      Component(const Graph_ &graph,
                const std::shared_ptr<uint32_t> &vertexCounter);

     private:
      bool canMerge(const uint32_t v0, const uint32_t v1) const;
      Merge_ merge(const uint32_t v0, const uint32_t v1);
      std::set<uint32_t> getEdges(
          const uint32_t v0,
          const std::pair<uint32_t, bool> &otherVertex) const;

     private:
      Graph_ graph_;
      std::shared_ptr<uint32_t> vertexCounter_;
    };

   public:
    static std::shared_ptr<Graph> create(
        const InterferencePlane &interferencePlane) {
      return std::shared_ptr<Graph>(new Graph(interferencePlane));
    }

    void debug() const {
      for (const auto &component : components_) {
        component->debug();
      }
    }

   protected:
    Graph(const InterferencePlane &interferencePlane)
        : vertexCounter_(std::make_shared<uint32_t>(
              interferencePlane.graph_.getMaxVertexIndex())) {
      for (const auto &component :
           interferencePlane.graph_.getConnectedComponents()) {
        Graph_ output;
        std::map<std::pair<uint32_t, uint32_t>, std::pair<uint32_t, uint32_t>>
            renamedEdges;
        for (const InterferencePlane::Graph::ConstVertex &vertex :
             component.getVertices()) {
          std::cout << vertex.getIndex() << std::endl;
          auto clusters = vertex.getValue().exportPoints();
          if (clusters.size() == 1) {
            output.emplaceVertex(
                vertex.getIndex(),
                VertexConnectivity(clusters.front(),
                                   vertex.getValue().isFullCircle()));
          } else {
            for (const auto &cluster : clusters) {
              auto newVertexIndex = ++(*vertexCounter_);
              output.emplaceVertex(newVertexIndex,
                                   VertexConnectivity(cluster, false));
              for (const auto &[otherVertex, isIncoming] : cluster) {
                auto edge = isIncoming ? std::pair<uint32_t, uint32_t>(
                                             otherVertex, vertex.getIndex())
                                       : std::pair<uint32_t, uint32_t>(
                                             vertex.getIndex(), otherVertex);
                auto &it = renamedEdges.emplace(edge, edge).first->second;
                auto &ref = (isIncoming ? it.second : it.first);
                ref = newVertexIndex;
              }
            }
          }
        }
        for (const InterferencePlane::Graph::ConstEdge &edge :
             component.getEdges()) {
          auto it = renamedEdges.find(
              std::pair<uint32_t, uint32_t>(edge.getSource(), edge.getDest()));
          if (it != renamedEdges.end()) {
            uint32_t oldSource = it->first.first;
            uint32_t oldDest = it->first.second;
            uint32_t newSource = it->second.first;
            uint32_t newDest = it->second.second;
            output.emplaceEdge(newSource, newDest).first->getValue() =
                edge.getValue()->edges;
            if (newSource != oldSource) {
              Graph_::unwrap(output.getVertex(newDest))
                  .getValue()
                  .rename(oldSource, newSource);
            }
            if (newDest != oldDest) {
              Graph_::unwrap(output.getVertex(newSource))
                  .getValue()
                  .rename(oldDest, newDest);
            }
          } else {
            output.emplaceEdge(edge.getSource(), edge.getDest())
                .first->getValue() = edge.getValue()->edges;
          }
        }
        components_.push_back(Component::create(output, vertexCounter_));
      }
    }

    std::vector<Graph> splitIntoComponents() const;

   private:
    std::shared_ptr<uint32_t> vertexCounter_;
    std::vector<std::shared_ptr<Component>> components_;
  };

 public:
  TopologyPlane(const InterferencePlane &interferencePlane) {
    auto graph = Graph::create(interferencePlane);
    graph->debug();
  }
};
}  // namespace stl3lasercut
