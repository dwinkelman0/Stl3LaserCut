// Copyright 2022 by Daniel Winkelman. All rights reserved.

#pragma once

#include <algo/Graph.h>
#include <gtest/gtest.h>
#include <stl3lasercut/Line.h>

#include <compare>
#include <list>
#include <map>

namespace stl3lasercut {
template <typename T>
class HierarchicalOrdering {
 public:
  template <typename V>
  class Output {
   public:
    void addData(const V &value, const Output &output) {
      std::pair<V, Output> item(value, output);
      data_.insert(std::upper_bound(data_.begin(), data_.end(), item), item);
    }
    const std::vector<std::pair<V, Output>> &getData() const { return data_; }
    bool operator<(const Output &other) const { return data_ < other.data_; }
    bool operator==(const Output &other) const { return data_ == other.data_; }

   private:
    std::vector<std::pair<V, Output>> data_;
  };

 private:
  struct PartitionComparator {
   public:
    bool operator()(const T &a, const T &b) const { return (a <=> b).second; }
  };

  /** A level is a collection of disjoint partitions, i.e. collections of T
   * that are equivalent. */
  class Level {
   private:
    using Partition = std::map<T, Level, PartitionComparator>;
    using PartitionList = std::list<Partition>;

   public:
    void insert(const T &item) {
      PartitionList children;
      typename PartitionList::iterator partitionIt = items_.begin();
      typename PartitionList::iterator partitionEnd = items_.end();
      for (; partitionIt != partitionEnd;) {
        // Check each sub-level within each partition
        typename PartitionList::iterator currentPartitionIt = partitionIt++;
        for (auto &[child, level] : *currentPartitionIt) {
          std::partial_ordering order = (item <=> child).first;
          if (order == std::partial_ordering::greater) {
            std::move(currentPartitionIt, partitionIt,
                      std::back_inserter(children));
            items_.erase(currentPartitionIt);
            break;
          } else if (order == std::partial_ordering::less) {
            level.insert(item);
            return;
          } else if (order == std::partial_ordering::equivalent) {
            currentPartitionIt->emplace(item, Level());
            return;
          }
        }
      }
      Partition newPartition;
      newPartition.emplace(item, Level()).first->second.items_ = children;
      items_.push_back(newPartition);
    }

    void map(const std::function<void(const T &)> &func) const {
      for (const auto &partition : items_) {
        for (const auto &[item, level] : partition) {
          func(item);
          level.map(func);
        }
      }
    }

    template <typename V>
    Output<V> map(const std::function<V(const T &)> &func) const {
      Output<V> output;
      for (const auto &partition : items_) {
        for (const auto &[item, level] : partition) {
          output.addData(func(item), level.map(func));
        }
      }
      return output;
    }

   private:
    std::list<std::map<T, Level, PartitionComparator>> items_;
  };

 public:
  void insert(const T &item) { root_.insert(item); }
  void map(const std::function<void(const T &)> &func) const {
    return root_.map(func);
  }
  template <typename V>
  Output<V> map(const std::function<V(const T &)> &func) const {
    return root_.map(func);
  }

 private:
  Level root_;
};
}  // namespace stl3lasercut
