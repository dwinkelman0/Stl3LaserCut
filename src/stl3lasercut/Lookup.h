// Copyright 2022 by Daniel Winkelman. All rights reserved.

#pragma once

#include <map>
#include <vector>

namespace stl3lasercut {
template <typename T>
class Lookup {
 public:
  uint32_t operator()(const T &object) {
    auto [it, success] = forward_.emplace(object, backward_.size());
    if (success) {
      backward_.push_back(object);
    }
    return it->second;
  }

  T operator()(const uint32_t index) const {
    if (index < backward_.size()) {
      return backward_[index];
    } else {
      throw std::out_of_range("Invalid reverse mapping");
    }
  }

 private:
  std::map<T, uint32_t> forward_;
  std::vector<T> backward_;
};
}  // namespace stl3lasercut
