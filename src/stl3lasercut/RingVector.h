// Copyright 2022 by Daniel Winkelman. All rights reserved.

#pragma once

#include <iostream>
#include <optional>
#include <vector>

template <typename T>
class RingVector {
 public:
  RingVector(const std::vector<T> &vec) : vec_(vec) {}

  uint32_t getSize() const { return vec_.size(); }

  bool operator==(const RingVector &other) const {
    if (getSize() == other.getSize()) {
      for (uint32_t i = 0; i < getSize(); ++i) {
        if (this->operator[](i) != other[i]) {
          return false;
        }
      }
      return true;
    } else {
      return false;
    }
  }

  bool operator<(const RingVector &other) const {
    if (getSize() == other.getSize()) {
      for (uint32_t i = 0; i < getSize(); ++i) {
        if (this->operator[](i) < other[i]) {
          return true;
        }
      }
      return false;
    } else {
      return getSize() < other.getSize();
    }
  }

  T &operator[](const uint32_t index) { return vec_[index % getSize()]; }
  const T &operator[](const uint32_t index) const {
    return vec_[index % getSize()];
  }

  void foreach (const std::function<void(const std::vector<T> &)> &callback,
                const uint32_t size) const {
    for (uint32_t i = 0; i < getSize(); ++i) {
      std::vector<T> input;
      for (uint32_t j = 0; j < size; ++j) {
        input.push_back(this->operator[](i + j));
      }
      callback(input);
    }
  }

  template <typename T2>
  std::vector<T> foreach (
      const std::function<T2(const std::vector<T> &)> &callback,
      const uint32_t size) const {
    std::vector<T2> output;
    for (uint32_t i = 0; i < getSize(); ++i) {
      std::vector<T> input;
      for (uint32_t j = 0; j < size; ++j) {
        input.push_back(this->operator[](i + j));
      }
      output.push_back(callback(input));
    }
    return output;
  }

  template <typename T2>
  void zip(const RingVector<T2> &other,
           const std::function<void(const T &, const T2 &)> &callback,
           const uint32_t offset) const {
    assert(getSize() == other.getSize());
    for (uint32_t i = 0; i < getSize(); ++i) {
      callback(this->operator[](i), other[i + offset]);
    }
  }

  template <typename T2, typename T3>
  RingVector<T3> zip(const RingVector<T2> &other,
                     const std::function<T3(const T &, const T2 &)> &callback,
                     const uint32_t offset) const {
    assert(getSize() == other.getSize());
    std::vector<T3> output;
    for (uint32_t i = 0; i < getSize(); ++i) {
      output.push_back(callback(this->operator[](i), other[i + offset]));
    }
    return RingVector<T3>(output);
  }

  template <typename T2>
  void sandwich(const RingVector<T2> &other,
                const std::function<void(const T &, const T2 &, const T &)>
                    &callback) const {
    for (uint32_t i = 0; i < getSize(); ++i) {
      callback(this->operator[](i), other[i], this->operator[](i + 1));
    }
  }

  template <typename T2, typename T3>
  RingVector<T3> sandwich(const RingVector<T2> &other,
                          const std::function<T3(const T &, const T2 &,
                                                 const T &)> &callback) const {
    std::vector<T3> output;
    for (uint32_t i = 0; i < getSize(); ++i) {
      output.push_back(
          callback(this->operator[](i), other[i], this->operator[](i + 1)));
    }
    return RingVector<T3>(output);
  }

  std::vector<RingVector<T>> splitCycles(
      const std::function<bool(const T &, const T &)> &equalityFunc) const {
    std::vector<RingVector<T>> output;
    RingVector<T> remainder = *this;
    while (remainder.getSize() > 0) {
      const auto [cycle, newRemainder] =
          remainder.splitOnShortestCycle(equalityFunc);
      output.push_back(cycle);
      remainder = newRemainder;
    }
    return output;
  }

  RingVector<T> slice(const uint32_t begin, const uint32_t end) const {
    uint32_t normBegin = begin % getSize();
    uint32_t normEnd = end % getSize();
    if (normBegin == normEnd) {
      return end - begin == 0 ? RingVector({}) : *this;
    } else if (normBegin < normEnd) {
      return RingVector(
          std::vector<T>(vec_.begin() + normBegin, vec_.begin() + normEnd));
    } else {
      std::vector<T> output(vec_.begin() + normBegin, vec_.end());
      output.insert(output.end(), vec_.begin(), vec_.begin() + normEnd);
      return RingVector(output);
    }
  }

 private:
  std::pair<RingVector<T>, RingVector<T>> splitOnShortestCycle(
      const std::function<bool(const T &, const T &)> &equalityFunc) const {
    uint32_t begin = 0;
    uint32_t end = getSize();
    for (uint32_t i = 0; i < getSize(); ++i) {
      for (uint32_t j = i + 1; j < i + getSize(); ++j) {
        if (equalityFunc(this->operator[](i), this->operator[](j)) &&
            j - i < end - begin) {
          begin = i;
          end = j;
          assert(begin < end);
        }
      }
    }
    return {slice(begin, end), slice(end, begin + getSize())};
  }

  std::vector<T> vec_;
};

template <typename T1, typename T2>
RingVector<std::pair<T1, T2>> zip(const RingVector<T1> &a,
                                  const RingVector<T2> &b) {
  return a.template zip<T2, std::pair<T1, T2>>(
      b, [](const T1 &t1, const T2 &t2) { return std::pair<T1, T2>(t1, t2); },
      0);
}
