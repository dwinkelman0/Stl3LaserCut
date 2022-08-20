// Copyright 2022 by Daniel Winkelman. All rights reserved.

#pragma once

#include <assert.h>

#include <compare>
#include <set>

namespace stl3lasercut {
template <class T, class Compare = std::less<T>>
class RangeUnion {
 public:
  using Range = std::pair<T, T>;

 private:
  class SetComparator {
   public:
    SetComparator(const Compare &comparator) : comparator_(comparator) {}

    bool isRangeNull(const Range &range) const {
      return !comparator_(range.first, range.second);
    }

    bool doRangesOverlap(const Range &a, const Range &b) const {
      assert(!isRangeNull(a) && !isRangeNull(b));
      return !comparator_(a.second, b.first) && !comparator_(b.second, a.first);
    }

    bool operator()(const Range &a, const Range &b) const {
      if (doRangesOverlap(a, b)) {
        return false;
      } else {
        return a.second < b.first;
      }
    }

   private:
    Compare comparator_;
  };

 public:
  using RangeSet = std::set<Range, SetComparator>;

 public:
  RangeUnion(const Compare &comparator)
      : comparator_(comparator), ranges_(SetComparator(comparator_)) {}

  void insert(const T &lower, const T &upper) { insert({lower, upper}); }

  bool isNull() const { return ranges_.empty(); }
  const RangeSet &getRanges() const { return ranges_; }
  bool operator!=(const RangeUnion &other) { return ranges_ != other.ranges_; }

  bool contains(const T &value) const {
    return std::any_of(ranges_.begin(), ranges_.end(),
                       [this, value](const Range &range) {
                         return !comparator_(value, range.first) &&
                                !comparator_(range.second, value);
                       });
  }

  RangeUnion operator&(const RangeUnion &other) const {
    RangeUnion output(comparator_);
    for (auto it1 = ranges_.begin(), it2 = other.ranges_.begin();
         it1 != ranges_.end() && it2 != other.ranges_.end();) {
      if (ranges_.key_comp()(*it1, *it2)) {
        ++it1;
      } else if (ranges_.key_comp()(*it2, *it1)) {
        ++it2;
      } else {
        output.insert(std::max(it1->first, it2->first, comparator_),
                      std::min(it1->second, it2->second, comparator_));
        if (!comparator_(it2->second, it1->second)) {
          ++it1;
          if (it1 == ranges_.end()) {
            break;
          }
        }
        if (!comparator_(it1->second, it2->second)) {
          ++it2;
        }
      }
    }
    return output;
  }

  RangeUnion operator|(const RangeUnion &other) const {
    RangeUnion output(comparator_);
    for (const Range &range : ranges_) {
      output.insert(range);
    }
    for (const Range &range : other.ranges_) {
      output.insert(range);
    }
    return output;
  }

  template <class T_, class Compare_>
  friend std::ostream &operator<<(std::ostream &os,
                                  const RangeUnion<T_, Compare_> &rangeUnion);

 private:
  void insert(const Range &range) {
    if (!ranges_.key_comp().isRangeNull(range)) {
      Range accum(range);
      auto it = ranges_.find(range);
      while (it != ranges_.end()) {
        assert(ranges_.key_comp().doRangesOverlap(range, *it));
        accum = Range(std::min(accum.first, it->first, comparator_),
                      std::max(accum.second, it->second, comparator_));
        ranges_.erase(it);
        it = ranges_.find(range);
      }
      ranges_.insert(accum);
    }
  }

 private:
  Compare comparator_;
  RangeSet ranges_;
};

template <class T, class Compare>
std::ostream &operator<<(std::ostream &os,
                         const RangeUnion<T, Compare> &rangeUnion) {
  for (const typename RangeUnion<T, Compare>::Range &range :
       rangeUnion.getRanges()) {
    os << "[" << range.first << ", " << range.second << "], ";
  }
  return os;
}
}  // namespace stl3lasercut
