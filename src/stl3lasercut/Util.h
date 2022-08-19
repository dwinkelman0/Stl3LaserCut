// Copyright 2022 by Daniel Winkelman. All rights reserved.

#pragma once

#include <iostream>
#include <set>
#include <tuple>
#include <vector>

namespace stl3lasercut {
using Vec2 = std::tuple<float, float>;
using Vec3 = std::tuple<float, float, float>;

namespace unit3 {
extern const Vec3 x, y, z;
}

Vec3 operator-(const Vec3 vec);
Vec2 operator+(const Vec2 vec1, const Vec2 vec2);
Vec2 operator-(const Vec2 vec1, const Vec2 vec2);
Vec3 operator-(const Vec3 vec1, const Vec3 vec2);
Vec2 operator/(const Vec2 vec, const float x);
Vec3 operator/(const Vec3 vec, const float x);
float dot(const Vec2 vec1, const Vec2 vec2);
float dot(const Vec3 vec1, const Vec3 vec2);
float cross(const Vec2 vec1, const Vec2 vec2);
Vec3 cross(const Vec3 vec1, const Vec3 vec2);
float abs(const Vec2 vec);
float abs(const Vec3 vec);
float angle(const Vec2 vec1, const Vec2 vec2);
float angle(const Vec3 vec1, const Vec3 vec2);
Vec2 rotate2D(const Vec2 vec, const float angle);
Vec3 rotateX(const Vec3 vec, const float angle);
Vec3 rotateY(const Vec3 vec, const float angle);
Vec3 rotateZ(const Vec3 vec, const float angle);

std::ostream &operator<<(std::ostream &os, const Vec2 &vec);
std::ostream &operator<<(std::ostream &os, const Vec3 &vec);

float getPolygonArea(const std::vector<Vec2> &points);

template <typename T>
typename T::const_iterator expectToFind(const T &container,
                                        const typename T::key_type &key) {
  typename T::const_iterator it = container.find(key);
  assert(it != container.end());
  return it;
}

template <typename T>
typename T::iterator expectToFind(T &container,
                                  const typename T::key_type &key) {
  typename T::iterator it = container.find(key);
  assert(it != container.end());
  return it;
}

template <typename T, class Compare>
bool areSetsDisjoint(const std::set<T, Compare> &set1, const std::set<T, Compare> &set2) {
  if (set1.empty() || set2.empty()) return true;

  typename std::set<T>::const_iterator it1 = set1.begin(), it1End = set1.end();
  typename std::set<T>::const_iterator it2 = set2.begin(), it2End = set2.end();

  if (*set2.rbegin() < *it1 || *set1.rbegin() < *it2) return true;

  while (it1 != it1End && it2 != it2End) {
    if (!(*it1 < *it2) && !(*it2 < *it1)) return false;
    if (*it1 < *it2) {
      it1++;
    } else {
      it2++;
    }
  }

  return true;
}

}  // namespace stl3lasercut
