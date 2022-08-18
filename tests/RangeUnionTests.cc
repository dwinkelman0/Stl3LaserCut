// Copyright 2022 by Daniel Winkelman. All rights reserved.

#include <gtest/gtest.h>
#include <stl3lasercut/RangeUnion.h>

namespace stl3lasercut {
TEST(RangeUnion, Insert) {
  RangeUnion<uint32_t> u;
  ASSERT_EQ(u.getRanges().size(), 0);
  u.insert(0, 4);
  ASSERT_EQ(u.getRanges().size(), 1);
  u.insert(5, 6);
  u.insert(14, 15);
  u.insert(8, 10);
  ASSERT_EQ(u.getRanges().size(), 4);
  u.insert(6, 14);
  ASSERT_EQ(u.getRanges().size(), 2);
  u.insert(7, 11);
  ASSERT_EQ(u.getRanges().size(), 2);
}

TEST(RangeUnion, Contains) {
  RangeUnion<uint32_t> u;
  u.insert(4, 7);
  ASSERT_FALSE(u.contains(2));
  ASSERT_TRUE(u.contains(4));
  ASSERT_TRUE(u.contains(5));
  ASSERT_TRUE(u.contains(7));
  ASSERT_FALSE(u.contains(9));
}

TEST(RangeUnion, Union) {
  RangeUnion<uint32_t> u1;
  u1.insert(1, 4);
  u1.insert(8, 9);
  RangeUnion<uint32_t> u2;
  u2.insert(9, 10);
  u2.insert(14, 16);
  RangeUnion<uint32_t> u3 = u1 | u2;
  ASSERT_EQ(u3.getRanges().size(), 3);
}

TEST(RangeUnion, Intersection) {
  RangeUnion<uint32_t> u1;
  u1.insert(3, 10);
  RangeUnion<uint32_t> u2;
  u2.insert(5, 12);
  RangeUnion<uint32_t> u3;
  u3.insert(10, 11);
  RangeUnion<uint32_t> u4;
  u4.insert(1, 3);
  u4.insert(5, 7);
  u4.insert(9, 11);
  u4.insert(14, 18);
  RangeUnion<uint32_t> u5;
  u5.insert(2, 10);
  u5.insert(13, 15);
  u5.insert(17, 19);
  ASSERT_EQ((u1 & u2).getRanges().size(), 1);
  ASSERT_EQ((u1 & u3).getRanges().size(), 0);
  ASSERT_EQ((u2 & u3).getRanges().size(), 1);
  ASSERT_EQ((u4 & u5).getRanges().size(), 4);
}
}  // namespace stl3lasercut
