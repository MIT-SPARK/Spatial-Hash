/** -----------------------------------------------------------------------------
 * Copyright (c) 2024 Massachusetts Institute of Technology.
 * All Rights Reserved.
 *
 * AUTHORS:     Lukas Schmid <lschmid@mit.edu>
 * AFFILIATION: MIT SPARK Lab, Massachusetts Institute of Technology
 * YEAR:        2024
 * LICENSE:     BSD 3-Clause
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * -------------------------------------------------------------------------- */
#include <random>

#include <gtest/gtest.h>
#include <spatial_hash/hash.h>

namespace spatial_hash {

template <typename IndexType, typename Type>
IndexType randomIndex(const Type range) {
  return IndexType(static_cast<Type>(rand() % (2 * range) - range),
                   static_cast<Type>(rand() % (2 * range) - range),
                   static_cast<Type>(rand() % (2 * range) - range));
}

// Compute the badness of a hash map. The lower the better the hash. 0 is optimal
template <class Map>
double computeMapBadness(const Map& map) {
  double const lambda = static_cast<double>(map.size()) / map.bucket_count();

  double cost = 0;
  for (auto const& [k, _] : map) cost += map.bucket_size(map.bucket(k));
  cost /= map.size();

  return std::max(0.0, cost / (1 + lambda) - 1);
}

TEST(Hash, HashMapAccess) {
  IndexHashMap<int> map;
  map[Index(0, 0, 0)] = 1;
  map[Index(1, 0, 0)] = 2;
  map[Index(0, 1, 0)] = 3;
  map[Index(0, 0, 1)] = 4;

  EXPECT_EQ(map[Index(0, 0, 0)], 1);
  EXPECT_EQ(map[Index(1, 0, 0)], 2);
  EXPECT_EQ(map[Index(0, 1, 0)], 3);
  EXPECT_EQ(map[Index(0, 0, 1)], 4);

  LongIndexHashMap<int> long_map;
  long_map[LongIndex(0, 0, 0)] = 1;
  long_map[LongIndex(1, 0, 0)] = 2;
  long_map[LongIndex(0, 1, 0)] = 3;
  long_map[LongIndex(0, 0, 1)] = 4;

  EXPECT_EQ(long_map[LongIndex(0, 0, 0)], 1);
  EXPECT_EQ(long_map[LongIndex(1, 0, 0)], 2);
  EXPECT_EQ(long_map[LongIndex(0, 1, 0)], 3);
  EXPECT_EQ(long_map[LongIndex(0, 0, 1)], 4);
}

// NOTE(lschmid): Takes a few seconds to compute and onlyu relevant if hash is updated (see if it
// gets better or worse, might need more thorough test cases though.)
TEST(Hash, DISABLED_HashMapCollisions) {
  IndexHashMap<int> map;
  int range = 1290;
  srand(42);

  // Within range.
  for (size_t i = 0; i < 100000; ++i) {
    map[randomIndex<Index>(range)] = i;
  }
  double badness = computeMapBadness(map);
  EXPECT_NEAR(badness, 0.002408, 1e-6);

  // Out of range.
  range = 10000;
  map.clear();
  for (size_t i = 0; i < 100000; ++i) {
    map[randomIndex<Index>(range)] = i;
  }
  badness = computeMapBadness(map);
  EXPECT_NEAR(badness, 0.0, 1e-6);

  // Dense.
  range = 200;
  map.clear();
  for (int x = -range; x < range; ++x) {
    for (int y = -range; y < range; ++y) {
      for (int z = -range; z < range; ++z) {
        map[Index(x, y, z)] = 0;
      }
    }
  }
  badness = computeMapBadness(map);
  EXPECT_NEAR(badness, 0.047845, 1e-6);
}

TEST(Hash, IndexSets) {
  IndexSet set;
  set.insert(Index(0, 0, 0));
  set.insert(Index(1, 0, 0));
  set.insert(Index(0, 1, 0));
  set.insert(Index(0, 0, 1));

  // Lookup.
  EXPECT_TRUE(set.find(Index(0, 0, 0)) != set.end());
  EXPECT_TRUE(set.find(Index(1, 0, 0)) != set.end());
  EXPECT_TRUE(set.find(Index(0, 1, 0)) != set.end());
  EXPECT_TRUE(set.find(Index(0, 0, 1)) != set.end());

  // Collisions.
  EXPECT_EQ(set.size(), 4);
  set.insert(Index(0, 0, 0));
  set.insert(Index(1, 0, 0));
  EXPECT_EQ(set.size(), 4);

  // Erase.
  set.erase(Index(0, 0, 0));
  EXPECT_TRUE(set.find(Index(0, 0, 0)) == set.end());
  EXPECT_EQ(set.size(), 3);

  LongIndexSet long_set;
  long_set.insert(LongIndex(0, 0, 0));
  long_set.insert(LongIndex(1, 0, 0));
  long_set.insert(LongIndex(0, 1, 0));
  long_set.insert(LongIndex(0, 0, 1));

  // Lookup.
  EXPECT_TRUE(long_set.find(LongIndex(0, 0, 0)) != long_set.end());
  EXPECT_TRUE(long_set.find(LongIndex(1, 0, 0)) != long_set.end());
  EXPECT_TRUE(long_set.find(LongIndex(0, 1, 0)) != long_set.end());
  EXPECT_TRUE(long_set.find(LongIndex(0, 0, 1)) != long_set.end());

  // Collisions.
  EXPECT_EQ(long_set.size(), 4);
  long_set.insert(LongIndex(0, 0, 0));
  long_set.insert(LongIndex(1, 0, 0));
  EXPECT_EQ(long_set.size(), 4);

  // Erase.
  long_set.erase(LongIndex(0, 0, 0));
  EXPECT_TRUE(long_set.find(LongIndex(0, 0, 0)) == long_set.end());
  EXPECT_EQ(long_set.size(), 3);
}

}  // namespace spatial_hash
