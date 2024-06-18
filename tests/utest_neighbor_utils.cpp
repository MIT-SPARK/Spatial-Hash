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
#include <gtest/gtest.h>
#include <spatial_hash/neighbor_utils.h>

namespace spatial_hash {

TEST(NeighborUtils, IndicesValid) {
  // Index search.
  const NeighborSearch search(6);
  const Index index(0, 0, 0);
  const auto neighbors = search.neighborIndices(index, true);
  EXPECT_EQ(7, neighbors.size());
  EXPECT_EQ(index, neighbors[0]);
  EXPECT_EQ(Index(-1, 0, 0), neighbors[1]);
  EXPECT_EQ(Index(1, 0, 0), neighbors[2]);
  EXPECT_EQ(Index(0, -1, 0), neighbors[3]);
  EXPECT_EQ(Index(0, 1, 0), neighbors[4]);
  EXPECT_EQ(Index(0, 0, -1), neighbors[5]);
  EXPECT_EQ(Index(0, 0, 1), neighbors[6]);

  // Long Index search.
  const LongIndex index2(1, 1, 1);
  const auto neighbors2 = search.neighborIndices(index2, false);
  EXPECT_EQ(6, neighbors2.size());
  EXPECT_EQ(LongIndex(0, 1, 1), neighbors2[0]);

  // 18-neighbor search.
  const NeighborSearch search2(18);
  const auto neighbors3 = search2.neighborIndices(index, true);
  EXPECT_EQ(19, neighbors3.size());

  // 26 neighbor search.
  const NeighborSearch search3(26);
  const auto neighbors4 = search3.neighborIndices(Index(0, 2, -2), false);
  EXPECT_EQ(26, neighbors4.size());
}

TEST(NeighborUtils, BlockNeighborSearch) {
  Layer<int> layer(1.0f);
  const BlockNeighborSearch search(layer, 6);

  // Index search.
  const BlockIndex block_index(1, 2, 3);
  auto neighbor_indices = search.neighborIndices(block_index, true);
  EXPECT_EQ(7, neighbor_indices.size());

  // Block search.
  auto neighbors = search.neighborBlocks(block_index, true);
  EXPECT_EQ(0, neighbors.size());

  // With blocks.
  layer.allocateBlock(block_index) = 123;
  layer.allocateBlock(BlockIndex(1, 2, 4)) = 124;
  layer.allocateBlock(BlockIndex(1, 3, 3)) = 133;
  EXPECT_EQ(3, layer.numBlocks());

  const auto neighbors2 = search.neighborBlocks(block_index, true);
  EXPECT_EQ(3, neighbors2.size());
  EXPECT_EQ(123, *neighbors2[0]);
  EXPECT_EQ(133, *neighbors2[1]);
  EXPECT_EQ(124, *neighbors2[2]);
}

TEST(NeighborUtils, VoxelNeighborSearch) {
  VoxelLayer<VoxelBlock<int>> layer(1.0f, 4);
  for (int x = 0; x < 8; ++x) {
    for (int y = 0; y < 8; ++y) {
      for (int z = 0; z < 8; ++z) {
        layer.allocateVoxel(GlobalIndex(x, y, z)) = x * 100 + y * 10 + z;
      }
    }
  }

  const VoxelNeighborSearch search(layer, 6);
  const GlobalIndex index(4, 5, 6);
  const VoxelKey key{BlockIndex(1, 1, 1), VoxelIndex(0, 1, 2)};

  // Index search.
  auto neighbors = search.neighborIndices(index, true);
  EXPECT_EQ(7, neighbors.size());
  EXPECT_EQ(index, neighbors[0]);
  EXPECT_EQ(GlobalIndex(3, 5, 6), neighbors[1]);

  // Key search.
  const auto neighbors2 = search.neighborKeys(key, true);
  EXPECT_EQ(7, neighbors2.size());
  EXPECT_EQ(BlockIndex(1, 1, 1), neighbors2[0].first);
  EXPECT_EQ(VoxelIndex(0, 1, 2), neighbors2[0].second);
  EXPECT_EQ(BlockIndex(0, 1, 1), neighbors2[1].first);
  EXPECT_EQ(VoxelIndex(3, 1, 2), neighbors2[1].second);

  // Voxel search.
  const auto neighbors3 = search.neighborVoxels(index, true);
  EXPECT_EQ(7, neighbors3.size());
  EXPECT_EQ(456, *neighbors3[0]);
  EXPECT_EQ(356, *neighbors3[1]);
  EXPECT_EQ(556, *neighbors3[2]);

  // On boundary.
  const VoxelNeighborSearch search2(layer, 26);
  auto neighbors4 = search2.neighborVoxels(index);
  EXPECT_EQ(26, neighbors4.size());

  neighbors4 = search2.neighborVoxels(GlobalIndex(0, 0, 0), true);
  EXPECT_EQ(8, neighbors4.size());

  neighbors4 = search2.neighborVoxels(GlobalIndex(1, 0, 0), true);
  EXPECT_EQ(12, neighbors4.size());

  neighbors4 = search2.neighborVoxels(GlobalIndex(1, 1, 0), true);
  EXPECT_EQ(18, neighbors4.size());
}

}  // namespace spatial_hash
