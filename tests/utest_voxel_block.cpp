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
#include <spatial_hash/voxel_block.h>

namespace spatial_hash {

TEST(VoxelBlock, IndicesValid) {
  const VoxelBlock<int> block(0.1, 16, BlockIndex(0, 0, 0));

  // Linear Index.
  EXPECT_TRUE(block.isValidLinearIndex(0));
  EXPECT_TRUE(block.isValidLinearIndex(4095));
  EXPECT_FALSE(block.isValidLinearIndex(4096));
  EXPECT_FALSE(block.isValidLinearIndex(-1));

  // Local voxel Index.
  EXPECT_TRUE(block.isValidVoxelIndex(VoxelIndex(0, 0, 0)));
  EXPECT_TRUE(block.isValidVoxelIndex(VoxelIndex(15, 15, 15)));
  EXPECT_FALSE(block.isValidVoxelIndex(VoxelIndex(16, 15, 15)));
  EXPECT_FALSE(block.isValidVoxelIndex(VoxelIndex(15, 16, 15)));
  EXPECT_FALSE(block.isValidVoxelIndex(VoxelIndex(15, 15, 16)));
  EXPECT_FALSE(block.isValidVoxelIndex(VoxelIndex(-1, 15, 15)));
  EXPECT_FALSE(block.isValidVoxelIndex(VoxelIndex(15, -1, 15)));
  EXPECT_FALSE(block.isValidVoxelIndex(VoxelIndex(15, 15, -1)));
}

TEST(VoxelBlock, IndexConversion) {
  const VoxelBlock<int> block(0.1, 16, BlockIndex(1, 2, 3));

  // Linear to local.
  EXPECT_EQ(VoxelIndex(0, 0, 0), block.getVoxelIndex(0));
  EXPECT_EQ(VoxelIndex(15, 15, 15), block.getVoxelIndex(4095));
  EXPECT_EQ(VoxelIndex(1, 0, 0), block.getVoxelIndex(1));
  EXPECT_EQ(VoxelIndex(0, 1, 0), block.getVoxelIndex(16));
  EXPECT_EQ(VoxelIndex(0, 0, 1), block.getVoxelIndex(256));

  // Local to linear.
  EXPECT_EQ(0, block.getLinearIndex(VoxelIndex(0, 0, 0)));
  EXPECT_EQ(4095, block.getLinearIndex(VoxelIndex(15, 15, 15)));
  EXPECT_EQ(1, block.getLinearIndex(VoxelIndex(1, 0, 0)));
  EXPECT_EQ(16, block.getLinearIndex(VoxelIndex(0, 1, 0)));
  EXPECT_EQ(256, block.getLinearIndex(VoxelIndex(0, 0, 1)));

  // Point to local.
  EXPECT_EQ(VoxelIndex(0, 0, 0), block.getVoxelIndex(Point(1.6, 3.2, 4.8)));
  EXPECT_EQ(VoxelIndex(15, 15, 15), block.getVoxelIndex(Point(3.15, 4.75, 6.35)));
}

TEST(VoxelBlock, Iterator) {
  VoxelBlock<int> block(0.1, 16, BlockIndex(0, 0, 0));
  for (size_t i = 0; i < block.numVoxels(); ++i) {
    block.getVoxel(i) = i;
  }

  // Check iterators follow the correct order.
  size_t i = 0;
  for (const auto& voxel : block) {
    EXPECT_EQ(voxel, i++);
  }
}

TEST(VoxelBlock, VoxelPosition) {
  const VoxelBlock<int> block(0.1, 16, BlockIndex(0, 0, 0));
  EXPECT_NEAR(
      (Point(0.05, 0.05, 0.05) - block.getVoxelPosition(VoxelIndex(0, 0, 0))).norm(), 0, 1e-6);
  EXPECT_NEAR(
      (Point(1.55, 1.55, 1.55) - block.getVoxelPosition(VoxelIndex(15, 15, 15))).norm(), 0, 1e-6);
  EXPECT_NEAR(
      (Point(0.15, 0.05, 0.05) - block.getVoxelPosition(VoxelIndex(1, 0, 0))).norm(), 0, 1e-6);
  EXPECT_NEAR(
      (Point(0.05, 0.15, 0.05) - block.getVoxelPosition(VoxelIndex(0, 1, 0))).norm(), 0, 1e-6);
  EXPECT_NEAR(
      (Point(0.05, 0.05, 0.15) - block.getVoxelPosition(VoxelIndex(0, 0, 1))).norm(), 0, 1e-6);
}

TEST(VoxelBlock, Assignment) {
  VoxelBlock<int> block(0.1, 16, BlockIndex(0, 0, 0));
  block.getVoxel(VoxelIndex(0, 0, 0)) = 42;
  block.getVoxel(VoxelIndex(15, 15, 15)) = 24;
  block.updated = true;

  // Assignment.
  VoxelBlock<int> block_copy = block;
  EXPECT_EQ(block_copy.getVoxel(VoxelIndex(0, 0, 0)), 42);
  EXPECT_EQ(block_copy.getVoxel(VoxelIndex(15, 15, 15)), 24);
  EXPECT_EQ(block_copy.updated, block.updated);
  EXPECT_EQ(block_copy.index, block.index);
  EXPECT_EQ(block_copy.numVoxels(), block.numVoxels());
}

}  // namespace spatial_hash
