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
#include <spatial_hash/grid.h>

namespace spatial_hash {

TEST(Grid, LinearIndices) {
  // Test conversion between linear and voxel indices.
  const size_t voxel_per_side = 16;
  VoxelIndex voxel_index(0, 0, 0);
  size_t linear_index = 0;
  EXPECT_EQ(voxel_index, voxelIndexFromLinearIndex(linear_index, voxel_per_side));
  EXPECT_EQ(linear_index, linearIndexFromVoxelIndex(voxel_index, voxel_per_side));

  voxel_index = VoxelIndex(5, 0, 0);
  linear_index = 5;
  EXPECT_EQ(voxel_index, voxelIndexFromLinearIndex(linear_index, voxel_per_side));
  EXPECT_EQ(linear_index, linearIndexFromVoxelIndex(voxel_index, voxel_per_side));

  voxel_index = VoxelIndex(0, 5, 0);
  linear_index = 5 * voxel_per_side;
  EXPECT_EQ(voxel_index, voxelIndexFromLinearIndex(linear_index, voxel_per_side));
  EXPECT_EQ(linear_index, linearIndexFromVoxelIndex(voxel_index, voxel_per_side));

  voxel_index = VoxelIndex(0, 0, 5);
  linear_index = 5 * voxel_per_side * voxel_per_side;
  EXPECT_EQ(voxel_index, voxelIndexFromLinearIndex(linear_index, voxel_per_side));
  EXPECT_EQ(linear_index, linearIndexFromVoxelIndex(voxel_index, voxel_per_side));

  voxel_index = VoxelIndex(2, 3, 4);
  linear_index = 2 + 3 * voxel_per_side + 4 * voxel_per_side * voxel_per_side;
  EXPECT_EQ(voxel_index, voxelIndexFromLinearIndex(linear_index, voxel_per_side));
  EXPECT_EQ(linear_index, linearIndexFromVoxelIndex(voxel_index, voxel_per_side));
}

TEST(Grid, GlobalIndices) {
  // Test block index from global index.
  const size_t voxels_per_side = 16;

  // Trivial index.
  GlobalIndex global_index(0, 0, 0);
  EXPECT_EQ(BlockIndex(0, 0, 0), blockIndexFromGlobalIndex(global_index, voxels_per_side));
  EXPECT_EQ(VoxelIndex(0, 0, 0), localIndexFromGlobalIndex(global_index, voxels_per_side));

  // Non-trivial index.
  global_index = GlobalIndex(1, 17, 33);
  EXPECT_EQ(BlockIndex(0, 1, 2), blockIndexFromGlobalIndex(global_index, voxels_per_side));
  EXPECT_EQ(VoxelIndex(1, 1, 1), localIndexFromGlobalIndex(global_index, voxels_per_side));

  // Corner.
  global_index = GlobalIndex(16, 16, 16);
  EXPECT_EQ(BlockIndex(1, 1, 1), blockIndexFromGlobalIndex(global_index, voxels_per_side));
  EXPECT_EQ(VoxelIndex(0, 0, 0), localIndexFromGlobalIndex(global_index, voxels_per_side));

  // Get global index from local index.
  EXPECT_EQ(GlobalIndex(0, 0, 0),
            globalIndexFromLocalIndices(BlockIndex(0, 0, 0), VoxelIndex(0, 0, 0), 16));
  EXPECT_EQ(GlobalIndex(1, 17, 33),
            globalIndexFromLocalIndices(BlockIndex(0, 1, 2), VoxelIndex(1, 1, 1), 16));
  EXPECT_EQ(GlobalIndex(16, 16, 16),
            globalIndexFromLocalIndices(BlockIndex(1, 1, 1), VoxelIndex(0, 0, 0), 16));
}

TEST(Grid, Positions) {
  // Test conversion between index and 3D position.
  const float voxel_size = 0.1;

  BlockIndex block_index(0, 0, 0);
  Point position(0.05, 0.05, 0.05);
  EXPECT_EQ(position, centerPointFromIndex(block_index, voxel_size));
  EXPECT_EQ(block_index, indexFromPoint<BlockIndex>(position, voxel_size));

  block_index = BlockIndex(1, 2, 3);
  position = Point(0.15, 0.25, 0.35);
  EXPECT_EQ(position, centerPointFromIndex(block_index, voxel_size));
  EXPECT_EQ(block_index, indexFromPoint<BlockIndex>(position, 1.0f / voxel_size));
}

}  // namespace spatial_hash
