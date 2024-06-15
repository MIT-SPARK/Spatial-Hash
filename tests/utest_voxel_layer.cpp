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
#include <spatial_hash/voxel_layer.h>

namespace spatial_hash {

TEST(VoxelLayer, Access) {
  VoxelLayer<VoxelBlock<int>> layer(0.1, 16);
  layer.allocateBlock(BlockIndex(0, 0, 0));

  // Existing voxel.
  const Point p_valid = Point::Zero();
  EXPECT_TRUE(layer.hasBlock(p_valid));
  EXPECT_TRUE(layer.hasVoxel(p_valid));
  const GlobalIndex i_valid = indexFromPoint<GlobalIndex>(p_valid, 10.0f);
  EXPECT_TRUE(layer.hasVoxel(i_valid));

  // Non-existing voxel.
  const Point p_invalid = Point(1.65, 0.15, 0.15);
  EXPECT_FALSE(layer.hasBlock(p_invalid));
  EXPECT_FALSE(layer.hasVoxel(p_invalid));
  const GlobalIndex i_invalid = indexFromPoint<GlobalIndex>(p_invalid, 10.0f);
  EXPECT_FALSE(layer.hasVoxel(i_invalid));

  // Get and set same voxel.
  auto& voxel = layer.getVoxel(p_valid);
  EXPECT_EQ(voxel, 0);
  voxel = 1;
  const auto& voxel_const = layer.getVoxel(i_valid);
  EXPECT_EQ(voxel_const, 1);
}

TEST(VoxelLayer, AllocateBlocks) {
  VoxelLayer<VoxelBlock<int>> layer(0.1, 16);
  const Point point = Point::Zero();
  const GlobalIndex index = indexFromPoint<GlobalIndex>(point, 10.0f);

  // Allocate voxels (includes the block).
  auto& voxel = layer.allocateVoxel(point);
  EXPECT_EQ(voxel, 0);
  voxel = 42;
  const auto& voxel_const = layer.allocateVoxel(index);
  EXPECT_EQ(voxel_const, 42);
  EXPECT_EQ(layer.numBlocks(), 1);

  // Allocate block.
  layer.allocateBlock(BlockIndex(1, 0, 0));
  const auto& block = layer.getBlock(BlockIndex(1, 0, 0));
  EXPECT_EQ(block.index, BlockIndex(1, 0, 0));
  EXPECT_EQ(block.voxel_size, 0.1f);
  EXPECT_EQ(block.voxels_per_side, 16);
  EXPECT_EQ(block.position(), Point(2.4, 0.8, 0.8));
  EXPECT_EQ(layer.numBlocks(), 2);
}

TEST(VoxelLayer, AllocateVoxels) {
  struct ComplicatedBlock : public VoxelBlock<int> {
    ComplicatedBlock(float voxel_size,
                     size_t voxels_per_side,
                     const BlockIndex& index,
                     const std::string& test)
        : VoxelBlock<int>(voxel_size, voxels_per_side, index), test(test) {}
    const std::string test;
  };

  VoxelLayer<ComplicatedBlock> layer(0.1, 16);
  const Point point(12, 23, 34);

  layer.allocateVoxel(point, "test1") = 42;
  EXPECT_TRUE(layer.hasVoxel(point));
  EXPECT_EQ(layer.getVoxel(point), 42);
  EXPECT_EQ(layer.getBlock(layer.getBlockIndex(point)).test, "test1");
}

TEST(VoxelLayer, Iterators) {
  VoxelLayer<VoxelBlock<int>> layer(0.1, 16);
  layer.allocateBlock(BlockIndex(0, 0, 0));
  layer.allocateBlock(BlockIndex(1, 0, 0));
  layer.allocateBlock(BlockIndex(0, 1, 0));
  layer.allocateBlock(BlockIndex(0, 0, 1));

  // Iterate over all voxels.
  size_t num_voxels = 0;
  for (auto& voxel : layer.voxels()) {
    voxel = 1;
  }
  for (const auto& voxel : const_cast<const VoxelLayer<VoxelBlock<int>>*>(&layer)->voxels()) {
    num_voxels += voxel;
  }
  EXPECT_EQ(num_voxels, std::pow(16, 3) * 4);
}

TEST(VoxelLayer, Assignment) {
  VoxelLayer<VoxelBlock<int>> layer(0.1, 16);
  const BlockIndex block_index(1, 2, 3);
  auto& block = layer.allocateBlock(block_index);
  block.getVoxel(12) = 34;

  // Copy constructor.
  VoxelLayer<VoxelBlock<int>> layer_copy(layer);
  EXPECT_EQ(layer_copy.numBlocks(), 1);
  EXPECT_TRUE(layer_copy.hasBlock(block_index));
  EXPECT_EQ(layer_copy.getBlock(block_index).getVoxel(12), 34);
  EXPECT_EQ(layer_copy.voxel_size, 0.1f);
  EXPECT_EQ(layer_copy.voxels_per_side, 16);

  // Copy assignment.
  VoxelLayer<VoxelBlock<int>> layer_copy_assign(0.2, 8);
  layer_copy_assign = layer;
  EXPECT_EQ(layer_copy_assign.numBlocks(), 1);
  EXPECT_TRUE(layer_copy_assign.hasBlock(block_index));
  EXPECT_EQ(layer_copy_assign.getBlock(block_index).getVoxel(12), 34);
  EXPECT_EQ(layer_copy_assign.voxel_size, 0.1f);
  EXPECT_EQ(layer_copy_assign.voxels_per_side, 16);

  // Move constructor.
  VoxelLayer<VoxelBlock<int>> layer_move(std::move(layer));
  EXPECT_EQ(layer_move.numBlocks(), 1);
  EXPECT_TRUE(layer_move.hasBlock(block_index));
  EXPECT_EQ(layer_move.getBlock(block_index).getVoxel(12), 34);
  EXPECT_EQ(layer_move.voxel_size, 0.1f);
  EXPECT_EQ(layer_move.voxels_per_side, 16);

  // Move assignment.
  VoxelLayer<VoxelBlock<int>> layer_move_assign(0.2, 8);
  layer_move_assign = std::move(layer_copy);
  EXPECT_EQ(layer_move_assign.numBlocks(), 1);
  EXPECT_TRUE(layer_move_assign.hasBlock(block_index));
  EXPECT_EQ(layer_move_assign.getBlock(block_index).getVoxel(12), 34);
  EXPECT_EQ(layer_move_assign.voxel_size, 0.1f);
  EXPECT_EQ(layer_move_assign.voxels_per_side, 16);
}

}  // namespace spatial_hash
