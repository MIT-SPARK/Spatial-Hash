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
#include <spatial_hash/block_layer.h>

namespace spatial_hash {

struct TestBlock : public Block {
  TestBlock(float block_size, const BlockIndex& index) : Block(block_size, index) {}
  int data = 0;
};

TEST(BlockLayer, Constructor) {
  BlockLayer<TestBlock> layer(2.0f);
  EXPECT_EQ(layer.blockSize(), 2.0f);
  EXPECT_EQ(layer.blockSizeInv(), 0.5f);
  EXPECT_EQ(layer.numBlocks(), 0);

  // Allocate some data.
  const BlockIndex i1(1, 2, 3);
  layer.allocateBlock(i1).data = 1;
  const BlockIndex i2(4, 5, 6);
  layer.allocateBlock(i2).data = 2;
  const BlockIndex i3(7, 8, 9);
  auto& block = layer.allocateBlock(i3);
  block.data = 3;
  block.updated = true;
  EXPECT_EQ(layer.numBlocks(), 3);

  // Copy constructor.
  const BlockLayer<TestBlock> layer_copy(layer);
  EXPECT_EQ(layer_copy.blockSize(), 2.0f);
  EXPECT_EQ(layer_copy.blockSizeInv(), 0.5f);
  EXPECT_EQ(layer_copy.numBlocks(), 3);
  EXPECT_EQ(layer_copy.getBlock(i1).data, 1);
  EXPECT_EQ(layer_copy.getBlock(i2).data, 2);
  EXPECT_EQ(layer_copy.getBlock(i3).data, 3);
  EXPECT_FALSE(layer_copy.getBlock(i1).updated);
  EXPECT_TRUE(layer_copy.getBlock(i3).updated);

  // Assignment operator.
  BlockLayer<TestBlock> layer_assign(1.0f);
  layer_assign = layer;
  EXPECT_EQ(layer_assign.blockSize(), 2.0f);
  EXPECT_EQ(layer_assign.blockSizeInv(), 0.5f);
  EXPECT_EQ(layer_assign.numBlocks(), 3);
  EXPECT_EQ(layer_assign.getBlock(i1).data, 1);
  EXPECT_EQ(layer_assign.getBlock(i2).data, 2);
  EXPECT_EQ(layer_assign.getBlock(i3).data, 3);
  EXPECT_FALSE(layer_copy.getBlock(i1).updated);
  EXPECT_TRUE(layer_copy.getBlock(i3).updated);

  // Check old block unchanged.
  EXPECT_EQ(layer.blockSize(), 2.0f);
  EXPECT_EQ(layer.blockSizeInv(), 0.5f);
  EXPECT_EQ(layer.numBlocks(), 3);
  EXPECT_EQ(layer.getBlock(i1).data, 1);
  EXPECT_EQ(layer.getBlock(i2).data, 2);
  EXPECT_EQ(layer.getBlock(i3).data, 3);
  EXPECT_FALSE(layer_copy.getBlock(i1).updated);
  EXPECT_TRUE(layer_copy.getBlock(i3).updated);
}

TEST(BlockLayer, BlockAttributes) {
  BlockLayer<TestBlock> layer(2.0f);
  const auto& block = layer.allocateBlock(BlockIndex(1, 2, 3));

  // Check block attributes.
  EXPECT_EQ(block.index, BlockIndex(1, 2, 3));
  EXPECT_EQ(block.block_size, 2.0f);
  EXPECT_EQ(block.position(), Point(3.0f, 5.0f, 7.0f));
  EXPECT_EQ(block.origin(), Point(2.0f, 4.0f, 6.0f));
  EXPECT_EQ(block.data, 0);
  EXPECT_FALSE(block.updated);
}

TEST(BlockLayer, BlockConstructors) {
  struct ComplicatedBlock : public Block {
    ComplicatedBlock(float block_size, const BlockIndex& index, int a, bool b)
        : Block(block_size, index), a(a), b(b) {}
    const int a;
    const bool b;
  };

  BlockLayer<ComplicatedBlock> layer(2.0f);
  const auto& block = layer.allocateBlock(BlockIndex(1, 2, 3), 4, true);
  EXPECT_EQ(block.a, 4);
  EXPECT_EQ(block.b, true);
}

TEST(BlockLayer, updatedBlocks) {
  BlockLayer<TestBlock> layer(1.0f);
  layer.allocateBlock(BlockIndex(0, 0, 0)).data = 0;
  layer.allocateBlock(BlockIndex(1, 0, 0)).data = 1;
  layer.allocateBlock(BlockIndex(0, 1, 0)).data = 2;
  layer.allocateBlock(BlockIndex(0, 0, 1)).data = 3;

  // Get blocks with condition.
  auto blocks_indices = layer.updatedBlockIndices();
  EXPECT_EQ(blocks_indices.size(), 0);

  // Update some blocks.
  layer.getBlock(BlockIndex(1, 0, 0)).updated = true;
  layer.getBlock(BlockIndex(0, 0, 1)).updated = true;

  blocks_indices = layer.updatedBlockIndices();
  EXPECT_EQ(blocks_indices.size(), 2);
  EXPECT_TRUE(std::find(blocks_indices.begin(), blocks_indices.end(), BlockIndex(1, 0, 0)) !=
              blocks_indices.end());  // hash is unordered.
  EXPECT_TRUE(std::find(blocks_indices.begin(), blocks_indices.end(), BlockIndex(0, 0, 1)) !=
              blocks_indices.end());

  // Iterate over blocks with condition.
  int sum = 0;
  for (auto block : const_cast<const BlockLayer<TestBlock>&>(layer).updatedBlocks()) {
    sum += block->data;
    block->updated = false;
  }
  EXPECT_EQ(sum, 4);
  EXPECT_EQ(layer.updatedBlockIndices().size(), 0);
}

}  // namespace spatial_hash
