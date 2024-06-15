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
#include <spatial_hash/layer.h>

namespace spatial_hash {

TEST(Layer, Constructor) {
  struct MyBlock {
    int data;
  };
  Layer<MyBlock> layer(2.0f);
  EXPECT_EQ(layer.blockSize(), 2.0f);
  EXPECT_EQ(layer.blockSizeInv(), 0.5f);
  EXPECT_EQ(layer.numBlocks(), 0);

  // Allocate some data.
  const BlockIndex i1(1, 2, 3);
  layer.allocateBlock(i1).data = 1;
  const BlockIndex i2(4, 5, 6);
  layer.allocateBlock(i2).data = 2;
  const BlockIndex i3(7, 8, 9);
  layer.allocateBlock(i3).data = 3;
  EXPECT_EQ(layer.numBlocks(), 3);

  // Copy constructor.
  const Layer<MyBlock> layer_copy(layer);
  EXPECT_EQ(layer_copy.blockSize(), 2.0f);
  EXPECT_EQ(layer_copy.blockSizeInv(), 0.5f);
  EXPECT_EQ(layer_copy.numBlocks(), 3);
  EXPECT_EQ(layer_copy.getBlock(i1).data, 1);
  EXPECT_EQ(layer_copy.getBlock(i2).data, 2);
  EXPECT_EQ(layer_copy.getBlock(i3).data, 3);

  // Assignment operator.
  Layer<MyBlock> layer_assign(1.0f);
  layer_assign = layer;
  EXPECT_EQ(layer_assign.blockSize(), 2.0f);
  EXPECT_EQ(layer_assign.blockSizeInv(), 0.5f);
  EXPECT_EQ(layer_assign.numBlocks(), 3);
  EXPECT_EQ(layer_assign.getBlock(i1).data, 1);
  EXPECT_EQ(layer_assign.getBlock(i2).data, 2);
  EXPECT_EQ(layer_assign.getBlock(i3).data, 3);

  // Check old block unchanged.
  EXPECT_EQ(layer.blockSize(), 2.0f);
  EXPECT_EQ(layer.blockSizeInv(), 0.5f);
  EXPECT_EQ(layer.numBlocks(), 3);
  EXPECT_EQ(layer.getBlock(i1).data, 1);
  EXPECT_EQ(layer.getBlock(i2).data, 2);
  EXPECT_EQ(layer.getBlock(i3).data, 3);
}

TEST(Layer, RemoveBlocks) {
  Layer<int> layer(1.0f);
  layer.allocateBlock(BlockIndex(0, 0, 0));
  layer.allocateBlock(BlockIndex(1, 0, 0));
  layer.allocateBlock(BlockIndex(0, 1, 0));
  layer.allocateBlock(BlockIndex(0, 0, 1));
  EXPECT_EQ(layer.numBlocks(), 4);

  // Remove block.
  layer.removeBlock(BlockIndex(0, 0, 0));
  EXPECT_EQ(layer.numBlocks(), 3);
  EXPECT_FALSE(layer.hasBlock(BlockIndex(0, 0, 0)));

  // Remove non-existing block.
  layer.removeBlock(BlockIndex(0, 0, 0));
  EXPECT_EQ(layer.numBlocks(), 3);

  // Remove all blocks.
  layer.clear();
  EXPECT_EQ(layer.numBlocks(), 0);

  // Iterables.
  layer.allocateBlock(BlockIndex(0, 0, 0));
  layer.allocateBlock(BlockIndex(1, 0, 0));
  layer.allocateBlock(BlockIndex(0, 1, 0));
  layer.allocateBlock(BlockIndex(0, 0, 1));

  BlockIndices block_indices = {BlockIndex(0, 0, 0), BlockIndex(1, 0, 0)};
  layer.removeBlocks(block_indices);
  EXPECT_EQ(layer.numBlocks(), 2);
  EXPECT_FALSE(layer.hasBlock(BlockIndex(0, 0, 0)));
  EXPECT_FALSE(layer.hasBlock(BlockIndex(1, 0, 0)));

  IndexSet block_indices_set = {BlockIndex(0, 0, 0), BlockIndex(0, 0, 1)};
  layer.removeBlocks(block_indices_set);
  EXPECT_EQ(layer.numBlocks(), 1);
  EXPECT_FALSE(layer.hasBlock(BlockIndex(0, 0, 1)));
}

TEST(Layer, Access) {
  Layer<int> layer(1.6);
  EXPECT_EQ(layer.numBlocks(), 0);

  // Alloacte a block.
  layer.allocateBlock(BlockIndex(0, 0, 0));
  EXPECT_EQ(layer.numBlocks(), 1);

  // Has block.
  EXPECT_TRUE(layer.hasBlock(BlockIndex(0, 0, 0)));
  EXPECT_FALSE(layer.hasBlock(BlockIndex(1, 0, 0)));
  EXPECT_TRUE(layer.hasBlock(Point(0, 0, 0)));  // lower bound included.
  EXPECT_TRUE(layer.hasBlock(Point(0.1, 0, 0)));
  EXPECT_FALSE(layer.hasBlock(Point(-0.1, 0, 0)));
  EXPECT_FALSE(layer.hasBlock(Point(1.6, 0, 0)));  // upper bound excluded.

  // Block list.
  const auto block_list = layer.allocatedBlockIndices();
  EXPECT_EQ(block_list.size(), 1);
  EXPECT_EQ(block_list[0], BlockIndex(0, 0, 0));

  // Get block.
  auto& block = layer.getBlock(BlockIndex(0, 0, 0));
  block = 123;
  const auto& block_const = layer.getBlock(Point(0, 0, 0));
  EXPECT_EQ(block_const, 123);

  // Get block ptr.
  auto block_ptr = layer.getBlockPtr(BlockIndex(0, 0, 0));
  EXPECT_NE(block_ptr, nullptr);
  block_ptr = layer.getBlockPtr(BlockIndex(1, 0, 0));
  EXPECT_EQ(block_ptr, nullptr);
}

TEST(Layer, Iterators) {
  Layer<int> layer(1.0f);
  layer.allocateBlock(BlockIndex(0, 0, 0));
  layer.allocateBlock(BlockIndex(1, 0, 0));
  layer.allocateBlock(BlockIndex(0, 1, 0));
  layer.allocateBlock(BlockIndex(0, 0, 1));

  // Iterate over all blocks.
  for (auto& block : layer) {
    block = 1;
  }
  size_t num_blocks = 0;
  for (const auto& block : *const_cast<const Layer<int>*>(&layer)) {
    num_blocks += block;
  }
  EXPECT_EQ(num_blocks, 4);
}

TEST(Layer, BlocksWithCondition) {
  struct Block {
    int data;
  };
  Layer<Block> layer(1.0f);
  layer.allocateBlock(BlockIndex(0, 0, 0)).data = 1;
  layer.allocateBlock(BlockIndex(1, 0, 0)).data = 2;
  layer.allocateBlock(BlockIndex(0, 1, 0)).data = 3;
  layer.allocateBlock(BlockIndex(0, 0, 1)).data = 4;

  // Get blocks with condition.
  const std::function<bool(const Block&)> greater2 = [](const Block& block) {
    return block.data > 2;
  };
  const auto blocks_indices = layer.blockIndicesWithCondition(greater2);
  EXPECT_EQ(blocks_indices.size(), 2);
  EXPECT_TRUE(std::find(blocks_indices.begin(), blocks_indices.end(), BlockIndex(0, 1, 0)) !=
              blocks_indices.end());  // hash is unordered.
  EXPECT_TRUE(std::find(blocks_indices.begin(), blocks_indices.end(), BlockIndex(0, 0, 1)) !=
              blocks_indices.end());

  // Iterate over blocks with condition.
  for (auto block : layer.blocksWithCondition(greater2)) {
    block->data = 0;
  }
  for (const auto& block : layer) {
    EXPECT_LE(block.data, 2);
  }
}

}  // namespace spatial_hash
