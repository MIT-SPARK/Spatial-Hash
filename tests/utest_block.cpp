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
#include <utility>

#include <gtest/gtest.h>
#include <spatial_hash/block.h>

namespace spatial_hash {

TEST(Block, Copy) {
  struct TestBlock : public Block {
    TestBlock(float block_size, const BlockIndex& index) : Block(block_size, index) {}
    int data = 0;
  };

  TestBlock block(1.0f, BlockIndex(1, 2, 3));
  block.updated = true;
  block.data = 42;

  // Copy constructor.
  TestBlock block_copy(block);
  EXPECT_EQ(block_copy.index, block.index);
  EXPECT_EQ(block_copy.block_size, block.block_size);
  EXPECT_EQ(block_copy.updated, block.updated);
  EXPECT_EQ(block_copy.data, block.data);

  // Copy assignment.
  TestBlock block_copy_assign(2.0f, BlockIndex(4, 5, 6));
  block_copy_assign = block;
  EXPECT_EQ(block_copy_assign.index, block.index);
  EXPECT_EQ(block_copy_assign.block_size, block.block_size);
  EXPECT_EQ(block_copy_assign.updated, block.updated);
  EXPECT_EQ(block_copy_assign.data, block.data);

  // Move constructor.
  TestBlock block_move(std::move(block));
  EXPECT_EQ(block_move.index, block_copy.index);
  EXPECT_EQ(block_move.block_size, block_copy.block_size);
  EXPECT_EQ(block_move.updated, block_copy.updated);
  EXPECT_EQ(block_move.data, block_copy.data);

  // Move assignment.
  TestBlock block_move_assign(3.0f, BlockIndex(7, 8, 9));
  block_move_assign = std::move(block_copy);
  EXPECT_EQ(block_move_assign.index, block_copy_assign.index);
  EXPECT_EQ(block_move_assign.block_size, block_copy_assign.block_size);
  EXPECT_EQ(block_move_assign.updated, block_copy_assign.updated);
  EXPECT_EQ(block_move_assign.data, block_copy_assign.data);
}

}  // namespace spatial_hash
