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
#pragma once

#include <memory>
#include <utility>
#include <vector>

#include <glog/logging.h>

#include "spatial_hash/grid.h"
#include "spatial_hash/types.h"

namespace spatial_hash {

/**
 * @brief Base data structure for data to be stored in a BlockGrid.
 * @tparam IndexT The type of the block index. Defaults to BlockIndex.
 * @tparam PointT The type of the point. Defaults to Point. Must match the dimensionality of IndexT.
 */
template <typename IndexT = BlockIndex, typename PointT = Point>
struct Block {
  // Types.
  using Ptr = std::shared_ptr<Block>;
  using ConstPtr = std::shared_ptr<const Block>;

  // Constructors.
  /**
   * @brief Construct a block with a given block size and its index in the grid.
   * @param block_size The side length of the block.
   * @param index Index of the block in the grid.
   */
  Block(float block_size, const IndexT& index) : block_size(block_size), index(index) {}

  Block(const Block& other) = default;

  Block(Block&& other) = default;

  virtual ~Block() = default;

  Block& operator=(const Block& other) {
    if (this == &other) {
      return *this;
    }

    const_cast<float&>(block_size) = other.block_size;
    const_cast<IndexT&>(index) = other.index;
    updated = other.updated;
    return *this;
  }

  Block& operator=(Block&& other) {
    if (this == &other) {
      return *this;
    }

    const_cast<float&>(block_size) = other.block_size;
    const_cast<IndexT&>(index) = other.index;
    updated = other.updated;
    return *this;
  }

  // Config.
  // Metric side length of the block.
  const float block_size;

  // Index of the block in the grid
  const IndexT index;

  // Attributes.
  // Flag indicating if the block has been updated. Can be set or unset by any user.
  mutable bool updated = false;

  // Block info.
  /**
   * @brief Get the inverse of the block size.
   */
  float blockSizeInv() const { return 1.0f / block_size; }

  /**
   * @brief Get the center point of the block.
   */
  PointT position() const { return centerPointFromIndex<IndexT, PointT>(index, block_size); }

  /**
   * @brief Get the origin point of the block.
   */
  PointT origin() const { return originPointFromIndex<IndexT, PointT>(index, block_size); }
};

using Block2D = Block<BlockIndex2D, Point2D>;

}  // namespace spatial_hash
