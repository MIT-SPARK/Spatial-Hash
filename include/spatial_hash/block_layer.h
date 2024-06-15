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

#include "spatial_hash/block.h"
#include "spatial_hash/grid.h"
#include "spatial_hash/hash.h"
#include "spatial_hash/layer.h"
#include "spatial_hash/types.h"

namespace spatial_hash {

/**
 * @brief Layer containing data blocks, where BlockT must be a subclass of Block and have
 * the same constructor as Block.
 * @tparam BlockT The type of blocks contained in the layer. Must be a subclass of Block and
 * have the same constructor as Block.
 */
template <typename BlockT>
class BlockLayer : public Layer<BlockT> {
 public:
  // Types.
  using Ptr = std::shared_ptr<BlockLayer>;
  using ConstPtr = std::shared_ptr<const BlockLayer>;
  using BlockType = BlockT;
  using BlockPtr = std::shared_ptr<BlockT>;

  // Constructors.
  /**
   * @brief Construct a layer of voxel blocks.
   * @param voxel_size The side length of each voxel.
   * @param voxels_per_side The number of voxels per side of each block. Must be a power of 2.
   */
  explicit BlockLayer(float block_size) : Layer<BlockT>(block_size) {
    static_assert(std::is_base_of<Block, BlockT>::value,
                  "BlockT must be a subclass of Block to create a BlockLayer.");
  }

  virtual ~BlockLayer() = default;

  // NOTE(lschmid): This is the only shadowing function since allocation requires different
  // constructor arguments.
  /**
   * @brief Gets the block at a given index or allocates a new block if it does not exist.
   * @param block_index The block index to retrieve or allocate.
   * @param args Additional arguments to pass to the block constructor.
   * @tparam Args The types of the additional arguments.
   */
  template <typename... Args>
  BlockT& allocateBlock(const BlockIndex& block_index, Args&&... args) {
    return *allocateBlockPtr(block_index, std::forward<Args>(args)...);
  }

  /**
   * @brief Gets the block at a given index or allocates a new block if it does not exist.
   * @param block_index The block index to retrieve or allocate.
   * @param args Additional arguments to pass to the block constructor.
   * @tparam Args The types of the additional arguments.
   */
  template <typename... Args>
  BlockPtr allocateBlockPtr(const BlockIndex& block_index, Args&&... args);

  /**
   * @brief Get the indices of all blocks with the updated flag set to true.
   */
  BlockIndices updatedBlockIndices() const {
    return this->blockIndicesWithCondition(kUpdatedCondition);
  }

  /**
   * @brief Get pointers to all blocks with the updated flag set to true.
   */
  std::vector<BlockT*> updatedBlocks() { return this->blocksWithCondition(kUpdatedCondition); }
  std::vector<const BlockT*> updatedBlocks() const {
    return this->blocksWithCondition(kUpdatedCondition);
  }

 protected:
  inline static const std::function<bool(const BlockT&)> kUpdatedCondition =
      [](const BlockT& block) { return block.updated; };
};

}  // namespace spatial_hash

#include "spatial_hash/impl/block_layer_impl.h"
