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
#include <vector>

#include "spatial_hash/layer.h"

namespace spatial_hash {

template <typename BlockT>
void Layer<BlockT>::copyBlocks(const Layer<BlockT>& other) {
  blocks_.clear();
  for (const auto& [index, block_ptr] : other.blocks_) {
    blocks_[index] = std::make_shared<BlockT>(*block_ptr);
  }
}

template <typename BlockT>
Layer<BlockT>& Layer<BlockT>::operator=(const Layer<BlockT>& other) {
  if (this == &other) {
    return *this;
  }

  const_cast<IndexGrid&>(block_grid_) = other.block_grid_;
  copyBlocks(other);
  return *this;
}

template <typename BlockT>
BlockIndices Layer<BlockT>::allocatedBlockIndices() const {
  BlockIndices indices;
  indices.reserve(blocks_.size());
  for (const auto& kv : blocks_) {
    indices.push_back(kv.first);
  }
  return indices;
}

template <typename BlockT>
BlockIndices Layer<BlockT>::blockIndicesWithCondition(
    std::function<bool(const BlockT&)> condition) const {
  BlockIndices indices;
  for (const auto& [index, block_ptr] : blocks_) {
    if (condition(*block_ptr)) {
      indices.push_back(index);
    }
  }
  return indices;
}

template <typename BlockT>
std::vector<BlockT*> Layer<BlockT>::blocksWithCondition(
    std::function<bool(const BlockT&)> condition) {
  std::vector<BlockT*> blocks;
  for (const auto& [index, block_ptr] : blocks_) {
    if (condition(*block_ptr)) {
      blocks.push_back(block_ptr.get());
    }
  }
  return blocks;
}

template <typename BlockT>
std::vector<const BlockT*> Layer<BlockT>::blocksWithCondition(
    std::function<bool(const BlockT&)> condition) const {
  std::vector<const BlockT*> blocks;
  for (const auto& [index, block_ptr] : blocks_) {
    if (condition(*block_ptr)) {
      blocks.push_back(block_ptr.get());
    }
  }
  return blocks;
}

template <typename BlockT>
BlockT& Layer<BlockT>::getBlock(const BlockIndex& block_index) {
  const auto it = blocks_.find(block_index);
  if (it == blocks_.end()) {
    LOG(FATAL) << "Accessed unallocated block at " << block_index.transpose();
  }
  return *it->second;
}

template <typename BlockT>
const BlockT& Layer<BlockT>::getBlock(const BlockIndex& block_index) const {
  const auto it = blocks_.find(block_index);
  if (it == blocks_.end()) {
    LOG(FATAL) << "Accessed unallocated block at " << block_index.transpose();
  }
  return *it->second;
}

template <typename BlockT>
typename Layer<BlockT>::BlockPtr Layer<BlockT>::getBlockPtr(const BlockIndex& block_index) {
  const auto it = blocks_.find(block_index);
  if (it == blocks_.end()) {
    return nullptr;
  }
  return it->second;
}

template <typename BlockT>
typename Layer<BlockT>::BlockConstPtr Layer<BlockT>::getBlockPtr(
    const BlockIndex& block_index) const {
  const auto it = blocks_.find(block_index);
  if (it == blocks_.end()) {
    return nullptr;
  }
  return it->second;
}

template <typename BlockT>
template <typename... Args>
typename Layer<BlockT>::BlockPtr Layer<BlockT>::allocateBlockPtr(const BlockIndex& block_index,
                                                                 Args&&... args) {
  const auto it = blocks_.find(block_index);
  if (it != blocks_.end()) {
    return it->second;
  }

  auto block = std::make_shared<BlockT>(args...);
  blocks_[block_index] = block;
  return block;
}

template <typename BlockT>
void Layer<BlockT>::removeBlock(const BlockIndex& block_index) {
  blocks_.erase(block_index);
}

template <typename BlockT>
template <typename BlockIndexIterable>
void Layer<BlockT>::removeBlocks(const BlockIndexIterable& block_indices) {
  for (const auto& block_index : block_indices) {
    removeBlock(block_index);
  }
}

template <typename BlockT>
void Layer<BlockT>::clear() {
  blocks_.clear();
}

}  // namespace spatial_hash
