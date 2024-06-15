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

#include "spatial_hash/voxel_layer.h"

namespace spatial_hash {

template <typename BlockT>
VoxelLayer<BlockT>& VoxelLayer<BlockT>::operator=(const VoxelLayer& other) {
  if (this == &other) {
    return *this;
  }

  const_cast<size_t&>(voxels_per_side) = other.voxels_per_side;
  Grid<GlobalIndex>::operator=(other);
  BlockLayer<BlockT>::operator=(other);
  return *this;
}

template <typename BlockT>
VoxelLayer<BlockT>& VoxelLayer<BlockT>::operator=(VoxelLayer&& other) {
  if (this == &other) {
    return *this;
  }

  const_cast<size_t&>(voxels_per_side) = other.voxels_per_side;
  Grid<GlobalIndex>::operator=(other);
  BlockLayer<BlockT>::operator=(other);
  return *this;
}

template <typename BlockT>
typename VoxelLayer<BlockT>::VoxelType* VoxelLayer<BlockT>::getVoxelPtr(
    const GlobalIndex& voxel_index) {
  const BlockIndex block_index = blockIndexFromGlobalIndex(voxel_index, voxels_per_side);
  const auto it = this->blocks_.find(block_index);
  if (it == this->blocks_.end()) {
    return nullptr;
  }

  const VoxelIndex local_index = localIndexFromGlobalIndex(voxel_index, voxels_per_side);
  return &it->second->getVoxel(local_index);
}

template <typename BlockT>
const typename VoxelLayer<BlockT>::VoxelType* VoxelLayer<BlockT>::getVoxelPtr(
    const GlobalIndex& voxel_index) const {
  const BlockIndex block_index = blockIndexFromGlobalIndex(voxel_index, voxels_per_side);
  const auto it = this->blocks_.find(block_index);
  if (it == this->blocks_.end()) {
    return nullptr;
  }

  const VoxelIndex local_index = localIndexFromGlobalIndex(voxel_index, voxels_per_side);
  return &it->second->getVoxel(local_index);
}

template <typename BlockT>
typename VoxelLayer<BlockT>::VoxelType* VoxelLayer<BlockT>::getVoxelPtr(const VoxelKey& key) {
  const auto it = this->blocks_.find(key.first);
  if (it == this->blocks_.end()) {
    return nullptr;
  }
  return &it->second->getVoxel(key.second);
}

template <typename BlockT>
const typename VoxelLayer<BlockT>::VoxelType* VoxelLayer<BlockT>::getVoxelPtr(
    const VoxelKey& key) const {
  const auto it = this->blocks_.find(key.first);
  if (it == this->blocks_.end()) {
    return nullptr;
  }
  return &it->second->getVoxel(key.second);
}

template <typename BlockT>
template <typename... Args>
typename VoxelLayer<BlockT>::BlockPtr VoxelLayer<BlockT>::allocateBlockPtr(
    const BlockIndex& block_index,
    Args&&... args) {
  const auto it = this->blocks_.find(block_index);
  if (it != this->blocks_.end()) {
    return it->second;
  }

  auto block = std::make_shared<BlockT>(voxel_size, voxels_per_side, block_index, args...);
  this->blocks_[block_index] = block;
  return block;
}

}  // namespace spatial_hash
