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

#include <utility>
#include <vector>

#include "spatial_hash/neighbor_utils.h"

namespace spatial_hash {

template <typename BlockT>
BlockNeighborSearch<BlockT>::BlockNeighborSearch(const Layer<BlockT>& layer, size_t connectivity)
    : NeighborSearch(connectivity), layer_(layer) {}

template <typename IndexT>
std::vector<IndexT> NeighborSearch::neighborIndices(const IndexT& index,
                                                    const bool include_self) const {
  const size_t offset = include_self ? 0 : 1;
  std::vector<IndexT> neighbors;
  neighbors.reserve(connectivity + offset);
  for (size_t i = offset; i <= connectivity; ++i) {
    neighbors.emplace_back(index + kNeighborOffsets.col(i).cast<typename IndexT::Scalar>());
  }
  return neighbors;
}

template <typename BlockT>
std::vector<const BlockT*> BlockNeighborSearch<BlockT>::neighborBlocks(
    const BlockIndex& block_index,
    const bool include_self) const {
  std::vector<const BlockT*> neighbors;
  neighbors.reserve(static_cast<size_t>(connectivity) + static_cast<size_t>(include_self));
  for (const auto& index : neighborIndices(block_index, include_self)) {
    const auto block = layer_.getBlockPtr(index);
    if (block) {
      neighbors.push_back(block.get());
    }
  }

  return neighbors;
}

template <typename BlockT>
VoxelNeighborSearch<BlockT>::VoxelNeighborSearch(const VoxelLayer<BlockT>& layer,
                                                 size_t connectivity)
    : NeighborSearch(connectivity), layer_(layer) {}

template <typename BlockT>
std::vector<VoxelKey> VoxelNeighborSearch<BlockT>::neighborKeys(const VoxelKey& key,
                                                                const bool include_self) const {
  std::vector<VoxelKey> neighbors;
  neighbors.reserve(static_cast<size_t>(connectivity) + static_cast<size_t>(include_self));
  for (const auto& neighbor_index :
       neighborIndices(globalIndexFromKey(key, layer_.voxels_per_side), include_self)) {
    neighbors.push_back(keyFromGlobalIndex(neighbor_index, layer_.voxels_per_side));
  }
  return neighbors;
}

template <typename BlockT>
std::vector<const typename VoxelNeighborSearch<BlockT>::VoxelType*>
VoxelNeighborSearch<BlockT>::neighborVoxels(const GlobalIndex& index,
                                            const bool include_self) const {
  std::vector<const VoxelType*> neighbors;
  neighbors.reserve(static_cast<size_t>(connectivity) + static_cast<size_t>(include_self));
  for (const auto& neighbor_index : neighborIndices(index, include_self)) {
    const auto voxel = layer_.getVoxelPtr(neighbor_index);
    if (voxel) {
      neighbors.push_back(voxel);
    }
  }
  return neighbors;
}

}  // namespace spatial_hash
