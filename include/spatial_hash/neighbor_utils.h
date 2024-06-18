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

#include "spatial_hash/grid.h"
#include "spatial_hash/types.h"
#include "spatial_hash/voxel_layer.h"

namespace spatial_hash {

/**
 * @brief Index-based neighbor search.
 */
struct NeighborSearch {
  // Constructors.
  explicit NeighborSearch(size_t connectivity);
  virtual ~NeighborSearch() = default;

  /**
   * @brief Get the n-connectivity neighbors of a given index.
   * @param index The index to find neighbors for.
   * @param include_self Whether to include the index itself in the list of neighbors.
   */
  template <typename IndexT>
  std::vector<IndexT> neighborIndices(const IndexT& index, const bool include_self = false) const;

  const size_t connectivity;

 protected:
  // Neighbor offsets ordered for self->6->18->26 connectivity.
  static const Eigen::Matrix<int, 3, 27> kNeighborOffsets;
  void checkConnectivity() const;
};

/**
 * @brief Neighbor search over block layers. For now only supports searching over const layers.
 */
template <typename BlockT>
struct BlockNeighborSearch : public NeighborSearch {
  BlockNeighborSearch(const Layer<BlockT>& layer, size_t connectivity);
  virtual ~BlockNeighborSearch() = default;

  /**
   * @brief Get pointers to all allocated neighbor blocks of a given block.
   * @param block_index The index of the block to find neighbors for.
   * @param include_self Whether to include the block itself in the list of neighbors.
   */
  std::vector<const BlockT*> neighborBlocks(const BlockIndex& block_index,
                                            const bool include_self = false) const;

  // TODO(lschmid): In the future support a non-const version of this that can also allocated
  // neighbor blocks.

 protected:
  const Layer<BlockT>& layer_;
};

/**
 * @brief Neighbor search over voxel layers. For now only supports searching over const layers.
 */
template <typename BlockT>
struct VoxelNeighborSearch : public NeighborSearch {
  using VoxelType = typename BlockT::VoxelType;

  VoxelNeighborSearch(const VoxelLayer<BlockT>& layer, size_t connectivity);

  virtual ~VoxelNeighborSearch() = default;

  /**
   * @brief Get the global voxel indices of all neighbors of a given voxel.
   * @param index The global voxel index of the voxel to find neighbors for.
   * @param include_self Whether to include the voxel itself in the list of neighbors.
   */
  GlobalIndices neighborIndices(const GlobalIndex& index, const bool include_self = false) const {
    return NeighborSearch::neighborIndices(index, include_self);
  }

  /**
   * @brief Get the indices of all neighbors of a given voxel.
   * @param key The key of the voxel to find neighbors for.
   * @param include_self Whether to include the voxel itself in the list of neighbors.
   */
  std::vector<VoxelKey> neighborKeys(const VoxelKey& key, const bool include_self = false) const;

  /**
   * @brief Get pointers to all allocated neighbor voxels of a given voxel.
   * @param index The global voxel index of the voxel to find neighbors for.
   * @param include_self Whether to include the voxel itself in the list of neighbors.
   */
  std::vector<const VoxelType*> neighborVoxels(const GlobalIndex& index,
                                               const bool include_self = false) const;

  /**
   * @brief Get pointers to all allocated neighbor voxels of a given voxel.
   * @param key The key of the voxel to find neighbors for.
   * @param include_self Whether to include the voxel itself in the list of neighbors.
   */
  std::vector<const VoxelType*> neighborVoxels(const VoxelKey& key,
                                               const bool include_self = false) const {
    return neighborVoxels(globalIndexFromKey(key, layer_.voxels_per_side), include_self);
  }

  // TODO(lschmid): In the future support a non-const version of this that can also allocated
  // neighbor voxel blocks.

 protected:
  const VoxelLayer<BlockT>& layer_;
};

}  // namespace spatial_hash

#include "spatial_hash/impl/neighbor_utils_impl.h"
