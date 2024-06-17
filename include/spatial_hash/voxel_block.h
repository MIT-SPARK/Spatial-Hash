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

namespace spatial_hash {

/**
 * @brief A data structure for blocks of densely allocated 3D voxels.
 * @tparam VoxelT The voxel type contained in the block.
 * @note Local voxel indices range from [0,0,0] at the bottom left front corner to
 * [vps-1,vps-1,vps-1] at the top right back corner. Linear indices range from 0 to vps^3-1 and
 * traverse x->y->z.
 */
template <typename VoxelT>
struct VoxelBlock : public IndexGrid, public Block {
  // Types.
  using VoxelType = VoxelT;
  using Ptr = std::shared_ptr<VoxelBlock>;
  using ConstPtr = std::shared_ptr<const VoxelBlock>;

  // Constructors.
  /**
   * @brief Construct a block with a given voxel size, number of voxels per side, and position.
   * @param voxel_size The side length of each voxel.
   * @param voxels_per_side The number of voxels per side of the block. Must be a power of 2.
   * @param index Index of the block in the grid.
   */
  VoxelBlock(float voxel_size, size_t voxels_per_side, const BlockIndex& index);

  VoxelBlock(const VoxelBlock& other);

  VoxelBlock(VoxelBlock&& other);

  virtual ~VoxelBlock() = default;

  VoxelBlock& operator=(const VoxelBlock& other);

  VoxelBlock& operator=(VoxelBlock&& other);

  // Config.
  // Number of voxels per side of the block.
  const size_t voxels_per_side;

  /**
   * @brief Get the number of voxels in the block.
   */
  size_t numVoxels() const { return voxels_per_side * voxels_per_side * voxels_per_side; }

  // Index checking and conversion.
  /**
   * @brief Check if a linear index is valid, i.e., it is within the block.
   * @param linear_index The linear index to check.
   */
  bool isValidLinearIndex(size_t linear_index) const { return linear_index < numVoxels(); }

  /**
   * @brief Check if a local voxel index is valid, i.e., it is within the block.
   * @param index The local voxel index to check.
   */
  bool isValidVoxelIndex(const VoxelIndex& index) const {
    return (index.array() >= 0).all() && (index.array() < voxels_per_side).all();
  }

  /**
   * @brief Check whether the given voxel index is on the boundary of the block.
   * @param index The local voxel index to check.
   */
  bool isOnBoundary(const VoxelIndex& index) const {
    return (index.array() == 0).any() || (index.array() == voxels_per_side - 1).any();
  }
  bool isOnBoundary(const size_t& index) const {
    return isOnBoundary(voxelIndexFromLinearIndex(index, voxels_per_side));
  }

  /**
   * @brief Compute the linear index from a local voxel index. This assumes the local index is
   * valid. Note that the resulting lienar index may be valid (but wrong) even if the local index is
   * not.
   * @param index The local voxel index.
   */
  size_t getLinearIndex(const VoxelIndex& index) const {
    return linearIndexFromVoxelIndex(index, voxels_per_side);
  }

  /**
   * @brief Get the local voxel index from a linear index. The voxel index will be invalid if the
   * linear index is invalid.
   * @param linear_index The linear index to convert.
   */
  VoxelIndex getVoxelIndex(size_t linear_index) const {
    return voxelIndexFromLinearIndex(linear_index, voxels_per_side);
  }

  /**
   * @brief Get the local voxel index from a global position. This voxel index may be invalid if the
   * point is outside the block.
   * @param position The global position to convert.
   */
  VoxelIndex getVoxelIndex(const Point& position) const { return toIndex(position - origin()); }

  /**
   * @brief Get the global voxel index from a linear index.
   * @param linear_index The linear index to convert.
   */
  GlobalIndex getGlobalVoxelIndex(size_t linear_index) const {
    return globalIndexFromLocalIndices(this->index, getVoxelIndex(linear_index), voxels_per_side);
  }

  /**
   * @brief Get the global voxel index from a local index.
   * @param index The local voxel index to convert.
   */
  GlobalIndex getGlobalVoxelIndex(const VoxelIndex& index) const {
    return globalIndexFromLocalIndices(this->index, index, voxels_per_side);
  }

  /**
   * @brief Get the key <block index, voxel index> from a local voxel index.
   * @param index The local voxel index.
   */
  VoxelKey getVoxelKey(const VoxelIndex& index) const { return std::make_pair(this->index, index); }

  /**
   * @brief Get the key <block index, voxel index> from a linear index.
   * @param linear_index The linear index.
   */
  VoxelKey getVoxelKey(size_t linear_index) const {
    return getVoxelKey(getVoxelIndex(linear_index));
  }

  // Access.
  /**
   * @brief Get the voxel at a linear index. This assumes the linear index is valid.
   */
  VoxelType& getVoxel(size_t linear_index) { return voxels[linear_index]; }
  const VoxelType& getVoxel(size_t linear_index) const { return voxels[linear_index]; }

  /**
   * @brief Get the voxel at a local index. This assumes the local index is valid.
   */
  VoxelType& getVoxel(const VoxelIndex& index) { return voxels[getLinearIndex(index)]; }
  const VoxelType& getVoxel(const VoxelIndex& index) const { return voxels[getLinearIndex(index)]; }

  // Additional functions.
  /**
   * @brief Get the position (cell center) of a voxel at a linear index in global frame.
   * @param linear_index The linear index.
   */
  Point getVoxelPosition(size_t linear_index) const {
    return getVoxelPosition(getVoxelIndex(linear_index));
  }

  /**
   * @brief Get the position (cell center) of a voxel at a local index in global frame.
   * @param index The local voxel index.
   */
  Point getVoxelPosition(const VoxelIndex& index) const {
    return origin() + index.cast<float>() * voxel_size + Point::Constant(voxel_size * 0.5f);
  }

  // Iterators.
  typename std::vector<VoxelType>::iterator begin() { return voxels.begin(); }
  typename std::vector<VoxelType>::iterator end() { return voxels.end(); }
  typename std::vector<VoxelType>::const_iterator begin() const { return voxels.begin(); }
  typename std::vector<VoxelType>::const_iterator end() const { return voxels.end(); }

 private:
  std::vector<VoxelType> voxels;
};

}  // namespace spatial_hash

#include "spatial_hash/impl/voxel_block_impl.h"
