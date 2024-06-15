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

#include "spatial_hash/voxel_block.h"

namespace spatial_hash {

template <typename VoxelT>
VoxelBlock<VoxelT>::VoxelBlock(float voxel_size, size_t voxels_per_side, const BlockIndex& index)
    : IndexGrid(voxel_size),
      Block(voxel_size * voxels_per_side, index),
      voxels_per_side(voxels_per_side),
      voxels(numVoxels()) {
  CHECK((voxels_per_side & (voxels_per_side - 1)) == 0) << "voxels_per_side must be a power of 2";
}

template <typename VoxelT>
VoxelBlock<VoxelT>::VoxelBlock(const VoxelBlock& other)
    : IndexGrid(other),
      Block(other),
      voxels_per_side(other.voxels_per_side),
      voxels(other.voxels) {}

template <typename VoxelT>
VoxelBlock<VoxelT>::VoxelBlock(VoxelBlock&& other)
    : IndexGrid(std::move(other)),
      Block(std::move(other)),
      voxels_per_side(std::move(other.voxels_per_side)),
      voxels(std::move(other.voxels)) {}

template <typename VoxelT>
VoxelBlock<VoxelT>& VoxelBlock<VoxelT>::operator=(const VoxelBlock& other) {
  IndexGrid::operator=(other);
  Block::operator=(other);
  const_cast<size_t&>(voxels_per_side) = other.voxels_per_side;
  voxels = other.voxels;
  return *this;
}

template <typename VoxelT>
VoxelBlock<VoxelT>& VoxelBlock<VoxelT>::operator=(VoxelBlock&& other) {
  IndexGrid::operator=(std::move(other));
  Block::operator=(std::move(other));
  const_cast<size_t&>(voxels_per_side) = std::move(other.voxels_per_side);
  voxels = std::move(other.voxels);
  return *this;
}

}  // namespace spatial_hash
