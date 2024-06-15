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
#include "spatial_hash/grid.h"

namespace spatial_hash {

BlockIndex blockIndexFromGlobalIndex(const GlobalIndex& global_index,
                                     const size_t voxels_per_side) {
  return BlockIndex(global_index.x() / voxels_per_side,
                    global_index.y() / voxels_per_side,
                    global_index.z() / voxels_per_side);
}

VoxelIndex localIndexFromGlobalIndex(const GlobalIndex& global_index,
                                     const size_t voxels_per_side) {
  return VoxelIndex(global_index.x() % voxels_per_side,
                    global_index.y() % voxels_per_side,
                    global_index.z() % voxels_per_side);
}

GlobalIndex globalIndexFromLocalIndices(const BlockIndex& block_index,
                                        const VoxelIndex& local_index,
                                        const size_t voxels_per_side) {
  return GlobalIndex(local_index.x() + block_index.x() * voxels_per_side,
                     local_index.y() + block_index.y() * voxels_per_side,
                     local_index.z() + block_index.z() * voxels_per_side);
}

GlobalIndex globalIndexFromKey(const VoxelKey& key, const size_t voxels_per_side) {
  return globalIndexFromLocalIndices(key.first, key.second, voxels_per_side);
}

VoxelKey keyFromGlobalIndex(const GlobalIndex& global_index, const size_t voxels_per_side) {
  return VoxelKey(blockIndexFromGlobalIndex(global_index, voxels_per_side),
                  localIndexFromGlobalIndex(global_index, voxels_per_side));
}

VoxelIndex voxelIndexFromLinearIndex(const size_t linear_index, const size_t voxels_per_side) {
  int rem = linear_index;
  VoxelIndex result;
  std::div_t div_temp = std::div(rem, voxels_per_side * voxels_per_side);
  rem = div_temp.rem;
  result.z() = div_temp.quot;
  div_temp = std::div(rem, voxels_per_side);
  result.y() = div_temp.quot;
  result.x() = div_temp.rem;
  return result;
}

size_t linearIndexFromVoxelIndex(const VoxelIndex& index, const size_t voxels_per_side) {
  return index.x() + voxels_per_side * (index.y() + index.z() * voxels_per_side);
}

}  // namespace spatial_hash
