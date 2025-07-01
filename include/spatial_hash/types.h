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

#include <Eigen/Core>

namespace spatial_hash {

// Coordinates in space. By default, spatial hash uses 3D coordinates, but 2D coordinates are also
// supported.
using Point = Eigen::Vector3f;
using Point2D = Eigen::Vector2f;

// Indices.
using IndexType = int;
using Index = Eigen::Matrix<IndexType, 3, 1>;
using Index2D = Eigen::Matrix<IndexType, 2, 1>;
using LongIndexType = int64_t;
using LongIndex = Eigen::Matrix<LongIndexType, 3, 1>;
using LongIndex2D = Eigen::Matrix<LongIndexType, 2, 1>;

// Predefined index types.
using BlockIndex = Index;                                  // Block indices.
using VoxelIndex = Index;                                  // Local voxel indices.
using GlobalIndex = Index;                                 // Global voxel indices.
using VoxelKey = std::pair<BlockIndex, VoxelIndex>;        // <BlockIndex, local VoxelIndex>
using BlockIndex2D = Index2D;                              // 2D block indices.
using VoxelIndex2D = Index2D;                              // 2D local voxel indices
using GlobalIndex2D = Index2D;                             // 2D global voxel indices
using VoxelKey2D = std::pair<BlockIndex2D, VoxelIndex2D>;  // <BlockIndex2D, local VoxelIndex2D>

// Index containers.
using BlockIndices = std::vector<BlockIndex>;
using VoxelIndices = std::vector<VoxelIndex>;
using GlobalIndices = std::vector<GlobalIndex>;
using VoxelKeys = std::vector<VoxelKey>;
using BlockIndices2D = std::vector<BlockIndex2D>;
using VoxelIndices2D = std::vector<VoxelIndex2D>;
using GlobalIndices2D = std::vector<GlobalIndex2D>;
using VoxelKeys2D = std::vector<VoxelKey2D>;

}  // namespace spatial_hash
