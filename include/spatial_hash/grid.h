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

#include <stdexcept>

#include <Eigen/Dense>
#include <glog/logging.h>

#include "spatial_hash/types.h"

namespace spatial_hash {

/**
 * @brief Compute the grid index from a coordinate point.
 * @param point The point to compute the grid index from.
 * @param voxel_size_inv The inverse of the grid size. Must be > 0.
 * @tparam IndexT The type of the grid index.
 * @tparam PointT The type of the point. Dimensionality must match IndexT.
 * @return The grid index corresponding to the point.
 */
template <typename IndexT, typename PointT>
IndexT indexFromPoint(const PointT& point, const float voxel_size_inv) {
  return (point * voxel_size_inv).array().floor().template cast<typename IndexT::Scalar>();
}

/**
 * @brief Compute the cell center point from a grid index.
 * @param index The grid index to compute the point from.
 * @param voxel_size The size of the grid.
 * @tparam IndexT The type of the grid index.
 * @tparam PointT The type of the point. Dimensionality must match IndexT.
 * @return The center point of the cell corresponding to the grid index.
 */
template <typename IndexT, typename PointT>
PointT centerPointFromIndex(const IndexT& index, const float voxel_size) {
  return (index.template cast<float>() + PointT::Ones() * 0.5f) * voxel_size;
}

/**
 * @brief Compute the cell origin point from a grid index.
 * @param index The grid index to compute the point from.
 * @param voxel_size The size of the grid.
 * @tparam IndexT The type of the grid index.
 * @tparam PointT The type of the point. Dimensionality must match IndexT.
 * @return The origin point of the cell corresponding to the grid index.
 */
template <typename IndexT, typename PointT>
PointT originPointFromIndex(const IndexT& index, const float voxel_size) {
  return index.template cast<float>() * voxel_size;
}

/**
 * @brief Compute the block index from a global voxel index.
 * @param global_index The global voxel index.
 * @param voxels_per_side The number of voxels per side of a block.
 */
BlockIndex blockIndexFromGlobalIndex(const GlobalIndex& global_index, const size_t voxels_per_side);
BlockIndex2D blockIndexFromGlobalIndex(const GlobalIndex2D& global_index,
                                       const size_t voxels_per_side);

/**
 * @brief Compute the local voxel index within a block from a global voxel index.
 * @param global_index The global voxel index.
 * @param voxels_per_side The number of voxels per side of a block.
 */
VoxelIndex localIndexFromGlobalIndex(const GlobalIndex& global_index, const size_t voxels_per_side);
VoxelIndex2D localIndexFromGlobalIndex(const GlobalIndex2D& global_index,
                                       const size_t voxels_per_side);

/**
 * @brief Compute the global voxel index from a local voxel index and block index.
 * @param local_index The local voxel index.
 * @param block_index The block index.
 * @param voxels_per_side The number of voxels per side of a block.
 */
GlobalIndex globalIndexFromLocalIndices(const BlockIndex& block_index,
                                        const VoxelIndex& local_index,
                                        const size_t voxels_per_side);
GlobalIndex2D globalIndexFromLocalIndices(const BlockIndex2D& block_index,
                                          const VoxelIndex2D& local_index,
                                          const size_t voxels_per_side);
/**
 * @brief Compute the global voxel index from a voxel key.
 * @param key The voxel key.
 * @param voxels_per_side The number of voxels per side of a block.
 */
GlobalIndex globalIndexFromKey(const VoxelKey& key, const size_t voxels_per_side);
GlobalIndex2D globalIndexFromKey(const VoxelKey2D& key, const size_t voxels_per_side);

/**
 * @brief Compute the voxel key from a global voxel index.
 * @param global_index The global voxel index.
 * @param voxels_per_side The number of voxels per side of a block.
 */
VoxelKey keyFromGlobalIndex(const GlobalIndex& global_index, const size_t voxels_per_side);
VoxelKey2D keyFromGlobalIndex(const GlobalIndex2D& global_index, const size_t voxels_per_side);

/**
 * @brief Compute the local voxel index from a linear index. Linear indices range from 0 to
 voxels_per_side^3-1 and traverse x->y->z.
 * @param linear_index The linear index to convert.
 * @param voxels_per_side The number of voxels per side of a block.
 * @tparam VoxelIndexT The type of the local voxel index.
 */
template <typename VoxelIndexT>
VoxelIndexT voxelIndexFromLinearIndex(const size_t linear_index, const size_t voxels_per_side) {
  throw std::invalid_argument("Unsupported voxel index type for linear index conversion.");
}

/**
 * @brief Compute the linear index from a local voxel index. Linear indices range from 0 to
 * voxels_per_side^3-1 and traverse x->y->z.
 * @param index The local voxel index.
 * @param voxels_per_side The number of voxels per side of a block.
 */
size_t linearIndexFromVoxelIndex(const VoxelIndex& index, const size_t voxels_per_side);
size_t linearIndexFromVoxelIndex(const VoxelIndex2D& index, const size_t voxels_per_side);

/**
 * @brief A fixed resolution grid to move between points and grid indices.
 * @tparam IndexT The type of the grid index.
 */
template <typename IndexT, typename PointT>
struct Grid {
  /**
   * @brief Construct a grid with a fixed resolution.
   * @param voxel_size The width of a cell.
   */
  explicit Grid(const float voxel_size) : voxel_size(voxel_size), voxel_size_inv(1.f / voxel_size) {
    CHECK(voxel_size > 0.0f);
  }

  Grid(const Grid& other) : voxel_size(other.voxel_size), voxel_size_inv(other.voxel_size_inv) {}

  Grid(Grid&& other) : voxel_size(other.voxel_size), voxel_size_inv(other.voxel_size_inv) {}

  virtual ~Grid() = default;

  Grid& operator=(const Grid& other) {
    const_cast<float&>(voxel_size) = other.voxel_size;
    const_cast<float&>(voxel_size_inv) = other.voxel_size_inv;
    return *this;
  }

  Grid& operator=(Grid&& other) {
    const_cast<float&>(voxel_size) = other.voxel_size;
    const_cast<float&>(voxel_size_inv) = other.voxel_size_inv;
    return *this;
  }

  /**
   * @brief Compute the grid index from a coordinate point.
   * @param point The point to compute the grid index from.
   */
  IndexT toIndex(const Point& point) const {
    return indexFromPoint<IndexT, PointT>(point, voxel_size_inv);
  }

  /**
   * @brief Compute the cell center point from a grid index.
   * @param index The grid index to compute the point from.
   */
  Point toPoint(const IndexT& index) const {
    return centerPointFromIndex<IndexT, PointT>(index, voxel_size);
  }

  // Side length of one voxel.
  const float voxel_size;

  // Inverse of the voxel size.
  const float voxel_size_inv;
};

// Common grid definitions.
using IndexGrid = Grid<Index, Point>;
using LongIndexGrid = Grid<LongIndex, Point>;
using IndexGrid2D = Grid<Index2D, Point2D>;
using LongIndexGrid2D = Grid<LongIndex2D, Point2D>;

}  // namespace spatial_hash
