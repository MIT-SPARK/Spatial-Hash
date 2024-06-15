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

#include "spatial_hash/block_layer.h"
#include "spatial_hash/grid.h"
#include "spatial_hash/types.h"
#include "spatial_hash/voxel_block.h"

namespace spatial_hash {

/**
 * @brief Layer of voxel blocks stored in a spatial hash grid, where BlockT is a subclass of
 * VoxelBlock and has the same constructor as VoxelBlock.
 * @tparam BlockT The type of blocks contained in the layer. Must be a subclass of VoxelBlock and
 * have the same constructor as VoxelBlock.
 */
template <typename BlockT>
class VoxelLayer : protected Grid<GlobalIndex>, public BlockLayer<BlockT> {
 public:
  // Types.
  using VoxelType = typename BlockT::VoxelType;
  using BlockType = BlockT;
  using Ptr = std::shared_ptr<VoxelLayer>;
  using ConstPtr = std::shared_ptr<const VoxelLayer>;
  using BlockPtr = std::shared_ptr<BlockT>;

  // Constructors.
  /**
   * @brief Construct a layer of voxel blocks.
   * @param voxel_size The side length of each voxel.
   * @param voxels_per_side The number of voxels per side of each block. Must be a power of 2.
   */
  VoxelLayer(float voxel_size, size_t voxels_per_side)
      : Grid<GlobalIndex>(voxel_size),
        BlockLayer<BlockT>(voxel_size * voxels_per_side),
        voxels_per_side(voxels_per_side) {
    static_assert(std::is_base_of<VoxelBlock<VoxelType>, BlockT>::value,
                  "BlockT must be a subclass of VoxelBlock to create a VoxelLayer.");
  }

  VoxelLayer(const VoxelLayer& other)
      : Grid<GlobalIndex>(other),
        BlockLayer<BlockT>(other),
        voxels_per_side(other.voxels_per_side) {}

  VoxelLayer(VoxelLayer&& other)
      : Grid<GlobalIndex>(std::move(other)),
        BlockLayer<BlockT>(std::move(other)),
        voxels_per_side(std::move(other.voxels_per_side)) {}

  virtual ~VoxelLayer() = default;

  VoxelLayer& operator=(const VoxelLayer& other);

  VoxelLayer& operator=(VoxelLayer&& other);

  // Config.
  // Number of voxels per side of each block.
  const size_t voxels_per_side;
  using Grid::voxel_size;
  using Grid::voxel_size_inv;

  // Index checking and conversion.
  /**
   * @brief Check if a voxel exists at a given global voxel index.
   * @param voxel_index The global voxel index.
   */
  bool hasVoxel(const GlobalIndex& voxel_index) const {
    return this->hasBlock(blockIndexFromGlobalIndex(voxel_index, voxels_per_side));
  }

  /**
   * @brief Check if a voxel exists at a given point.
   * @param point Coordinates to check.
   */
  bool hasVoxel(const Point& point) const { return this->hasBlock(point); }

  /**
   * @brief Check if a voxel exists at a given voxel key.
   * @param key The voxel key to lookup.
   */
  bool hasVoxel(const VoxelKey& key) const { return hasVoxel(key.first); }

  /**
   * @brief Get the global index of a voxel at a given point.
   * @param point The coordinates to check.
   */
  GlobalIndex getGlobalVoxelIndex(const Point& point) const { return toIndex(point); }

  /**
   * @brief Get the key <block_index, voxel_index> of a voxel at a given point.
   */
  VoxelKey getVoxelKey(const Point& point) const {
    return keyFromGlobalIndex(toIndex(point), voxels_per_side);
  }

  /**
   * @brief Get the key <block_index, voxel_index> of a voxel at a given global voxel index.
   * @param index The global voxel index.
   */
  VoxelKey getVoxelKey(const GlobalIndex& index) const {
    return keyFromGlobalIndex(index, voxels_per_side);
  }

  /**
   * @brief Get the center point of a voxel in global coordinates.
   * @param index The global voxel index.
   */
  Point getVoxelPosition(const GlobalIndex& index) const { return toPoint(index); }

  /**
   * @brief Get the center point of a voxel in global coordinates.
   * @param key The voxel key <block_index, voxel_index>.
   */
  Point getVoxelPosition(const VoxelKey& key) const {
    return toPoint(globalIndexFromKey(key, voxels_per_side));
  }

  // Access.
  /**
   * @brief Get the voxel at a given global voxel index. This assumes the voxel exists.
   * @param voxel_index The global voxel index.
   */
  VoxelType& getVoxel(const GlobalIndex& voxel_index) {
    return this->getBlock(blockIndexFromGlobalIndex(voxel_index, voxels_per_side))
        .getVoxel(localIndexFromGlobalIndex(voxel_index, voxels_per_side));
  }
  const VoxelType& getVoxel(const GlobalIndex& voxel_index) const {
    return this->getBlock(blockIndexFromGlobalIndex(voxel_index, voxels_per_side))
        .getVoxel(localIndexFromGlobalIndex(voxel_index, voxels_per_side));
  }

  /**
   * @brief Get the voxel at a given point. This assumes the voxel exists.
   * @param point Global coordinates to check.
   */
  VoxelType& getVoxel(const Point& point) { return getVoxel(toIndex(point)); }
  const VoxelType& getVoxel(const Point& point) const { return getVoxel(toIndex(point)); }

  /**
   * @brief Get the voxel at a given voxel key. This assumes the voxel exists.
   * @param key The voxel key <block_index, voxel_index>.
   */
  VoxelType& getVoxel(const VoxelKey& key) {
    return this->getBlock(key.first).getVoxel(key.second);
  }
  const VoxelType& getVoxel(const VoxelKey& key) const {
    return this->getBlock(key.first).getVoxel(key.second);
  }

  /**
   * @brief Get the voxel at a given global voxel index or nullptr if it does not exist.
   * @param voxel_index The global voxel index.
   */
  VoxelType* getVoxelPtr(const GlobalIndex& voxel_index);
  const VoxelType* getVoxelPtr(const GlobalIndex& voxel_index) const;

  /**
   * @brief Get the voxel at a given point or nullptr if it does not exist.
   * @param point Position of the voxel in global coordinates.
   */
  VoxelType* getVoxelPtr(const Point& point) { return getVoxelPtr(toIndex(point)); }
  const VoxelType* getVoxelPtr(const Point& point) const { return getVoxelPtr(toIndex(point)); }

  /**
   * @brief Get the voxel at a given voxel key or nullptr if it does not exist.
   * @param key The voxel key <block_index, voxel_index>.
   */
  VoxelType* getVoxelPtr(const VoxelKey& key);
  const VoxelType* getVoxelPtr(const VoxelKey& key) const;

  /**
   * @brief Get the voxel at a given global voxel or allocate the corresponding block if it does not
   * exist.
   * @param voxel_index The global voxel index.
   * @param args Additional arguments to pass to the block constructor.
   * @tparam Args The types of the additional arguments.
   */
  template <typename... Args>
  VoxelType& allocateVoxel(const GlobalIndex& voxel_index, Args&&... args) {
    return allocateBlock(blockIndexFromGlobalIndex(voxel_index, voxels_per_side),
                         std::forward<Args>(args)...)
        .getVoxel(localIndexFromGlobalIndex(voxel_index, voxels_per_side));
  }

  /**
   * @brief Get the voxel at a given point or allocate the corresponding block if it does not exist.
   * @param point Global coordinates to check.
   * @param args Additional arguments to pass to the block constructor.
   * @tparam Args The types of the additional arguments.
   */
  template <typename... Args>
  VoxelType& allocateVoxel(const Point& point, Args&&... args) {
    return allocateVoxel(toIndex(point), std::forward<Args>(args)...);
  }

  /**
   * @brief Get the voxel at a given voxel key or allocate the corresponding block if it does not
   * exist.
   * @param key The voxel key <block_index, voxel_index>.
   * @param args Additional arguments to pass to the block constructor.
   * @tparam Args The types of the additional arguments.
   */
  template <typename... Args>
  VoxelType& allocateVoxel(const VoxelKey& key, Args&&... args) {
    return allocateBlock(key.first, std::forward<Args>(args)...).getVoxel(key.second);
  }

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

  // Iterators.
  struct VoxelIterable {
    using BlockMap = typename Layer<BlockT>::BlockMap;
    using MapIt = typename BlockMap::const_iterator;

    VoxelIterable(const BlockMap& blocks, size_t num_voxels)
        : blocks_(blocks), num_voxels_(num_voxels) {}
    virtual ~VoxelIterable() = default;

    class iterator : public std::iterator<std::forward_iterator_tag,
                                          VoxelType,
                                          std::ptrdiff_t,
                                          VoxelType*,
                                          VoxelType&> {
     public:
      iterator(MapIt map_it, MapIt map_end, size_t voxel_idx, size_t num_voxels)
          : map_it_(map_it), map_end_(map_end), voxel_idx_(voxel_idx), num_voxels_(num_voxels) {}

      iterator& operator++() {
        voxel_idx_++;
        if (voxel_idx_ >= num_voxels_ && map_it_ != map_end_) {
          map_it_++;
          if (map_it_ != map_end_) {
            voxel_idx_ = 0;
          }
        }
        return *this;
      }

      iterator operator++(int) {
        iterator retval = *this;
        ++(*this);
        return retval;
      }
      bool operator==(iterator other) const {
        return map_it_ == other.map_it_ && voxel_idx_ == other.voxel_idx_;
      }
      bool operator!=(iterator other) const { return !(*this == other); }
      VoxelType& operator*() const { return map_it_->second->getVoxel(voxel_idx_); }
      VoxelType* operator->() const { return &map_it_->second->getVoxel(voxel_idx_); }

     private:
      MapIt map_it_;
      const MapIt map_end_;
      size_t voxel_idx_;
      const size_t num_voxels_;
    };

    class const_iterator : public std::iterator<std::forward_iterator_tag,
                                                VoxelType,
                                                std::ptrdiff_t,
                                                VoxelType*,
                                                VoxelType&> {
     public:
      const_iterator(MapIt map_it, MapIt map_end, size_t voxel_idx, size_t num_voxels)
          : map_it_(map_it), map_end_(map_end), voxel_idx_(voxel_idx), num_voxels_(num_voxels) {}

      const_iterator& operator++() {
        voxel_idx_++;
        if (voxel_idx_ >= num_voxels_ && map_it_ != map_end_) {
          map_it_++;
          if (map_it_ != map_end_) {
            voxel_idx_ = 0;
          }
        }
        return *this;
      }

      const_iterator operator++(int) {
        const_iterator retval = *this;
        ++(*this);
        return retval;
      }
      bool operator==(const_iterator other) const {
        return map_it_ == other.map_it_ && voxel_idx_ == other.voxel_idx_;
      }
      bool operator!=(const_iterator other) const { return !(*this == other); }
      const VoxelType& operator*() const { return map_it_->second->getVoxel(voxel_idx_); }
      const VoxelType* operator->() const { return &map_it_->second->getVoxel(voxel_idx_); }

     private:
      MapIt map_it_;
      const MapIt map_end_;
      size_t voxel_idx_;
      const size_t num_voxels_;
    };

    iterator begin() { return iterator(blocks_.begin(), blocks_.end(), 0, num_voxels_); }
    iterator end() { return iterator(blocks_.end(), blocks_.end(), num_voxels_, num_voxels_); }
    const_iterator begin() const {
      return const_iterator(blocks_.begin(), blocks_.end(), 0, num_voxels_);
    }
    const_iterator end() const {
      return const_iterator(blocks_.end(), blocks_.end(), num_voxels_, num_voxels_);
    }

   private:
    const BlockMap& blocks_;
    const size_t num_voxels_;
  };

  /**
   * @brief Returns a struct that allows iterating through all voxels in the layer.
   */
  VoxelIterable voxels() {
    return VoxelIterable(this->blocks_, voxels_per_side * voxels_per_side * voxels_per_side);
  }
  const VoxelIterable voxels() const {
    return VoxelIterable(this->blocks_, voxels_per_side * voxels_per_side * voxels_per_side);
  }
};

}  // namespace spatial_hash

#include "spatial_hash/impl/voxel_layer_impl.h"
