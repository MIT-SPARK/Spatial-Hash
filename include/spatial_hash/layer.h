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

#include <functional>
#include <memory>
#include <utility>
#include <vector>

#include <glog/logging.h>

#include "spatial_hash/block.h"
#include "spatial_hash/grid.h"
#include "spatial_hash/hash.h"
#include "spatial_hash/types.h"

namespace spatial_hash {

/**
 * @brief Layer of data blocks stored in a spatial hash grid, where BlockT can be any data structure
 * but should be copy constructable.
 * @tparam BlockT The type of blocks contained in the layer. Can be any data structure but should be
 * copy constructable.
 */
template <typename BlockT>
class Layer {
 public:
  // Types.
  using Ptr = std::shared_ptr<Layer>;
  using ConstPtr = std::shared_ptr<const Layer>;
  using BlockType = BlockT;
  using BlockPtr = std::shared_ptr<BlockT>;
  using BlockConstPtr = std::shared_ptr<const BlockT>;
  using BlockMap = IndexHashMap<BlockPtr>;

  // Constructors.
  /**
   * @brief Construct a layer of data blocks.
   * @param block_size The side length of each block.
   */
  explicit Layer(float block_size) : block_grid_(block_size) {}

  virtual ~Layer() = default;

  // Copy constructor and assignment create deep copies of the blocks. This assumes blocks are copy
  // constructible.
  Layer(const Layer& other) : block_grid_(other.block_grid_) { copyBlocks(other); }

  Layer& operator=(const Layer& other);

  // Layer info.
  float blockSize() const { return block_grid_.voxel_size; }
  float blockSizeInv() const { return block_grid_.voxel_size_inv; }

  /**
   * @brief Get the number of allocated blocks in the layer.
   */
  size_t numBlocks() const { return blocks_.size(); }

  /**
   * @brief Get the indices of all allocated blocks in the layer.
   */
  BlockIndices allocatedBlockIndices() const;

  /**
   * @brief Get the indices of all blocks that satisfy a given condition.
   * @param condition The condition to check.
   */
  BlockIndices blockIndicesWithCondition(std::function<bool(const BlockT&)> condition) const;

  /**
   * @brief Get pointers to all blocks that satisfy a given condition.
   * @param condition The condition to check.
   */
  std::vector<BlockT*> blocksWithCondition(std::function<bool(const BlockT&)> condition);
  std::vector<const BlockT*> blocksWithCondition(
      std::function<bool(const BlockT&)> condition) const;

  // Index checking and conversion.
  /**
   * @brief Check if a block exists at a given index.
   * @param block_index The block index.
   */
  bool hasBlock(const BlockIndex& block_index) const { return blocks_.count(block_index) > 0; }

  /**
   * @brief Check if a block exists at a given point.
   * @param point Coordinates to check.
   */
  bool hasBlock(const Point& point) const { return hasBlock(block_grid_.toIndex(point)); }

  /**
   * @brief Get the block index of a given point.
   * @param point Global coordinates to check.
   */
  BlockIndex getBlockIndex(const Point& point) const { return block_grid_.toIndex(point); }

  // Access.
  /**
   * @brief Get the block at a given index. This assumes the block exists.
   * @param block_index The block index.
   */
  BlockT& getBlock(const BlockIndex& block_index);
  const BlockT& getBlock(const BlockIndex& block_index) const;

  /**
   * @brief Get the block at a given point. This assumes the block exists.
   * @param point Coordinates to check.
   */
  BlockT& getBlock(const Point& point) { return getBlock(block_grid_.toIndex(point)); }
  const BlockT& getBlock(const Point& point) const { return getBlock(block_grid_.toIndex(point)); }

  /**
   * @brief Get the block at a given index. Returns nullptr if the block does not exist.
   * @param block_index The block index.
   */
  BlockPtr getBlockPtr(const BlockIndex& block_index);
  BlockConstPtr getBlockPtr(const BlockIndex& block_index) const;

  /**
   * @brief Get the block at a given point. Returns nullptr if the block does not exist.
   */
  BlockPtr getBlockPtr(const Point& point) { return getBlockPtr(block_grid_.toIndex(point)); }
  BlockConstPtr getBlockPtr(const Point& point) const {
    return getBlockPtr(block_grid_.toIndex(point));
  }

  /**
   * @brief Gets the block at a given index or allocates a new block if it does not exist.
   * @param block_index The block index to retrieve or allocate.
   * @param args The arguments to pass to the block constructor.
   * @tparam Args The types of the arguments to pass to the block constructor.
   */
  template <typename... Args>
  BlockT& allocateBlock(const BlockIndex& block_index, Args&&... args) {
    return *allocateBlockPtr(block_index, std::forward<Args>(args)...);
  }

  /**
   * @brief Gets the block at a given point or allocates a new block if it does not exist.
   * @param block_index The point to retrieve or allocate.
   * @param args The arguments to pass to the block constructor.
   * @tparam Args The types of the arguments to pass to the block constructor.
   */
  template <typename... Args>
  BlockPtr allocateBlockPtr(const BlockIndex& block_index, Args&&... args);

  /**
   * @brief Remove a block from the layer.
   * @param block_index The index of the block to remove.
   */
  void removeBlock(const BlockIndex& block_index);

  /**
   * @brief Remove several blocks from the layer.
   * @param block_indices The indices of the blocks to remove.
   * @tparam BlockIndexIterable An iterable type containing block indices.
   */
  template <typename BlockIndexIterable>
  void removeBlocks(const BlockIndexIterable& block_indices);

  /**
   * @brief Remove all blocks from the layer.
   */
  void clear();

 protected:
  const IndexGrid block_grid_;
  BlockMap blocks_;

  void copyBlocks(const Layer& other);

 public:
  // Iterators.
  class iterator
      : public std::iterator<std::forward_iterator_tag, BlockT, std::ptrdiff_t, BlockT*, BlockT&> {
   public:
    using MapIt = typename BlockMap::iterator;

    explicit iterator(MapIt it) : it_(it) {}
    iterator& operator++() {
      it_++;
      return *this;
    }
    iterator operator++(int) {
      iterator retval = *this;
      ++(*this);
      return retval;
    }
    bool operator==(iterator other) const { return it_ == other.it_; }
    bool operator!=(iterator other) const { return it_ != other.it_; }
    BlockT& operator*() const { return *it_->second; }
    BlockT* operator->() const { return it_->second.get(); }

   private:
    MapIt it_;
  };

  class const_iterator : public std::iterator<std::forward_iterator_tag,
                                              const BlockT,
                                              std::ptrdiff_t,
                                              BlockT const*,
                                              const BlockT&> {
   public:
    using MapIt = typename BlockMap::const_iterator;

    explicit const_iterator(MapIt it) : it_(it) {}
    const_iterator& operator++() {
      it_++;
      return *this;
    }
    const_iterator operator++(int) {
      const_iterator retval = *this;
      ++(*this);
      return retval;
    }
    bool operator==(const_iterator other) const { return it_ == other.it_; }
    bool operator!=(const_iterator other) const { return it_ != other.it_; }
    const BlockT& operator*() const { return *it_->second; }
    const BlockT* operator->() const { return it_->second.get(); }

   private:
    MapIt it_;
  };

  iterator begin() { return iterator(blocks_.begin()); }
  iterator end() { return iterator(blocks_.end()); }
  const_iterator begin() const { return const_iterator(blocks_.begin()); }
  const_iterator end() const { return const_iterator(blocks_.end()); }
};

}  // namespace spatial_hash

#include "spatial_hash/impl/layer_impl.h"
