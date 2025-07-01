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
#include <unordered_map>
#include <unordered_set>
#include <utility>

#include "spatial_hash/types.h"

namespace spatial_hash {

/**
 * @brief Hash function for indices.
 */
struct IndexHash {
  // 1290 is the maximum number to fill a 32-bit integer. Afterwards, collisions will occur through
  // overflow.
  inline static const auto s = Index(1, 1290, 1290 * 1290);
  inline static const auto s2d = Index2D(1, 1290);

  int operator()(const Index& index) const { return index.dot(s); }
  int operator()(const Index2D& index) const { return index.dot(s2d); }
};

/**
 * @brief Hash function for long indices.
 */
struct LongIndexHash {
  // 2097152 is the maximum number to fill a 64-bit integer. Afterwards, collisions will occur
  // through overflow.
  inline static const auto s = LongIndex(1, 2097152, 2097152 * 2097152);
  inline static const auto s2d = LongIndex2D(1, 2097152);

  int64_t operator()(const LongIndex& index) const { return index.dot(s); }
  int64_t operator()(const LongIndex2D& index) const { return index.dot(s2d); }
};

// Base templates for easier parsing of types in the hash maps and sets.
template <typename ValueT, typename IndexT, typename HashT>
using HashMapBase = std::unordered_map<IndexT,
                                       ValueT,
                                       HashT,
                                       std::equal_to<IndexT>,
                                       Eigen::aligned_allocator<std::pair<const IndexT, ValueT>>>;

template <typename IndexT, typename HashT>
using IndexSetBase =
    std::unordered_set<IndexT, HashT, std::equal_to<IndexT>, Eigen::aligned_allocator<IndexT>>;

// Common maps and sets.
template <typename ValueT>
using IndexMap = HashMapBase<ValueT, Index, IndexHash>;

template <typename ValueT>
using LongIndexMap = HashMapBase<ValueT, LongIndex, LongIndexHash>;

template <typename ValueT>
using IndexMap2D = HashMapBase<ValueT, Index2D, IndexHash>;

template <typename ValueT>
using LongIndexMap2D = HashMapBase<ValueT, LongIndex2D, LongIndexHash>;

using IndexSet = IndexSetBase<Index, IndexHash>;

using LongIndexSet = IndexSetBase<LongIndex, LongIndexHash>;

using IndexSet2D = IndexSetBase<Index2D, IndexHash>;

using LongIndexSet2D = IndexSetBase<LongIndex2D, LongIndexHash>;

}  // namespace spatial_hash
