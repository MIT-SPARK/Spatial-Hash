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
  static constexpr unsigned int s1 = 1290;
  static constexpr unsigned int s2 = s1 * s1;

  int operator()(const Index& index) const {
    return static_cast<unsigned int>(index.x()) + static_cast<unsigned int>(index.y()) * s1 +
           static_cast<unsigned int>(index.z()) * s2;
  }
};

/**
 * @brief Hash function for long indices.
 */
struct LongIndexHash {
  // 2097152 is the maximum number to fill a 64-bit integer. Afterwards, collisions will occur
  // through overflow.
  static constexpr int64_t s1 = 2097152;
  static constexpr int64_t s2 = s1 * s1;

  int64_t operator()(const LongIndex& index) const {
    return index.x() + index.y() * s1 + index.z() * s2;
  }
};

template <typename ValueType>
using IndexHashMap =
    std::unordered_map<Index,
                       ValueType,
                       IndexHash,
                       std::equal_to<Index>,
                       Eigen::aligned_allocator<std::pair<const Index, ValueType>>>;

template <typename ValueType>
using LongIndexHashMap =
    std::unordered_map<LongIndex,
                       ValueType,
                       LongIndexHash,
                       std::equal_to<LongIndex>,
                       Eigen::aligned_allocator<std::pair<const LongIndex, ValueType>>>;
using IndexSet =
    std::unordered_set<Index, IndexHash, std::equal_to<Index>, Eigen::aligned_allocator<Index>>;

using LongIndexSet = std::unordered_set<LongIndex,
                                        LongIndexHash,
                                        std::equal_to<LongIndex>,
                                        Eigen::aligned_allocator<LongIndex>>;

}  // namespace spatial_hash
