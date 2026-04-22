// Adapted from OpenMVG

// Copyright (c) 2016 Pierre MOULON
//               2018 Nikolaus DEMMEL

// This file was originally part of OpenMVG, an Open Multiple View Geometry C++
// library.

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#pragma once

#include <cassert>
#include <cstdint>
#include <limits>
#include <numeric>
#include <vector>

// Union-Find/Disjoint-Set data structure
//--
// A disjoint-set data structure also called a union–find data structure
// or merge–find set, is a data structure that keeps track of a set of elements
// partitioned into a number of disjoint (non-overlapping) subsets.
// It supports two operations:
// - Find: Determine which subset a particular element is in.
//   - It returns an item from this set that serves as its "representative";
// - Union: Join two subsets into a single subset.
// Sometime a Connected method is implemented:
// - Connected:
//   - By comparing the result of two Find operations, one can determine whether
//      two elements are in the same subset.
//--
struct UnionFind {
  using ValueType = uint32_t;

  // Special Value for invalid parent index
  static ValueType InvalidIndex() {
    return std::numeric_limits<ValueType>::max();
  }

  // Represent the DS/UF forest thanks to two array:
  // A parent 'pointer tree' where each node holds a reference to its parent
  // node
  std::vector<ValueType> m_cc_parent;
  // A rank array used for union by rank
  std::vector<ValueType> m_cc_rank;
  // A 'size array' to know the size of each connected component
  std::vector<ValueType> m_cc_size;

  // Init the UF structure with num_cc nodes
  void InitSets(const ValueType num_cc) {
    // all set size are 1 (independent nodes)
    m_cc_size.resize(num_cc, 1);
    // Parents id have their own CC id {0,n}
    m_cc_parent.resize(num_cc);
    std::iota(m_cc_parent.begin(), m_cc_parent.end(), 0);
    // Rank array (0)
    m_cc_rank.resize(num_cc, 0);
  }

  // Append a single new independent node (self-parent) to the UF forest.
  // Returns the DSU index of the newly added node — equal to GetNumNodes()
  // before the call. Used by incremental trackers that discover new features
  // one-at-a-time rather than from a known batch.
  ValueType AddIndex() {
    const ValueType new_idx = static_cast<ValueType>(m_cc_size.size());
    m_cc_parent.push_back(new_idx);
    m_cc_rank.push_back(0);
    m_cc_size.push_back(1);
    return new_idx;
  }

  // Mark root_idx as invalid. Subsequent Find() calls on any node in this
  // set will return InvalidIndex() via path compression.
  // PRECONDITION: root_idx must currently be a root (Find(root_idx)==root_idx).
  // The caller is responsible for clearing any external maps keyed on
  // root_idx (e.g. mpUFNodeIndexToTrackIdMap).
  void InvalidateRoot(ValueType root_idx) {
    assert(root_idx < m_cc_parent.size());
    m_cc_parent[root_idx] = InvalidIndex();
  }

  // Return the number of nodes that have been initialized in the UF tree
  std::size_t GetNumNodes() const { return m_cc_size.size(); }

  // Return the representative set id of I nth component
  ValueType Find(ValueType i) {
    // Recursively set all branch as children of root (Path compression)
    if (m_cc_parent[i] != i && m_cc_parent[i] != InvalidIndex())
      m_cc_parent[i] = Find(m_cc_parent[i]);
    return m_cc_parent[i];
  }

  // Replace sets containing I and J with their union
  void Union(ValueType i, ValueType j) {
    i = Find(i);
    j = Find(j);
    if (i == j) {  // Already in the same set. Nothing to do
      return;
    }

    // x and y are not already in same set. Merge them.
    // Perform an union by rank:
    //  - always attach the smaller tree to the root of the larger tree
    if (m_cc_rank[i] < m_cc_rank[j]) {
      m_cc_parent[i] = j;
      m_cc_size[j] += m_cc_size[i];

    } else {
      m_cc_parent[j] = i;
      m_cc_size[i] += m_cc_size[j];
      if (m_cc_rank[i] == m_cc_rank[j]) ++m_cc_rank[i];
    }
  }
};
