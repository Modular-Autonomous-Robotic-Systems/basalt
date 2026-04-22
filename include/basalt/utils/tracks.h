// Adapted from OpenMVG

// Copyright (c) 2012, 2013 Pierre MOULON
//               2018 Nikolaus DEMMEL

// This file was originally part of OpenMVG, an Open Multiple View Geometry C++
// library.

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

// Implementation of [1] an efficient algorithm to compute track from pairwise
//  correspondences.
//
//  [1] Pierre Moulon and Pascal Monasse,
//    "Unordered feature tracking made fast and easy" CVMP 2012.
//
// It tracks the position of features along the series of image from pairwise
//  correspondences.
//
// From map<[imageI,ImageJ], [indexed matches array] > it builds tracks.
//
// Usage (batch / offline):
//  TrackBuilder trackBuilder;
//  FeatureTracks tracks;
//  trackBuilder.Build(matches); // Build: Efficient fusion of correspondences
//  trackBuilder.Filter();       // Filter: Remove tracks that have conflict
//  trackBuilder.Export(tracks); // Export tree to usable data structure
//
// Usage (incremental / online — used by LocalMapper):
//  // First keyframe batch:
//  trackBuilder.Build(initial_matches);
//  trackBuilder.Filter();
//  trackBuilder.Export(tracks);
//
//  // Each subsequent batch:
//  trackBuilder.AddNewMatches(new_matches, lmdb, updated_tracks, retired_ids);
//
//  // When a keyframe is culled:
//  trackBuilder.DeleteTracksAfterCulling({culled_frame_id});

#pragma once

#include <basalt/utils/common_types.h>
#include <basalt/utils/union_find.h>
#include <basalt/vi_estimator/landmark_database.h>

#include <cassert>
#include <cstdint>
#include <functional>
#include <iterator>
#include <map>
#include <memory>
#include <set>
#include <unordered_map>
#include <utility>
#include <vector>

namespace basalt {

/// TrackBuilder creates, maintains and manages feature tracks from pairwise
/// matches.
///
/// Points to Remember:
///   - DSU array index (uint32_t): internal identifier of a node in the
///     UnionFind. Not stable across CompactTree() — callers must never
///     persist these. These are only used internally.
///   - TrackId (int64_t): externally-visible stable identifier for a track.
///     Assigned from mpNextTrackId on first sighting of a new track; the
///     value is invariant for the track's entire lifetime, even across
///     merges (the surviving track keeps its TrackId; the retired one's
///     TrackId is reported in retired_ids so the caller can remove the
///     corresponding landmark).
struct TrackBuilder {
    std::map<ImageFeaturePair, uint32_t> map_node_to_index;
    UnionFind uf_tree;

    void Build(const Matches& map_pair_wise_matches) {
        map_node_to_index.clear();
        mpUFNodeIndexToTrackIdMap.clear();
        uf_tree = UnionFind();
        // 1. We need to know how much single set we will have.
        //   i.e each set is made of a tuple : (imageIndex, featureIndex)
        // Phase 1 — collect all ImageFeaturePairs into a sorted set for
        // deterministic iteration.
        std::set<ImageFeaturePair> allFeatures;
        for (const auto& iter : map_pair_wise_matches) {
            const auto I = iter.first.first;
            const auto J = iter.first.second;
            const MatchData& matchData = iter.second;
            for (const auto& match : matchData.inliers) {
                allFeatures.emplace(I, match.first);
                allFeatures.emplace(J, match.second);
            }
        }

        // Phase 2 — assign DSU indices via AddIndex so that subsequent
        // incremental inserts (from AddNewMatches) remain consistent with the
        // same allocation strategy.
        for (const auto& feat : allFeatures) {
            map_node_to_index.emplace(feat, uf_tree.AddIndex());
        }
        allFeatures.clear();

        // Phase 3 — union matched feature pairs.
        for (const auto& iter : map_pair_wise_matches) {
            const auto I = iter.first.first;
            const auto J = iter.first.second;
            const MatchData& matchData = iter.second;
            for (const auto& match : matchData.inliers) {
                const ImageFeaturePair pairI(I, match.first);
                const ImageFeaturePair pairJ(J, match.second);
                uf_tree.Union(map_node_to_index[pairI],
                              map_node_to_index[pairJ]);
            }
        }

        // Phase 4 — assign a fresh stable TrackId to every distinct root.
        for (const auto& kv : map_node_to_index) {
            const uint32_t root = uf_tree.Find(kv.second);
            if (root != UnionFind::InvalidIndex() &&
                mpUFNodeIndexToTrackIdMap.count(root) == 0) {
                mpUFNodeIndexToTrackIdMap[root] = mpNextTrackId++;
            }
        }
    }

    /// Remove tracks that are too short or that contain two observations from
    /// the same image. Invalidates the root so subsequent Find() returns
    /// InvalidIndex via path compression; also drops the corresponding entry
    /// from mpUFNodeIndexToTrackIdMap.
    ///
    /// Note: does NOT populate `retired_ids` — this method is typically called
    /// immediately after Build (so the only tracks it removes are ones that
    /// have never been published to the caller). AddNewMatches calls Filter
    /// internally and handles TrackId retirement via its own survivor-loss
    /// heuristic before Filter runs.
    bool Filter(size_t minimumTrackLength = 2) {
        // Build per-root {TimeCamId set} to detect duplicates (same image twice
        // in one track is a conflict) and under-length tracks.
        std::map<uint32_t, std::set<TimeCamId>> tracks_by_root;
        std::set<uint32_t> problematic_roots;

        for (const auto& kv : map_node_to_index) {
            const uint32_t root = uf_tree.Find(kv.second);
            if (root == UnionFind::InvalidIndex()) continue;
            if (problematic_roots.count(root)) continue;

            const TimeCamId& tcid = kv.first.first;
            if (tracks_by_root[root].count(tcid)) {
                problematic_roots.insert(root);
            } else {
                tracks_by_root[root].insert(tcid);
            }
        }

        for (const auto& root_set : tracks_by_root) {
            if (root_set.second.size() < minimumTrackLength) {
                problematic_roots.insert(root_set.first);
            }
        }

        // Invalidate each bad root exactly once. Children of an invalidated
        // root will resolve to InvalidIndex() via path compression on next
        // Find().
        //
        // We deliberately do NOT erase the feature pairs from
        // map_node_to_index. CompactTree relies on every live feature being
        // present in this map; leaving invalidated entries in place still lets
        // CompactTree ignore them (Find returns InvalidIndex), and also
        // preserves the node count so that DSU indices allocated by AddIndex
        // remain consistent.
        for (const uint32_t bad_root : problematic_roots) {
            if (uf_tree.m_cc_parent[bad_root] != UnionFind::InvalidIndex()) {
                uf_tree.InvalidateRoot(bad_root);
                mpUFNodeIndexToTrackIdMap.erase(bad_root);
            }
        }
        return false;
    }

    /// Return the number of live tracks currently held in the TrackBuilder.
    /// A track is live iff its root is present in mpUFNodeIndexToTrackIdMap.
    size_t TrackCount() const { return mpUFNodeIndexToTrackIdMap.size(); }

    /// Export all live tracks keyed by stable TrackId.
    void Export(FeatureTracks& tracks) {
        tracks.clear();
        for (const auto& kv : map_node_to_index) {
            const uint32_t root = uf_tree.Find(kv.second);
            if (root == UnionFind::InvalidIndex()) continue;
            auto tid_it = mpUFNodeIndexToTrackIdMap.find(root);
            if (tid_it == mpUFNodeIndexToTrackIdMap.end()) continue;
            tracks[tid_it->second].emplace(kv.first);
        }
    }

    /// Incrementally incorporate a new batch of pairwise matches into the
    /// existing track set. Populates `updated_tracks` with ONLY the tracks
    /// touched by this call (for efficient downstream triangulation) and
    /// `retired_ids` with TrackIds whose tracks were merged into a survivor
    /// (the caller should remove the corresponding landmarks from its lmdb).
    void AddNewMatches(const Matches& new_matches,
                       const LandmarkDatabase<double>& lmdb,
                       FeatureTracks& updated_tracks,
                       std::set<TrackId>& retired_ids) {
        retired_ids.clear();
        updated_tracks.clear();

        // Step A — collect all image-feature pairs appearing in the new batch.
        std::set<ImageFeaturePair> new_feats;
        for (const auto& iter : new_matches) {
            const auto I = iter.first.first;
            const auto J = iter.first.second;
            const MatchData& matchData = iter.second;
            for (const auto& match : matchData.inliers) {
                new_feats.emplace(I, match.first);
                new_feats.emplace(J, match.second);
            }
        }

        // Step B — add DSU nodes for every previously-unseen feature pair.
        for (const auto& feat : new_feats) {
            if (map_node_to_index.count(feat) == 0) {
                map_node_to_index[feat] = uf_tree.AddIndex();
            }
        }

        // Step C — union matches with the survivor-preference heuristic:
        //   (1) existing landmark wins — preserves optimised geometry.
        //   (2) larger track wins       — stability under merges.
        //   (3) DSU union-by-rank       — default tie-break.
        for (const auto& iter : new_matches) {
            const auto I = iter.first.first;
            const auto J = iter.first.second;
            const MatchData& matchData = iter.second;

            for (const auto& match : matchData.inliers) {
                const uint32_t idx_i = map_node_to_index.at({I, match.first});
                const uint32_t idx_j = map_node_to_index.at({J, match.second});
                const uint32_t root_i = uf_tree.Find(idx_i);
                const uint32_t root_j = uf_tree.Find(idx_j);
                if (root_i == root_j || root_i == UnionFind::InvalidIndex() ||
                    root_j == UnionFind::InvalidIndex())
                    continue;

                const TrackId tid_i = mpUFNodeIndexToTrackIdMap.count(root_i)
                                          ? mpUFNodeIndexToTrackIdMap.at(root_i)
                                          : TrackId(-1);
                const TrackId tid_j = mpUFNodeIndexToTrackIdMap.count(root_j)
                                          ? mpUFNodeIndexToTrackIdMap.at(root_j)
                                          : TrackId(-1);
                const bool i_has_lm =
                    (tid_i >= 0 && lmdb.landmarkExists(tid_i));
                const bool j_has_lm =
                    (tid_j >= 0 && lmdb.landmarkExists(tid_j));

                const bool force_i_wins =
                    (i_has_lm && !j_has_lm) ||
                    (i_has_lm == j_has_lm &&
                     uf_tree.m_cc_size[root_i] > uf_tree.m_cc_size[root_j]);
                const bool force_j_wins =
                    (j_has_lm && !i_has_lm) ||
                    (!force_i_wins && i_has_lm == j_has_lm &&
                     uf_tree.m_cc_size[root_j] > uf_tree.m_cc_size[root_i]);

                uint32_t survivor_root;
                uint32_t retired_root;
                if (force_i_wins) {
                    uf_tree.m_cc_parent[root_j] = root_i;
                    uf_tree.m_cc_size[root_i] += uf_tree.m_cc_size[root_j];
                    survivor_root = root_i;
                    retired_root = root_j;
                } else if (force_j_wins) {
                    uf_tree.m_cc_parent[root_i] = root_j;
                    uf_tree.m_cc_size[root_j] += uf_tree.m_cc_size[root_i];
                    survivor_root = root_j;
                    retired_root = root_i;
                } else {
                    uf_tree.Union(idx_i, idx_j);
                    const uint32_t new_root = uf_tree.Find(idx_i);
                    survivor_root = new_root;
                    retired_root = (new_root == root_i) ? root_j : root_i;
                }
                (void)survivor_root;

                auto retired_it = mpUFNodeIndexToTrackIdMap.find(retired_root);
                if (retired_it != mpUFNodeIndexToTrackIdMap.end()) {
                    retired_ids.insert(retired_it->second);
                    mpUFNodeIndexToTrackIdMap.erase(retired_it);
                }
            }
        }

        // Step D — assign TrackIds to any brand-new singleton / still-unnamed
        // tracks that appeared as a result of AddIndex in Step B.
        for (const auto& kv : map_node_to_index) {
            const uint32_t root = uf_tree.Find(kv.second);
            if (root == UnionFind::InvalidIndex()) continue;
            if (mpUFNodeIndexToTrackIdMap.count(root) == 0) {
                mpUFNodeIndexToTrackIdMap[root] = mpNextTrackId++;
            }
        }

        // Step E — filter. May retire additional tracks but only ones that
        // have never been published (they wouldn't have existed before this
        // call), so retired_ids does not need to receive them.
        Filter(/*min_track_length=*/2);

        // Step F — resolve the set of TrackIds actually touched by new_matches
        // after all merges/invalidations have settled.
        std::set<TrackId> touched_tids;
        for (const auto& feat : new_feats) {
            auto map_it = map_node_to_index.find(feat);
            if (map_it == map_node_to_index.end()) continue;
            const uint32_t root = uf_tree.Find(map_it->second);
            if (root == UnionFind::InvalidIndex()) continue;
            auto tid_it = mpUFNodeIndexToTrackIdMap.find(root);
            if (tid_it != mpUFNodeIndexToTrackIdMap.end()) {
                touched_tids.insert(tid_it->second);
            }
        }

        // Step G — export only touched tracks.
        for (const auto& kv : map_node_to_index) {
            const uint32_t root = uf_tree.Find(kv.second);
            if (root == UnionFind::InvalidIndex()) continue;
            auto tid_it = mpUFNodeIndexToTrackIdMap.find(root);
            if (tid_it == mpUFNodeIndexToTrackIdMap.end()) continue;
            if (touched_tids.count(tid_it->second)) {
                updated_tracks[tid_it->second].emplace(kv.first);
            }
        }

        // Step H — compact if the proportion of ghost/invalid nodes is high.
        // Ghost: erased from map_node_to_index via RemoveObservations.
        // Invalid: root has been invalidated.
        const size_t live = CountLiveNodes();
        const size_t total = uf_tree.GetNumNodes();
        if (total > 0 && (total - live) * 2 >= total) {
            CompactTree();
        }
    }

    /// Drop all observations (ImageFeaturePairs) that belong to any of the
    /// given frame IDs. Affected TrackIds are recorded in mpTracksWithCulledObs
    /// and mpTracksWithLiveObs for DeleteTracksAfterCulling to consult.
    void RemoveObservations(const std::set<FrameId>& culled_frame_ids) {
        // 1. Identify affected feature pairs and the TrackIds they belong to.
        std::vector<ImageFeaturePair> to_erase;
        for (const auto& kv : map_node_to_index) {
            if (culled_frame_ids.count(kv.first.first.frame_id)) {
                to_erase.push_back(kv.first);
                const uint32_t root = uf_tree.Find(kv.second);
                if (root != UnionFind::InvalidIndex()) {
                    auto tid_it = mpUFNodeIndexToTrackIdMap.find(root);
                    if (tid_it != mpUFNodeIndexToTrackIdMap.end()) {
                        mpTracksWithCulledObs.insert(tid_it->second);
                    }
                }
            }
        }

        // 2. Erase from map_node_to_index. The corresponding DSU slots become
        //    "ghost" nodes that CompactTree will eventually reclaim.
        for (const auto& feat : to_erase) {
            map_node_to_index.erase(feat);
        }

        // 3. For every affected track, check whether it still has a live
        //    observation; if so, record it in mpTracksWithLiveObs.
        for (const auto& kv : map_node_to_index) {
            const uint32_t root = uf_tree.Find(kv.second);
            if (root == UnionFind::InvalidIndex()) continue;
            auto tid_it = mpUFNodeIndexToTrackIdMap.find(root);
            if (tid_it == mpUFNodeIndexToTrackIdMap.end()) continue;
            if (mpTracksWithCulledObs.count(tid_it->second)) {
                mpTracksWithLiveObs.insert(tid_it->second);
            }
        }
    }

    /// Called after a keyframe is culled. Invalidates any track that (a) lost
    /// at least one observation due to the cull and (b) no longer has any
    /// live observation remaining. The culled keyframe's observations are
    /// removed from map_node_to_index unconditionally.
    void DeleteTracksAfterCulling(const std::set<FrameId>& culled_frame_ids) {
        RemoveObservations(culled_frame_ids);

        std::set<TrackId> to_kill;
        for (const TrackId tid : mpTracksWithCulledObs) {
            if (mpTracksWithLiveObs.count(tid) == 0) {
                to_kill.insert(tid);
            }
        }

        if (!to_kill.empty()) {
            // Build reverse {TrackId -> root} exactly once (O(n)).
            std::unordered_map<TrackId, uint32_t> tid_to_root;
            tid_to_root.reserve(mpUFNodeIndexToTrackIdMap.size());
            for (const auto& kv : mpUFNodeIndexToTrackIdMap) {
                tid_to_root[kv.second] = kv.first;
            }
            for (const TrackId tid : to_kill) {
                auto it = tid_to_root.find(tid);
                if (it != tid_to_root.end()) {
                    uf_tree.InvalidateRoot(it->second);
                    mpUFNodeIndexToTrackIdMap.erase(it->second);
                }
            }
        }

        mpTracksWithCulledObs.clear();
        mpTracksWithLiveObs.clear();
    }

    /// Rebuild uf_tree so that only live nodes remain, densely indexed
    /// [0, N). Preserves set-equivalence (same TrackIds map to the same
    /// groups of ImageFeaturePairs). mpNextTrackId is NOT reset.
    void CompactTree() {
        // 1. Group live feature pairs by their current root.
        std::unordered_map<uint32_t, std::vector<ImageFeaturePair>>
            old_root_to_feats;
        std::unordered_map<uint32_t, TrackId> old_root_to_tid;
        for (const auto& kv : map_node_to_index) {
            const uint32_t root = uf_tree.Find(kv.second);
            if (root == UnionFind::InvalidIndex()) continue;
            auto tid_it = mpUFNodeIndexToTrackIdMap.find(root);
            if (tid_it == mpUFNodeIndexToTrackIdMap.end()) continue;
            old_root_to_feats[root].push_back(kv.first);
            old_root_to_tid[root] = tid_it->second;
        }

        // 2. Build a fresh UnionFind containing exactly the live nodes.
        UnionFind new_uf;
        std::map<ImageFeaturePair, uint32_t> new_map;
        std::unordered_map<uint32_t, TrackId> new_tid_map;
        for (const auto& group : old_root_to_feats) {
            const std::vector<ImageFeaturePair>& feats = group.second;
            if (feats.empty()) continue;
            const uint32_t first_new_idx = new_uf.AddIndex();
            new_map[feats[0]] = first_new_idx;
            for (size_t i = 1; i < feats.size(); ++i) {
                const uint32_t idx = new_uf.AddIndex();
                new_map[feats[i]] = idx;
                new_uf.Union(first_new_idx, idx);
            }
            const uint32_t new_root = new_uf.Find(first_new_idx);
            new_tid_map[new_root] = old_root_to_tid.at(group.first);
        }

        // 3. Swap in the compacted structures.
        uf_tree = std::move(new_uf);
        map_node_to_index = std::move(new_map);
        mpUFNodeIndexToTrackIdMap = std::move(new_tid_map);
    }

private:
    // Returns the number of entries in map_node_to_index whose Find() resolves
    // to a non-invalid root. Used by AddNewMatches to decide when to compact.
    size_t CountLiveNodes() {
        size_t count = 0;
        for (const auto& kv : map_node_to_index) {
            if (uf_tree.Find(kv.second) != UnionFind::InvalidIndex()) {
                ++count;
            }
        }
        return count;
    }

    // Maps the current root DSU index of a live track to its stable TrackId.
    // The key is volatile (may change when merges occur or CompactTree is run);
    // the value is the invariant track identifier.
    std::unordered_map<uint32_t, TrackId> mpUFNodeIndexToTrackIdMap;

    // Monotonically increasing TrackId allocator. Never reset by Build/Compact
    // because TrackIds must remain unique for the lifetime of the mapper.
    TrackId mpNextTrackId = 0;

    // Populated by RemoveObservations; consumed by DeleteTracksAfterCulling
    // to decide which tracks have lost all live observations.
    std::set<TrackId> mpTracksWithCulledObs;
    std::set<TrackId> mpTracksWithLiveObs;
};

/// Find common tracks between images.
inline bool GetTracksInImages(const std::set<TimeCamId>& image_ids,
                              const FeatureTracks& all_tracks,
                              std::vector<TrackId>& shared_track_ids) {
    shared_track_ids.clear();

    // Go along the tracks
    for (const auto& kv_track : all_tracks) {
        // Look if the track contains the provided view index & save the point
        // ids
        size_t observed_image_count = 0;
        for (const auto& imageId : image_ids) {
            if (kv_track.second.count(imageId) > 0) {
                ++observed_image_count;
            } else {
                break;
            }
        }

        if (observed_image_count == image_ids.size()) {
            shared_track_ids.push_back(kv_track.first);
        }
    }
    return !shared_track_ids.empty();
}

/// Find all tracks in an image.
inline bool GetTracksInImage(const TimeCamId& image_id,
                             const FeatureTracks& all_tracks,
                             std::vector<TrackId>& track_ids) {
    std::set<TimeCamId> image_set;
    image_set.insert(image_id);
    return GetTracksInImages(image_set, all_tracks, track_ids);
}

/// Find shared tracks between map and image
inline bool GetSharedTracks(const TimeCamId& image_id,
                            const FeatureTracks& all_tracks,
                            const Landmarks& landmarks,
                            std::vector<TrackId>& track_ids) {
    track_ids.clear();
    for (const auto& kv : landmarks) {
        const TrackId trackId = kv.first;
        if (all_tracks.at(trackId).count(image_id) > 0) {
            track_ids.push_back(trackId);
        }
    }
    return !track_ids.empty();
}

}  // namespace basalt
