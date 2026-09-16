/**
BSD 3-Clause License

This file is part of the Basalt project.
https://gitlab.com/VladyslavUsenko/basalt.git

Copyright (c) 2019, Vladyslav Usenko and Nikolaus Demmel.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <basalt/vi_estimator/landmark_database.h>

#include <algorithm>
#include <iterator>
#include <set>

namespace basalt {

template <class Scalar_>
void LandmarkDatabase<Scalar_>::addLandmark(KeypointId lm_id,
                                            const Keypoint<Scalar>& pos) {
    auto& kpt = kpts[lm_id];
    kpt.direction = pos.direction;
    kpt.inv_dist = pos.inv_dist;
    kpt.host_kf_id = pos.host_kf_id;
}

template <class Scalar_>
void LandmarkDatabase<Scalar_>::removeFrame(const FrameId& frame) {
    for (auto it = kpts.begin(); it != kpts.end();) {
        for (auto it2 = it->second.obs.begin(); it2 != it->second.obs.end();) {
            if (it2->first.frame_id == frame)
                it2 = removeLandmarkObservationHelper(it, it2);
            else
                it2++;
        }

        if (it->second.obs.size() < min_num_obs) {
            it = removeLandmarkHelper(it);
        } else {
            ++it;
        }
    }
}

template <class Scalar_>
void LandmarkDatabase<Scalar_>::removeKeyframes(
    const std::set<FrameId>& kfs_to_marg,
    const std::set<FrameId>& poses_to_marg,
    const std::set<FrameId>& states_to_marg_all) {
    for (auto it = kpts.begin(); it != kpts.end();) {
        if (kfs_to_marg.count(it->second.host_kf_id.frame_id) > 0) {
            it = removeLandmarkHelper(it);
        } else {
            for (auto it2 = it->second.obs.begin();
                 it2 != it->second.obs.end();) {
                FrameId fid = it2->first.frame_id;
                if (poses_to_marg.count(fid) > 0 ||
                    states_to_marg_all.count(fid) > 0 ||
                    kfs_to_marg.count(fid) > 0)
                    it2 = removeLandmarkObservationHelper(it, it2);
                else
                    it2++;
            }

            if (it->second.obs.size() < min_num_obs) {
                it = removeLandmarkHelper(it);
            } else {
                ++it;
            }
        }
    }
}

template <class Scalar_>
std::vector<TimeCamId> LandmarkDatabase<Scalar_>::getHostKfs() const {
    std::vector<TimeCamId> res;

    for (const auto& kv : observations) res.emplace_back(kv.first);

    return res;
}

template <class Scalar_>
std::vector<const Keypoint<Scalar_>*>
LandmarkDatabase<Scalar_>::getLandmarksForHost(const TimeCamId& tcid) const {
    std::vector<const Keypoint<Scalar>*> res;

    // A landmark appears in one row per observing keyframe, so emitting it per
    // row publishes it once per observation
    std::set<KeypointId> seen;
    for (const auto& [k, obs] : observations.at(tcid))
        for (const auto& v : obs)
            if (seen.insert(v).second) res.emplace_back(&kpts.at(v));

    return res;
}

template <class Scalar_>
void LandmarkDatabase<Scalar_>::addObservation(
    const TimeCamId& tcid_target, const KeypointObservation<Scalar>& o) {
    auto it = kpts.find(o.kpt_id);
    BASALT_ASSERT(it != kpts.end());

    // setup_opt re-adds every observation of every live
    // track on every cycle, so the covisibility hook needs the novelty that a
    // plain assignment discards. The else branch reproduces this line.
    // it->second.obs[tcid_target] = o.pos;
    const std::pair<typename Keypoint<Scalar>::MapIter, bool> ins =
        it->second.obs.emplace(tcid_target, o.pos);
    if (ins.second)
        CovisAddObserver(it->first, tcid_target);
    else
        ins.first->second = o.pos;

    observations[it->second.host_kf_id][tcid_target].insert(it->first);
}

template <class Scalar_>
Keypoint<Scalar_>& LandmarkDatabase<Scalar_>::getLandmark(KeypointId lm_id) {
    return kpts.at(lm_id);
}

template <class Scalar_>
const Keypoint<Scalar_>& LandmarkDatabase<Scalar_>::getLandmark(
    KeypointId lm_id) const {
    return kpts.at(lm_id);
}

template <class Scalar_>
const std::unordered_map<TimeCamId, std::map<TimeCamId, std::set<KeypointId>>>&
LandmarkDatabase<Scalar_>::getObservations() const {
    return observations;
}

template <class Scalar_>
const Eigen::aligned_unordered_map<KeypointId, Keypoint<Scalar_>>&
LandmarkDatabase<Scalar_>::getLandmarks() const {
    return kpts;
}

template <class Scalar_>
bool LandmarkDatabase<Scalar_>::landmarkExists(int lm_id) const {
    return kpts.count(lm_id) > 0;
}

template <class Scalar_>
size_t LandmarkDatabase<Scalar_>::numLandmarks() const {
    return kpts.size();
}

template <class Scalar_>
int LandmarkDatabase<Scalar_>::numObservations() const {
    int num_observations = 0;

    for (const auto& [_, val_map] : observations) {
        for (const auto& [_, val] : val_map) {
            num_observations += val.size();
        }
    }

    return num_observations;
}

template <class Scalar_>
int LandmarkDatabase<Scalar_>::numObservations(KeypointId lm_id) const {
    return kpts.at(lm_id).obs.size();
}

template <class Scalar_>
size_t LandmarkDatabase<Scalar_>::getObservationsCountForPair(
    const TimeCamId& tid_a, const TimeCamId& tid_b) const {
    const auto it = observations.find(tid_a);
    if (it != observations.end()) {
        const auto jt = it->second.find(tid_b);
        if (jt != it->second.end()) {
            return jt->second.size();
        }
    }
    return 0;
}

template <class Scalar_>
size_t LandmarkDatabase<Scalar_>::getNonLandmarkObservationsCountForKeyFrame(
    const TimeCamId& tid) const {
    size_t observations_count = 0;
    for (auto it = observations.begin(); it != observations.end(); ++it) {
        if (it->first == tid) continue;
        const auto jt = it->second.find(tid);
        if (jt != it->second.end()) {
            observations_count += jt->second.size();
        }
    }
    return observations_count;
}

template <class Scalar_>
typename LandmarkDatabase<Scalar_>::MapIter
LandmarkDatabase<Scalar_>::removeLandmarkHelper(
    LandmarkDatabase<Scalar>::MapIter it) {
    // Ahead of the early return below. The covisibility state is keyed on
    // frames rather than hosts, so it must be torn down on that path too.
    CovisRemoveLandmark(it->first);

    auto host_it = observations.find(it->second.host_kf_id);

    // host_it may be end() in two legitimate situations:
    //   (a) removeLandmarkObservationHelper already drained every observation
    //       for this landmark and erased the host bucket (e.g., removeFrame
    //       removes all of a landmark's observations before calling this
    //       helper);
    //   (b) addLandmark() was called but no addObservation() followed
    //       (e.g., RehostLandmark produced a landmark with no transferable
    //       obs).
    // In both cases the observations index is already clean; just erase the
    // kpts entry and return.
    if (host_it == observations.end()) {
        return kpts.erase(it);
    }

    for (const auto& [k, v] : it->second.obs) {
        auto target_it = host_it->second.find(k);
        if (target_it != host_it->second.end()) {
            target_it->second.erase(it->first);
            if (target_it->second.empty()) host_it->second.erase(target_it);
        }
    }

    if (host_it->second.empty()) observations.erase(host_it);

    return kpts.erase(it);
}

template <class Scalar_>
typename Keypoint<Scalar_>::MapIter
LandmarkDatabase<Scalar_>::removeLandmarkObservationHelper(
    LandmarkDatabase<Scalar>::MapIter it,
    typename Keypoint<Scalar>::MapIter it2) {
    CovisRemoveObserver(it->first, it2->first);

    auto host_it = observations.find(it->second.host_kf_id);
    auto target_it = host_it->second.find(it2->first);
    target_it->second.erase(it->first);

    if (target_it->second.empty()) host_it->second.erase(target_it);
    if (host_it->second.empty()) observations.erase(host_it);

    return it->second.obs.erase(it2);
}

template <class Scalar_>
void LandmarkDatabase<Scalar_>::removeLandmark(KeypointId lm_id) {
    auto it = kpts.find(lm_id);
    if (it != kpts.end()) removeLandmarkHelper(it);
}

template <class Scalar_>
void LandmarkDatabase<Scalar_>::removeObservations(
    KeypointId lm_id, const std::set<TimeCamId>& obs) {
    auto it = kpts.find(lm_id);
    BASALT_ASSERT(it != kpts.end());

    for (auto it2 = it->second.obs.begin(); it2 != it->second.obs.end();) {
        if (obs.count(it2->first) > 0) {
            it2 = removeLandmarkObservationHelper(it, it2);
        } else
            it2++;
    }

    if (it->second.obs.size() < min_num_obs) {
        removeLandmarkHelper(it);
    }
}

template <class Scalar_>
void LandmarkDatabase<Scalar_>::EnableCovisibilityTracking(bool enable) {
    mpCovisEnabled = enable;
}

template <class Scalar_>
const CovisMatrix& LandmarkDatabase<Scalar_>::GetCovisibility() const {
    return mpCovisibility;
}

template <class Scalar_>
const ObservedByFrameMap& LandmarkDatabase<Scalar_>::GetObservedByFrame()
    const {
    return mpObservedByFrame;
}

template <class Scalar_>
const ObserverFrameMap& LandmarkDatabase<Scalar_>::GetObserverFrames(
    KeypointId lm_id) const {
    static const ObserverFrameMap kEmpty;
    const ObserverIndex::const_iterator it = mpObserverFrames.find(lm_id);
    return it == mpObserverFrames.end() ? kEmpty : it->second;
}

template <class Scalar_>
void LandmarkDatabase<Scalar_>::CovisDecrementDirected(FrameId from,
                                                       FrameId to) {
    const CovisMatrix::iterator row = mpCovisibility.find(from);
    if (row == mpCovisibility.end()) return;
    const CovisRow::iterator cell = row->second.find(to);
    if (cell == row->second.end()) return;
    BASALT_ASSERT(cell->second > 0);
    if (--cell->second == 0) row->second.erase(cell);
    if (row->second.empty()) mpCovisibility.erase(row);
}

template <class Scalar_>
void LandmarkDatabase<Scalar_>::CovisDecrementPair(FrameId a, FrameId b) {
    CovisDecrementDirected(a, b);
    CovisDecrementDirected(b, a);
}

// A stereo pair contributes one observer frame and not two, so the covisibility
// counters move only on the zero to one transition of the camera count.
template <class Scalar_>
void LandmarkDatabase<Scalar_>::CovisAddObserver(KeypointId lm_id,
                                                 const TimeCamId& tcid) {
    if (!mpCovisEnabled) return;
    ObserverFrameMap& frames = mpObserverFrames[lm_id];
    if (++frames[tcid.frame_id] > 1) return;

    // The size guard is not an optimisation. operator[] materialises the row,
    // so on a landmark's first observation it would leave an empty row that a
    // rebuild from lmdb never produces.
    if (frames.size() > 1) {
        CovisRow& row = mpCovisibility[tcid.frame_id];
        for (ObserverFrameMap::const_iterator f = frames.begin();
             f != frames.end(); ++f) {
            if (f->first == tcid.frame_id) continue;
            ++row[f->first];
            ++mpCovisibility[f->first][tcid.frame_id];
        }
    }
    mpObservedByFrame[tcid.frame_id].insert(lm_id);
}

template <class Scalar_>
void LandmarkDatabase<Scalar_>::CovisRemoveObserver(KeypointId lm_id,
                                                    const TimeCamId& tcid) {
    if (!mpCovisEnabled) return;
    const ObserverIndex::iterator lm_it = mpObserverFrames.find(lm_id);
    if (lm_it == mpObserverFrames.end()) return;
    ObserverFrameMap& frames = lm_it->second;
    const ObserverFrameMap::iterator f_it = frames.find(tcid.frame_id);
    if (f_it == frames.end()) return;
    BASALT_ASSERT(f_it->second > 0);
    if (--f_it->second > 0) return;
    frames.erase(f_it);

    for (ObserverFrameMap::const_iterator f = frames.begin(); f != frames.end();
         ++f)
        CovisDecrementPair(tcid.frame_id, f->first);

    const ObservedByFrameMap::iterator obf =
        mpObservedByFrame.find(tcid.frame_id);
    if (obf != mpObservedByFrame.end()) {
        obf->second.erase(lm_id);
        if (obf->second.empty()) mpObservedByFrame.erase(obf);
    }
    if (frames.empty()) mpObserverFrames.erase(lm_it);
}

// Driven off mpObserverFrames rather than Keypoint::obs, so it stays correct
// where the host-keyed observations index has gone stale.
template <class Scalar_>
void LandmarkDatabase<Scalar_>::CovisRemoveLandmark(KeypointId lm_id) {
    if (!mpCovisEnabled) return;
    const ObserverIndex::iterator lm_it = mpObserverFrames.find(lm_id);
    if (lm_it == mpObserverFrames.end()) return;

    const ObserverFrameMap& frames = lm_it->second;
    for (ObserverFrameMap::const_iterator it = frames.begin();
         it != frames.end(); ++it) {
        for (ObserverFrameMap::const_iterator jt = std::next(it);
             jt != frames.end(); ++jt)
            CovisDecrementPair(it->first, jt->first);
        const ObservedByFrameMap::iterator obf =
            mpObservedByFrame.find(it->first);
        if (obf != mpObservedByFrame.end()) {
            obf->second.erase(lm_id);
            if (obf->second.empty()) mpObservedByFrame.erase(obf);
        }
    }
    mpObserverFrames.erase(lm_it);
}

// //////////////////////////////////////////////////////////////////
// instatiate templates

// Note: double specialization is unconditional, b/c NfrMapper depends on it.
// #ifdef BASALT_INSTANTIATIONS_DOUBLE
template class LandmarkDatabase<double>;
// #endif

#ifdef BASALT_INSTANTIATIONS_FLOAT
template class LandmarkDatabase<float>;
#endif

}  // namespace basalt
