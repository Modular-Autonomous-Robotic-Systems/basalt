# Keyframe Selection Driven Local Mapping

## 1. Introduction

The local mapping stage of this Basalt integration exists to serve three purposes. It refines the visual-inertial estimate over a window larger than the VIO sliding window and feeds corrected keyframe poses back to the estimator. It publishes a metric point cloud and keyframe graph for downstream planning and perception. It stands in for a LiDAR as a low cost source of local structure. Each of these consumers reads the map while the vehicle is flying, so the map must exist as early as possible and must track keyframe creation rather than lag behind it.

The present implementation does not meet that requirement, because the local mapper is driven by marginalisation. A keyframe becomes visible to the mapper only when the VIO sliding window overflows and ejects some other keyframe, so the map does not exist at all until `vio_max_kfs` keyframes have been selected, and its cadence is dictated by a memory management policy inside the estimator rather than by the map's own needs. The change described here moves the driver to keyframe selection. A dedicated queue carries every selected keyframe, with its pose and its images, from the VIO thread to the mapping thread the moment the keyframe is created, and marginalisation is demoted to a secondary, non-blocking input whose sole remaining role is to supply refined pose estimates and nonlinear factors.

The impact is threefold. The map is built from the first keyframe onward and no longer depends on a marginalisation event ever occurring. Images reach the mapper along exactly one path, which removes the current duplication in which every marginalisation packet re-ships the images of every keyframe in the VIO window. Keyframe admission into the local map becomes an explicit act performed once per keyframe, which closes a defect whereby a keyframe culled by the mapper is silently resurrected by the next marginalisation packet.

## 2. Current Implementation

### 2.1 Pipeline

```
 ┌─────────────────────────────────────────────────────────────────────────────┐
 │ VIO thread — SqrtKeypointVioEstimator::measure()   src/vi_estimator/         │
 │                                                    sqrt_keypoint_vio.cpp     │
 └─────────────────────────────────────────────────────────────────────────────┘
  images ─► OpticalFlow ─► measure(opt_flow_meas, imu_meas)
                              │
                              ├─ prev_opt_flow_res[t] = opt_flow_meas          :446
                              ├─ landmark association, connectivity ratio
                              ├─ take_kf = true                                :483-486
                              ├─ if (take_kf) { kf_ids.emplace(t); ... }       :493-498
                              │      ▲ keyframe is born here and NOTHING
                              │        is emitted to the local mapper
                              └─ optimize_and_marg()                           :601, :1591
                                    ├─ optimize()
                                    └─ marginalize()
                                          ├─ kfs_to_marg selected only when
                                          │  kf_ids.size() > vio_max_kfs (=7)  :735
                                          └─ if (out_marg_queue && !kfs_to_marg.empty())
                                                MargData m {                    :887-916
                                                   abs_H, abs_b, aom,
                                                   frame_poses, frame_states,
                                                   kfs_all, kfs_to_marg,
                                                   opt_flow_res[ every kfs_all ] }
                                                out_marg_queue->push(m)   BLOCKING
                                                          │
   ═══════════════════════════════════════════════════════▼══════════════════════
        local_map_input_queue_   tbb::concurrent_bounded_queue   capacity 10
                                 src/controller.cpp:172-173
   ═══════════════════════════════════════════════════════╤══════════════════════
                                                          │
 ┌────────────────────────────────────────────────────────▼────────────────────┐
 │ LocalMapper thread — MapLocally()   src/vi_estimator/local_mapper.cpp:66     │
 └─────────────────────────────────────────────────────────────────────────────┘
      mpMargInputQueue->pop(data)   BLOCKING, sole wake source          :82
          │
          ├─ try_pop drain of further packets                           :89-96
          ├─ for each packet  IngestMargData(packet)                    :302-346
          │      ├─ new KF set  =  kfs_all \ frame_poses
          │      ├─ processMargData()      Schur complement,
          │      │                         img_data ← opt_flow_res      nfr_mapper.cpp:150
          │      ├─ extractNonlinearFactors()   RelPose + RollPitch
          │      └─ frame_poses ← data->frame_poses     INSERTS
          ├─ img_data filtered down to new keyframes                    :111-117
          ├─ detect_keypoints → match_stereo → MatchLocal → build_tracks → setup_opt
          ├─ CullRedundantKeyframes    erases frame_poses entries       :819
          ├─ optimize → filterOutliers → optimize
          ├─ out_vis_queue->try_push(snapshot)                          :250-262
          └─ mpVioPoseUpdateCallback(frame_poses) ──► VIO mpPosesToUpdate
```

### 2.2 Discussion

A `MargData` packet is assembled inside `marginalize` and pushed at `src/vi_estimator/sqrt_keypoint_vio.cpp:915`. It carries the dense information matrix of the whole marginalisation sub-problem, the estimator's `frame_poses` and `frame_states` maps, the keyframe identity sets `kfs_all` and `kfs_to_marg`, and one `OpticalFlowResult::Ptr` for every keyframe in `kfs_all`. The mapper consumes it in `IngestMargData`, where `processMargData` applies the Schur complement that strips velocity and bias, promotes keyframe states into poses, and fills `img_data` from `opt_flow_res`, and where `extractNonlinearFactors` inverts the information matrix to produce one `RollPitchFactor` for the marginalised keyframe and one `RelPoseFactor` from it to every other keyframe in the window.

The push at `:915` is gated on `!kfs_to_marg.empty()`, and `kfs_to_marg` is filled only inside the loop at `:735` whose condition is `kf_ids.size() > max_kfs`. Marginalisation of states therefore runs on nearly every frame, since `vio_max_states` is three, but a packet is emitted only when a keyframe is actually ejected from the estimator's keyframe set. Since `kf_ids` grows only when a keyframe is selected, at most one packet is emitted between two consecutive keyframe selections, and none at all until the eighth keyframe.

### 2.3 Drawbacks

The map does not exist during the warm-up. With `vio_max_kfs` at seven, no packet is produced for the first seven keyframes, so `frame_poses`, `lmdb` and the visualisation snapshot are empty throughout that interval. A consumer that needs local structure at take-off receives nothing.

The map's existence is conditional on an estimator policy that has nothing to do with mapping. An estimator configured with a large keyframe budget, as `src/mapper_sim_naive.cpp:758` does with `setMaxKfs(10000)`, never marginalises a keyframe and therefore never produces a packet, and the local mapper would idle indefinitely.

Image delivery is duplicated and wasteful. Every packet re-ships an `OpticalFlowResult::Ptr` for every keyframe in `kfs_all`, the mapper writes all of them into `img_data` inside `processMargData`, and `MapLocally` at `:111-117` then erases everything that is not a new keyframe. The images survive only because `prev_opt_flow_res` in the estimator holds the same shared pointers, so the waste is in bookkeeping rather than in memory, but the path is redundant and it couples image arrival to marginalisation.

Culled keyframes are resurrected. `IngestMargData` updates `frame_poses` with `frame_poses[kv.first] = p`, which inserts when the key is absent. `CullRedundantKeyframes` erases entries from `frame_poses` while the same keyframe is usually still inside the estimator's `kfs_all`, so the next packet re-inserts it. The resurrected entry carries a pose and nothing else, because its images, corners, landmarks, tracks and bag-of-words entries were destroyed by the cull. It then enters the bundle adjustment through the `aom` built at `src/vi_estimator/nfr_mapper.cpp:255`, contributes no residual, is skipped by the redundancy criterion in `SelectKeyframesToCull` because its observation count is zero, and inflates the keyframe list published to the visualiser. The cull is then repeated on the following cycle, producing a thrash loop.

One ingestion loop is dead. After `processMargData` returns, `m.frame_states` is empty, because every entry of `frame_states` appears in `aom` with `POSE_VEL_BIAS_SIZE` and is either moved into `m.frame_poses` when it belongs to `kfs_all` or erased outright when it does not, at `src/vi_estimator/nfr_mapper.cpp:112-119`. The third loop of `IngestMargData`, which iterates `data->frame_states`, therefore never executes a single iteration. The same dead loop exists in `NfrMapper::addMargData`.

The blocking push is the documented shutdown hazard. `BUG.md` records the deadlock in which `Controller::Stop` hangs joining the mapping thread while that thread sits in `mpMargInputQueue->pop`, and the capacity of ten on `local_map_input_queue_` means a slow mapper stalls the estimator through backpressure.

A correction to the framing that motivated this work is warranted here. It is not the case that non-keyframe poses enter the local map. `frame_poses` inside the estimator is an invariant subset of `kf_ids`, because the only writer is the `states_to_marg_vel_bias` promotion at `src/vi_estimator/sqrt_keypoint_vio.cpp:1053-1060` which handles keyframes only, and because every entry not in `kf_ids` is placed into `poses_to_marg` at `:698` and erased in the same call at `:1062`. Every identity reaching the mapper is consequently a keyframe already. The defect that does exist, and that the keyframe filter is nonetheless required to fix, is the unconditional insertion described above, which readmits keyframes the mapper has deliberately discarded.

## 3. Implementation Plan

### 3.1 Updated pipeline

```
 ┌─────────────────────────────────────────────────────────────────────────────┐
 │ VIO thread — SqrtKeypointVioEstimator::measure()                            │
 └─────────────────────────────────────────────────────────────────────────────┘
  images ─► OpticalFlow ─► measure(opt_flow_meas, imu_meas)
                              │
                              ├─ prev_opt_flow_res[t] = opt_flow_meas
                              ├─ take_kf = true
                              ├─ if (take_kf) { kf_ids.emplace(t);
                              │                 mpIsCurrentFrameKF = true; }   NEW
                              └─ optimize_and_marg()
                                    ├─ optimize()
                                    ├─ marginalize()
                                    │     └─ if (!kfs_to_marg.empty())
                                    │           out_marg_queue->push(m)   ── pose refinement only
                                    │                        │
                                    └─ PublishKeyframe()                       NEW
                                          if (mpIsCurrentFrameKF)
                                             Keyframe k {                      NEW struct
                                                timestamp    = last_state_t_ns,
                                                pose         = optimised T_w_i,
                                                opt_flow_res = prev_opt_flow_res[t] }
                                             mpKFOutputQueue->push(k)
                                             mpIsCurrentFrameKF = false
                                                       │           │
   ════════════════════════════════════════════════════│═══════════▼═══════════
     local_map_input_queue_  cap 10   ◄────────────────┘   local_map_kf_queue_
     MargData, drained non-blocking                        Keyframe, cap 100
   ════════════════════════════════╤══════════════════════════════╤════════════
                                   │                              │
 ┌─────────────────────────────────▼──────────────────────────────▼───────────┐
 │ LocalMapper thread — MapLocally()                                          │
 └────────────────────────────────────────────────────────────────────────────┘
      mpKFInputQueue->pop(kf)   BLOCKING, sole wake source           CHANGED
          │
          ├─ try_pop drain of further keyframes
          ├─ for each keyframe  IngestKeyframe(kf)                   NEW
          │      ├─ frame_poses[t] = pose          insert, once per keyframe
          │      ├─ img_data[t]    = opt_flow_res->input_images   sole image path
          │      └─ mpNewKeyframesForTracking.insert(t)
          │
          ├─ while (mpMargInputQueue->try_pop(data))   NON-BLOCKING     CHANGED
          ├─ for each packet  IngestMargData(packet)                    CHANGED
          │      ├─ data->opt_flow_res.clear()      image path severed
          │      ├─ processMargData()               Schur complement
          │      ├─ extractNonlinearFactors()       RelPose + RollPitch
          │      ├─ refresh frame_poses for kfs_all ∩ frame_poses   NEVER INSERTS
          │      └─ prune factors referencing unknown keyframes      NEW
          │
          ├─ detect_keypoints → match_stereo → MatchLocal → build_tracks → setup_opt
          ├─ CullRedundantKeyframes        culled keyframes stay culled
          ├─ optimize → filterOutliers → optimize
          ├─ out_vis_queue->try_push(snapshot)
          └─ mpVioPoseUpdateCallback(frame_poses) ──► VIO mpPosesToUpdate
```

### 3.2 Why this pipeline is better

Mapping begins at the first keyframe, because the trigger is keyframe selection and not sliding window overflow. The warm-up gap of `vio_max_kfs` keyframes disappears, and an estimator configured never to marginalise still produces a map.

The two inputs are separated by concern. The keyframe queue carries identity, pose and images, which is everything needed to admit a keyframe into the map. The marginalisation queue carries refined poses and nonlinear factors, which is everything needed to improve a keyframe already admitted. Neither input can now do the other's job, so the ambiguity that produced the resurrection defect is removed by construction rather than by a filter.

The ordering of the two queues is safe without further synchronisation, and the pipeline cannot deadlock. This point requires proof rather than assertion, because the mapper no longer blocks on the marginalisation queue, so an estimator blocked on a full marginalisation queue while the mapper waits on an empty keyframe queue would hang the whole pipeline.

The proof rests on the emission bound. A packet is pushed only when `kfs_to_marg` is non-empty, and `kfs_to_marg` is filled only inside the loop at `sqrt_keypoint_vio.cpp:735` whose guard is `kf_ids.size() > max_kfs`. Every iteration of that loop erases one identity from `kf_ids` at `:807`, and `kf_ids` grows only in the keyframe commit block at `:498`. The number of packets the estimator can emit without an intervening keyframe selection is therefore bounded by the excess of `kf_ids` over `max_kfs` at that moment, which is at most one in steady state since the set grows one identity at a time. Deadlock would require the marginalisation queue to reach its capacity of ten while the keyframe queue is empty, which the bound forbids.

The ordering within a cycle follows from the call sequence. `marginalize` pushes its packet before `PublishKeyframe` pushes the keyframe, the mapper wakes on that keyframe, admits it, and only then drains the marginalisation queue, so the packet describing a keyframe is always consumed after that keyframe has been admitted.

The mapper still amortises. The blocking pop followed by a non-blocking drain is retained on the keyframe queue, so a mapper that falls behind processes a batch of keyframes in one visual pipeline pass, which is the behaviour established in `local_mapper_optimisation.md` and already implemented.

### 3.3 Changes

#### 3.3.1 `struct Keyframe` in `include/basalt/utils/imu_types.h`

Add immediately after `struct MargData`.

```cpp
struct Keyframe {
    typedef std::shared_ptr<Keyframe> Ptr;

    int64_t timestamp;
    PoseStateWithLin<double> pose;
    OpticalFlowResult::Ptr opt_flow_res;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
```

Rationale. The three fields are the complete admission record for a keyframe. The timestamp is the map key used throughout `NfrMapper`. The pose seeds `frame_poses` and therefore the triangulation baselines in `setup_opt`. The optical flow result carries `input_images`, which is the only thing `detect_keypoints` consumes, and it is a shared pointer already held by the estimator's `prev_opt_flow_res`, so passing it costs no image copy.

`PoseStateWithLin<double>` embeds a `Sophus::SE3d` by value, whose `Quaterniond` is over-aligned under the `EIGEN_MAX_ALIGN_BYTES` pin asserted at `src/controller.cpp:47-52`. `EIGEN_MAKE_ALIGNED_OPERATOR_NEW` is therefore mandatory, and the struct must be allocated with plain `new` so that the class operator new is used, matching `MargData::Ptr m(new MargData)` at `sqrt_keypoint_vio.cpp:891` and `VioVisualizationData::Ptr data(new VioVisualizationData)` at `:612`. `std::make_shared` must not be used, since it allocates through the allocator and bypasses the class operator new. Where a shared allocation is preferred, `std::allocate_shared<Keyframe>(Eigen::aligned_allocator<Keyframe>{})` is the correct form, which is the idiom already used for `MatchData` at `local_mapper.cpp:451`.

Velocity and bias are deliberately excluded. The local map optimises pose and structure only, its `frame_poses` is typed `PoseStateWithLin<double>`, and carrying a `PoseVelBiasStateWithLin` would add state the mapper cannot use and would invite a future reader to believe the mapper performs inertial refinement.

Backwards compatibility. A new struct in a header that is already included everywhere `MargData` is visible. No existing type, signature or serialisation is touched. `Keyframe` is deliberately not registered with cereal, because nothing persists it.

#### 3.3.2 Keyframe output queue in `include/basalt/vi_estimator/vio_estimator.h`

Add alongside the existing output queue pointers in `class VioEstimatorBase`.

```cpp
    tbb::concurrent_bounded_queue<MargData::Ptr> *out_marg_queue = nullptr;
    tbb::concurrent_bounded_queue<Keyframe::Ptr> *mpKFOutputQueue = nullptr;
```

Rationale, and a departure from the task as stated. The task placed this pointer in `SqrtKeypointVioEstimator`. That will not compile at the wiring site, because `Controller` holds `basalt::VioEstimatorBase<double>::Ptr` at `include/basalt/controller.h:76` and reaches `out_marg_queue` through the base class at `src/controller.cpp:173`. Placing the keyframe queue in the derived class would force a `dynamic_pointer_cast` in the controller and would silently disable local mapping whenever the estimator is the visual-only variant. The base class placement mirrors the `out_marg_queue` contract exactly, defaults to `nullptr`, and is therefore inert for every estimator that does not set it.

Backwards compatibility. A new member with a null default in a base class that is constructed only through `VioEstimatorFactory`. No existing consumer of `VioEstimatorBase` observes any change.

#### 3.3.3 Keyframe selection flag and publication in the estimator

In `include/basalt/vi_estimator/sqrt_keypoint_vio.h`, add a private member beside `take_kf`, and declare the publication helper next to `optimize_and_marg`.

```cpp
    void PublishKeyframe();
...
    bool take_kf;
    bool mpIsCurrentFrameKF = false;
```

In `src/vi_estimator/sqrt_keypoint_vio.cpp`, raise the flag where the keyframe is committed rather than where the threshold fires.

```cpp
    if (take_kf) {
        // Triangulate new points from one of the observations (with sufficient
        // baseline) and make keyframe for camera 0
        take_kf = false;
        mpIsCurrentFrameKF = true;
        frames_after_kf = 0;
        kf_ids.emplace(last_state_t_ns);
```

Rationale, and a second departure from the task as stated. The task set the flag at the threshold test at `:483-486`. That site is not the only writer of `take_kf`. The constructor initialises `take_kf(true)` at `:62`, which is what makes the very first frame a keyframe, and that path never passes through the threshold test. Raising the flag inside the `if (take_kf)` block at `:493` captures both writers, is the single point at which `kf_ids` actually grows, and is the only site that cannot drift out of agreement with the estimator's own notion of a keyframe.

Publish after optimisation and marginalisation.

```cpp
template <class Scalar_>
void SqrtKeypointVioEstimator<Scalar_>::optimize_and_marg(
    const std::map<int64_t, int>& num_points_connected,
    const std::unordered_set<KeypointId>& lost_landmaks) {
    optimize();
    marginalize(num_points_connected, lost_landmaks);
    PublishKeyframe();
}

template <class Scalar_>
void SqrtKeypointVioEstimator<Scalar_>::PublishKeyframe() {
    if (!mpIsCurrentFrameKF) return;
    mpIsCurrentFrameKF = false;

    if (!this->mpKFOutputQueue) return;

    const auto it_state = frame_states.find(last_state_t_ns);
    const auto it_flow = prev_opt_flow_res.find(last_state_t_ns);
    if (it_state == frame_states.end() || it_flow == prev_opt_flow_res.end())
        return;

    Keyframe::Ptr kf(new Keyframe);
    kf->timestamp = last_state_t_ns;
    kf->pose = PoseStateWithLin<double>(
        last_state_t_ns,
        it_state->second.getState().T_w_i.template cast<double>());
    kf->opt_flow_res = it_flow->second;

    this->mpKFOutputQueue->push(kf);
}
```

Rationale. Publishing after `marginalize` means the pose shipped is the one produced by the joint optimisation of the current window rather than the inertial prediction available at selection time, which is the best estimate the estimator will ever hold for a keyframe at the moment of its creation. The flag is cleared before the queue check, so an unwired queue cannot latch it and cause the next keyframe to be published twice.

Two lookups are guarded rather than asserted. `last_state_t_ns` is in fact always present in both maps after marginalisation, because `states_to_marg_all` and `poses_to_marg` are drawn from entries strictly older than `last_state_to_marg` at `:684-688`, and the newest state is never among them. The guard costs nothing and keeps the estimator from aborting should that invariant be altered later.

The pose is reconstructed through the two argument `PoseStateWithLin` constructor, whose `linearized` parameter defaults to false. This is required and not incidental. `NfrMapper::optimize` asserts `BASALT_ASSERT(!kv.second.isLinearized())` before applying an increment, at `src/vi_estimator/nfr_mapper.cpp:333` and `:414`. The estimator marks marginalised states linearised through `setLinTrue` at `sqrt_keypoint_vio.cpp:1044`, so a state copied verbatim from the estimator would abort the mapper's bundle adjustment.

The push is blocking, matching `out_marg_queue`. Dropping a keyframe silently would leave the map with a hole that no later input repairs, since the marginalisation path is now forbidden to insert. With the queue capacity set to one hundred in the controller, blocking requires the mapper to fall one hundred keyframes behind, which is a failure worth surfacing as backpressure rather than concealing as silent loss.

Backwards compatibility. `optimize_and_marg` gains one call whose body returns immediately when the flag is clear or the queue is unwired, so `src/vio.cpp`, `src/rs_t265_vio.cpp`, `src/vio_sim.cpp` and the `basalt_mapper` tools, none of which set `mpKFOutputQueue`, are unaffected.

#### 3.3.4 Shutdown sentinels

The mapping thread's blocking wait moves from the marginalisation queue to the keyframe queue, so the shutdown sentinel must move with it or `Controller::Stop` will hang exactly as recorded in `BUG.md`. Add the keyframe sentinel at both existing sentinel sites in `src/vi_estimator/sqrt_keypoint_vio.cpp`, at `:184-186` in the producer consumer loop and at `:209-211` in the null frame branch of `ProcessFrame`.

```cpp
            if (this->out_vis_queue) this->out_vis_queue->push(nullptr);
            if (this->out_marg_queue) this->out_marg_queue->push(nullptr);
            if (this->mpKFOutputQueue) this->mpKFOutputQueue->push(nullptr);
            if (this->out_state_queue) this->out_state_queue->push(nullptr);
```

Backwards compatibility. Guarded on a null pointer, so no existing consumer sees an extra push.

#### 3.3.5 The visual-only estimator

`Controller` constructs a `LocalMapper` for both members of `enum class SlamMode`, and selects `SqrtKeypointVoEstimator` when the mode is `VO` through `use_imu` at `src/controller.cpp:156`. That estimator has the identical structure, with `take_kf(true)` in its constructor at `sqrt_keypoint_vo.cpp:63`, the threshold at `:319`, the commit block at `:329-338`, `optimize_and_marg` at `:1389` and the marginalisation push at `:804`. Left untouched, the visual-only mode would lose local mapping entirely and would hang at shutdown, which is a regression for a caller that did not ask for this change. The same four edits are therefore mirrored into `src/vi_estimator/sqrt_keypoint_vo.cpp` and `include/basalt/vi_estimator/sqrt_keypoint_vo.h`, namely the `mpIsCurrentFrameKF` member, the flag being raised in the commit block, `PublishKeyframe` called from `optimize_and_marg`, and the sentinel added beside the existing one at `:151-152`.

One difference emerged during implementation. The visual-only `ProcessFrame` had no sentinel branch at all, returning `nullptr` on a null frame without pushing to any output queue, whereas the inertial one pushes to all of them at `sqrt_keypoint_vio.cpp:209-213`. In the event-driven model `proc_func` never runs, so visual-only mode emitted no sentinel on any queue and `LocalMapper::Stop` would already have hung on the marginalisation queue today. The branch was therefore given the full sentinel cascade and `finished = true`, mirroring the inertial estimator exactly. This repairs a pre-existing defect rather than merely preserving behaviour.

The remaining difference is benign. The visual-only estimator has no inertial state, but its `frame_states` entries are still `PoseVelBiasStateWithLin` and the same accessor applies, so `PublishKeyframe` is identical in both.

#### 3.3.6 `LocalMapper` queue and ingestion, `include/basalt/vi_estimator/local_mapper.h`

```cpp
    void SetMarginalisationDataInputQueue(
        tbb::concurrent_bounded_queue<MargData::Ptr>* queue);
    void SetKFInputQueue(tbb::concurrent_bounded_queue<Keyframe::Ptr>* queue);
...
    void IngestMargData(MargData::Ptr& data);
    void IngestKeyframe(Keyframe::Ptr& kf);
...
    std::atomic<bool> mpIsMargDataInputQueueSet{false};
    std::atomic<bool> mpIsKFInputQueueSet{false};
...
private:
    tbb::concurrent_bounded_queue<MargData::Ptr>* mpMargInputQueue = nullptr;
    tbb::concurrent_bounded_queue<Keyframe::Ptr>* mpKFInputQueue = nullptr;
...
    void PruneFactorsWithUnknownKeyframes(size_t relBegin, size_t rpBegin);
```

`IngestKeyframe` is public for the same reason `IngestMargData` is, namely testability without a live estimator.

#### 3.3.7 `LocalMapper` implementation, `src/vi_estimator/local_mapper.cpp`

The setter mirrors the existing one.

```cpp
void LocalMapper::SetKFInputQueue(
    tbb::concurrent_bounded_queue<Keyframe::Ptr>* queue) {
    mpKFInputQueue = queue;
    mpIsKFInputQueueSet = true;
}
```

The startup gate in `MapLocally` waits on the queue that now drives the loop.

```cpp
    while (!mpStopLocalMapping && !mpIsKFInputQueueSet) {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
```

The head of the loop is replaced. The blocking pop moves to the keyframe queue, the marginalisation queue is drained non-blockingly after the keyframes have been admitted, and both queues can deliver the shutdown sentinel.

```cpp
        std::vector<Keyframe::Ptr> vKeyframes;
        bool nullReceived = false;
        Keyframe::Ptr kf;

        mpKFInputQueue->pop(kf);  // blocking — sleeps until VIO selects a KF
        if (!kf) {
            std::cout << "[Local Mapper] got shutdown sentinel" << std::endl;
            break;
        }
        vKeyframes.push_back(kf);

        while (mpKFInputQueue->try_pop(kf)) {
            if (!kf) {
                nullReceived = true;
                break;
            }
            vKeyframes.push_back(kf);
        }

        mpNewKeyframesForTracking.clear();
        mpLatestKeyframesMatches.clear();

        for (Keyframe::Ptr& k : vKeyframes) IngestKeyframe(k);
        vKeyframes.clear();

        // Marginalisation is now a refinement input. Every packet available at
        // this instant is consumed, and an empty queue is not a reason to wait.
        std::vector<MargData::Ptr> vecData;
        MargData::Ptr data;
        while (mpMargInputQueue && mpMargInputQueue->try_pop(data)) {
            if (!data) {
                nullReceived = true;
                break;
            }
            vecData.push_back(data);
        }
        for (MargData::Ptr& packet : vecData) IngestMargData(packet);
        vecData.clear();
```

Everything below this point in `MapLocally` is unchanged, including the `img_data` filter at `:112-118`, which becomes redundant but is retained as a guard.

One guard below that point did have to change. The early exit read `if (mpNewKeyframesForTracking.empty()) continue;`, which skips the `if (nullReceived) break;` at the foot of the loop and returns to the blocking pop. Under the old driver that path was reachable only when a packet contributed no new keyframe. Under the new one the shutdown sentinel can arrive from either queue while the batch yields no admissible keyframe, so the exit must consume it.

```cpp
        // A sentinel drained alongside real data must still terminate the
        // thread. Skipping the visual pipeline must not skip that check, or
        // the next blocking pop never returns.
        if (mpNewKeyframesForTracking.empty()) {
            if (nullReceived) break;
            continue;
        }
```

Keyframe admission is the new function.

```cpp
void LocalMapper::IngestKeyframe(Keyframe::Ptr& kf) {
    if (!kf->opt_flow_res || !kf->opt_flow_res->input_images) return;

    const int64_t t_ns = kf->timestamp;
    if (frame_poses.count(t_ns) > 0) return;

    // Rebuilt rather than copied so that linearized is false, which
    // NfrMapper::optimize asserts before applying an increment.
    frame_poses[t_ns] = PoseStateWithLin<double>(t_ns, kf->pose.getPose());
    img_data[t_ns] = kf->opt_flow_res->input_images;
    mpNewKeyframesForTracking.emplace(t_ns);
}
```

A keyframe without images is rejected outright rather than admitted pose-only, because a pose with no corners, no landmarks and no bag-of-words entry is precisely the zombie state this change exists to eliminate. The `frame_poses.count` guard makes the function idempotent and prevents a keyframe from being readmitted after the mapper has culled it, should the estimator ever republish an identity.

Marginalisation ingestion is reduced to refinement.

```cpp
void LocalMapper::IngestMargData(MargData::Ptr& data) {
    // Images arrive with the keyframe. Dropping the packet's copy makes
    // NfrMapper::processMargData's img_data write inert without altering the
    // shared base class, which the offline mapper still depends on.
    data->opt_flow_res.clear();

    const size_t relBegin = rel_pose_factors.size();
    const size_t rpBegin = roll_pitch_factors.size();

    processMargData(*data);
    const bool valid = extractNonlinearFactors(*data);
    if (!valid) return;

    // Refresh only keyframes the mapper already holds. Insertion is
    // deliberately absent, so a keyframe removed by CullRedundantKeyframes is
    // never resurrected by a later marginalisation packet.
    for (const auto& kv : data->frame_poses) {
        if (data->kfs_all.count(kv.first) == 0) continue;
        auto it = frame_poses.find(kv.first);
        if (it == frame_poses.end()) continue;
        it->second =
            PoseStateWithLin<double>(kv.second.getT_ns(), kv.second.getPose());
    }

    PruneFactorsWithUnknownKeyframes(relBegin, rpBegin);
}
```

Three deletions are recorded here. The first loop of the old body, which derived `mpNewKeyframesForTracking` from `kfs_all`, is removed because keyframe admission now belongs solely to `IngestKeyframe`. The third loop, over `data->frame_states`, is removed because it was already unreachable, `m.frame_states` being emptied by `processMargData` as established in section 2.3. The unconditional insertion is replaced by the `find` based refresh above. Per the workspace convention on superseded decisions, the three blocks are commented out in place beside the replacement with a note naming the reason and the date, and the user removes them at commit time.

The `kfs_all` membership test is retained even though `frame_poses` inside the estimator is provably a subset of `kf_ids`. It costs one set lookup, it states the intended contract at the point of use rather than in a comment, and it holds the invariant if the estimator's pose bookkeeping is ever changed.

Factor pruning is the new guard.

```cpp
void LocalMapper::PruneFactorsWithUnknownKeyframes(size_t relBegin,
                                                   size_t rpBegin) {
    auto known = [this](int64_t t) { return frame_poses.count(t) > 0; };

    rel_pose_factors.erase(
        std::remove_if(rel_pose_factors.begin() + relBegin,
                       rel_pose_factors.end(),
                       [&](const RelPoseFactor& f) {
                           return !known(f.t_i_ns) || !known(f.t_j_ns);
                       }),
        rel_pose_factors.end());

    roll_pitch_factors.erase(
        std::remove_if(roll_pitch_factors.begin() + rpBegin,
                       roll_pitch_factors.end(),
                       [&](const RollPitchFactor& f) { return !known(f.t_ns); }),
        roll_pitch_factors.end());
}
```

This guard is mandatory, not defensive. `extractNonlinearFactors` emits one `RollPitchFactor` for the marginalised keyframe and one `RelPoseFactor` from it to every other member of `kfs_all`, drawn from the packet's own maps and therefore indifferent to what the mapper holds. Those factors are consumed with `frame_poses.at(...)` in `MapperLinearizeAbsReduce::operator()` at `src/vi_estimator/nfr_mapper.h:114` and `:135`, and again in `computeRelPose` at `nfr_mapper.cpp:456` and `computeRollPitch` at `:469`, all of which throw `std::out_of_range` on a missing key. `config.mapper_use_factors` defaults to true at `src/utils/vio_config.cpp:105`, so this path is live. Until now the unconditional insertion in `IngestMargData` accidentally guaranteed that every factor endpoint existed. Removing the insertion removes that accident, and without this prune the mapper's bundle adjustment throws the first time a marginalisation packet names a keyframe the mapper has culled. `CullRedundantKeyframes` already prunes factors for keyframes it removes, at `local_mapper.cpp:885-897`, so the two guards together cover both orderings.

The prune is restricted to the range appended by this packet, so factors accumulated earlier are not rescanned on every ingestion.

#### 3.3.8 Controller wiring, `include/basalt/controller.h` and `src/controller.cpp`

Add the queue beside the existing one.

```cpp
    tbb::concurrent_bounded_queue<basalt::MargData::Ptr> local_map_input_queue_;
    tbb::concurrent_bounded_queue<basalt::Keyframe::Ptr> local_map_kf_queue_;
```

Wire both before the mapper thread is started.

```cpp
    // 4. Wire VIO outputs to the local mapper input queues. The keyframe queue
    //    drives the mapper; the marginalisation queue refines what it holds.
    local_map_input_queue_.set_capacity(10);
    vio_estimator_->out_marg_queue = &local_map_input_queue_;

    local_map_kf_queue_.set_capacity(100);
    vio_estimator_->mpKFOutputQueue = &local_map_kf_queue_;

    // 5. Create and wire the local mapper.
    local_mapper_ = std::make_shared<basalt::LocalMapper>(calib_, vio_config_);
    local_mapper_->SetMarginalisationDataInputQueue(&local_map_input_queue_);
    local_mapper_->SetKFInputQueue(&local_map_kf_queue_);
    local_mapper_->SetVIOPoseUpdateCallback(
        [this](const auto& poses) { vio_estimator_->QueuePoseUpdates(poses); });
    local_mapper_->Initialise();
```

The marginalisation capacity of ten is retained deliberately. The queue can gain at most one packet per keyframe and is fully drained once per keyframe, so ten is an order of magnitude of headroom. The keyframe capacity of one hundred sizes the backpressure threshold at roughly one hundred keyframes of mapper lag.

`Controller::Stop` needs no change. Its comments at `:76-88` describe the sentinel cascade and should be amended to name both queues.

### 3.4 Blast radius

| Consumer | Reached through | Effect |
|---|---|---|
| `src/basalt_slam.cpp` | `Controller`, `SlamMode::VIO` | Local mapping now starts at the first keyframe rather than the eighth |
| `Controller` in `SlamMode::VO` | `SqrtKeypointVoEstimator` | Preserved by mirroring the estimator changes, section 3.3.5 |
| `src/vio.cpp:325`, `src/rs_t265_vio.cpp:195` | `out_marg_queue` into `MargDataSaver` | Unaffected, `MargData::opt_flow_res` is still populated by the estimator and only the mapper discards it |
| `src/mapper.cpp:570`, `src/mapper_sim.cpp:317`, `src/mapper_sim_naive.cpp:589` | `NfrMapper::addMargData` offline | Unaffected, `NfrMapper` is not modified |
| `src/io/marg_data_io.cpp` | cereal serialisation of `MargData` | Unaffected, `Keyframe` is not serialised and `MargData` is unchanged |
| `src/visualisation/visualiser.cpp:56` | `LocalMapper::out_vis_queue` | Unaffected in type, richer in content, since snapshots now begin earlier and contain no zombie keyframes |
| `SqrtKeypointVioEstimator::QueuePoseUpdates` | mapper feedback callback | Unaffected, though it now receives updates earlier in the run |

No test or Python binding references `LocalMapper`, `IngestMargData` or `SetMarginalisationDataInputQueue`, so the blast radius is confined to the table above.

### 3.5 Validation

Compilation of the `basalt` target is the first gate, since the base class placement of `mpKFOutputQueue` and the mirrored visual-only edits are both compile-time propositions. It could not be run. The development container has neither `cmake` nor the TBB headers, and the stale `build/` tree it carries predates `local_mapper.cpp` entering the library source list. Verification was therefore a symbol-level audit, confirming that every new declaration has a matching definition, that both estimators carry explicit template instantiations so `PublishKeyframe` is emitted, that no blocking pop on the marginalisation queue survives, and that all four keyframe sentinel sites are present. The compile gate remains outstanding and must be run on a build host before this is trusted.

A run of `basalt_slam` on `data/machine_hall/MH_01_easy` establishes the functional claims and is likewise outstanding. The mapper's debug output under `vio_debug` should report a non-empty batch for the first keyframe rather than after the eighth. The keyframe count published to the visualiser should be monotone in admission and should never exceed `mpMaxLocalMapSize` once culling engages, which is the direct observable for the resurrection defect. The run must complete a graceful shutdown from the GUI close, which exercises both sentinels and is the regression test for `BUG.md`.

An instrumented count of `frame_poses` entries lacking any entry in `lmdb.getObservations()` should be zero at every cycle, where the current implementation shows a non-zero count once culling begins.

The visual-only mode has no application entry point in this repository, so its validation is limited to compilation and to a targeted unit exercise of `LocalMapper::IngestKeyframe` and `IngestMargData` should such a harness be added.

### 3.6 Deliberately not done

Merging marginalisation packets is not attempted, for the reasons established in `local_mapper_optimisation.md` section 3, namely that the shared variables of two packets carry different linearisation points and naive addition double counts information.

The estimator continues to populate `MargData::opt_flow_res`. Removing it there would be the tidier severing of the duplicate image path, but `MargDataSaver` at `src/io/marg_data_io.cpp:74` serialises exactly that field and the offline `basalt_mapper` pipeline reads it back, so the field stays and the mapper discards its copy instead.

`NfrMapper::addMargData` retains its own unreachable loop over `data->frame_states`. It is dead rather than wrong, it belongs to the offline mapper's call path, and removing it is outside the scope of this change.
