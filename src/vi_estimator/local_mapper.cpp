#include "basalt/vi_estimator/local_mapper.h"

#include <basalt/hash_bow/hash_bow.h>
#include <basalt/utils/keypoints.h>

#include <algorithm>
#include <basalt/camera/stereographic_param.hpp>
#include <chrono>
#include <iostream>

namespace basalt {

// ═══════════════════════════════════════════════════════════════════
// Construction / Destruction / Lifecycle
// ═══════════════════════════════════════════════════════════════════

LocalMapper::LocalMapper(const Calibration<double>& calib,
                         const VioConfig& config, const Logger::Ptr& logger)
    : NfrMapper(calib, config, logger) {
    // Replace NfrMapper's TBB-backed HashBow with the STL-backed variant that
    // supports RemoveKeyframes (needed for keyframe culling).
    hash_bow_database =
        std::make_shared<HashBowStl<256>>(config.mapper_bow_num_bits);
    // Only this estimator reads covisibility.
    lmdb.EnableCovisibilityTracking(true);

    mpMaxLocalMapSize = config.local_mapper_max_local_map_size;
    mpMinLocalMapSize = config.local_mapper_min_local_map_size;
    mpMinRedundantObservers = config.local_mapper_min_redundant_observers;
    mpCullRedundancyThresh = config.local_mapper_cull_redundancy_thresh;
    mpMinObservedForCull = config.local_mapper_min_observed_for_cull;
    mpMaxCullPerPass = config.local_mapper_max_cull_per_pass;
    mpOptIterations = config.local_mapper_opt_iterations;
    mpFilterOutlierThreshold = config.local_mapper_filter_outlier_threshold;
}

LocalMapper::~LocalMapper() { Stop(); }

void LocalMapper::Stop() {
    std::cout << "inside stop function" << std::endl;
    mpStopLocalMapping = true;
    if (mpLocalMappingThread.joinable()) {
        std::cout << "attempting to stop" << std::endl;
        mpLocalMappingThread.join();
        std::cout << "able to stop local mapping thread" << std::endl;
    }
    mpTrackBuilder.ResetTrackId();
    std::cout << "track builder ID reset and mapper stopped" << std::endl;
}

void LocalMapper::Initialise() {
    mpStopLocalMapping = false;
    mpIsTrackBuilderInitialised = false;
    mpRetiredTrackIds.clear();
    mpNewKeyframesForTracking.clear();
    mpLatestKeyframesMatches.clear();

    // Spawn the mapping thread. It busy-waits in MapLocally until the queue
    // that drives it is wired (mpIsKFInputQueueSet == true).
    mpLocalMappingThread = std::thread(&LocalMapper::MapLocally, this);
}

void LocalMapper::SetMarginalisationDataInputQueue(
    tbb::concurrent_bounded_queue<MargData::Ptr>* queue) {
    mpMargInputQueue = queue;
    mpIsMargDataInputQueueSet = true;
}

void LocalMapper::SetKFInputQueue(
    tbb::concurrent_bounded_queue<Keyframe::Ptr>* queue) {
    mpKFInputQueue = queue;
    mpIsKFInputQueueSet = true;
}

void LocalMapper::SetVIOPoseUpdateCallback(PoseUpdateCallback cb) {
    mpVioPoseUpdateCallback = std::move(cb);
}

// ═══════════════════════════════════════════════════════════════════
// MapLocally — thread entry point
// ═══════════════════════════════════════════════════════════════════

void LocalMapper::MapLocally() {
    // Wait until the driving queue is wired.
    while (!mpStopLocalMapping && !mpIsKFInputQueueSet) {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    while (!mpStopLocalMapping) {
        auto t1 = std::chrono::high_resolution_clock::now();

        // Block on the first keyframe so the thread sleeps (zero CPU) when
        // idle, then try_pop any further keyframes that accumulated during the
        // last cycle.
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
        auto tStep = std::chrono::high_resolution_clock::now();
        const double queueWaitSeconds =
            std::chrono::duration<double>(tStep - t1).count();

        const int numKeyframes = int(vKeyframes.size());
        mpNewKeyframesForTracking.clear();
        mpLatestKeyframesMatches.clear();

        for (Keyframe::Ptr& k : vKeyframes) {
            IngestKeyframe(k);
        }
        vKeyframes.clear();

        std::vector<MargData::Ptr> vecData;
        MargData::Ptr data;
        while (mpMargInputQueue && mpMargInputQueue->try_pop(data)) {
            if (!data) {
                nullReceived = true;
                break;
            }
            vecData.push_back(data);
        }
        const int numMargPackets = int(vecData.size());
        for (MargData::Ptr& packet : vecData) {
            IngestMargData(packet);
        }
        vecData.clear();

        for (auto it = img_data.begin(); it != img_data.end();) {
            if (mpNewKeyframesForTracking.count(it->first) == 0) {
                it = img_data.erase(it);
            } else
                ++it;
        }
        auto tEnd = std::chrono::high_resolution_clock::now();
        const double ingestSeconds =
            std::chrono::duration<double>(tEnd - tStep).count();

        if (mpNewKeyframesForTracking.empty()) {
            if (nullReceived) break;
            continue;
        }

        tStep = std::chrono::high_resolution_clock::now();
        detect_keypoints();  // inherited — only touches img_data (now filtered)
        tEnd = std::chrono::high_resolution_clock::now();
        const double detectKeypointsSeconds =
            std::chrono::duration<double>(tEnd - tStep).count();

        double matchStereoSeconds = 0.0;
        if (calib.T_i_c.size() > 1) {
            tStep = std::chrono::high_resolution_clock::now();
            match_stereo();  // inherited — writes to feature_matches
            tEnd = std::chrono::high_resolution_clock::now();
            matchStereoSeconds =
                std::chrono::duration<double>(tEnd - tStep).count();
        }

        tStep = std::chrono::high_resolution_clock::now();
        MatchLocal();  // BoW cross-frame matching for new KFs
        tEnd = std::chrono::high_resolution_clock::now();
        const double matchLocalSeconds =
            std::chrono::duration<double>(tEnd - tStep).count();

        tStep = std::chrono::high_resolution_clock::now();
        CollectNewKeyframesAfterMatching();  // promote to
                                             // mpLatestKeyframesMatches
        tEnd = std::chrono::high_resolution_clock::now();
        const double collectNewKfsSeconds =
            std::chrono::duration<double>(tEnd - tStep).count();

        tStep = std::chrono::high_resolution_clock::now();
        build_tracks();  // LocalMapper (shadowing NfrMapper)
        tEnd = std::chrono::high_resolution_clock::now();
        const double buildTracksSeconds =
            std::chrono::duration<double>(tEnd - tStep).count();

        tStep = std::chrono::high_resolution_clock::now();
        setup_opt();  // LocalMapper (shadowing NfrMapper)
        tEnd = std::chrono::high_resolution_clock::now();
        const double setupOptSeconds =
            std::chrono::duration<double>(tEnd - tStep).count();

        tStep = std::chrono::high_resolution_clock::now();
        CullRedundantKeyframes();
        tEnd = std::chrono::high_resolution_clock::now();
        const double cullSeconds =
            std::chrono::duration<double>(tEnd - tStep).count();

        tStep = std::chrono::high_resolution_clock::now();
        optimize(mpOptIterations);
        tEnd = std::chrono::high_resolution_clock::now();
        const double optimize1Seconds =
            std::chrono::duration<double>(tEnd - tStep).count();

        tStep = std::chrono::high_resolution_clock::now();
        // The hard-coded 4 deleted every landmark left
        // with fewer than four inlier observations, and the measured histogram
        // peaks at two and three, so it removed 150 to 250 landmarks per cycle.
        // filterOutliers(mpFilterOutlierThreshold, 4);
        const size_t nLmBeforeFilter = lmdb.numLandmarks();
        filterOutliers(mpFilterOutlierThreshold, mpFilterMinObs);
        const int64_t cycleTNs =
            frame_poses.empty() ? 0 : frame_poses.rbegin()->first;
        mpLogger->AddMapFilter(cycleTNs, int(nLmBeforeFilter),
                               int(lmdb.numLandmarks()), mpFilterMinObs,
                               mpFilterOutlierThreshold);
        mpLogger->PrintMapFilter();
        tEnd = std::chrono::high_resolution_clock::now();
        const double filterOutliersSeconds =
            std::chrono::duration<double>(tEnd - tStep).count();

        tStep = std::chrono::high_resolution_clock::now();
        optimize(mpOptIterations);
        tEnd = std::chrono::high_resolution_clock::now();
        const double optimize2Seconds =
            std::chrono::duration<double>(tEnd - tStep).count();

        // Publish an immutable local-map snapshot for the GUI. This is the only
        // point in the cycle where frame_poses and lmdb are quiescent on the
        // mapping thread, so the copy is internally consistent and race-free.
        tStep = std::chrono::high_resolution_clock::now();
        if (out_vis_queue) {
            LocalMapperVisualizationData::Ptr vis_data =
                std::make_shared<LocalMapperVisualizationData>();
            vis_data->t_ns =
                frame_poses.empty() ? 0 : frame_poses.rbegin()->first;
            get_current_points(vis_data->points, vis_data->point_ids);
            vis_data->keyframes.reserve(frame_poses.size());
            for (const auto& kv : frame_poses)
                vis_data->keyframes.emplace_back(kv.second.getPose());
            mpLogger->AddMapPublish(
                vis_data->t_ns, int(vis_data->points.size()),
                int(vis_data->keyframes.size()), int(lmdb.numLandmarks()),
                int(lmdb.getHostKfs().size()));
            mpLogger->PrintMapPublish();
            out_vis_queue->try_push(std::move(vis_data));  // never block mapper
        }
        tEnd = std::chrono::high_resolution_clock::now();
        const double publishVisSeconds =
            std::chrono::duration<double>(tEnd - tStep).count();

        tStep = std::chrono::high_resolution_clock::now();
        if (mpVioPoseUpdateCallback) mpVioPoseUpdateCallback(frame_poses);
        tEnd = std::chrono::high_resolution_clock::now();
        const double poseUpdateCbSeconds =
            std::chrono::duration<double>(tEnd - tStep).count();

        const double totalSeconds =
            std::chrono::duration<double>(tEnd - t1).count();

        mpLogger->AddMapCycleTiming(
            cycleTNs, numKeyframes, numMargPackets, queueWaitSeconds,
            ingestSeconds, detectKeypointsSeconds, matchStereoSeconds,
            matchLocalSeconds, collectNewKfsSeconds, buildTracksSeconds,
            setupOptSeconds, cullSeconds, optimize1Seconds,
            filterOutliersSeconds, optimize2Seconds, publishVisSeconds,
            poseUpdateCbSeconds, totalSeconds);
        mpLogger->PrintMapCycleTiming();

        // If the sentinel arrived mid-drain (alongside real data), we still
        // processed all data above — now shut down cleanly.
        if (nullReceived) break;
    }
}

// ═══════════════════════════════════════════════════════════════════
// IngestKeyframe
// ═══════════════════════════════════════════════════════════════════

void LocalMapper::IngestKeyframe(Keyframe::Ptr& kf) {
    if (!kf->opt_flow_res || !kf->opt_flow_res->input_images) return;

    const int64_t t_ns = kf->timestamp;
    if (frame_poses.count(t_ns) > 0) return;

    // Rebuilt rather than copied so linearized stays false, which
    // NfrMapper::optimize asserts before applying an increment.
    frame_poses[t_ns] = PoseStateWithLin<double>(t_ns, kf->pose.getPose());
    img_data[t_ns] = kf->opt_flow_res->input_images;
    mpNewKeyframesForTracking.emplace(t_ns);
}

// ═══════════════════════════════════════════════════════════════════
// IngestMargData
// ═══════════════════════════════════════════════════════════════════

void LocalMapper::IngestMargData(MargData::Ptr& data) {
    // Images arrive with the keyframe. Dropping the packet's copy makes
    // processMargData's img_data write inert without altering NfrMapper,
    // which the offline mapper still depends on.
    data->opt_flow_res.clear();

    const size_t relBegin = rel_pose_factors.size();
    const size_t rpBegin = roll_pitch_factors.size();

    // Inherited factor extraction (mutates data).
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

// ═══════════════════════════════════════════════════════════════════
// PruneFactorsWithUnknownKeyframes
// ═══════════════════════════════════════════════════════════════════

// extractNonlinearFactors draws its endpoints from the packet, so it can name
// keyframes the mapper has culled. Both the BA reduce and computeRelPose /
// computeRollPitch read those endpoints with frame_poses.at, which throws.
void LocalMapper::PruneFactorsWithUnknownKeyframes(size_t relBegin,
                                                   size_t rpBegin) {
    auto known = [this](int64_t t) { return frame_poses.count(t) > 0; };

    rel_pose_factors.erase(
        std::remove_if(
            rel_pose_factors.begin() + static_cast<std::ptrdiff_t>(relBegin),
            rel_pose_factors.end(),
            [&](const RelPoseFactor& f) {
                return !known(f.t_i_ns) || !known(f.t_j_ns);
            }),
        rel_pose_factors.end());

    roll_pitch_factors.erase(
        std::remove_if(
            roll_pitch_factors.begin() + static_cast<std::ptrdiff_t>(rpBegin),
            roll_pitch_factors.end(),
            [&](const RollPitchFactor& f) { return !known(f.t_ns); }),
        roll_pitch_factors.end());
}

// ═══════════════════════════════════════════════════════════════════
// CollectNewKeyframesAfterMatching
// ═══════════════════════════════════════════════════════════════════

void LocalMapper::CollectNewKeyframesAfterMatching() {
    for (const auto& kv : feature_matches) {
        const auto& pair = kv.first;
        if (mpNewKeyframesForTracking.count(pair.first.frame_id) ||
            mpNewKeyframesForTracking.count(pair.second.frame_id)) {
            mpLatestKeyframesMatches[pair] = kv.second;
        }
    }
}

// ═══════════════════════════════════════════════════════════════════
// MatchLocal — BoW cross-frame matching restricted to new KFs
// ═══════════════════════════════════════════════════════════════════

void LocalMapper::MatchLocal() {
    int neighbourPairs = 0;
    std::vector<TimeCamId> keys;
    std::unordered_map<TimeCamId, size_t> id_to_key_idx;

    for (const auto& kv : feature_corners) {
        id_to_key_idx[kv.first] = keys.size();
        keys.push_back(kv.first);
    }

    auto t1 = std::chrono::high_resolution_clock::now();

    struct match_pair {
        size_t i;
        size_t j;
        double score;
    };

    tbb::concurrent_vector<match_pair> ids_to_match;

    tbb::blocked_range<size_t> keys_range(0, keys.size());
    auto compute_pairs = [&](const tbb::blocked_range<size_t>& r) {
        for (size_t i = r.begin(); i != r.end(); ++i) {
            const TimeCamId& tcid = keys[i];
            // Only query from new KFs — this is the key difference from
            // match_all.
            if (mpNewKeyframesForTracking.count(tcid.frame_id) > 0) {
                const KeypointsData& kd = feature_corners.at(tcid);

                std::vector<std::pair<TimeCamId, double>> results;

                hash_bow_database->querry_database(
                    kd.bow_vector, config.mapper_num_frames_to_match, results,
                    &tcid.frame_id);

                double best_score = 0.0;
                size_t above = 0;
                for (const auto& otcid_score : results) {
                    if (otcid_score.first.frame_id == tcid.frame_id) continue;
                    best_score = std::max(best_score, otcid_score.second);
                    if (otcid_score.second <=
                        config.mapper_frames_to_match_threshold)
                        continue;
                    const auto idx_it = id_to_key_idx.find(otcid_score.first);
                    if (idx_it == id_to_key_idx.end()) continue;
                    ++above;
                    match_pair m;
                    m.i = i;
                    m.j = idx_it->second;
                    m.score = otcid_score.second;
                    ids_to_match.emplace_back(m);
                }

                mpLogger->AddMapBowQuery(
                    tcid.frame_id, tcid.frame_id, int(kd.corners.size()),
                    int(kd.corner_descriptors.size()),
                    int(kd.bow_vector.size()), int(results.size()), int(above),
                    best_score, config.mapper_frames_to_match_threshold);
                mpLogger->PrintMapBowQuery();
            }
        }
    };

    tbb::parallel_for(keys_range, compute_pairs);

    // Temporal neighbours, added unconditionally. Bag-of-words retrieval is a
    // place-recognition mechanism and says nothing useful about the keyframes
    // immediately preceding the current one, which a local map already knows
    // are its neighbours. Relying on retrieval alone left track building
    // with no inputs. BoW Matching to be investigated later to ensure
    // appropriate number of matches are available
    {
        std::set<std::pair<size_t, size_t>> already;
        for (const auto& m : ids_to_match) already.emplace(m.i, m.j);

        std::vector<TimeCamId> vOlder;
        for (const auto& tcid : keys)
            if (mpNewKeyframesForTracking.count(tcid.frame_id) == 0)
                vOlder.push_back(tcid);
        std::sort(vOlder.begin(), vOlder.end(),
                  [](const TimeCamId& a, const TimeCamId& b) {
                      return a.frame_id > b.frame_id;  // newest first
                  });

        size_t nNeighbourPairs = 0;
        for (size_t i = 0; i < keys.size(); ++i) {
            if (mpNewKeyframesForTracking.count(keys[i].frame_id) == 0)
                continue;
            size_t taken = 0;
            for (const TimeCamId& other : vOlder) {
                if (taken >= mpLocalMatchNeighbours) break;
                if (other.frame_id >= keys[i].frame_id) continue;
                const size_t j = id_to_key_idx.at(other);
                ++taken;
                if (already.count({i, j})) continue;
                match_pair m;
                m.i = i;
                m.j = j;
                m.score = 0.0;  // not retrieved, admitted on adjacency
                ids_to_match.emplace_back(m);
                ++nNeighbourPairs;
            }
        }
        neighbourPairs = int(nNeighbourPairs);
    }

    auto t2 = std::chrono::high_resolution_clock::now();

    std::atomic<int> total_matched{0};

    tbb::blocked_range<size_t> range(0, ids_to_match.size());
    auto match_func = [&](const tbb::blocked_range<size_t>& r) {
        int matched = 0;

        for (size_t j = r.begin(); j != r.end(); ++j) {
            const TimeCamId& id1 = keys[ids_to_match[j].i];
            const TimeCamId& id2 = keys[ids_to_match[j].j];

            const KeypointsData& f1 = feature_corners[id1];
            const KeypointsData& f2 = feature_corners[id2];

            MatchData md;

            matchDescriptors(f1.corner_descriptors, f2.corner_descriptors,
                             md.matches, config.mapper_max_hamming_distance,
                             config.mapper_second_best_test_ratio);

            if (static_cast<int>(md.matches.size()) >
                config.mapper_min_matches) {
                matched++;
                findInliersRansac(f1, f2, config.mapper_ransac_threshold,
                                  config.mapper_min_matches, md);
            }

            mpLogger->AddMapPairMatch(
                id1.frame_id, id2.frame_id, int(md.matches.size()),
                config.mapper_min_matches, int(md.inliers.size()),
                !md.inliers.empty());
            mpLogger->PrintMapPairMatch();

            if (!md.inliers.empty()) {
                feature_matches[std::make_pair(id1, id2)] =
                    std::allocate_shared<MatchData>(
                        Eigen::aligned_allocator<MatchData>{}, md);
            }
        }
        total_matched += matched;
    };

    tbb::parallel_for(range, match_func);

    auto t3 = std::chrono::high_resolution_clock::now();

    auto elapsed1 =
        std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1);
    auto elapsed2 =
        std::chrono::duration_cast<std::chrono::microseconds>(t3 - t2);

    int num_matches = 0;
    int num_inliers = 0;
    for (const auto& kv : mpLatestKeyframesMatches) {
        num_matches += kv.second->matches.size();
        num_inliers += kv.second->inliers.size();
    }

    const int64_t matchTNs = mpNewKeyframesForTracking.empty()
                                 ? 0
                                 : *mpNewKeyframesForTracking.rbegin();
    mpLogger->AddMapMatchSummary(
        matchTNs, int(ids_to_match.size()), neighbourPairs,
        int(mpLocalMatchNeighbours), total_matched, num_inliers, num_matches,
        elapsed1.count() * 1e-6, elapsed2.count() * 1e-6);
    mpLogger->PrintMapMatchSummary();
}

// ═══════════════════════════════════════════════════════════════════
// build_tracks — incremental track building (shadows NfrMapper)
// ═══════════════════════════════════════════════════════════════════

void LocalMapper::build_tracks() {
    // Convert mpLatestKeyframesMatches (std::unordered_map) to Matches (TBB
    // map) because TrackBuilder::Build uses the Matches API.
    Matches tbb_matches;
    for (const auto& kv : mpLatestKeyframesMatches)
        tbb_matches.insert({kv.first, kv.second});

    if (!mpIsTrackBuilderInitialised) {
        mpTrackBuilder.Build(tbb_matches);
        mpTrackBuilder.Filter(2);
        feature_tracks.clear();
        mpTrackBuilder.Export(feature_tracks);
        mpIsTrackBuilderInitialised = true;
    } else {
        feature_tracks.clear();
        mpTrackBuilder.AddNewMatches(tbb_matches, lmdb, feature_tracks,
                                     mpRetiredTrackIds);
    }

    std::map<size_t, size_t> hist;
    for (const auto& kv : feature_tracks) hist[kv.second.size()]++;
    // Index i holds length i + 2 (the shortest track Filter(2) admits), and
    // index 9 accumulates every length at or above 11.
    Eigen::VectorXd lenHist = Eigen::VectorXd::Zero(10);
    for (const auto& [len, count] : hist) {
        if (len < 2) continue;
        lenHist[std::min(len, size_t(11)) - 2] += double(count);
    }

    const int64_t tracksTNs = mpNewKeyframesForTracking.empty()
                                  ? 0
                                  : *mpNewKeyframesForTracking.rbegin();
    mpLogger->AddMapTracks(tracksTNs, int(feature_tracks.size()),
                           int(mpTrackBuilder.TrackCount()),
                           int(mpLatestKeyframesMatches.size()),
                           int(mpNewKeyframesForTracking.size()),
                           int(feature_corners.size()), lenHist);
    mpLogger->PrintMapTracks();
}

// ═══════════════════════════════════════════════════════════════════
// setup_opt — landmark DB update (shadows NfrMapper)
// ═══════════════════════════════════════════════════════════════════

void LocalMapper::setup_opt() {
    const double min_triang_dist2 = config.mapper_min_triangulation_dist *
                                    config.mapper_min_triangulation_dist;

    // Triangulation accounting. Every landmark that fails to enter lmdb does so
    // for exactly one of these reasons, so the totals localise the failure.
    size_t nTracks = 0, nShortTrack = 0, nAlreadyKnown = 0, nNewOk = 0;
    size_t nNoCorners = 0, nNoHostPose = 0, nNoObsPose = 0, nShortBaseline = 0;
    size_t nBadDepth = 0, nNoTriangulation = 0, nObsAdded = 0;
    size_t nBehind = 0, nTooClose = 0, nNonFinite = 0, nLowParallax = 0;
    const size_t nLandmarksBefore = lmdb.numLandmarks();

    // Step A — retire merged landmarks.
    for (const TrackId retired : mpRetiredTrackIds) {
        if (lmdb.landmarkExists(retired)) {
            lmdb.removeLandmark(retired);
        }
    }
    const size_t nRetired = mpRetiredTrackIds.size();
    mpRetiredTrackIds.clear();

    // Step B — iterate updated feature tracks.
    for (const auto& kv : feature_tracks) {
        ++nTracks;
        if (kv.second.size() < 2) {
            ++nShortTrack;
            continue;
        }

        // Add landmark iff not yet known to lmdb — skip-if-exists preserves
        // optimised geometry from previous iterations.
        if (lmdb.landmarkExists(kv.first)) ++nAlreadyKnown;
        if (!lmdb.landmarkExists(kv.first)) {
            auto it_h = kv.second.begin();
            TimeCamId tcid_h = it_h->first;
            FeatureId fid_h = it_h->second;

            Eigen::Vector2d pos_2d_h =
                feature_corners.at(tcid_h).corners[fid_h];
            Eigen::Vector4d pos_3d_h;
            calib.intrinsics[tcid_h.cam_id].unproject(pos_2d_h, pos_3d_h);

            if (!frame_poses.count(tcid_h.frame_id)) {
                ++nNoHostPose;
                continue;
            }
            const Sophus::SE3d T_w_h =
                frame_poses.at(tcid_h.frame_id).getPose() *
                calib.T_i_c[tcid_h.cam_id];

            std::vector<std::pair<double, decltype(kv.second.begin())>> vCands;
            for (auto it_o = std::next(it_h); it_o != kv.second.end(); ++it_o) {
                if (feature_corners.count(it_o->first) == 0) {
                    ++nNoCorners;
                    continue;
                }
                if (!frame_poses.count(it_o->first.frame_id)) {
                    ++nNoObsPose;
                    continue;
                }
                const Sophus::SE3d T_w_o =
                    frame_poses.at(it_o->first.frame_id).getPose() *
                    calib.T_i_c[it_o->first.cam_id];
                vCands.emplace_back(
                    (T_w_h.inverse() * T_w_o).translation().squaredNorm(),
                    it_o);
            }
            std::sort(
                vCands.begin(), vCands.end(),
                [](const auto& a, const auto& b) { return a.first > b.first; });

            bool triangulated = false;
            for (const auto& cand : vCands) {
                auto it_o = cand.second;
                TimeCamId tcid_o = it_o->first;
                FeatureId fid_o = it_o->second;

                Eigen::Vector2d pos_2d_o =
                    feature_corners.at(tcid_o).corners[fid_o];
                Eigen::Vector4d pos_3d_o;
                calib.intrinsics[tcid_o.cam_id].unproject(pos_2d_o, pos_3d_o);

                Sophus::SE3d T_w_o = frame_poses.at(tcid_o.frame_id).getPose() *
                                     calib.T_i_c[tcid_o.cam_id];
                Sophus::SE3d T_h_o = T_w_h.inverse() * T_w_o;

                if (cand.first < min_triang_dist2) {
                    ++nShortBaseline;
                    continue;  // sorted descending, but keep the accounting
                }

                // ORB-SLAM3's parallax gate
                // Near-parallel bearing rays make the
                // linear solve ill-conditioned, and it then returns a point
                // behind the camera or at absurd depth rather than failing.
                const Eigen::Vector3d ray_h = pos_3d_h.head<3>().normalized();
                const Eigen::Vector3d ray_o =
                    (T_h_o.so3() * pos_3d_o.head<3>()).normalized();
                if (ray_h.dot(ray_o) > mpMaxCosParallax) {
                    ++nLowParallax;
                    continue;
                }

                Eigen::Vector4d pos_3d =
                    triangulate(pos_3d_h.head<3>(), pos_3d_o.head<3>(), T_h_o);
                if (!pos_3d.array().isFinite().all()) {
                    ++nNonFinite;
                    ++nBadDepth;
                    continue;
                }
                if (pos_3d[3] <= 0) {
                    ++nBehind;
                    ++nBadDepth;
                    continue;
                }
                if (pos_3d[3] > mpMaxInvDist) {
                    ++nTooClose;
                    ++nBadDepth;
                    continue;
                }

                Keypoint<Scalar> kpt;
                kpt.host_kf_id = tcid_h;
                kpt.direction = StereographicParam<double>::project(pos_3d);
                kpt.inv_dist = pos_3d[3];
                lmdb.addLandmark(kv.first, kpt);
                triangulated = true;
                ++nNewOk;
                break;
            }
            if (!triangulated) {
                ++nNoTriangulation;
                continue;
            }
        }

        // Add observations (idempotent — safe to repeat).
        for (const auto& obs_kv : kv.second) {
            if (!frame_poses.count(obs_kv.first.frame_id)) continue;
            if (feature_corners.count(obs_kv.first) == 0) continue;
            KeypointObservation<Scalar> ko;
            ko.kpt_id = kv.first;
            ko.pos = feature_corners.at(obs_kv.first).corners[obs_kv.second];
            lmdb.addObservation(obs_kv.first, ko);
            ++nObsAdded;
        }
    }

    const int64_t setupOptTNs =
        frame_poses.empty() ? 0 : frame_poses.rbegin()->first;
    mpLogger->AddMapSetupOpt(
        setupOptTNs, int(nTracks), int(nShortTrack), int(nAlreadyKnown),
        int(nRetired), int(nNewOk), int(nNoTriangulation), int(nNoCorners),
        int(nNoHostPose), int(nNoObsPose), int(nShortBaseline),
        int(nLowParallax), int(nBadDepth), int(nBehind), int(nTooClose),
        int(nNonFinite), int(nObsAdded), int(nLandmarksBefore),
        int(lmdb.numLandmarks()), config.mapper_min_triangulation_dist,
        mpMaxCosParallax, mpMaxInvDist);
    mpLogger->PrintMapSetupOpt();
}

// ═══════════════════════════════════════════════════════════════════
// ComputeCovisibility
// ═══════════════════════════════════════════════════════════════════

// size_t LocalMapper::ComputeCovisibility(int64_t tid_a, int64_t tid_b,
//                                         int num_cameras) {
//     size_t covisibility = 0;
//     for (int i = 0; i < num_cameras; i++) {
//         for (int j = 0; j < num_cameras; j++) {
//             covisibility += lmdb.getObservationsCountForPair(
//                 TimeCamId(tid_a, i), TimeCamId(tid_b, j));
//             covisibility += lmdb.getObservationsCountForPair(
//                 TimeCamId(tid_b, j), TimeCamId(tid_a, i));
//         }
//     }
//     return covisibility;
// }

// ═══════════════════════════════════════════════════════════════════
// SelectKeyframeToCull
// ═══════════════════════════════════════════════════════════════════

ObservedByFrameMap LocalMapper::BuildObservedSets() const {
    ObservedByFrameMap seen;
    for (const auto& [host, targets] : lmdb.getObservations())
        for (const auto& [target, lms] : targets)
            seen[target.frame_id].insert(lms.begin(), lms.end());
    return seen;
}

CovisMatrix LocalMapper::BuildCovisibilityMatrix(
    const ObservedByFrameMap& seen) const {
    CovisMatrix m;
    for (auto it = seen.begin(); it != seen.end(); ++it)
        for (auto jt = std::next(it); jt != seen.end(); ++jt) {
            std::vector<KeypointId> shared;
            std::set_intersection(it->second.begin(), it->second.end(),
                                  jt->second.begin(), jt->second.end(),
                                  std::back_inserter(shared));
            if (shared.empty()) continue;
            m[it->first][jt->first] = shared.size();
            m[jt->first][it->first] = shared.size();
        }
    return m;
}

double LocalMapper::RedundancyScore(int64_t kf, const ObservedByFrameMap& seen,
                                    const std::set<int64_t>& alive,
                                    size_t& nObservedOut) const {
    const ObservedByFrameMap::const_iterator it = seen.find(kf);
    if (it == seen.end() || it->second.empty()) {
        nObservedOut = 0;
        return 0.0;
    }
    size_t redundant = 0;
    for (const KeypointId lm : it->second) {
        size_t others = 0;
        const ObserverFrameMap& observers = lmdb.GetObserverFrames(lm);
        for (ObserverFrameMap::const_iterator f = observers.begin();
             f != observers.end(); ++f) {
            if (f->first == kf || alive.count(f->first) == 0) continue;
            if (++others >= mpMinRedundantObservers) break;
        }
        if (others >= mpMinRedundantObservers) ++redundant;
    }
    nObservedOut = it->second.size();
    return static_cast<double>(redundant) / static_cast<double>(nObservedOut);
}

// ═══════════════════════════════════════════════════════════════════
// SelectKeyframesToCull
// ═══════════════════════════════════════════════════════════════════

bool LocalMapper::SelectKeyframesToCull(std::vector<int64_t>& keyframesToCull) {
    if (frame_poses.size() <= mpMinLocalMapSize) return false;

    const ObservedByFrameMap& seen = lmdb.GetObservedByFrame();

    if (mpVioDebugMode) {
        const ObservedByFrameMap ref_seen = BuildObservedSets();
        const CovisMatrix ref_covis = BuildCovisibilityMatrix(ref_seen);
        const bool ok = ref_seen == seen && ref_covis == lmdb.GetCovisibility();
        const int64_t covisTNs =
            frame_poses.empty() ? 0 : frame_poses.rbegin()->first;
        mpLogger->AddMapCovis(
            covisTNs, ok, int(seen.size()), int(ref_seen.size()),
            int(lmdb.GetCovisibility().size()), int(ref_covis.size()));
        mpLogger->PrintMapCovis();
    }

    std::vector<int64_t> ordered;
    ordered.reserve(frame_poses.size());
    for (const auto& kv : frame_poses) ordered.push_back(kv.first);
    std::sort(ordered.begin(), ordered.end());

    const size_t keep_recent =
        std::min<size_t>(mpNewKeyframesForTracking.size(), ordered.size());
    const size_t eligible_end = ordered.size() - keep_recent;
    const size_t max_cull =
        std::min(frame_poses.size() - mpMinLocalMapSize, mpMaxCullPerPass);

    std::set<int64_t> alive(ordered.begin(), ordered.end());

    // Remove one keyframe at a time, always the most redundant survivor, and
    // re-score against `alive` after each removal. Re-scoring reproduces
    // ORB-SLAM3's live-state semantics inside a batch selection, and it is what
    // stops both members of a mutually covisible pair being culled together.
    while (keyframesToCull.size() < max_cull) {
        int64_t worst = -1;
        double worstScore = mpCullRedundancyThresh;
        size_t worstObserved = 0;
        for (size_t i = 0; i < eligible_end; ++i) {
            const int64_t a = ordered[i];
            if (!alive.count(a)) continue;
            size_t nObserved = 0;
            const double score = RedundancyScore(a, seen, alive, nObserved);
            if (nObserved < mpMinObservedForCull) continue;
            if (score > worstScore) {
                worstScore = score;
                worst = a;
                worstObserved = nObserved;
            }
        }
        if (worst < 0) break;
        if (mpVioDebugMode)
            std::cout << "[Local Mapper][cull-select] redundancy pick kf="
                      << worst << " score=" << worstScore
                      << " observed=" << worstObserved << std::endl;
        keyframesToCull.push_back(worst);
        alive.erase(worst);
    }

    // Criterion 2 — capacity. Retained because a keyframe that observes nothing
    // scores zero under the redundancy criterion and is exempted by
    // mpMinObservedForCull, so this is the only path that can remove it.
    bool capacityRuleFired = false;
    if (frame_poses.size() > mpMaxLocalMapSize && keyframesToCull.empty()) {
        if (mpVioDebugMode)
            std::cout << "[Local Mapper][cull-select] capacity rule, culling "
                         "oldest keyframe"
                      << std::endl;
        keyframesToCull.push_back(ordered.front());
        capacityRuleFired = true;
    }

    // Observer-count histogram over the whole map. Index i holds landmarks
    // with i observers, and index 9 accumulates 9 or more.
    Eigen::VectorXd observersHist = Eigen::VectorXd::Zero(10);
    {
        std::set<KeypointId> all_lms;
        for (const auto& [kf, lms] : seen)
            all_lms.insert(lms.begin(), lms.end());
        for (const KeypointId lm : all_lms)
            observersHist[std::min(lmdb.GetObserverFrames(lm).size(),
                                   size_t(9))]++;
    }

    const int64_t cullSelectTNs =
        frame_poses.empty() ? 0 : frame_poses.rbegin()->first;
    mpLogger->AddMapCullSelect(cullSelectTNs, int(frame_poses.size()),
                               int(eligible_end), int(keep_recent),
                               int(max_cull), int(mpMinRedundantObservers),
                               int(keyframesToCull.size()), capacityRuleFired,
                               mpCullRedundancyThresh, observersHist);
    mpLogger->PrintMapCullSelect();

    if (mpVioDebugMode) {
        std::cout << "[Local Mapper][cull-select] selected [";
        for (const int64_t kf : keyframesToCull) std::cout << kf << " ";
        std::cout << "]" << std::endl;
    }

    return !keyframesToCull.empty();
}

// ═══════════════════════════════════════════════════════════════════
// FindBestRehostKf
// ═══════════════════════════════════════════════════════════════════

int64_t LocalMapper::FindBestRehostKf(int64_t culled_kf, TrackId lm_id,
                                      const std::set<int64_t>& candidates) {
    int64_t best = -1;
    size_t best_covis = 0;
    if (lm_id < 0 || !lmdb.landmarkExists(lm_id)) return -1;
    const auto& obs_set = lmdb.getLandmark(lm_id).obs;

    for (const int64_t c : candidates) {
        if (c == culled_kf) continue;
        // lm_id must be observed by c in at least one camera.
        bool observed = false;
        for (size_t cam = 0; cam < calib.intrinsics.size(); ++cam) {
            if (obs_set.count(TimeCamId(c, cam))) {
                observed = true;
                break;
            }
        }
        if (!observed) continue;

        size_t cv = 0;
        const CovisMatrix& covis_matrix = lmdb.GetCovisibility();
        const CovisMatrix::const_iterator row = covis_matrix.find(c);
        if (row != covis_matrix.end())
            for (CovisRow::const_iterator cell = row->second.begin();
                 cell != row->second.end(); ++cell)
                if (candidates.count(cell->first)) cv += cell->second;

        if (cv > best_covis) {
            best_covis = cv;
            best = c;
        }
    }
    return best;
}

// ═══════════════════════════════════════════════════════════════════
// RehostLandmark
// ═══════════════════════════════════════════════════════════════════

void LocalMapper::RehostLandmark(TrackId lm_id, int64_t culled_kf,
                                 int64_t new_host_kf) {
    const int64_t rehostTNs =
        frame_poses.empty() ? 0 : frame_poses.rbegin()->first;
    const Keypoint<double>& old_kpt = lmdb.getLandmark(lm_id);

    // Preserve observations and the existing geometry before removeLandmark
    // tears them down. The geometry feeds the reprojection fallback below.
    Eigen::aligned_map<TimeCamId, Eigen::Vector2d> obs_copy(old_kpt.obs.begin(),
                                                            old_kpt.obs.end());
    const TimeCamId old_host_tcid = old_kpt.host_kf_id;
    const Eigen::Vector2d old_direction = old_kpt.direction;
    const double old_inv_dist = old_kpt.inv_dist;

    // Choose the cam_id of the new host that actually observed this landmark.
    // Prefer camera 0 for determinism when both cameras observed it.
    TimeCamId new_host_tcid(new_host_kf, 0);
    if (obs_copy.count(new_host_tcid) == 0 && calib.intrinsics.size() > 1) {
        bool found_obs = false;
        for (size_t i = 1; i < calib.intrinsics.size(); i++) {
            TimeCamId alt(new_host_kf, i);
            if (obs_copy.count(alt)) {
                new_host_tcid = alt;
                found_obs = true;
                break;
            }
        }
        if (!found_obs) {
            mpLogger->AddMapRehost(rehostTNs, lm_id, culled_kf, new_host_kf,
                                   int(obs_copy.size()), 0, 3, 1);
            mpLogger->PrintMapRehost();
            return;
        }
    } else if (obs_copy.count(new_host_tcid) == 0) {
        mpLogger->AddMapRehost(rehostTNs, lm_id, culled_kf, new_host_kf,
                               int(obs_copy.size()), 0, 3, 1);
        mpLogger->PrintMapRehost();
        return;
    }

    // Compute new landmark parameters in the new host frame.
    const Eigen::Vector2d& pos_2d_new = obs_copy.at(new_host_tcid);
    Eigen::Vector4d pos_3d_new_hom;
    if (!calib.intrinsics[new_host_tcid.cam_id].unproject(pos_2d_new,
                                                          pos_3d_new_hom)) {
        mpLogger->AddMapRehost(rehostTNs, lm_id, culled_kf, new_host_kf,
                               int(obs_copy.size()), 0, 3, 2);
        mpLogger->PrintMapRehost();
        return;
    }

    Sophus::SE3d T_w_newh = frame_poses.at(new_host_kf).getPose() *
                            calib.T_i_c[new_host_tcid.cam_id];

    Keypoint<double> new_kpt;
    new_kpt.host_kf_id = new_host_tcid;
    bool triangulated = false;
    for (const auto& o : obs_copy) {
        if (o.first == new_host_tcid) continue;
        if (o.first.frame_id == culled_kf) continue;
        if (!frame_poses.count(o.first.frame_id)) continue;

        Eigen::Vector4d p_o_hom;
        if (!calib.intrinsics[o.first.cam_id].unproject(o.second, p_o_hom))
            continue;
        Sophus::SE3d T_w_o = frame_poses.at(o.first.frame_id).getPose() *
                             calib.T_i_c[o.first.cam_id];
        Sophus::SE3d T_newh_o = T_w_newh.inverse() * T_w_o;
        if (T_newh_o.translation().squaredNorm() <
            config.mapper_min_triangulation_dist *
                config.mapper_min_triangulation_dist)
            continue;
        Eigen::Vector4d p_3d =
            triangulate(pos_3d_new_hom.head<3>(), p_o_hom.head<3>(), T_newh_o);
        if (!p_3d.array().isFinite().all() || p_3d[3] <= 0 || p_3d[3] > 2.0)
            continue;
        new_kpt.direction = StereographicParam<double>::project(p_3d);
        new_kpt.inv_dist = p_3d[3];
        triangulated = true;
        break;
    }
    // Fallback. A landmark with only the culled host and the new host as
    // observers has no third view, so the loop above cannot run at all and the
    // landmark was destroyed unconditionally.
    if (!triangulated) {
        const Sophus::SE3d T_w_oldh = frame_poses.at(culled_kf).getPose() *
                                      calib.T_i_c[old_host_tcid.cam_id];
        Eigen::Vector4d p_oldh =
            StereographicParam<double>::unproject(old_direction);
        p_oldh[3] = old_inv_dist;
        Eigen::Vector4d p_newh =
            (T_w_newh.inverse() * T_w_oldh).matrix() * p_oldh;
        // triangulate() guarantees a unit-length direction head with the
        // inverse distance in the last component, and the rest of the pipeline
        // relies on that. Rescaling the homogeneous point leaves the 3D
        // position unchanged while restoring the invariant.
        const double dir_norm = p_newh.head<3>().norm();
        if (dir_norm > 0) p_newh /= dir_norm;
        if (p_newh.array().isFinite().all() && dir_norm > 0 && p_newh[3] > 0 &&
            p_newh[3] <= 2.0) {
            new_kpt.direction = StereographicParam<double>::project(p_newh);
            new_kpt.inv_dist = p_newh[3];
            triangulated = true;
            mpLogger->AddMapRehost(rehostTNs, lm_id, culled_kf, new_host_kf,
                                   int(obs_copy.size()), 0, 2, 0);
            mpLogger->PrintMapRehost();
        }
    }

    if (!triangulated) {
        mpLogger->AddMapRehost(rehostTNs, lm_id, culled_kf, new_host_kf,
                               int(obs_copy.size()), 0, 4, 3);
        mpLogger->PrintMapRehost();
        lmdb.removeLandmark(lm_id);
        return;
    }

    // Perform the swap.
    lmdb.removeLandmark(lm_id);
    lmdb.addLandmark(lm_id, new_kpt);
    int obs_added = 0;
    for (const auto& o : obs_copy) {
        if (o.first.frame_id == culled_kf) continue;
        KeypointObservation<double> ko;
        ko.kpt_id = lm_id;
        ko.pos = o.second;
        lmdb.addObservation(o.first, ko);
        ++obs_added;
    }

    // If every observation was either from the culled frame or from the new
    // host itself, no entry was added to the observations index for this
    // landmark. The landmark would be dangling in kpts with an empty obs set,
    // which would cause removeLandmarkHelper to receive observations.end()
    // later. Remove it immediately instead.
    const int outcome = obs_added == 0 ? 4 : (obs_added < 2 ? 1 : 0);
    mpLogger->AddMapRehost(rehostTNs, lm_id, culled_kf, new_host_kf,
                           int(obs_copy.size()), obs_added, outcome,
                           obs_added == 0 ? 4 : 0);
    mpLogger->PrintMapRehost();

    if (obs_added == 0) {
        lmdb.removeLandmark(lm_id);
    }
}

// ═══════════════════════════════════════════════════════════════════
// CullRedundantKeyframes
// ═══════════════════════════════════════════════════════════════════

void LocalMapper::CullRedundantKeyframes() {
    std::vector<KeypointId> landmarksToRemove;
    std::vector<int64_t> keyframesToCull;
    bool toCull = SelectKeyframesToCull(keyframesToCull);
    if (!toCull) {
        // Still clear transient per-step data even if no culling occurred.
        img_data.clear();
        feature_tracks.clear();
        mpLatestKeyframesMatches.clear();
        return;
    }

    // ─── Step 1 — rehost landmarks hosted by culled KF ─────────────────
    for (int64_t culled : keyframesToCull) {
        const size_t nLmBeforeKf = lmdb.numLandmarks();
        const int mapBefore = int(frame_poses.size());
        size_t nRehosted = 0, nNoHost = 0;

        // A keyframe still queued for culling must not become a host, or the
        // landmark pays the rehost cost again when that keyframe's turn comes.
        // frame_poses.erase only removes victims already processed.
        const std::set<int64_t> doomed(keyframesToCull.begin(),
                                       keyframesToCull.end());
        std::set<int64_t> candidates;
        for (const auto& kv : frame_poses)
            if (!doomed.count(kv.first)) candidates.insert(kv.first);

        // Collect unique landmark IDs hosted by the culled KF via the
        // observations index — avoids getLandmarksForHost (which throws on
        // missing keys) and produces deduplicated IDs in O(N).
        const auto& obs = lmdb.getObservations();
        std::set<KeypointId> hosted_lm_set;
        for (size_t cam = 0; cam < calib.intrinsics.size(); ++cam) {
            TimeCamId tcid(culled, cam);
            auto obs_it = obs.find(tcid);
            if (obs_it == obs.end()) continue;
            for (const auto& [target, kpt_set] : obs_it->second) {
                hosted_lm_set.insert(kpt_set.begin(), kpt_set.end());
            }
        }
        std::vector<TrackId> hosted_lm_ids(hosted_lm_set.begin(),
                                           hosted_lm_set.end());

        landmarksToRemove.clear();
        for (const TrackId lm : hosted_lm_ids) {
            const int64_t new_host = FindBestRehostKf(culled, lm, candidates);
            if (new_host < 0) {
                ++nNoHost;
                mpLogger->AddMapRehost(culled, lm, culled, -1,
                                       int(candidates.size()), 0, 4, 5);
                mpLogger->PrintMapRehost();
                if (lmdb.landmarkExists(lm)) {
                    landmarksToRemove.push_back(lm);
                }
            } else {
                ++nRehosted;
                // A candidate that is itself queued for culling is currently
                // accepted here. That is the defect under investigation.
                if (std::find(keyframesToCull.begin(), keyframesToCull.end(),
                              new_host) != keyframesToCull.end()) {
                    mpLogger->AddMapRehost(culled, lm, culled, new_host, 0, 0,
                                           5, 0);
                    mpLogger->PrintMapRehost();
                }
                RehostLandmark(lm, culled, new_host);
            }
        }
        for (const KeypointId id : landmarksToRemove) {
            lmdb.removeLandmark(id);
        }

        // ─── Step 2 — remove remaining observations of culled KF ───────────
        lmdb.removeFrame(culled);

        // ─── Step 3 — prune NFR factors referencing culled KF ──────────────
        rel_pose_factors.erase(
            std::remove_if(rel_pose_factors.begin(), rel_pose_factors.end(),
                           [culled](const RelPoseFactor& f) {
                               return f.t_i_ns == culled || f.t_j_ns == culled;
                           }),
            rel_pose_factors.end());
        roll_pitch_factors.erase(
            std::remove_if(roll_pitch_factors.begin(), roll_pitch_factors.end(),
                           [culled](const RollPitchFactor& f) {
                               return f.t_ns == culled;
                           }),
            roll_pitch_factors.end());

        // ─── Step 4 — erase frame_poses and feature_corners ─────────────────
        frame_poses.erase(culled);
        for (size_t cam = 0; cam < calib.intrinsics.size(); ++cam)
            feature_corners.unsafe_erase(TimeCamId(culled, cam));

        // feature_matches: erase any entry involving culled.
        for (auto kv = feature_matches.begin(); kv != feature_matches.end();) {
            if (kv->first.first.frame_id == culled ||
                kv->first.second.frame_id == culled) {
                kv = feature_matches.unsafe_erase(kv);
            } else {
                ++kv;
            }
        }

        // ─── Step 5 — BoW database ──────────────────────────────────────────
        std::vector<TimeCamId> bow_to_drop;
        for (size_t cam = 0; cam < calib.intrinsics.size(); ++cam)
            bow_to_drop.emplace_back(TimeCamId(culled, cam));
        hash_bow_database->RemoveKeyframes(bow_to_drop);

        // ─── Step 6 — TrackBuilder cleanup ──────────────────────────────────
        mpTrackBuilder.DeleteTracksAfterCulling({culled});

        mpLogger->AddMapCull(culled, culled, int(keyframesToCull.size()),
                             int(hosted_lm_ids.size()), int(nRehosted),
                             int(nNoHost), int(nLmBeforeKf),
                             int(lmdb.numLandmarks()), mapBefore,
                             int(frame_poses.size()));
        mpLogger->PrintMapCull();
    }

    // ─── Step 7 — clear transient per-step data ─────────────────────────
    img_data.clear();
    feature_tracks.clear();
    mpLatestKeyframesMatches.clear();
}

}  // namespace basalt
