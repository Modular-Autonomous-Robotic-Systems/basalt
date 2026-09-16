#include <basalt/utils/logger.h>

#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <sstream>

namespace basalt {

// ═══════════════════════════════════════════════════════════════════
// AggregateStats
// ═══════════════════════════════════════════════════════════════════

AggregateStats::AggregateStats(std::string name) : mpName(std::move(name)) {}

AggregateStats::~AggregateStats() { CloseStream(); }

void AggregateStats::RegisterStats(const Registry& registry) {
    std::lock_guard<std::mutex> lock(mpMtxRows);
    mpRegistry = registry;
}

void AggregateStats::SetPrefixForLogging(const std::string& prefix) {
    std::lock_guard<std::mutex> lock(mpMtxRows);
    mpPrefix = prefix;
}

std::string AggregateStats::FormatFor(const std::string& name) const {
    const Registry::const_iterator it = mpRegistry.find(name);
    return it == mpRegistry.end() ? std::string("f") : it->second;
}

AggregateStats::Meta& AggregateStats::add(const std::string& name,
                                          double value) {
    std::lock_guard<std::mutex> lock(mpMtxRows);
    Meta& meta = ExecutionStats::add(name, value);
    meta.format(FormatFor(name));
    return meta;
}

AggregateStats::Meta& AggregateStats::add(const std::string& name,
                                          const Eigen::VectorXd& value) {
    std::lock_guard<std::mutex> lock(mpMtxRows);
    Meta& meta = ExecutionStats::add(name, value);
    meta.format(FormatFor(name));
    return meta;
}

AggregateStats::Meta& AggregateStats::add(const std::string& name,
                                          const Eigen::VectorXf& value) {
    // Not ExecutionStats::add(name, value): that overload casts and calls
    // add(name, VectorXd) through virtual dispatch, which would re-enter the
    // override below and deadlock on mpMtxRows.
    return add(name, value.cast<double>().eval());
}

AggregateStats::Meta& AggregateStats::add_int(const std::string& name,
                                              int64_t value) {
    std::lock_guard<std::mutex> lock(mpMtxRows);
    Meta& meta = ExecutionStats::add_int(name, value);
    meta.format(FormatFor(name));
    return meta;
}

std::string AggregateStats::RenderValue(const Meta& meta) const {
    const std::string& fmt = meta.format_;
    std::ostringstream os;
    std::visit(
        overload{
            [&](const std::vector<double>& d) {
                if (d.empty()) return;
                const double v = d.back();
                if (fmt == "count" || fmt == "flag" || fmt == "enum")
                    os << int64_t(v);
                else if (fmt == "ms")
                    os << std::fixed << std::setprecision(3) << v * 1e3;
                else if (fmt == "ratio")
                    os << std::fixed << std::setprecision(4) << v;
                else if (fmt == "e")
                    os << std::scientific << std::setprecision(4) << v;
                else
                    os << std::setprecision(6) << v;
            },
            [&](const std::vector<Eigen::VectorXd>& d) {
                if (d.empty()) return;
                os << d.back().transpose();
            },
            [&](const std::vector<int64_t>& d) {
                if (d.empty()) return;
                os << d.back();
            }},
        meta.data_);
    return os.str();
}

std::string AggregateStats::PrintLatest() const { return PrintLatest(mpPrefix); }

std::string AggregateStats::PrintLatest(const std::string& prefix) const {
    std::lock_guard<std::mutex> lock(mpMtxRows);
    std::ostringstream os;
    if (!prefix.empty()) os << prefix << ' ';
    bool first = true;
    for (const std::string& name : order_) {
        const Meta& meta = stats_.at(name);
        if (meta.format_ == "none") continue;
        if (!first) os << ' ';
        first = false;
        os << name << '=' << RenderValue(meta);
    }
    return os.str();
}

std::vector<std::string> AggregateStats::ColumnNames() const {
    std::vector<std::string> vNames;
    for (const std::string& name : order_) {
        const std::string fmt = FormatFor(name);
        if (fmt.rfind("vec", 0) == 0) {
            const int width = std::atoi(fmt.c_str() + 3);
            for (int i = 0; i < width; i++)
                vNames.push_back(name + "_" + std::to_string(i));
        } else {
            vNames.push_back(name);
        }
    }
    return vNames;
}

void AggregateStats::WriteHeaderLocked() {
    const std::vector<std::string> names = ColumnNames();
    for (size_t i = 0; i < names.size(); i++) {
        if (i) mpStream << ',';
        mpStream << names[i];
    }
    mpStream << '\n';
    mpHeaderWritten = true;
}

void AggregateStats::WriteRowLocked() {
    bool first = true;
    for (const std::string& name : order_) {
        const Meta& meta = stats_.at(name);
        const std::string& fmt = meta.format_;
        std::visit(
            overload{
                [&](const std::vector<double>& d) {
                    if (!first) mpStream << ',';
                    first = false;
                    if (d.empty()) return;
                    const double v = d.back();
                    if (fmt == "count" || fmt == "flag" || fmt == "enum")
                        mpStream << int64_t(v);
                    else if (fmt == "ms")
                        mpStream << v * 1e3;
                    else
                        mpStream << v;
                },
                [&](const std::vector<Eigen::VectorXd>& d) {
                    const int width =
                        fmt.rfind("vec", 0) == 0 ? std::atoi(fmt.c_str() + 3) : 0;
                    const bool blank =
                        mpBlankEmptyVectors && !d.empty() && d.back().size() == 0;
                    for (int i = 0; i < width; i++) {
                        if (!first) mpStream << ',';
                        first = false;
                        if (blank) continue;
                        mpStream << (!d.empty() && i < d.back().size()
                                         ? d.back()[i]
                                         : 0.0);
                    }
                },
                [&](const std::vector<int64_t>& d) {
                    if (!first) mpStream << ',';
                    first = false;
                    if (!d.empty()) mpStream << d.back();
                }},
            meta.data_);
    }
    mpStream << '\n';
}

bool AggregateStats::OpenStream(const std::string& path) {
    std::lock_guard<std::mutex> lock(mpMtxRows);
    mpStream.open(path, std::ios::out | std::ios::trunc);
    mpHeaderWritten = false;
    mpRowsWrittenToFile = 0;
    return mpStream.is_open();
}

void AggregateStats::CloseStream() {
    std::lock_guard<std::mutex> lock(mpMtxRows);
    if (mpStream.is_open()) mpStream.close();
}

void AggregateStats::CommitRow() {
    std::lock_guard<std::mutex> lock(mpMtxRows);
    if (mpStream.is_open()) {
        if (!mpHeaderWritten) WriteHeaderLocked();
        WriteRowLocked();
        mpRowsWrittenToFile++;
        if (mpRowsWrittenToFile % kFlushPeriod == 0) mpStream.flush();
    }
    mpRowCount++;
    if (mpRowCallback) mpRowCallback(*this);
    TrimLocked();
}

void AggregateStats::TrimLocked() {
    if (mpRetainedRows == 0) return;
    for (const std::string& name : order_) {
        Meta& meta = stats_.at(name);
        std::visit(
            [&](auto& data) {
                if (data.size() > mpRetainedRows)
                    data.erase(data.begin(),
                              data.end() - std::ptrdiff_t(mpRetainedRows));
            },
            meta.data_);
    }
}

void AggregateStats::SetRetainedRows(size_t rows) {
    std::lock_guard<std::mutex> lock(mpMtxRows);
    mpRetainedRows = rows;
}

void AggregateStats::SetBlankEmptyVectors(bool blank) {
    std::lock_guard<std::mutex> lock(mpMtxRows);
    mpBlankEmptyVectors = blank;
}

void AggregateStats::SetRowCallback(
    std::function<void(const AggregateStats&)> cb) {
    std::lock_guard<std::mutex> lock(mpMtxRows);
    mpRowCallback = std::move(cb);
}

const std::string& AggregateStats::Name() const { return mpName; }

size_t AggregateStats::RowCount() const {
    std::lock_guard<std::mutex> lock(mpMtxRows);
    return mpRowCount;
}

bool AggregateStats::SaveCsv(const std::string& path) const {
    std::lock_guard<std::mutex> lock(mpMtxRows);
    std::ofstream ofs(path, std::ios::out | std::ios::trunc);
    if (!ofs.is_open()) return false;

    const std::vector<std::string> names = ColumnNames();
    for (size_t i = 0; i < names.size(); i++) {
        if (i) ofs << ',';
        ofs << names[i];
    }
    ofs << '\n';

    size_t rows = 0;
    for (const std::string& name : order_)
        std::visit([&](const auto& d) { rows = std::max(rows, d.size()); },
                   stats_.at(name).data_);

    for (size_t r = 0; r < rows; r++) {
        bool first = true;
        for (const std::string& name : order_) {
            const Meta& meta = stats_.at(name);
            const std::string& fmt = meta.format_;
            std::visit(
                overload{
                    [&](const std::vector<double>& d) {
                        if (!first) ofs << ',';
                        first = false;
                        const double v = r < d.size() ? d[r] : 0.0;
                        if (fmt == "count" || fmt == "flag" || fmt == "enum")
                            ofs << int64_t(v);
                        else if (fmt == "ms")
                            ofs << v * 1e3;
                        else
                            ofs << v;
                    },
                    [&](const std::vector<Eigen::VectorXd>& d) {
                        const int width = fmt.rfind("vec", 0) == 0
                                              ? std::atoi(fmt.c_str() + 3)
                                              : 0;
                        const bool blank = mpBlankEmptyVectors &&
                                           r < d.size() && d[r].size() == 0;
                        for (int i = 0; i < width; i++) {
                            if (!first) ofs << ',';
                            first = false;
                            if (blank) continue;
                            ofs << (r < d.size() && i < d[r].size() ? d[r][i]
                                                                    : 0.0);
                        }
                    },
                    [&](const std::vector<int64_t>& d) {
                        if (!first) ofs << ',';
                        first = false;
                        ofs << (r < d.size() ? d[r] : int64_t(0));
                    }},
                meta.data_);
        }
        ofs << '\n';
    }
    return true;
}

// ═══════════════════════════════════════════════════════════════════
// Logger
// ═══════════════════════════════════════════════════════════════════

Logger::Logger(const std::string& outputDirectory, bool consoleEnabled)
    : mpEnabled(true),
      mpConsoleEnabled(consoleEnabled),
      mpOutputDirectory(outputDirectory) {
    Register("of_frame", "[Optical Flow]",
             {{"t_ns", "ns"}, {"frame", "count"}, {"dt", "ms"},
              {"in", "count"}, {"attempted", "count"}, {"tracked", "count"},
              {"fwd_fail", "count"}, {"bwd_fail", "count"},
              {"recov_rej", "count"}, {"survival", "ratio"},
              {"detected", "count"}, {"epi_rej", "count"}, {"out", "count"},
              {"flow_px_mean", "f"}, {"flow_px_max", "f"}});

    Register("of_timing", "[Optical Flow][timing]",
             {{"t_ns", "ns"}, {"pyramid", "ms"}, {"track", "ms"},
              {"detect_add", "ms"}, {"total", "ms"}, {"queue", "count"}});

    Register("vio_imu_ingest", "[VIO][imu-ingest]",
             {{"t_ns", "ns"}, {"n", "count"}, {"stamp_rate_hz", "f"},
              {"wall_rate_hz", "f"}, {"max_gap", "ms"},
              {"non_monotonic", "count"}, {"queue", "count"},
              {"accel", "vec3"}, {"gyro", "vec3"}});

    Register("vio_init", "[VIO][init]",
             {{"t_ns", "ns"}, {"imu_t_ns", "ns"}, {"imu_minus_frame", "ms"},
              {"imu_queue", "count"}, {"skipped_behind", "count"},
              {"accel", "vec3"}, {"accel_norm", "f"}, {"gyro", "vec3"},
              {"gyro_norm", "f"}, {"bg", "vec3"}, {"ba", "vec3"},
              {"g", "vec3"}, {"q_w_i", "vec4"}, {"tilt_deg", "f"}});

    Register("vio_imu_frame", "[VIO][imu]",
             {{"t_ns", "ns"}, {"frame_dt", "ms"}, {"integrated", "count"},
              {"expected", "count"}, {"skipped_behind", "count"},
              {"first_imu_ns", "ns"}, {"last_imu_ns", "ns"},
              {"coverage", "ratio"}, {"meas_dt", "ms"}, {"stretch", "flag"},
              {"stretch_span", "ms"}, {"imu_queue_in", "count"},
              {"imu_queue_out", "count"}, {"head_minus_frame", "ms"}});

    Register("vio_pose_update", "[VIO][pose-update]",
             {{"t_ns", "ns"}, {"pending", "count"}, {"applied", "count"},
              {"rejected", "count"}, {"max_trans_err", "f"},
              {"max_rot_err", "f"}, {"limit_trans", "f"},
              {"limit_rot", "f"}});

    Register("vio_assoc", "[VIO][assoc]",
             {{"t_ns", "ns"}, {"of_obs", "count"}, {"connected0", "count"},
              {"unconnected0", "count"}, {"connected_ratio", "ratio"},
              {"thresh", "f"}, {"frames_after_kf", "count"},
              {"lmdb_landmarks", "count"}, {"kfs", "count"},
              {"frame_states", "count"}, {"frame_poses", "count"},
              {"take_kf", "flag"}});

    Register("vio_triang", "[VIO][triang]",
             {{"t_ns", "ns"}, {"candidates", "count"}, {"added", "count"},
              {"no_prior_obs", "count"}, {"unproject_fail", "count"},
              {"short_baseline", "count"}, {"not_finite", "count"},
              {"behind", "count"}, {"too_close", "count"},
              {"min_triang_dist", "f"}, {"baseline_m_min", "f"},
              {"baseline_m_max", "f"}, {"depth_m_min", "f"},
              {"depth_m_mean", "f"}, {"depth_m_max", "f"}});

    Register("vio_landmarks", "[VIO][landmarks]",
             {{"t_ns", "ns"}, {"lost", "count"}, {"lmdb", "count"},
              {"marg_lost", "flag"}});

    Register("gt_eval", "[GT]",
             {{"t_ns", "ns"}, {"p_w_i", "vec3"}, {"q_w_i", "vec4"},
              {"p_gt", "vec3"}, {"q_gt", "vec4"}});
    mpAggregates.at("gt_eval").SetBlankEmptyVectors(true);

    Register("vio_state", "[VIO][state]",
             {{"t_ns", "ns"}, {"p_w_i", "vec3"}, {"p_norm", "f"},
              {"q_w_i", "vec4"}, {"v_w_i", "vec3"}, {"v_norm", "f"},
              {"bg", "vec3"}, {"bg_norm", "f"}, {"ba", "vec3"},
              {"ba_norm", "f"}});

    Register("vio_timing", "[VIO][timing]",
             {{"t_ns", "ns"}, {"pose_update", "ms"}, {"association", "ms"},
              {"triangulation", "ms"}, {"lost_scan", "ms"},
              {"opt_and_marg", "ms"}, {"publish", "ms"}, {"measure", "ms"},
              {"imu_drain", "ms"}, {"process_frame", "ms"},
              {"vision_queue", "count"}, {"imu_queue", "count"}});

    Register("vio_linearise", "[VIO][linearize]",
             {{"t_ns", "ns"}, {"iter", "count"}, {"error_total", "e"},
              {"lambda", "e"}, {"landmarks", "count"}, {"states", "count"},
              {"poses", "count"}, {"numerically_valid", "flag"}});

    Register("vio_solver_iter", "[VIO][solver]",
             {{"t_ns", "ns"}, {"iter", "count"}, {"inner", "count"},
              {"status", "enum"}, {"error_total", "e"}, {"f_diff", "e"},
              {"l_diff", "e"}, {"step_quality", "e"}, {"step_size", "e"},
              {"vision", "e"}, {"imu", "e"}, {"bias_g", "e"},
              {"bias_a", "e"}, {"marg_prior", "e"}, {"lambda", "e"},
              {"it_time", "ms"}, {"total_time", "ms"}});

    Register("vio_solver_summary", "[VIO][solver-summary]",
             {{"t_ns", "ns"}, {"num_it", "count"},
              {"num_it_rejected", "count"}, {"converged", "flag"},
              {"terminated", "flag"}, {"reason", "enum"},
              {"optimize", "ms"}});

    Register("vio_solver_timing", "[VIO][solver-timing]",
             {{"num_cams", "count"}, {"num_lms", "count"},
              {"num_obs", "count"}, {"allocateLMB", "ms"},
              {"linearizeProblem", "ms"}, {"performQR", "ms"},
              {"get_dense_H_b", "ms"}, {"solve", "ms"},
              {"backSubstitute", "ms"}, {"computerError2", "ms"},
              {"iteration", "ms"}, {"resident_memory", "f"},
              {"resident_memory_peak", "f"}});

    Register("vio_marg", "[VIO][marg]",
             {{"t_ns", "ns"}, {"states_to_remove", "count"},
              {"poses_to_marg", "count"}, {"states_to_marg", "count"},
              {"states_to_marg_vel_bias", "count"},
              {"kfs_to_marg", "count"}, {"kf_ids", "count"},
              {"keeping", "count"}, {"marg", "count"}, {"total", "count"},
              {"frame_poses", "count"}, {"frame_states", "count"},
              {"last_state_to_marg", "ns"}});

    Register("vio_marg_timing", "[VIO][marg-timing]",
             {{"frame_id", "ns"}, {"measure", "ms"},
              {"marg_linearize", "ms"}, {"marg_helper", "ms"},
              {"marg", "ms"}, {"marg_log", "ms"}, {"marginalize", "ms"}});

    Register("vio_marg_nullspace", "[VIO][marg-ns]",
             {{"t_ns", "ns"}, {"marg_ns", "vec7"}, {"ev_min", "e"},
              {"ev_max", "e"}, {"ev_negative", "count"},
              {"ev_condition", "e"}});

    Register("map_cycle_timing", "[Local Mapper][timing]",
             {{"t_ns", "ns"}, {"keyframes", "count"},
              {"marg_packets", "count"}, {"queue_wait", "ms"},
              {"ingest", "ms"}, {"detect_keypoints", "ms"},
              {"match_stereo", "ms"}, {"match_local", "ms"},
              {"collect_new_kfs", "ms"}, {"build_tracks", "ms"},
              {"setup_opt", "ms"}, {"cull", "ms"}, {"optimize_1", "ms"},
              {"filter_outliers", "ms"}, {"optimize_2", "ms"},
              {"publish_vis", "ms"}, {"pose_update_cb", "ms"},
              {"total", "ms"}});

    Register("map_bow_query", "[Local Mapper][bow]",
             {{"t_ns", "ns"}, {"kf", "ns"}, {"keypoints", "count"},
              {"descriptors", "count"}, {"bow_buckets", "count"},
              {"db_hits", "count"}, {"above_thresh", "count"},
              {"best_score", "f"}, {"thresh", "f"}});

    Register("map_pair_match", "[Local Mapper][match]",
             {{"kf1", "ns"}, {"kf2", "ns"},
              {"descriptor_matches", "count"}, {"min_matches", "count"},
              {"ransac_inliers", "count"}, {"stored", "flag"}});

    Register("map_match_summary", "[Local Mapper][match-summary]",
             {{"t_ns", "ns"}, {"pairs", "count"},
              {"neighbour_pairs", "count"}, {"neighbour_limit", "count"},
              {"verification_attempts", "count"},
              {"inlier_matches", "count"}, {"total_matches", "count"},
              {"db_query", "ms"}, {"matching", "ms"}});

    Register("map_tracks", "[Local Mapper][tracks]",
             {{"t_ns", "ns"}, {"exported", "count"},
              {"live_in_builder", "count"}, {"matches", "count"},
              {"new_kfs", "count"}, {"corners", "count"},
              {"len_hist", "vec10"}});

    Register("map_setup_opt", "[Local Mapper][setup_opt]",
             {{"t_ns", "ns"}, {"tracks", "count"}, {"short_track", "count"},
              {"known", "count"}, {"retired", "count"}, {"new_ok", "count"},
              {"no_triang", "count"}, {"no_corners", "count"},
              {"no_host_pose", "count"}, {"no_obs_pose", "count"},
              {"short_baseline", "count"}, {"low_parallax", "count"},
              {"bad_depth", "count"}, {"behind", "count"},
              {"too_close", "count"}, {"nan", "count"},
              {"obs_added", "count"}, {"lmdb_before", "count"},
              {"lmdb_after", "count"}, {"min_triang_dist", "f"},
              {"max_cos_parallax", "f"}, {"max_inv_dist", "f"}});

    Register("map_covis", "[Local Mapper][covis]",
             {{"t_ns", "ns"}, {"parity", "flag"}, {"frames", "count"},
              {"ref_frames", "count"}, {"cells", "count"},
              {"ref_cells", "count"}});

    Register("map_cull_select", "[Local Mapper][cull-select]",
             {{"t_ns", "ns"}, {"map", "count"}, {"eligible", "count"},
              {"keep_recent", "count"}, {"max_cull", "count"},
              {"min_observers", "count"}, {"selected", "count"},
              {"capacity_rule", "flag"}, {"thresh", "f"},
              {"observers_hist", "vec10"}});

    Register("map_rehost", "[Local Mapper][rehost]",
             {{"t_ns", "ns"}, {"lm", "ns"}, {"culled_host", "ns"},
              {"new_host", "ns"}, {"obs_before", "count"},
              {"obs_readded", "count"}, {"outcome", "enum"},
              {"reason", "enum"}});

    Register("map_cull", "[Local Mapper][cull]",
             {{"t_ns", "ns"}, {"kf", "ns"}, {"victims", "count"},
              {"hosted_lms", "count"}, {"rehost_attempted", "count"},
              {"no_host", "count"}, {"lm_before", "count"},
              {"lm_after", "count"}, {"map_before", "count"},
              {"map_after", "count"}});

    Register("map_filter", "[Local Mapper][filter]",
             {{"t_ns", "ns"}, {"lm_before", "count"}, {"lm_after", "count"},
              {"min_obs", "count"}, {"thresh", "f"}});

    Register("map_publish", "[Local Mapper][publish]",
             {{"t_ns", "ns"}, {"points", "count"}, {"keyframes", "count"},
              {"lmdb_landmarks", "count"}, {"host_kfs", "count"}});

    Register("mapper_linearise", "[Mapper][linearize]",
             {{"iter", "count"}, {"vision", "e"}, {"rel_error", "e"},
              {"roll_pitch_error", "e"}, {"total", "e"},
              {"landmarks", "count"}, {"observations", "count"},
              {"rel_pose_factors", "count"},
              {"roll_pitch_factors", "count"}});

    Register("mapper_solver_iter", "[Mapper][solver]",
             {{"iter", "count"}, {"inner", "count"}, {"status", "enum"},
              {"lambda", "e"}, {"f_diff", "e"}, {"max_inc", "e"},
              {"vision_error", "e"}, {"rel_error", "e"},
              {"roll_pitch_error", "e"}, {"total", "e"},
              {"converged", "flag"}, {"error_increased", "flag"}});

    Register("mapper_iter_summary", "[Mapper][iter]",
             {{"iter", "count"}, {"iteration", "ms"},
              {"num_states", "count"}, {"num_poses", "count"},
              {"inner_steps", "count"}});

    Register("mapper_detect", "[Mapper][detect]",
             {{"frames", "count"}, {"detection", "ms"},
              {"corners_total", "count"}});

    Register("mapper_stereo", "[Mapper][stereo]",
             {{"pairs", "count"}, {"matches", "count"},
              {"inliers", "count"}});
}

Logger::~Logger() = default;

Logger::Ptr Logger::Disabled() {
    static Ptr instance = [] {
        Ptr p = std::make_shared<Logger>();
        p->mpEnabled = false;
        p->mpConsoleEnabled = false;
        return p;
    }();
    return instance;
}

void Logger::Register(const std::string& name, const std::string& prefix,
                      const AggregateStats::Registry& registry) {
    auto [it, inserted] = mpAggregates.try_emplace(name, name);
    it->second.RegisterStats(registry);
    it->second.SetPrefixForLogging(prefix);
    if (!mpOutputDirectory.empty())
        it->second.OpenStream(mpOutputDirectory + "/" + name + ".csv");
}

void Logger::Emit(const std::string& name) {
    if (!IsConsoleEnabled()) return;
    const std::string line = mpAggregates.at(name).PrintLatest();
    // '\n' rather than std::endl deliberately, since std::endl flushes the
    // stream on every call and a flush is itself a synchronising operation.
    std::lock_guard<std::mutex> lock(mpMtxConsole);
    std::cout << line << '\n';
}

AggregateStats& Logger::Aggregate(const std::string& name) {
    return mpAggregates.at(name);
}

// ── Optical flow ────────────────────────────────────────────────────

void Logger::AddOpticalFlowFrame(int64_t tNs, int64_t frame, double dt,
                                 int in, int attempted, int tracked,
                                 int fwdFail, int bwdFail, int recovRej,
                                 int detected, int epiRej, int out,
                                 double flowMean, double flowMax) {
    if (!mpEnabled) return;
    AggregateStats& s = mpAggregates.at("of_frame");
    s.add_int("t_ns", tNs);
    s.add_int("frame", frame);
    s.add("dt", dt);
    s.add("in", double(in));
    s.add("attempted", double(attempted));
    s.add("tracked", double(tracked));
    s.add("fwd_fail", double(fwdFail));
    s.add("bwd_fail", double(bwdFail));
    s.add("recov_rej", double(recovRej));
    s.add("survival", in ? double(tracked) / double(in) : 0.0);
    s.add("detected", double(detected));
    s.add("epi_rej", double(epiRej));
    s.add("out", double(out));
    s.add("flow_px_mean", flowMean);
    s.add("flow_px_max", flowMax);
    s.CommitRow();
}

void Logger::PrintOpticalFlowFrame() { Emit("of_frame"); }

void Logger::AddOpticalFlowTiming(int64_t tNs, double pyramid, double track,
                                  double detectAdd, double total, int queue) {
    if (!mpEnabled) return;
    AggregateStats& s = mpAggregates.at("of_timing");
    s.add_int("t_ns", tNs);
    s.add("pyramid", pyramid);
    s.add("track", track);
    s.add("detect_add", detectAdd);
    s.add("total", total);
    s.add("queue", double(queue));
    s.CommitRow();
}

void Logger::PrintOpticalFlowTiming() { Emit("of_timing"); }

// ── Inertial ingestion ───────────────────────────────────────────────

void Logger::AddVioImuIngest(int64_t tNs, int64_t n, double stampRateHz,
                             double wallRateHz, double maxGap,
                             int64_t nonMonotonic, int queue,
                             const Eigen::Vector3d& accel,
                             const Eigen::Vector3d& gyro) {
    if (!mpEnabled) return;
    AggregateStats& s = mpAggregates.at("vio_imu_ingest");
    s.add_int("t_ns", tNs);
    s.add_int("n", n);
    s.add("stamp_rate_hz", stampRateHz);
    s.add("wall_rate_hz", wallRateHz);
    s.add("max_gap", maxGap);
    s.add_int("non_monotonic", nonMonotonic);
    s.add("queue", double(queue));
    s.add("accel", Eigen::VectorXd(accel));
    s.add("gyro", Eigen::VectorXd(gyro));
    s.CommitRow();
}

void Logger::PrintVioImuIngest() { Emit("vio_imu_ingest"); }

// ── Estimator ─────────────────────────────────────────────────────────

void Logger::AddVioInit(int64_t tNs, int64_t imuTNs, int64_t imuQueue,
                        int64_t skippedBehind, const Eigen::Vector3d& accel,
                        const Eigen::Vector3d& gyro,
                        const Eigen::Vector3d& bg, const Eigen::Vector3d& ba,
                        const Eigen::Vector3d& g, const Eigen::Vector4d& qWI,
                        double tiltDeg) {
    if (!mpEnabled) return;
    AggregateStats& s = mpAggregates.at("vio_init");
    s.add_int("t_ns", tNs);
    s.add_int("imu_t_ns", imuTNs);
    s.add("imu_minus_frame", double(imuTNs - tNs) * 1e-9);
    s.add_int("imu_queue", imuQueue);
    s.add_int("skipped_behind", skippedBehind);
    s.add("accel", Eigen::VectorXd(accel));
    s.add("accel_norm", accel.norm());
    s.add("gyro", Eigen::VectorXd(gyro));
    s.add("gyro_norm", gyro.norm());
    s.add("bg", Eigen::VectorXd(bg));
    s.add("ba", Eigen::VectorXd(ba));
    s.add("g", Eigen::VectorXd(g));
    s.add("q_w_i", Eigen::VectorXd(qWI));
    s.add("tilt_deg", tiltDeg);
    s.CommitRow();
}

void Logger::PrintVioInit() { Emit("vio_init"); }

void Logger::AddVioImuFrame(int64_t tNs, double frameDt, int integrated,
                            int expected, int skippedBehind,
                            int64_t firstImuNs, int64_t lastImuNs,
                            double coverage, double measDt, bool stretch,
                            double stretchSpan, int imuQueueIn,
                            int imuQueueOut, double headMinusFrame) {
    if (!mpEnabled) return;
    AggregateStats& s = mpAggregates.at("vio_imu_frame");
    s.add_int("t_ns", tNs);
    s.add("frame_dt", frameDt);
    s.add("integrated", double(integrated));
    s.add("expected", double(expected));
    s.add("skipped_behind", double(skippedBehind));
    s.add_int("first_imu_ns", firstImuNs);
    s.add_int("last_imu_ns", lastImuNs);
    s.add("coverage", coverage);
    s.add("meas_dt", measDt);
    s.add("stretch", stretch ? 1.0 : 0.0);
    s.add("stretch_span", stretchSpan);
    s.add("imu_queue_in", double(imuQueueIn));
    s.add("imu_queue_out", double(imuQueueOut));
    s.add("head_minus_frame", headMinusFrame);
    s.CommitRow();
}

void Logger::PrintVioImuFrame() { Emit("vio_imu_frame"); }

void Logger::AddVioPoseUpdate(int64_t tNs, int pending, int applied,
                              int rejected, double maxTransErr,
                              double maxRotErr, double limitTrans,
                              double limitRot) {
    if (!mpEnabled) return;
    AggregateStats& s = mpAggregates.at("vio_pose_update");
    s.add_int("t_ns", tNs);
    s.add("pending", double(pending));
    s.add("applied", double(applied));
    s.add("rejected", double(rejected));
    s.add("max_trans_err", maxTransErr);
    s.add("max_rot_err", maxRotErr);
    s.add("limit_trans", limitTrans);
    s.add("limit_rot", limitRot);
    s.CommitRow();
}

void Logger::PrintVioPoseUpdate() { Emit("vio_pose_update"); }

void Logger::AddVioAssoc(int64_t tNs, int ofObs, int connected0,
                         int unconnected0, double thresh, int framesAfterKf,
                         int lmdbLandmarks, int kfs, int frameStates,
                         int framePoses, bool takeKf) {
    if (!mpEnabled) return;
    AggregateStats& s = mpAggregates.at("vio_assoc");
    const int total = connected0 + unconnected0;
    s.add_int("t_ns", tNs);
    s.add("of_obs", double(ofObs));
    s.add("connected0", double(connected0));
    s.add("unconnected0", double(unconnected0));
    s.add("connected_ratio", total ? double(connected0) / double(total) : 0.0);
    s.add("thresh", thresh);
    s.add("frames_after_kf", double(framesAfterKf));
    s.add("lmdb_landmarks", double(lmdbLandmarks));
    s.add("kfs", double(kfs));
    s.add("frame_states", double(frameStates));
    s.add("frame_poses", double(framePoses));
    s.add("take_kf", takeKf ? 1.0 : 0.0);
    s.CommitRow();
}

void Logger::PrintVioAssoc() { Emit("vio_assoc"); }

void Logger::AddVioTriang(int64_t tNs, int candidates, int added,
                          int noPriorObs, int unprojectFail,
                          int shortBaseline, int notFinite, int behind,
                          int tooClose, double minTriangDist,
                          double baselineMin, double baselineMax,
                          double depthMin, double depthSum,
                          double depthMax) {
    if (!mpEnabled) return;
    AggregateStats& s = mpAggregates.at("vio_triang");
    s.add_int("t_ns", tNs);
    s.add("candidates", double(candidates));
    s.add("added", double(added));
    s.add("no_prior_obs", double(noPriorObs));
    s.add("unproject_fail", double(unprojectFail));
    s.add("short_baseline", double(shortBaseline));
    s.add("not_finite", double(notFinite));
    s.add("behind", double(behind));
    s.add("too_close", double(tooClose));
    s.add("min_triang_dist", minTriangDist);
    s.add("baseline_m_min", baselineMin);
    s.add("baseline_m_max", baselineMax);
    s.add("depth_m_min", depthMin);
    s.add("depth_m_mean", added ? depthSum / double(added) : 0.0);
    s.add("depth_m_max", depthMax);
    s.CommitRow();
}

void Logger::PrintVioTriang() { Emit("vio_triang"); }

void Logger::AddVioLandmarks(int64_t tNs, int lost, int lmdb, bool margLost) {
    if (!mpEnabled) return;
    AggregateStats& s = mpAggregates.at("vio_landmarks");
    s.add_int("t_ns", tNs);
    s.add("lost", double(lost));
    s.add("lmdb", double(lmdb));
    s.add("marg_lost", margLost ? 1.0 : 0.0);
    s.CommitRow();
}

void Logger::PrintVioLandmarks() { Emit("vio_landmarks"); }

void Logger::AddGtEval(int64_t tNs, const Eigen::VectorXd& pWI,
                       const Eigen::VectorXd& qWI, const Eigen::VectorXd& pGt,
                       const Eigen::VectorXd& qGt) {
    if (!mpEnabled) return;
    AggregateStats& s = mpAggregates.at("gt_eval");
    s.add_int("t_ns", tNs);
    s.add("p_w_i", pWI);
    s.add("q_w_i", qWI);
    s.add("p_gt", pGt);
    s.add("q_gt", qGt);
    s.CommitRow();
}

void Logger::AddVioState(int64_t tNs, const Eigen::Vector3d& pWI,
                         const Eigen::Vector4d& qWI,
                         const Eigen::Vector3d& vWI,
                         const Eigen::Vector3d& bg,
                         const Eigen::Vector3d& ba) {
    if (!mpEnabled) return;
    AggregateStats& s = mpAggregates.at("vio_state");
    s.add_int("t_ns", tNs);
    s.add("p_w_i", Eigen::VectorXd(pWI));
    s.add("p_norm", pWI.norm());
    s.add("q_w_i", Eigen::VectorXd(qWI));
    s.add("v_w_i", Eigen::VectorXd(vWI));
    s.add("v_norm", vWI.norm());
    s.add("bg", Eigen::VectorXd(bg));
    s.add("bg_norm", bg.norm());
    s.add("ba", Eigen::VectorXd(ba));
    s.add("ba_norm", ba.norm());
    s.CommitRow();
}

void Logger::PrintVioState() { Emit("vio_state"); }

void Logger::AddVioTiming(int64_t tNs, double poseUpdate, double association,
                          double triangulation, double lostScan,
                          double optAndMarg, double publish, double measure,
                          double imuDrain, double processFrame,
                          int visionQueue, int imuQueue) {
    if (!mpEnabled) return;
    AggregateStats& s = mpAggregates.at("vio_timing");
    s.add_int("t_ns", tNs);
    s.add("pose_update", poseUpdate);
    s.add("association", association);
    s.add("triangulation", triangulation);
    s.add("lost_scan", lostScan);
    s.add("opt_and_marg", optAndMarg);
    s.add("publish", publish);
    s.add("measure", measure);
    s.add("imu_drain", imuDrain);
    s.add("process_frame", processFrame);
    s.add("vision_queue", double(visionQueue));
    s.add("imu_queue", double(imuQueue));
    s.CommitRow();
}

void Logger::PrintVioTiming() { Emit("vio_timing"); }

void Logger::AddVioLinearise(int64_t tNs, int iter, double errorTotal,
                             double lambda, int landmarks, int states,
                             int poses, bool numericallyValid) {
    if (!mpEnabled) return;
    AggregateStats& s = mpAggregates.at("vio_linearise");
    s.add_int("t_ns", tNs);
    s.add("iter", double(iter));
    s.add("error_total", errorTotal);
    s.add("lambda", lambda);
    s.add("landmarks", double(landmarks));
    s.add("states", double(states));
    s.add("poses", double(poses));
    s.add("numerically_valid", numericallyValid ? 1.0 : 0.0);
    s.CommitRow();
}

void Logger::PrintVioLinearise() { Emit("vio_linearise"); }

void Logger::AddVioSolverIter(int64_t tNs, int iter, int inner, int status,
                              double errorTotal, double fDiff, double lDiff,
                              double stepQuality, double stepSize,
                              double vision, double imu, double biasG,
                              double biasA, double margPrior, double lambda,
                              double itTime, double totalTime) {
    if (!mpEnabled) return;
    AggregateStats& s = mpAggregates.at("vio_solver_iter");
    s.add_int("t_ns", tNs);
    s.add("iter", double(iter));
    s.add("inner", double(inner));
    s.add("status", double(status));
    s.add("error_total", errorTotal);
    s.add("f_diff", fDiff);
    s.add("l_diff", lDiff);
    s.add("step_quality", stepQuality);
    s.add("step_size", stepSize);
    s.add("vision", vision);
    s.add("imu", imu);
    s.add("bias_g", biasG);
    s.add("bias_a", biasA);
    s.add("marg_prior", margPrior);
    s.add("lambda", lambda);
    s.add("it_time", itTime);
    s.add("total_time", totalTime);
    s.CommitRow();
}

void Logger::PrintVioSolverIter() { Emit("vio_solver_iter"); }

void Logger::AddVioSolverSummary(int64_t tNs, int numIt, int numItRejected,
                                 bool converged, bool terminated, int reason,
                                 double optimize) {
    if (!mpEnabled) return;
    AggregateStats& s = mpAggregates.at("vio_solver_summary");
    s.add_int("t_ns", tNs);
    s.add("num_it", double(numIt));
    s.add("num_it_rejected", double(numItRejected));
    s.add("converged", converged ? 1.0 : 0.0);
    s.add("terminated", terminated ? 1.0 : 0.0);
    s.add("reason", double(reason));
    s.add("optimize", optimize);
    s.CommitRow();
}

void Logger::PrintVioSolverSummary() { Emit("vio_solver_summary"); }

void Logger::AddVioSolverTiming(int numCams, int numLms, int numObs,
                                double allocateLmb, double linearizeProblem,
                                double performQr, double getDenseHB,
                                double solve, double backSubstitute,
                                double computeError2, double iteration,
                                double residentMemory,
                                double residentMemoryPeak) {
    if (!mpEnabled) return;
    AggregateStats& s = mpAggregates.at("vio_solver_timing");
    s.add("num_cams", double(numCams));
    s.add("num_lms", double(numLms));
    s.add("num_obs", double(numObs));
    s.add("allocateLMB", allocateLmb);
    s.add("linearizeProblem", linearizeProblem);
    s.add("performQR", performQr);
    s.add("get_dense_H_b", getDenseHB);
    s.add("solve", solve);
    s.add("backSubstitute", backSubstitute);
    s.add("computerError2", computeError2);
    s.add("iteration", iteration);
    s.add("resident_memory", residentMemory);
    s.add("resident_memory_peak", residentMemoryPeak);
    s.CommitRow();
}

void Logger::PrintVioSolverTiming() { Emit("vio_solver_timing"); }

void Logger::AddVioMarg(int64_t tNs, int statesToRemove, int posesToMarg,
                        int statesToMarg, int statesToMargVelBias,
                        int kfsToMarg, int kfIds, int keeping, int marg,
                        int total, int framePoses, int frameStates,
                        int64_t lastStateToMarg) {
    if (!mpEnabled) return;
    AggregateStats& s = mpAggregates.at("vio_marg");
    s.add_int("t_ns", tNs);
    s.add("states_to_remove", double(statesToRemove));
    s.add("poses_to_marg", double(posesToMarg));
    s.add("states_to_marg", double(statesToMarg));
    s.add("states_to_marg_vel_bias", double(statesToMargVelBias));
    s.add("kfs_to_marg", double(kfsToMarg));
    s.add("kf_ids", double(kfIds));
    s.add("keeping", double(keeping));
    s.add("marg", double(marg));
    s.add("total", double(total));
    s.add("frame_poses", double(framePoses));
    s.add("frame_states", double(frameStates));
    s.add_int("last_state_to_marg", lastStateToMarg);
    s.CommitRow();
}

void Logger::PrintVioMarg() { Emit("vio_marg"); }

void Logger::AddVioMargNullspace(int64_t tNs, const Eigen::VectorXd& margNs,
                                 double evMin, double evMax, int evNegative,
                                 double evCondition) {
    if (!mpEnabled) return;
    AggregateStats& s = mpAggregates.at("vio_marg_nullspace");
    s.add_int("t_ns", tNs);
    s.add("marg_ns", margNs);
    s.add("ev_min", evMin);
    s.add("ev_max", evMax);
    s.add("ev_negative", double(evNegative));
    s.add("ev_condition", evCondition);
    s.CommitRow();
}

void Logger::PrintVioMargNullspace() { Emit("vio_marg_nullspace"); }

void Logger::AddVioMargTiming(int64_t frameId, double measure,
                              double margLinearize, double margHelper,
                              double marg, double margLog,
                              double marginalize) {
    if (!mpEnabled) return;
    AggregateStats& s = mpAggregates.at("vio_marg_timing");
    s.add_int("frame_id", frameId);
    s.add("measure", measure);
    s.add("marg_linearize", margLinearize);
    s.add("marg_helper", margHelper);
    s.add("marg", marg);
    s.add("marg_log", margLog);
    s.add("marginalize", marginalize);
    s.CommitRow();
}

void Logger::PrintVioMargTiming() { Emit("vio_marg_timing"); }

// ── Local mapper ──────────────────────────────────────────────────────

void Logger::AddMapCycleTiming(int64_t tNs, int keyframes, int margPackets,
                               double queueWait, double ingest,
                               double detectKeypoints, double matchStereo,
                               double matchLocal, double collectNewKfs,
                               double buildTracks, double setupOpt,
                               double cull, double optimize1,
                               double filterOutliers, double optimize2,
                               double publishVis, double poseUpdateCb,
                               double total) {
    if (!mpEnabled) return;
    AggregateStats& s = mpAggregates.at("map_cycle_timing");
    s.add_int("t_ns", tNs);
    s.add("keyframes", double(keyframes));
    s.add("marg_packets", double(margPackets));
    s.add("queue_wait", queueWait);
    s.add("ingest", ingest);
    s.add("detect_keypoints", detectKeypoints);
    s.add("match_stereo", matchStereo);
    s.add("match_local", matchLocal);
    s.add("collect_new_kfs", collectNewKfs);
    s.add("build_tracks", buildTracks);
    s.add("setup_opt", setupOpt);
    s.add("cull", cull);
    s.add("optimize_1", optimize1);
    s.add("filter_outliers", filterOutliers);
    s.add("optimize_2", optimize2);
    s.add("publish_vis", publishVis);
    s.add("pose_update_cb", poseUpdateCb);
    s.add("total", total);
    s.CommitRow();
}

void Logger::PrintMapCycleTiming() { Emit("map_cycle_timing"); }

void Logger::AddMapBowQuery(int64_t tNs, int64_t kf, int keypoints,
                            int descriptors, int bowBuckets, int dbHits,
                            int aboveThresh, double bestScore,
                            double thresh) {
    if (!mpEnabled) return;
    AggregateStats& s = mpAggregates.at("map_bow_query");
    s.add_int("t_ns", tNs);
    s.add_int("kf", kf);
    s.add("keypoints", double(keypoints));
    s.add("descriptors", double(descriptors));
    s.add("bow_buckets", double(bowBuckets));
    s.add("db_hits", double(dbHits));
    s.add("above_thresh", double(aboveThresh));
    s.add("best_score", bestScore);
    s.add("thresh", thresh);
    s.CommitRow();
}

void Logger::PrintMapBowQuery() { Emit("map_bow_query"); }

void Logger::AddMapPairMatch(int64_t kf1, int64_t kf2, int descriptorMatches,
                             int minMatches, int ransacInliers,
                             bool stored) {
    if (!mpEnabled) return;
    AggregateStats& s = mpAggregates.at("map_pair_match");
    s.add_int("kf1", kf1);
    s.add_int("kf2", kf2);
    s.add("descriptor_matches", double(descriptorMatches));
    s.add("min_matches", double(minMatches));
    s.add("ransac_inliers", double(ransacInliers));
    s.add("stored", stored ? 1.0 : 0.0);
    s.CommitRow();
}

void Logger::PrintMapPairMatch() { Emit("map_pair_match"); }

void Logger::AddMapMatchSummary(int64_t tNs, int pairs, int neighbourPairs,
                                int neighbourLimit, int verificationAttempts,
                                int inlierMatches, int totalMatches,
                                double dbQuery, double matching) {
    if (!mpEnabled) return;
    AggregateStats& s = mpAggregates.at("map_match_summary");
    s.add_int("t_ns", tNs);
    s.add("pairs", double(pairs));
    s.add("neighbour_pairs", double(neighbourPairs));
    s.add("neighbour_limit", double(neighbourLimit));
    s.add("verification_attempts", double(verificationAttempts));
    s.add("inlier_matches", double(inlierMatches));
    s.add("total_matches", double(totalMatches));
    s.add("db_query", dbQuery);
    s.add("matching", matching);
    s.CommitRow();
}

void Logger::PrintMapMatchSummary() { Emit("map_match_summary"); }

void Logger::AddMapTracks(int64_t tNs, int exported, int liveInBuilder,
                          int matches, int newKfs, int corners,
                          const Eigen::VectorXd& lenHist) {
    if (!mpEnabled) return;
    AggregateStats& s = mpAggregates.at("map_tracks");
    s.add_int("t_ns", tNs);
    s.add("exported", double(exported));
    s.add("live_in_builder", double(liveInBuilder));
    s.add("matches", double(matches));
    s.add("new_kfs", double(newKfs));
    s.add("corners", double(corners));
    s.add("len_hist", lenHist);
    s.CommitRow();
}

void Logger::PrintMapTracks() { Emit("map_tracks"); }

void Logger::AddMapSetupOpt(int64_t tNs, int tracks, int shortTrack,
                            int known, int retired, int newOk, int noTriang,
                            int noCorners, int noHostPose, int noObsPose,
                            int shortBaseline, int lowParallax, int badDepth,
                            int behind, int tooClose, int nan, int obsAdded,
                            int lmdbBefore, int lmdbAfter,
                            double minTriangDist, double maxCosParallax,
                            double maxInvDist) {
    if (!mpEnabled) return;
    AggregateStats& s = mpAggregates.at("map_setup_opt");
    s.add_int("t_ns", tNs);
    s.add("tracks", double(tracks));
    s.add("short_track", double(shortTrack));
    s.add("known", double(known));
    s.add("retired", double(retired));
    s.add("new_ok", double(newOk));
    s.add("no_triang", double(noTriang));
    s.add("no_corners", double(noCorners));
    s.add("no_host_pose", double(noHostPose));
    s.add("no_obs_pose", double(noObsPose));
    s.add("short_baseline", double(shortBaseline));
    s.add("low_parallax", double(lowParallax));
    s.add("bad_depth", double(badDepth));
    s.add("behind", double(behind));
    s.add("too_close", double(tooClose));
    s.add("nan", double(nan));
    s.add("obs_added", double(obsAdded));
    s.add("lmdb_before", double(lmdbBefore));
    s.add("lmdb_after", double(lmdbAfter));
    s.add("min_triang_dist", minTriangDist);
    s.add("max_cos_parallax", maxCosParallax);
    s.add("max_inv_dist", maxInvDist);
    s.CommitRow();
}

void Logger::PrintMapSetupOpt() { Emit("map_setup_opt"); }

void Logger::AddMapCovis(int64_t tNs, bool parity, int frames, int refFrames,
                         int cells, int refCells) {
    if (!mpEnabled) return;
    AggregateStats& s = mpAggregates.at("map_covis");
    s.add_int("t_ns", tNs);
    s.add("parity", parity ? 1.0 : 0.0);
    s.add("frames", double(frames));
    s.add("ref_frames", double(refFrames));
    s.add("cells", double(cells));
    s.add("ref_cells", double(refCells));
    s.CommitRow();
}

void Logger::PrintMapCovis() { Emit("map_covis"); }

void Logger::AddMapCullSelect(int64_t tNs, int map, int eligible,
                              int keepRecent, int maxCull, int minObservers,
                              int selected, bool capacityRule,
                              double thresh,
                              const Eigen::VectorXd& observersHist) {
    if (!mpEnabled) return;
    AggregateStats& s = mpAggregates.at("map_cull_select");
    s.add_int("t_ns", tNs);
    s.add("map", double(map));
    s.add("eligible", double(eligible));
    s.add("keep_recent", double(keepRecent));
    s.add("max_cull", double(maxCull));
    s.add("min_observers", double(minObservers));
    s.add("selected", double(selected));
    s.add("capacity_rule", capacityRule ? 1.0 : 0.0);
    s.add("thresh", thresh);
    s.add("observers_hist", observersHist);
    s.CommitRow();
}

void Logger::PrintMapCullSelect() { Emit("map_cull_select"); }

void Logger::AddMapRehost(int64_t tNs, int64_t lm, int64_t culledHost,
                          int64_t newHost, int obsBefore, int obsReadded,
                          int outcome, int reason) {
    if (!mpEnabled) return;
    AggregateStats& s = mpAggregates.at("map_rehost");
    s.add_int("t_ns", tNs);
    s.add_int("lm", lm);
    s.add_int("culled_host", culledHost);
    s.add_int("new_host", newHost);
    s.add("obs_before", double(obsBefore));
    s.add("obs_readded", double(obsReadded));
    s.add("outcome", double(outcome));
    s.add("reason", double(reason));
    s.CommitRow();
}

void Logger::PrintMapRehost() { Emit("map_rehost"); }

void Logger::AddMapCull(int64_t tNs, int64_t kf, int victims, int hostedLms,
                        int rehostAttempted, int noHost, int lmBefore,
                        int lmAfter, int mapBefore, int mapAfter) {
    if (!mpEnabled) return;
    AggregateStats& s = mpAggregates.at("map_cull");
    s.add_int("t_ns", tNs);
    s.add_int("kf", kf);
    s.add("victims", double(victims));
    s.add("hosted_lms", double(hostedLms));
    s.add("rehost_attempted", double(rehostAttempted));
    s.add("no_host", double(noHost));
    s.add("lm_before", double(lmBefore));
    s.add("lm_after", double(lmAfter));
    s.add("map_before", double(mapBefore));
    s.add("map_after", double(mapAfter));
    s.CommitRow();
}

void Logger::PrintMapCull() { Emit("map_cull"); }

void Logger::AddMapFilter(int64_t tNs, int lmBefore, int lmAfter, int minObs,
                          double thresh) {
    if (!mpEnabled) return;
    AggregateStats& s = mpAggregates.at("map_filter");
    s.add_int("t_ns", tNs);
    s.add("lm_before", double(lmBefore));
    s.add("lm_after", double(lmAfter));
    s.add("min_obs", double(minObs));
    s.add("thresh", thresh);
    s.CommitRow();
}

void Logger::PrintMapFilter() { Emit("map_filter"); }

void Logger::AddMapPublish(int64_t tNs, int points, int keyframes,
                           int lmdbLandmarks, int hostKfs) {
    if (!mpEnabled) return;
    AggregateStats& s = mpAggregates.at("map_publish");
    s.add_int("t_ns", tNs);
    s.add("points", double(points));
    s.add("keyframes", double(keyframes));
    s.add("lmdb_landmarks", double(lmdbLandmarks));
    s.add("host_kfs", double(hostKfs));
    s.CommitRow();
}

void Logger::PrintMapPublish() { Emit("map_publish"); }

// ── Shared mapping base ───────────────────────────────────────────────

void Logger::AddMapperLinearise(int iter, double vision, double relError,
                                double rollPitchError, double total,
                                int landmarks, int observations,
                                int relPoseFactors, int rollPitchFactors) {
    if (!mpEnabled) return;
    AggregateStats& s = mpAggregates.at("mapper_linearise");
    s.add("iter", double(iter));
    s.add("vision", vision);
    s.add("rel_error", relError);
    s.add("roll_pitch_error", rollPitchError);
    s.add("total", total);
    s.add("landmarks", double(landmarks));
    s.add("observations", double(observations));
    s.add("rel_pose_factors", double(relPoseFactors));
    s.add("roll_pitch_factors", double(rollPitchFactors));
    s.CommitRow();
}

void Logger::PrintMapperLinearise() { Emit("mapper_linearise"); }

void Logger::AddMapperSolverIter(int iter, int inner, int status,
                                 double lambda, double fDiff, double maxInc,
                                 double visionError, double relError,
                                 double rollPitchError, double total,
                                 bool converged, bool errorIncreased) {
    if (!mpEnabled) return;
    AggregateStats& s = mpAggregates.at("mapper_solver_iter");
    s.add("iter", double(iter));
    s.add("inner", double(inner));
    s.add("status", double(status));
    s.add("lambda", lambda);
    s.add("f_diff", fDiff);
    s.add("max_inc", maxInc);
    s.add("vision_error", visionError);
    s.add("rel_error", relError);
    s.add("roll_pitch_error", rollPitchError);
    s.add("total", total);
    s.add("converged", converged ? 1.0 : 0.0);
    s.add("error_increased", errorIncreased ? 1.0 : 0.0);
    s.CommitRow();
}

void Logger::PrintMapperSolverIter() { Emit("mapper_solver_iter"); }

void Logger::AddMapperIterSummary(int iter, double iteration, int numStates,
                                  int numPoses, int innerSteps) {
    if (!mpEnabled) return;
    AggregateStats& s = mpAggregates.at("mapper_iter_summary");
    s.add("iter", double(iter));
    s.add("iteration", iteration);
    s.add("num_states", double(numStates));
    s.add("num_poses", double(numPoses));
    s.add("inner_steps", double(innerSteps));
    s.CommitRow();
}

void Logger::PrintMapperIterSummary() { Emit("mapper_iter_summary"); }

void Logger::AddMapperDetect(int frames, double detection, int cornersTotal) {
    if (!mpEnabled) return;
    AggregateStats& s = mpAggregates.at("mapper_detect");
    s.add("frames", double(frames));
    s.add("detection", detection);
    s.add("corners_total", double(cornersTotal));
    s.CommitRow();
}

void Logger::PrintMapperDetect() { Emit("mapper_detect"); }

void Logger::AddMapperStereo(int pairs, int matches, int inliers) {
    if (!mpEnabled) return;
    AggregateStats& s = mpAggregates.at("mapper_stereo");
    s.add("pairs", double(pairs));
    s.add("matches", double(matches));
    s.add("inliers", double(inliers));
    s.CommitRow();
}

void Logger::PrintMapperStereo() { Emit("mapper_stereo"); }

// ── Legacy ExecutionStats artefacts ───────────────────────────────────

ExecutionStats& Logger::SolverScratch() { return mpSolverScratch; }

void Logger::FinishVioOptimize() {
    if (!mpEnabled) return;
    mpStatsAll.merge_all(mpSolverScratch);
    mpStatsSums.merge_sums(mpSolverScratch);
    mpSolverScratch = ExecutionStats();
}

void Logger::SaveLegacyStats() {
    if (!mpEnabled) return;
    const std::string dir =
        mpOutputDirectory.empty() ? std::string(".") : mpOutputDirectory;
    mpStatsAll.save_json(dir + "/stats_all.json");
    mpStatsSums.save_json(dir + "/stats_sums.json");
}

void Logger::PrintSummary() {
    if (!mpEnabled) return;
    std::cout << "=== stats all ===\n";
    mpStatsAll.print();
    std::cout << "=== stats sums ===\n";
    mpStatsSums.print();
}

void Logger::SaveAll() {
    if (!mpEnabled) return;
    for (auto& kv : mpAggregates) kv.second.CloseStream();
}

}  // namespace basalt
