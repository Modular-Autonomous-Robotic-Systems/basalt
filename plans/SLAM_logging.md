# Centralised SLAM Logging and Data Aggregation

## Introduction

Basalt emits its diagnostic record through free standing `std::cout` statements scattered across the optical flow front end, the visual inertial estimator, the local mapper and the shared mapping base class. Each site repeats the same three concerns, it evaluates a debug predicate, it assembles a line by stream insertion, and it writes that line to a stream shared with every other thread in the process. The record that results is the primary evidence in every investigation this repository has run, including the divergence analysis in [`vio_divergence_fix.md`](vio_divergence_fix.md) and the drift analysis in [`vio_drift_analysis.md`](vio_drift_analysis.md), yet it is recovered only by regular expression parsing of interleaved console text.

This document specifies a logging subsystem that separates the act of recording a measurement from the act of rendering it. Two new files, `include/basalt/utils/logger.h` and `src/utils/logger.cpp`, introduce `class AggregateStats`, which extends the existing `class ExecutionStats`, and `class Logger`, which owns one aggregate per class of measurement and offers a named method for each. The pipeline classes receive the logger by constructor injection from `Controller::initialize`. Every quantity currently printed is registered, every print site is replaced by a call that records the row and a separate call that optionally renders it, and the rendered text is serialised against a single mutex. Each aggregate writes a comma separated file whose columns are the registered statistics, which removes the parsing layer entirely.

The subsystem is designed so that a later addition, publishing the same rows on ROS2 topics for real time health monitoring, requires a callback registration rather than a second traversal of the pipeline.

## Motivation

### The debug predicate is evaluated at every site

`config.vio_debug` is tested independently at twenty nine sites in `src/vi_estimator/sqrt_keypoint_vio.cpp`, at five sites in `include/basalt/optical_flow/frame_to_frame_optical_flow.h` and, as `mpVioDebugMode`, at forty two sites in `src/vi_estimator/local_mapper.cpp`. The predicate governs two different things that happen to coincide today, whether a value is computed and whether it is printed. Conflating them is what makes the instrumentation expensive to keep. The covisibility parity check at `src/vi_estimator/local_mapper.cpp:937-946` recomputes `BuildObservedSets` and `BuildCovisibilityMatrix` from nothing purely so that the printed line can compare them, an operation quadratic in the number of live keyframes, while the flow magnitude loop at `include/basalt/optical_flow/frame_to_frame_optical_flow.h:234-244` is linear in the track count and would be affordable unconditionally. A single predicate cannot express that distinction.

### Console output from three threads interleaves

With `useProducerConsumerArchitecture = false`, established in [`../context/vio_optical_flow_instrumentation.md`](../context/vio_optical_flow_instrumentation.md), the front end and the estimator run on the image callback thread, `Controller::GrabIMU` runs on the inertial callback thread, and `LocalMapper::MapLocally` runs on its own thread. Three writers share `std::cout`, and `operator<<` offers no atomicity across a chain of insertions. The consequence is measured rather than hypothetical. The extraction script `scripts/vio_log_extract.py` carries four distinct defences against it, and on the 2,002 frame capture of 2026-09-12 it rejected 699 records as corrupted. One splice inflated the reported maximum of `opt_and_marg` from a true 18.25 ms to 52,034 ms, a figure that would have redirected the whole investigation had it not been caught.

### The analysis path is brittle by construction

`scripts/vio_log_extract.py` reconstructs structured records from prose. It delimits fields by the position of the next key rather than by whitespace, because Eigen prints a vector as space separated components. It tracks a bracket group prefix so that `baseline_m[min=.. max=..]` does not collide with `depth_m[min=.. max=..]`. It knows the final field each record type writes so that a silently truncated line can be detected. It arbitrates timestamps against a short frame marker because a corrupted record can carry foreign digits spliced into its own timestamp. Every one of those behaviours exists to undo damage that a structured writer would never inflict, and each is a dependency of the analysis on the exact punctuation of a print statement.

### Nanosecond timestamps do not survive the current storage

`ExecutionStats::add` stores every scalar as `double`. `src/vi_estimator/sqrt_keypoint_vio.cpp:452` already passes an `int64_t` nanosecond timestamp through that path. A `double` represents integers exactly only below two to the fifty third, which is 9.007e15 ns, or 104 days of clock. Simulation time near 1.4e12 ns is safe, but the capture recorded in [`../context/ap_dds_imu_stream.md`](../context/ap_dds_imu_stream.md) carried inertial stamps on a UTC epoch near 1.787e18 ns, where the representable granularity is 256 ns. Since the timestamp is the join key across every record type, it must be stored as an integer.

### There is no single point at which the state of the system is known

Publishing SLAM health on a ROS2 topic, correlating front end survival against estimator cost within one frame, or comparing the estimator's triangulation tally against the mapper's, all require the per frame record to exist as data rather than as text. No such object exists today. Each thread formats and discards.

### What the current implementation does well and must be preserved

`class ExecutionStats` in `include/basalt/utils/time_utils.hpp:108-139` already solves the storage problem for one family of measurement. It holds a named, insertion ordered map of samples, it tags each with a format string, it merges two collections either sample wise or as sums, and it serialises to JSON and UBJSON. The estimator uses it for solver timing through `stats_all_` and `stats_sums_`, declared at `include/basalt/vi_estimator/sqrt_keypoint_vio.h:275-276`. Those two objects are not internal. `python/basalt/log.py:93-95` loads `stats_all`, `stats_sums` and `stats_vio`, `python/basalt/run.py:107-112` collects the six resulting files, and `scripts/batch/generate-tables.py` builds the batch evaluation tables from them, a workflow documented in `doc/BatchEvaluation.md`. The new subsystem therefore extends `ExecutionStats` rather than displacing it, and continues to emit `stats_all.json` and `stats_sums.json` with the key names and formats they carry today.

## Architecture Design

### The central idea

Every diagnostic line in the pipeline is a row of a table. The columns of that table are fixed by the site that emits it, the rows arrive one per frame, per keyframe, per solver iteration or per culled landmark, and the only thing that varies between two emissions of the same site is the values. Recognising a print statement as a row means a site needs to declare its columns once, record its values once, and leave rendering and persistence to a component that owns both sinks.

`class AggregateStats` is that table. It inherits the storage, ordering, merging and JSON serialisation of `class ExecutionStats` and adds three things it lacks, a registry that binds each column to a format so that no call site repeats a format string, a renderer that turns the most recently added row into one line of text, and a comma separated writer whose header is the registered column set. `class Logger` owns one `AggregateStats` per site, registers all of them in its constructor, and exposes one named recording method and one named rendering method per site so that call sites remain type checked and self documenting.

### Class hierarchy

```text
                    basalt::ExecutionStats                (time_utils.hpp, extended)
                    ├── struct Meta
                    │     ├── data_  : variant< vector<double>,
                    │     │                     vector<Eigen::VectorXd>,
                    │     │                     vector<int64_t> >        <- new arm
                    │     └── format_: string
                    ├── virtual add(name, double)      -> Meta&          <- made virtual
                    ├── virtual add(name, VectorXd)    -> Meta&          <- made virtual
                    ├── virtual add(name, VectorXf)    -> Meta&          <- made virtual
                    ├── virtual add_int(name, int64_t) -> Meta&          <- new, not an overload
                    ├── merge_all / merge_sums / print / save_json
                    └── protected: stats_, order_                        <- was private
                              ▲
                              │ public inheritance
                    basalt::AggregateStats                (logger.h, new)
                    ├── register_stats(map<string,string>)
                    ├── set_prefix_for_logging(string)
                    ├── add(...) overrides, apply the registered format
                    ├── print_latest()        -> string
                    ├── print_latest(prefix)  -> string
                    ├── save_csv(path)        -> bool
                    ├── OpenStream / CommitRow / CloseStream
                    ├── SetRetainedRows(size_t)
                    └── SetRowCallback(function<void(const AggregateStats&)>)
                              ▲
                              │ composition, one instance per record class
                    basalt::Logger                        (logger.h, new)
                    ├── ctor(outputDirectory, consoleEnabled)
                    │     registers all 35 aggregates, formats and prefixes
                    ├── AddOpticalFlowFrame(...)      PrintOpticalFlowFrame()
                    ├── AddVioState(...)              PrintVioState()
                    ├── ... one pair per aggregate ...
                    ├── mpStatsAll, mpStatsSums  : ExecutionStats   (legacy artefacts)
                    ├── SaveAll() / PrintSummary() / SaveLegacyStats()
                    └── static Disabled() -> Ptr                    (null object)
```

### Pipeline

```text
  image cb thread            imu cb thread           local mapper thread
  ───────────────            ─────────────           ───────────────────
  FrameToFrameOpticalFlow    Controller::GrabIMU     LocalMapper::MapLocally
        │                          │                        │
        │ AddOpticalFlowFrame      │ AddVioImuIngest        │ AddMapperCycleTiming
        │ AddOpticalFlowTiming     │                        │ AddMapperSetupOpt
        ▼                          │                        │ AddMapperCullSelect
  SqrtKeypointVioEstimator         │                        │ ...
        │ AddVioImuFrame           │                        │
        │ AddVioAssoc              │                        ▼
        │ AddVioTriangulation      │                  NfrMapper::optimize
        │ AddVioState              │                        │ AddMapperSolverIter
        │ AddVioSolverIteration    │                        │
        ▼                          ▼                        ▼
        └──────────────────┬───────┴────────────────────────┘
                           │
                    basalt::Logger            (one instance, shared_ptr)
                           │
         ┌─────────────────┼──────────────────────┬────────────────────┐
         ▼                 ▼                      ▼                    ▼
   AggregateStats     AggregateStats  ...   mpStatsAll /          row callback
   "of_frame"         "vio_state"          mpStatsSums           (unset today,
         │                 │                      │               ROS2 tap later)
         │ CommitRow       │ CommitRow            │
         ▼                 ▼                      ▼
   of_frame.csv      vio_state.csv          stats_all.json
                                            stats_sums.json
         └─────────── print_latest() ────────────┘
                           │
                    mpMtxConsole guarded
                           ▼
                       std::cout
```

### The four decisions that shape the design

Recording is separated from rendering. `Add*` always stores the row, and `Print*` renders it only when the console sink is enabled. That is what lets the per landmark rehost record, which reached 2,891 attempts across the 2026-09-08 run and is recorded in the comment at `src/vi_estimator/local_mapper.cpp:1172-1177`, exist in the comma separated file without flooding the console. It is also what makes the console predicate a property of the logger rather than a predicate repeated at every site.

Rows are streamed rather than dumped. `BASALT_ASSERT` is compiled into this build, established in [`../context/vio_optical_flow_instrumentation.md`](../context/vio_optical_flow_instrumentation.md), so a failing invariant terminates the process by `std::abort`. A logger that accumulates in memory and writes at shutdown loses its entire record in exactly the case the record is most needed. Each aggregate therefore appends its row on `CommitRow` and flushes on a fixed period, and `SetRetainedRows` bounds the in memory copy so that a multi hour run does not grow without limit. `save_csv` is retained for a caller that wants a single dump of whatever is held.

The logger is never null at a call site. `Logger::Disabled()` returns a shared instance whose `Add*` methods return immediately, so `NfrMapper`, which `src/mapper.cpp:566` and `src/mapper_sim.cpp:163` construct directly with no logger, needs no null guard anywhere and no change at those two call sites.

Column order comes from insertion, not from the registry. `std::map<std::string, std::string>`, which the registry signature uses, is ordered by key rather than by declaration, so it cannot carry column order. `ExecutionStats::order_` already records the order in which names were first added, and since each `Logger::Add*` method adds its fields in a fixed sequence, that order is deterministic and is the column order of the file. The header is therefore written on the first `CommitRow` rather than at open time.

### What the format string means

Every duration is stored in seconds and declared `ms`, which matches the existing convention in `ExecutionStats::print` at `src/utils/time_utils.cpp:118-120`. The vocabulary is extended as follows.

| Token | Storage | Console rendering | Column in the file |
|---|---|---|---|
| `count` | `double`, integral valued | integer | integer |
| `flag` | `double`, zero or one | `0` or `1` | integer |
| `ns` | `int64_t` | integer | integer |
| `ms` | `double`, seconds | value times `1e3`, three decimals | value times `1e3` |
| `ratio` | `double` | four decimals | raw |
| `f` | `double` | four significant figures | raw |
| `e` | `double` | scientific, four significant figures | raw |
| `enum` | `double`, integral code | integer | integer |
| `vecN` | `Eigen::VectorXd` of width `N` | components, space separated | `N` columns named `name_0` to `name_{N-1}` |
| `none` | any | omitted from the line, still stored | present |

`class ExecutionStats` holds no string channel, so the four sites that currently print text are encoded. The solver step status becomes `enum` with `0` accepted, `1` rejected and `2` backtracking. The solver termination message becomes `enum` with `0` converged, `1` small step, `2` maximum iterations, `3` maximum damping. The covisibility parity result becomes `flag`. The list of selected keyframe identifiers in `[cull-select]` has no fixed width, so the count is stored and the identifiers are rendered to the console only.

### Thread safety

Each `AggregateStats` guards `add`, `add_int`, `CommitRow`, `print_latest` and `save_csv` with its own `std::mutex`. In practice each aggregate has exactly one writer thread, so the lock is uncontended and costs a few nanoseconds against the tens of microseconds a frame already spends, but holding it removes an invariant that a future contributor could violate silently. `Logger` holds a separate `mpMtxConsole` taken only around the write of a finished line to `std::cout`, which is the lock that actually eliminates the interleaving. The two are never nested in a cycle, `Print*` takes the aggregate lock, releases it, then takes the console lock.

### What the field does instead, and why this design does not follow it

A single mutex around the console write is the simplest correct answer to the interleaving problem, and simplest is not automatically right, so the question of what a high throughput logger would do instead was researched directly rather than assumed. Four architectures were examined, spanning the range from the conservative end of the design space to the aggressive end, and each is grounded below against its own documentation and source rather than against recollection.

`spdlog`, the library nearest to this one in spirit, offers two tiers. Its default tier is synchronous, and a sink templated on `base_sink<std::mutex>` takes that one mutex only around `sink_it_`, the call that performs the write, with formatting done by the caller beforehand and outside the lock, exactly the shape `Logger::Emit` already has [4. Sinks, gabime/spdlog wiki](https://github.com/gabime/spdlog/wiki/4.-Sinks). Its second tier is asynchronous, where the formatted message is pushed onto a shared `mpmc_blocking_queue`, a bounded circular buffer sized 8,192 by default, and a small pool of background threads, one worker by default, drains it and performs the write, so every calling thread is decoupled from the write entirely and the mutex disappears from the hot path at the cost of a copy into the queue and a configurable overflow policy for when it fills [Asynchronous logging, gabime/spdlog wiki](https://github.com/gabime/spdlog/wiki/Asynchronous-logging).

`Quill` and Stanford PlatformLab's `NanoLog` push the same idea further. Each caller thread owns its own lock free single producer single consumer queue, so two callers never contend with each other at all, only ever with the one background thread that eventually drains that particular queue. `Quill` binary serialises the raw arguments into the queue and defers the `{fmt}` formatting itself to the single backend thread, which also merges the per thread queues in timestamp order before writing [Overview, Quill v12.1.0 documentation](https://quillcpp.readthedocs.io/en/latest/overview.html). `NanoLog` goes one step further again and defers formatting past the process entirely, to an offline post processing binary, having first stripped every compile time constant, the format string, the file, the line, the severity, out of the hot path at compile time, leaving the runtime call to copy only the arguments that actually vary, which is how it reaches a measured 80 million log calls per second at a 7 nanosecond median latency [NanoLog README, PlatformLab/NanoLog](https://github.com/PlatformLab/NanoLog/blob/master/README.md).

`Log4j2`'s asynchronous logger and the `LMAX Disruptor` it is built on are the ring buffer end of the spectrum. A disruptor is a preallocated, fixed size, circular array, 262,144 slots by default in `Log4j2`, in which a producer claims a slot with an atomic increment of a shared sequence counter rather than a lock, writes into that slot under the single writer principle, meaning that slot is never touched by another thread until the consumer has read it, and a background thread trails behind reading the same sequence counter to know what is ready [Asynchronous loggers, Apache Log4j manual](https://logging.apache.org/log4j/2.x/manual/async.html), [Disruptor (software), Wikipedia](<https://en.wikipedia.org/wiki/Disruptor_(software)>). `moodycamel::ConcurrentQueue`, the queue `spdlog`'s async tier is built on, reaches a similar result by a different route, emulating one multi producer multi consumer queue as a collection of near lock free single producer sub-queues internally, which is the same insight the disruptor's single writer principle expresses more explicitly [Detailed design of a lock free queue, moodycamel.com](https://moodycamel.com/blog/2014/detailed-design-of-a-lock-free-queue).

Two lower rungs of the same ladder are worth naming because they show what happens when the mutex is skipped rather than shrunk. `glog` locks a single mutex across every severity level for the duration of a write, which is the least sophisticated design surveyed and is the one this document's own baseline most resembles before the console lock was added [Overview, Google Logging Library](https://google.github.io/glog/0.7.1/). More pointedly, `rcutils`, the console logging layer underneath every `rclcpp` node in this workspace, documents its own default output handler as explicitly not thread safe, and a tracked issue records that even the read path of the per logger level cache lost its thread safety in a later change and has not regained it [Make logging functionality truly thread safe, ros2/rcutils#397](https://github.com/ros2/rcutils/issues/397). The stack this pipeline already runs on top of, in other words, offers less protection than `Logger::Emit` provides today.

Every one of the four sophisticated designs shares one structural idea, that the calling thread should copy data and return, and that formatting and the write itself belong to a different thread entirely. Where they differ is only in how much is copied and how far downstream the expensive part is pushed, a shared queue with a thread pool for `spdlog`, one queue per caller thread for `Quill`, the same again plus deferred formatting for `NanoLog`, and a preallocated ring buffer with sequence numbers instead of a queue at all for the `Disruptor`.

Two things specific to this pipeline argue against adopting any of that machinery now, rather than a general reluctance to add complexity. The measured median frame rate is `18.52` Hz in flight 1 and `19.61` Hz in flight 2, recorded in [`vio_drift_analysis.md:384`](vio_drift_analysis.md), with keyframes at roughly a third of that, `3.00` and `3.08` frames per keyframe, recorded at [`vio_drift_analysis.md:326`](vio_drift_analysis.md), so the sustained row rate across every aggregate together is in the low hundreds per second, five to six orders of magnitude below the tens of millions of calls per second `NanoLog` was built for and below the sustained millions per second the `Disruptor` and `Quill` target, and at that gap the lock is not the bottleneck, the `std::ofstream` write already dominates it as the plan's own thread safety paragraph above states. And every one of the four designs is durability optional, `NanoLog` explicitly moves formatting to an offline step that only ever runs after the process has exited cleanly, `Quill` and `spdlog`'s async tier hold a message in a queue for an interval during which a crash loses it, which is precisely the failure this design's row per commit choice, argued earlier in this section under `CommitRow`, exists to rule out on a build where `BASALT_ASSERT` calls `std::abort`. Adopting a background thread architecture here would trade a mutex that costs nanoseconds and is already outside the formatting path for a new class of failure, an unread queue at the moment of a crash, in exchange for headroom this pipeline is nowhere near needing.

The decision is therefore to keep the design as specified, and to take only the one concrete lesson the survey yields that costs nothing to apply, which is that a flush is itself a synchronising system call and should not ride on every line. `Logger::Emit`'s `std::cout << line << std::endl` becomes `std::cout << line << '\n'`, matching the periodic flush `AggregateStats::CommitRow` already gives the file sink through `kFlushPeriod`. Should a future sensor or a future deployment of `class Logger` outside this pipeline need the throughput the survey's upper tier targets, `AggregateStats::SetRowCallback` is already the seam for it, since it receives a complete row under the aggregate's own lock, and the concrete path at that point is to adopt `Quill` rather than to hand roll a ring buffer, since it is MIT licensed, header only and already C++17, matching this project's `CMAKE_CXX_STANDARD` at `CMakeLists.txt:96`, not to reimplement it.

## Implementation Plan

### Change 1, extend `class ExecutionStats`

#### Need

`AggregateStats` must read the stored samples in order to render a line and to write a file, and `ExecutionStats::stats_` and `ExecutionStats::order_` are private at `include/basalt/utils/time_utils.hpp:136-138`. It must also intercept every `add` so that the registered format is applied, including the `add` calls that `merge_all` issues internally at `src/utils/time_utils.cpp:63-70`, which non virtual name hiding would bypass. Finally the nanosecond timestamp needs an integer channel.

#### Code snippet

```cpp
class ExecutionStats {
 public:
  virtual ~ExecutionStats() = default;

  struct Meta {
    inline Meta& format(const std::string& s) {
      format_ = s;
      return *this;
    }

    // overwrite the meta data from another object
    void set_meta(const Meta& other) { format_ = other.format_; }

    std::variant<std::vector<double>, std::vector<Eigen::VectorXd>,
                 std::vector<int64_t>>
        data_;

    std::string format_;
  };

  virtual Meta& add(const std::string& name, double value);
  virtual Meta& add(const std::string& name, const Eigen::VectorXd& value);
  virtual Meta& add(const std::string& name, const Eigen::VectorXf& value);

  // Integer channel for nanosecond timestamps. Deliberately not an overload of
  // add, because int converts to double and to int64_t at the same rank and
  // every existing add("num_it", int) call site would become ambiguous.
  virtual Meta& add_int(const std::string& name, int64_t value);

  void merge_all(const ExecutionStats& other);
  void merge_sums(const ExecutionStats& other);
  void print() const;
  bool save_json(const std::string& path) const;

 protected:
  std::unordered_map<std::string, Meta> stats_;
  std::vector<std::string> order_;
};
```

The implementation in `src/utils/time_utils.cpp` gains `add_int`, which mirrors the existing `add` bodies onto the third variant arm.

```cpp
ExecutionStats::Meta& ExecutionStats::add_int(const std::string& name,
                                              int64_t value) {
  auto [it, new_item] = stats_.try_emplace(name);
  if (new_item) {
    order_.push_back(name);
    it->second.data_ = std::vector<int64_t>();
  }
  std::get<std::vector<int64_t>>(it->second.data_).push_back(value);
  return it->second;
}
```

`merge_all` currently visits with a generic lambda that calls `add(name, v)` for every element, which would not compile for `int64_t` once the arm exists. It becomes an explicit three arm overload so that the integer channel routes to `add_int`.

```cpp
void ExecutionStats::merge_all(const ExecutionStats& other) {
  for (const auto& name : other.order_) {
    const auto& meta = other.stats_.at(name);
    std::visit(overload{[&](const std::vector<double>& data) {
                          for (double v : data) add(name, v);
                        },
                        [&](const std::vector<Eigen::VectorXd>& data) {
                          for (const auto& v : data) add(name, v);
                        },
                        [&](const std::vector<int64_t>& data) {
                          for (int64_t v : data) add_int(name, v);
                        }},
               meta.data_);
    stats_.at(name).set_meta(meta);
  }
}
```

`merge_sums` gains a third arm that sums the integer channel and adds it back through `add_int`. `print` gains a third arm that reports the count, matching the treatment the vector arm already receives at `src/utils/time_utils.cpp:107-110`. `save_json` gains a third arm that writes the integer vector directly, which `nlohmann::json` accepts without conversion. The `overload` helper already exists in the anonymous namespace at `src/utils/time_utils.cpp:78-88`, so `merge_all` only needs to start using it. Note that `merge_all` is defined above that anonymous namespace today and must be moved below it, or the helper moved above, for the explicit overload form to compile.

#### Significance

Making `add` virtual costs one indirect call per recorded sample. At roughly ninety samples per frame and twenty frames per second that is 1,800 virtual dispatches per second, which is not measurable against the 9 ms median that `opt_and_marg` already spends. Making `stats_` and `order_` protected is a source compatible widening of access, no existing expression changes meaning.

#### Backwards compatibility impact

The three existing `add` overloads keep their signatures, their return type and their behaviour, so `src/vio.cpp:638-646`, `src/vi_estimator/sqrt_keypoint_vo.cpp` and the three `log_problem_stats` implementations are unaffected. Adding a virtual destructor and virtual methods changes the ABI of `ExecutionStats` by introducing a vtable pointer, which matters only if a prebuilt object file were linked against a newly built header. Everything in this repository is rebuilt from source by the same `colcon build`, so the change is contained. The `std::variant` gains a third alternative, which is a compile time change only. Every `std::visit` site is in `src/utils/time_utils.cpp` and each is updated in the same edit, and a missed site is a compile error rather than a silent fault.

### Change 2, `include/basalt/utils/logger.h`

#### Need

The header is public and is included by `include/basalt/optical_flow/frame_to_frame_optical_flow.h`, `include/basalt/vi_estimator/sqrt_keypoint_vio.h`, `include/basalt/vi_estimator/nfr_mapper.h` and `include/basalt/controller.h`. It must therefore depend only on `time_utils.hpp` and the standard library, because `nlohmann::json` and `fmt::fmt` are linked `PRIVATE` at `CMakeLists.txt:402-404` and are not available to consumers of the public interface.

#### Code snippet

```cpp
#pragma once

#include <basalt/utils/time_utils.hpp>

#include <fstream>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace basalt {

/// One table of diagnostic measurements. Columns are declared once through
/// RegisterStats, rows are appended through the inherited add methods and
/// closed with CommitRow, and the two sinks are a rendered line and a comma
/// separated file.
class AggregateStats : public ExecutionStats {
public:
    using Registry = std::map<std::string, std::string>;

    AggregateStats() = default;
    explicit AggregateStats(std::string name);
    ~AggregateStats() override;

    AggregateStats(const AggregateStats&) = delete;
    AggregateStats& operator=(const AggregateStats&) = delete;

    void RegisterStats(const Registry& registry);
    void SetPrefixForLogging(const std::string& prefix);

    Meta& add(const std::string& name, double value) override;
    Meta& add(const std::string& name, const Eigen::VectorXd& value) override;
    Meta& add(const std::string& name, const Eigen::VectorXf& value) override;
    Meta& add_int(const std::string& name, int64_t value) override;

    std::string PrintLatest() const;
    std::string PrintLatest(const std::string& prefix) const;

    bool SaveCsv(const std::string& path) const;

    bool OpenStream(const std::string& path);
    void CommitRow();
    void CloseStream();

    void SetRetainedRows(size_t rows);
    void SetRowCallback(std::function<void(const AggregateStats&)> cb);

    const std::string& Name() const;
    size_t RowCount() const;

private:
    std::string FormatFor(const std::string& name) const;
    std::string RenderValue(const std::string& name, const Meta& meta) const;
    std::vector<std::string> ColumnNames() const;
    void WriteHeaderLocked();
    void WriteRowLocked();
    void TrimLocked();

    Registry mpRegistry;
    std::string mpPrefix;
    std::string mpName;

    std::ofstream mpStream;
    bool mpHeaderWritten = false;
    size_t mpRowsWritten = 0;
    size_t mpRetainedRows = 0;  // zero keeps every row in memory
    static constexpr size_t kFlushPeriod = 32;

    std::function<void(const AggregateStats&)> mpRowCallback;
    mutable std::mutex mpMtxRows;
};

}  // namespace basalt
```

#### Design rationale

`RegisterStats`, `SetPrefixForLogging`, `PrintLatest` and `SaveCsv` carry the workspace naming convention rather than the upstream lower case spelling, because they are new symbols. The four `add` methods keep the upstream spelling because an override has no choice. That split is deliberate and should not be normalised in either direction.

The copy constructor is deleted because the class owns a `std::ofstream` and a `std::mutex`, neither of which is copyable, which in turn dictates how `Logger` stores its aggregates. A `std::map<std::string, AggregateStats>` supports in place construction through `try_emplace` and never moves its elements, so it is the correct container and is what `Logger` uses.

`PrintLatest` has two overloads because the requirement names both a stored prefix and a prefix argument. The no argument form uses whatever `SetPrefixForLogging` last recorded, and the empty string if it was never called. The argument form overrides the stored prefix for that call only.

`SetRowCallback` is the extension point for real time health monitoring. It is invoked from `CommitRow` while the aggregate lock is held, which guarantees that the callback observes a complete row, and it is unset today so it costs one branch.

#### Backwards compatibility impact

The file is new and is included by nothing that exists. The only risk is include order, and it is eliminated by depending on no basalt header other than `time_utils.hpp`, which itself includes only `chrono`, `string`, `unordered_map`, `variant`, `vector` and `Eigen/Dense`.

### Change 3, `src/utils/logger.cpp`, the `AggregateStats` half

#### Code snippet

The `add` overrides delegate to the base and then stamp the registered format. This is the whole mechanism by which no call site ever repeats a format string.

```cpp
AggregateStats::Meta& AggregateStats::add(const std::string& name,
                                          double value) {
    std::lock_guard<std::mutex> lock(mpMtxRows);
    Meta& meta = ExecutionStats::add(name, value);
    meta.format(FormatFor(name));
    return meta;
}

std::string AggregateStats::FormatFor(const std::string& name) const {
    const Registry::const_iterator it = mpRegistry.find(name);
    return it == mpRegistry.end() ? std::string("f") : it->second;
}
```

The `Eigen::VectorXd` and `Eigen::VectorXf` overrides are identical in shape. `add_int` is likewise, over `ExecutionStats::add_int`. An unregistered name is not an error, it defaults to `f`, which keeps a quick ad hoc measurement usable without a registry edit.

`RenderValue` reads the last element of whichever variant arm the name holds and applies the format table from the architecture section.

```cpp
std::string AggregateStats::RenderValue(const std::string& name,
                                        const Meta& meta) const {
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
        os << name << '=' << RenderValue(name, meta);
    }
    return os.str();
}
```

`RenderValue` is called with the lock already held by `PrintLatest`, so it must not take it itself. That is why the locking sits in the public methods only.

`ColumnNames` expands a vector column into one column per component, using the width declared by the `vecN` format token so that the header can be written before a second row exists.

```cpp
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
```

`CommitRow` writes the header on the first row, appends the row, flushes on a period, trims the in memory copy if a retention limit was set, and fires the callback.

```cpp
void AggregateStats::CommitRow() {
    std::lock_guard<std::mutex> lock(mpMtxRows);
    if (mpStream.is_open()) {
        if (!mpHeaderWritten) WriteHeaderLocked();
        WriteRowLocked();
        mpRowsWritten++;
        if (mpRowsWritten % kFlushPeriod == 0) mpStream.flush();
    }
    if (mpRowCallback) mpRowCallback(*this);
    TrimLocked();
}
```

`TrimLocked` is a no operation when `mpRetainedRows` is zero, which is the default and is what `SaveCsv` and the legacy JSON path depend on. When a limit is set it erases from the front of each sample vector, which keeps the recent history available to `PrintLatest` and to a future ROS2 tap while bounding memory.

#### Intuition

The class is deliberately thin. It adds a registry lookup on the write path, a last element read on the render path, and an append on the persistence path. Everything else, ordering, storage, merging and JSON, is inherited unchanged from a class that has been in this codebase since upstream.

#### Backwards compatibility impact

None, the file is new. `src/utils/logger.cpp` needs the same `overload` helper that `src/utils/time_utils.cpp` defines in an anonymous namespace. Rather than duplicating it, it moves to `include/basalt/utils/time_utils.hpp` as a public helper and both translation units use it. That is a pure relocation, no behaviour changes, and the anonymous namespace in `time_utils.cpp` is then removed.

### Change 4, `class Logger`

#### Code snippet, the class

```cpp
class Logger {
public:
    using Ptr = std::shared_ptr<Logger>;

    /// outputDirectory empty disables the comma separated sink and leaves the
    /// console sink alone, which reproduces today's behaviour exactly.
    explicit Logger(const std::string& outputDirectory = std::string(),
                    bool consoleEnabled = true);
    ~Logger();

    /// Shared null object. Every Add and Print method returns immediately, so a
    /// component constructed without a logger needs no null guard.
    static Ptr Disabled();

    bool IsEnabled() const { return mpEnabled; }
    bool IsConsoleEnabled() const { return mpEnabled && mpConsoleEnabled; }

    // ── Optical flow ────────────────────────────────────────────────
    void AddOpticalFlowFrame(int64_t tNs, int64_t frame, double dt, int in,
                             int attempted, int tracked, int fwdFail,
                             int bwdFail, int recovRej, int detected,
                             int epiRej, int out, double flowMean,
                             double flowMax);
    void PrintOpticalFlowFrame();
    void AddOpticalFlowTiming(int64_t tNs, double pyramid, double track,
                              double detectAdd, double total, int queue);
    void PrintOpticalFlowTiming();

    // ── ... one pair per aggregate, tabulated below ... ─────────────

    // ── Legacy ExecutionStats artefacts, preserved for the batch tools ──
    ExecutionStats& SolverScratch();
    void FinishVioOptimize();
    void SaveLegacyStats();

    void PrintSummary();
    void SaveAll();

    AggregateStats& Aggregate(const std::string& name);

private:
    void Register(const std::string& name, const std::string& prefix,
                  const AggregateStats::Registry& registry);
    void Emit(const std::string& name);

    bool mpEnabled = false;
    bool mpConsoleEnabled = true;
    std::string mpOutputDirectory;
    std::map<std::string, AggregateStats> mpAggregates;

    ExecutionStats mpStatsAll;
    ExecutionStats mpStatsSums;
    ExecutionStats mpSolverScratch;

    std::mutex mpMtxConsole;
};
```

#### Code snippet, the constructor and the two mechanical halves

```cpp
Logger::Logger(const std::string& outputDirectory, bool consoleEnabled)
    : mpEnabled(true),
      mpConsoleEnabled(consoleEnabled),
      mpOutputDirectory(outputDirectory) {
    Register("of_frame", "[Optical Flow]",
             {{"t_ns", "ns"},        {"frame", "count"},
              {"dt", "ms"},          {"in", "count"},
              {"attempted", "count"},{"tracked", "count"},
              {"fwd_fail", "count"}, {"bwd_fail", "count"},
              {"recov_rej", "count"},{"survival", "ratio"},
              {"detected", "count"}, {"epi_rej", "count"},
              {"out", "count"},      {"flow_px_mean", "f"},
              {"flow_px_max", "f"}});
    // ... the remaining thirty four Register calls, one per table below ...
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
    // The survey below is where this choice is justified.
    std::lock_guard<std::mutex> lock(mpMtxConsole);
    std::cout << line << '\n';
}
```

Every `Add*` body follows one shape, and every `Print*` body is a single `Emit` call.

```cpp
void Logger::AddOpticalFlowFrame(int64_t tNs, int64_t frame, double dt, int in,
                                 int attempted, int tracked, int fwdFail,
                                 int bwdFail, int recovRej, int detected,
                                 int epiRej, int out, double flowMean,
                                 double flowMax) {
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
```

Two rules make every remaining body derivable without further judgement. The fields are added in the order the registry table below lists them, which fixes the column order of the file. A field that is a derived quantity rather than an argument, such as `survival` here or `connected_ratio` in `vio_assoc`, is computed inside the `Add` method so that the derivation exists in exactly one place.

`Disabled` returns a singleton whose `mpEnabled` is false, constructed through a private tag so that the public constructor always yields an enabled logger.

```cpp
Logger::Ptr Logger::Disabled() {
    static Ptr instance = [] {
        Ptr p = std::make_shared<Logger>();
        p->mpEnabled = false;
        p->mpConsoleEnabled = false;
        return p;
    }();
    return instance;
}
```

### Change 5, the registry, which is the complete specification

Thirty five aggregates cover every quantity the pipeline prints today plus the gaps named in the requirement. Each table gives the column, its format token and where the value comes from. The `Add` method signature is the column list minus the derived rows, in the same order, and the `Print` method name is the aggregate name in title case.

#### Optical flow, `include/basalt/optical_flow/frame_to_frame_optical_flow.h`

`of_frame`, prefix `[Optical Flow]`, one row per frame, replacing `:188-194` and `:254-277`.

| Column | Format | Source |
|---|---|---|
| `t_ns` | `ns` | `curr_t_ns` |
| `frame` | `count` | `frame_counter` |
| `dt` | `ms` | `(curr_t_ns - prevFrameTNs) * 1e-9`, zero on the first frame |
| `in` | `count` | `numPointsBefore` |
| `attempted` | `count` | `trackStats.mAttempted` |
| `tracked` | `count` | `trackStats.mTracked` |
| `fwd_fail` | `count` | `trackStats.mForwardFailed` |
| `bwd_fail` | `count` | `trackStats.mBackwardFailed` |
| `recov_rej` | `count` | `trackStats.mRecoveryRejected` |
| `survival` | `ratio` | derived, `tracked / in`, zero when `in` is zero |
| `detected` | `count` | `numDetected` |
| `epi_rej` | `count` | `numEpipolarRejected` |
| `out` | `count` | `transforms->observations.at(0).size()` |
| `flow_px_mean` | `f` | derived, `flowSum / tracked` |
| `flow_px_max` | `f` | `flowMax` |

`of_timing`, prefix `[Optical Flow][timing]`, one row per frame, replacing `:280-286`.

| Column | Format | Source |
|---|---|---|
| `t_ns` | `ns` | `curr_t_ns` |
| `pyramid` | `ms` | `pyramidSeconds` |
| `track` | `ms` | `trackSeconds` |
| `detect_add` | `ms` | `detectSeconds` |
| `total` | `ms` | `tTotal.elapsed()` |
| `queue` | `count` | `input_queue.size()` |

#### Inertial ingestion, `src/controller.cpp`

`vio_imu_ingest`, prefix `[VIO][imu-ingest]`, one row per `kImuLogPeriod` samples, replacing `:251-262`.

| Column | Format | Source |
|---|---|---|
| `t_ns` | `ns` | `data->t_ns` |
| `n` | `count` | `mpImuSampleCount` |
| `stamp_rate_hz` | `f` | derived from `mpImuWindowFirstTNs` |
| `wall_rate_hz` | `f` | derived from `mpImuWindowWallStart` |
| `max_gap` | `ms` | `mpImuWindowMaxGapNs * 1e-9` |
| `non_monotonic` | `count` | `mpImuNonMonotonic` |
| `queue` | `count` | `imu_data_queue.size()` |
| `accel` | `vec3` | `data->accel` |
| `gyro` | `vec3` | `data->gyro` |

#### Estimator, `src/vi_estimator/sqrt_keypoint_vio.cpp`

`vio_init`, prefix `[VIO][init]`, one row, merging `:255-258` and `:295-302`.

| Column | Format | Source |
|---|---|---|
| `t_ns` | `ns` | `curr_frame->t_ns` |
| `imu_t_ns` | `ns` | `imuData->t_ns` |
| `imu_minus_frame` | `ms` | derived, `(imuData->t_ns - curr_frame->t_ns) * 1e-9` |
| `imu_queue` | `count` | `imuQueueSizeOnEntry` |
| `skipped_behind` | `count` | `numImuSkippedBehind` |
| `accel` | `vec3` | `imuData->accel` |
| `accel_norm` | `f` | derived |
| `gyro` | `vec3` | `imuData->gyro` |
| `gyro_norm` | `f` | derived |
| `bg` | `vec3` | `mpBg` |
| `ba` | `vec3` | `mpBa` |
| `g` | `vec3` | `g` |
| `q_w_i` | `vec4` | `T_w_i_init.unit_quaternion().coeffs()`, new |
| `tilt_deg` | `f` | derived, angle between the seeded body z axis and world z, new |

The last two columns close the Phase 0 item of [`vio_drift_analysis.md`](vio_drift_analysis.md), which recorded that the attitude seeded by `Eigen::Quaternion::FromTwoVectors` at `:275-276` is never logged and that the Flight A gyroscope mechanism therefore rests on inference rather than measurement.

`vio_imu_frame`, prefix `[VIO][imu]`, one row per frame, replacing `:371-388`.

| Column | Format | Source |
|---|---|---|
| `t_ns` | `ns` | `curr_frame->t_ns` |
| `frame_dt` | `ms` | `frameDtNs * 1e-9` |
| `integrated` | `count` | `numImuIntegrated` |
| `expected` | `count` | `frameDtNs * 1e-9 * calib.imu_update_rate`, rounded |
| `skipped_behind` | `count` | `numImuSkippedBehind` |
| `first_imu_ns` | `ns` | `firstIntegratedImuTNs` |
| `last_imu_ns` | `ns` | `lastIntegratedImuTNs` |
| `coverage` | `ratio` | `coverage` |
| `meas_dt` | `ms` | `meas->get_dt_ns() * 1e-9` |
| `stretch` | `flag` | `stretchFallbackFired` |
| `stretch_span` | `ms` | `stretchSpanNs * 1e-9` |
| `imu_queue_in` | `count` | `imuQueueSizeOnEntry` |
| `imu_queue_out` | `count` | `imu_data_queue.size()` |
| `head_minus_frame` | `ms` | derived |

`vio_pose_update`, prefix `[VIO][pose-update]`, one row per frame, replacing `:481-487`.

| Column | Format | Source |
|---|---|---|
| `t_ns` | `ns` | `opt_flow_meas->t_ns` |
| `pending` | `count` | `mpPosesToUpdate.size()` on entry |
| `applied` | `count` | new tally |
| `rejected` | `count` | new tally |
| `max_trans_err` | `f` | new, maximum over the pass |
| `max_rot_err` | `f` | new, maximum over the pass |
| `limit_trans` | `f` | `kRelinThresholdTrans` |
| `limit_rot` | `f` | `kRelinThresholdRot` |

The existing site prints one line per rejection, which produced 9,498 lines in Flight B of `/ws/log2.log` and 2,232 in Flight A, and reduced poorly. A per frame tally answers the same question in one row, and `pending` also exposes the unbounded growth of `mpPosesToUpdate` recorded in [`../context/vio_optical_flow_instrumentation.md`](../context/vio_optical_flow_instrumentation.md), which the current line does not.

`vio_assoc`, prefix `[VIO][assoc]`, one row per frame, replacing `:576-590`.

| Column | Format | Source |
|---|---|---|
| `t_ns` | `ns` | `opt_flow_meas->t_ns` |
| `of_obs` | `count` | `obs0` |
| `connected0` | `count` | `connected0` |
| `unconnected0` | `count` | `unconnected_obs0.size()` |
| `connected_ratio` | `ratio` | derived |
| `thresh` | `f` | `config.vio_new_kf_keypoints_thresh` |
| `frames_after_kf` | `count` | `frames_after_kf` |
| `lmdb_landmarks` | `count` | `lmdb.numLandmarks()` |
| `kfs` | `count` | `kf_ids.size()` |
| `frame_states` | `count` | `frame_states.size()` |
| `frame_poses` | `count` | `frame_poses.size()` |
| `take_kf` | `flag` | `take_kf` |

`vio_triang`, prefix `[VIO][triang]`, one row per keyframe, replacing `:716-732`.

| Column | Format | Source |
|---|---|---|
| `t_ns` | `ns` | `opt_flow_meas->t_ns` |
| `candidates` | `count` | `unconnected_obs0.size()` |
| `added` | `count` | `num_points_added` |
| `no_prior_obs` | `count` | `numNoPriorObs` |
| `unproject_fail` | `count` | `numUnprojectFail` |
| `short_baseline` | `count` | `numShortBaseline` |
| `not_finite` | `count` | `numNotFinite` |
| `behind` | `count` | `numBehind` |
| `too_close` | `count` | `numTooClose` |
| `min_triang_dist` | `f` | `config.vio_min_triangulation_dist` |
| `baseline_m_min` | `f` | `minBaseline` |
| `baseline_m_max` | `f` | `maxBaseline` |
| `depth_m_min` | `f` | `minDepth` |
| `depth_m_mean` | `f` | derived |
| `depth_m_max` | `f` | `maxDepth` |

The flat column names remove the bracket group that `scripts/vio_log_extract.py` has to disambiguate, described in [`../context/vio_optical_flow_instrumentation.md`](../context/vio_optical_flow_instrumentation.md).

`vio_landmarks`, prefix `[VIO][landmarks]`, one row per frame, replacing `:753-756`, columns `t_ns`, `lost` as `count`, `lmdb` as `count`, `marg_lost` as `flag`.

`vio_state`, prefix `[VIO][state]`, one row per frame, replacing `:766-772`.

| Column | Format | Source |
|---|---|---|
| `t_ns` | `ns` | `p.getT_ns()` |
| `p_w_i` | `vec3` | `st.T_w_i.translation()` |
| `p_norm` | `f` | derived |
| `q_w_i` | `vec4` | `st.T_w_i.unit_quaternion().coeffs()`, new |
| `v_w_i` | `vec3` | `st.vel_w_i` |
| `v_norm` | `f` | derived |
| `bg` | `vec3` | `st.bias_gyro` |
| `bg_norm` | `f` | derived, new |
| `ba` | `vec3` | `st.bias_accel` |
| `ba_norm` | `f` | derived |

`vio_timing`, prefix `[VIO][timing]`, one row per frame, replacing `:774-777` and adding the per stage breakdown the requirement asks for.

| Column | Format | Source |
|---|---|---|
| `t_ns` | `ns` | `opt_flow_meas->t_ns` |
| `pose_update` | `ms` | new timer around the `mpPosesToUpdate` pass |
| `association` | `ms` | new timer around the connected and unconnected scan |
| `triangulation` | `ms` | new timer around the keyframe triangulation block |
| `lost_scan` | `ms` | new timer around the lost landmark scan |
| `opt_and_marg` | `ms` | `optMargSeconds` |
| `publish` | `ms` | new timer around `PublishKeyframe` and the output queues |
| `measure` | `ms` | `t_total.elapsed()` |
| `imu_drain` | `ms` | new timer in `ProcessFrame` around the inertial drain |
| `process_frame` | `ms` | new timer spanning `ProcessFrame` |
| `vision_queue` | `count` | `vision_data_queue.size()` |
| `imu_queue` | `count` | `imu_data_queue.size()` |

`vio_linearise`, prefix `[VIO][linearize]`, one row per outer iteration, replacing `:1451-1458`, columns `t_ns`, `iter` as `count`, `error_total` as `e`, `lambda` as `e`, `landmarks` as `count`, `states` as `count`, `poses` as `count`, `numerically_valid` as `flag`.

`vio_solver_iter`, prefix `[VIO][solver]`, one row per inner iteration, merging `:1484-1485`, `:1640-1654`, `:1693-1697` and `:1727-1731` into one record.

| Column | Format | Source |
|---|---|---|
| `t_ns` | `ns` | `last_state_t_ns` |
| `iter` | `count` | `it` |
| `inner` | `count` | `j` |
| `status` | `enum` | `0` accepted, `1` rejected, `2` backtracking |
| `error_total` | `e` | `after_error_total` |
| `f_diff` | `e` | `f_diff` |
| `l_diff` | `e` | `l_diff` |
| `step_quality` | `e` | `relative_decrease` |
| `step_size` | `e` | `step_norminf` |
| `vision` | `e` | `after_update_vision_error` |
| `imu` | `e` | `after_update_imu_error` |
| `bias_g` | `e` | `after_bg_error` |
| `bias_a` | `e` | `after_ba_error` |
| `marg_prior` | `e` | `after_update_marg_prior_error` |
| `lambda` | `e` | `lambda` |
| `it_time` | `ms` | `iteration_time` |
| `total_time` | `ms` | `cumulative_time` |

`vio_solver_summary`, prefix `[VIO][solver-summary]`, one row per `optimize` call, replacing `:1754-1756` and `:1765-1773`, columns `t_ns`, `num_it` as `count`, `num_it_rejected` as `count`, `converged` as `flag`, `terminated` as `flag`, `reason` as `enum`, `optimize` as `ms`.

`vio_solver_timing`, prefix `[VIO][solver-timing]`, one row per inner iteration, carrying the keys the local `ExecutionStats stats` object holds today so that the legacy artefacts are unchanged, `num_cams`, `num_lms`, `num_obs`, `allocateLMB`, `linearizeProblem`, `performQR`, `get_dense_H_b`, `solve`, `backSubstitute`, `computerError2`, `iteration`, `resident_memory`, `resident_memory_peak`. The three count keys use `count`, the two memory keys use `f`, the rest use `ms`. The misspelling of `computerError2` is retained deliberately, because `python/basalt/log.py` indexes by key name.

`vio_marg`, prefix `[VIO][marg]`, one row per marginalisation, merging `:995-1001` and `:1126-1130`, columns `t_ns`, `states_to_remove`, `poses_to_marg`, `states_to_marg`, `states_to_marg_vel_bias`, `kfs_to_marg`, `kf_ids`, `keeping`, `marg`, `total`, `frame_poses`, `frame_states` as `count`, and `last_state_to_marg` as `ns`.

`vio_marg_timing`, prefix `[VIO][marg-timing]`, one row per marginalisation, carrying the `stats_sums_` keys `frame_id` as `ns`, and `measure`, `marg_linearize`, `marg_helper`, `marg`, `marg_log`, `marginalize` as `ms`.

`vio_marg_nullspace`, prefix `[VIO][marg-ns]`, one row per nullspace log, replacing `:830-835`.

| Column | Format | Source |
|---|---|---|
| `t_ns` | `ns` | `last_state_t_ns` |
| `marg_ns` | `vec7` | `checkMargNullspace()` |
| `ev_min` | `e` | derived, smallest eigenvalue |
| `ev_max` | `e` | derived, largest eigenvalue |
| `ev_negative` | `count` | derived, eigenvalues below zero |
| `ev_condition` | `e` | derived, `ev_max / ev_min` |

`SqrtBundleAdjustmentBase::checkNullspace` returns a fixed width seven vector at `src/vi_estimator/sqrt_ba_base.cpp:200-226`, whose components are the increments along world x, y and z, roll, pitch, yaw, and a random direction, so `marg_ns_0` to `marg_ns_6` are self describing in that order. `checkEigenvalues` at `:230-256` returns `eigensolver.eigenvalues()`, whose width is the dimension of the marginalisation prior and therefore changes with the window, so it cannot occupy a fixed column set. The four summary scalars answer the question the raw spectrum is consulted for, whether the prior has lost rank or become ill conditioned, and the full spectrum continues to reach `stats_sums.json` unchanged through the variable width encoding at `src/utils/time_utils.cpp:124-134`.

#### Local mapper, `src/vi_estimator/local_mapper.cpp`

`map_cycle_timing`, prefix `[Local Mapper][timing]`, one row per mapping cycle, replacing the fifteen paired `high_resolution_clock` timing prints and the two count prints at `:113-345`. Every column is `ms` except the two counts.

| Column | Format | Source |
|---|---|---|
| `t_ns` | `ns` | newest ingested keyframe timestamp |
| `keyframes` | `count` | `vKeyframes.size()` |
| `marg_packets` | `count` | `vecData.size()` |
| `queue_wait` | `ms` | `:113-118` |
| `ingest` | `ms` | `:155-162` |
| `detect_keypoints` | `ms` | `:171-178` |
| `match_stereo` | `ms` | `:183-191`, zero in a monocular configuration |
| `match_local` | `ms` | `:196-203` |
| `collect_new_kfs` | `ms` | `:208-216` |
| `build_tracks` | `ms` | `:220-227` |
| `setup_opt` | `ms` | `:231-238` |
| `cull` | `ms` | `:242-249` |
| `optimize_1` | `ms` | `:253-260` |
| `filter_outliers` | `ms` | `:274-281` |
| `optimize_2` | `ms` | `:285-292` |
| `publish_vis` | `ms` | `:316-323` |
| `pose_update_cb` | `ms` | `:327-334` |
| `total` | `ms` | `:336-340` |

Replacing seventeen separate prints with one row is the single largest reduction in this change. The current form emits eighteen lines per cycle counting the separator at `:341-344`, most of them carrying one number, which is the shape hardest to parse and the shape most likely to interleave with the estimator running on another thread.

`map_bow_query`, prefix `[Local Mapper][bow]`, one row per new keyframe queried, replacing `:499-508`, columns `t_ns`, `kf` as `ns`, `keypoints`, `descriptors`, `bow_buckets`, `db_hits`, `above_thresh` as `count`, `best_score` as `f`, `thresh` as `f`.

`map_pair_match`, prefix `[Local Mapper][match]`, one row per candidate pair, replacing `:592-599`, columns `kf1` and `kf2` as `ns`, `descriptor_matches`, `min_matches`, `ransac_inliers` as `count`, `stored` as `flag`.

`map_match_summary`, prefix `[Local Mapper][match-summary]`, one row per cycle, replacing `:553-557` and `:626-635`, columns `t_ns`, `pairs`, `neighbour_pairs`, `neighbour_limit`, `verification_attempts`, `inlier_matches`, `total_matches` as `count`, `db_query` and `matching` as `ms`.

`map_tracks`, prefix `[Local Mapper][tracks]`, one row per cycle, replacing `:661-672`.

| Column | Format | Source |
|---|---|---|
| `t_ns` | `ns` | newest ingested keyframe timestamp |
| `exported` | `count` | `feature_tracks.size()` |
| `live_in_builder` | `count` | `mpTrackBuilder.TrackCount()` |
| `matches` | `count` | `mpLatestKeyframesMatches.size()` |
| `new_kfs` | `count` | `mpNewKeyframesForTracking.size()` |
| `corners` | `count` | `feature_corners.size()` |
| `len_hist` | `vec10` | histogram of track length, index nine holding every length at or above eleven |

The variable width `std::map<size_t, size_t>` the current site prints inside braces becomes a fixed width vector so that it occupies stable columns. The two observation track ceiling recorded in [`../context/keyframe_driven_local_mapping.md`](../context/keyframe_driven_local_mapping.md) is then simply column `len_hist_2`.

`map_setup_opt`, prefix `[Local Mapper][setup_opt]`, one row per cycle, replacing `:827-844`, columns `t_ns`, `tracks`, `short_track`, `known`, `retired`, `new_ok`, `no_triang`, `no_corners`, `no_host_pose`, `no_obs_pose`, `short_baseline`, `low_parallax`, `bad_depth`, `behind`, `too_close`, `nan`, `obs_added`, `lmdb_before`, `lmdb_after` as `count`, and `min_triang_dist`, `max_cos_parallax`, `max_inv_dist` as `f`.

`map_covis`, prefix `[Local Mapper][covis]`, one row per cycle, replacing `:937-946`, columns `t_ns`, `parity` as `flag`, `frames`, `ref_frames`, `cells`, `ref_cells` as `count`. The reference recomputation at `:938-939` stays behind `mpVioDebugMode` rather than moving behind the logger, because it is quadratic in the live keyframe count and exists only to validate an incremental index against a rebuild.

`map_cull_select`, prefix `[Local Mapper][cull-select]`, one row per cycle, replacing `:1001-1022`, columns `t_ns`, `map`, `eligible`, `keep_recent`, `max_cull`, `min_observers`, `selected`, `capacity_rule` as `count` or `flag`, `thresh` as `f`, and `observers_hist` as `vec10`. The per pick line at `:982-985` and the identifier list at `:1008` are rendered to the console only, since neither has a fixed width.

`map_rehost`, prefix `[Local Mapper][rehost]`, one row per landmark, replacing the eight sites at `:1112-1251` and `:1314-1332`, columns `t_ns`, `lm` as `ns`, `culled_host` as `ns`, `new_host` as `ns`, `obs_before`, `obs_readded` as `count`, `outcome` as `enum` with `0` rehosted, `1` rehosted fragile, `2` reprojected, `3` kept stale, `4` removed, `5` rehosted onto a doomed keyframe, and `reason` as `enum` with `0` none, `1` new host never observed it, `2` unprojection failed, `3` retriangulation and reprojection failed, `4` no observation survived, `5` no candidate host observes it. Eight prose sites collapse to two enumerated columns, which is what makes an outcome histogram a single group by rather than eight regular expressions.

`map_cull`, prefix `[Local Mapper][cull]`, one row per culled keyframe, replacing `:1274-1277`, `:1398-1404` and `:1407-1409`, columns `t_ns`, `kf` as `ns`, `victims`, `hosted_lms`, `rehost_attempted`, `no_host`, `lm_before`, `lm_after`, `map_before`, `map_after` as `count`.

`map_filter`, prefix `[Local Mapper][filter]`, one row per cycle, replacing `:269-273`, columns `t_ns`, `lm_before`, `lm_after`, `min_obs` as `count`, `thresh` as `f`.

`map_publish`, prefix `[Local Mapper][publish]`, one row per cycle, replacing `:307-313`, columns `t_ns`, `points`, `keyframes`, `lmdb_landmarks`, `host_kfs` as `count`.

#### Shared mapping base, `src/vi_estimator/nfr_mapper.cpp`

The requirement is that every print in this file carries a `[Mapper]` prefix and that the optimisation statistics it lacks are added. Four of its prints are currently untagged, `[LINEARIZE]` at `:300`, the two `\t[REJECTED]` and `\t[ACCEPTED]` forms at `:367` and `:380`, and the `iter ... time` line at `:434`. The first of those collides exactly with the estimator's own marker, which is why the estimator's was tagged `[VIO]` on 2026-09-12.

`mapper_linearise`, prefix `[Mapper][linearize]`, one row per outer iteration, replacing `:299-305`, columns `iter` as `count`, `vision` as `e`, `rel_error` as `e`, `roll_pitch_error` as `e`, `total` as `e`, and the four new columns `landmarks`, `observations`, `rel_pose_factors`, `roll_pitch_factors` as `count`, read from `lmdb.numLandmarks()`, `lmdb.numObservations()`, `rel_pose_factors.size()` and `roll_pitch_factors.size()`.

`mapper_solver_iter`, prefix `[Mapper][solver]`, one row per inner Levenberg Marquardt step, replacing `:364-400`, columns `iter` as `count`, `inner` as `count`, `status` as `enum` with the same encoding the estimator uses, `lambda` as `e`, `f_diff` as `e`, `max_inc` as `e`, `vision_error` as `e`, `rel_error` as `e`, `roll_pitch_error` as `e`, `total` as `e`, `converged` as `flag`, and `error_increased` as `flag`, the last carrying the condition currently printed unconditionally at `:397-400`.

`mapper_iter_summary`, prefix `[Mapper][iter]`, one row per outer iteration, replacing `:433-437`, columns `iter` as `count`, `iteration` as `ms`, `num_states` as `count`, `num_poses` as `count`, and the new `inner_steps` as `count`.

`mapper_detect`, prefix `[Mapper][detect]`, one row per call, replacing `:532-538`, columns `frames` as `count`, `detection` as `ms`, and the new `corners_total` as `count`.

`mapper_stereo`, prefix `[Mapper][stereo]`, one row per call, replacing `:549-585`, columns `pairs`, `matches`, `inliers` as `count`.

`NfrMapper::addMargData` at `:61-62` and `NfrMapper::match_all` at `:644-706` are left as they stand. The first is one line per packet and already carries `[Mapper]`. The second is reachable only from the offline `src/mapper.cpp` path, which is outside the scope of this change and is recorded below as deliberately untouched.

### Change 6, constructor injection

#### Need

The logger must reach three classes that are constructed through two factories and one direct `make_shared`, while four other translation units construct the same classes for offline tools and must not change. Every injection point therefore takes a trailing `Logger::Ptr` defaulted to `nullptr`, and every recipient normalises it against the null object so that no call site acquires a null check.

#### Code snippet, the recipient pattern

```cpp
FrameToFrameOpticalFlow(const VioConfig& config,
                        const basalt::Calibration<double>& calib,
                        bool useProducerConsumerArchitecture = false,
                        const Logger::Ptr& logger = nullptr)
    : t_ns(-1),
      frame_counter(0),
      last_keypoint_id(0),
      config(config),
      mpUseProducerConsumerArchitecture(useProducerConsumerArchitecture),
      mpLogger(logger ? logger : Logger::Disabled()) {
```

`SqrtKeypointVioEstimator`, `NfrMapper` and `LocalMapper` take the identical trailing parameter and perform the identical normalisation. `LocalMapper` forwards to `NfrMapper` so that both halves of the mapper share one logger.

#### Code snippet, the two factories

```cpp
static OpticalFlowBase::Ptr getOpticalFlow(
    const VioConfig& config, const Calibration<double>& cam,
    bool useProducerConsumerArchitecture = false,
    const Logger::Ptr& logger = nullptr);
```

In `src/optical_flow/optical_flow.cpp` only the four `frame_to_frame` branches forward the logger. `PatchOpticalFlow` and `MultiscaleFrameToFrameOpticalFlow` derive from `OpticalFlowBase` directly rather than from `FrameToFrameOpticalFlow`, established in [`../context/vio_optical_flow_instrumentation.md`](../context/vio_optical_flow_instrumentation.md), carry no instrumentation today, and keep their current signatures.

```cpp
template <class Scalar>
static typename VioEstimatorBase<Scalar>::Ptr getVioEstimator(
    const VioConfig& config, const Calibration<Scalar>& cam,
    const Eigen::Vector3d& g, bool use_imu,
    bool useProducerConsumerArchitecture = false,
    const Logger::Ptr& logger = nullptr);
```

`factory_helper` in `src/vi_estimator/vio_estimator.cpp:45-62` gains the same trailing parameter and forwards it to both estimators. `SqrtKeypointVoEstimator` accepts and stores it but is otherwise untouched, so the parameter is inert on the visual only path until a later change mirrors this work there.

#### Code snippet, `Controller`

```cpp
void initialize(int64_t t_ns, const Sophus::SE3d& T_w_i,
                const Eigen::Vector3d& vel_w_i, const Eigen::Vector3d& bg,
                const Eigen::Vector3d& ba,
                bool useProducerConsumerArchitecture = false,
                bool enableVisualisation = false,
                const std::string& logDirectory = std::string());
```

```cpp
    // Construct the logger before anything that receives it. The console sink
    // reproduces the previous vio_debug behaviour exactly, the file sink is
    // silent unless a directory was named.
    mpLogger = std::make_shared<basalt::Logger>(logDirectory,
                                                vio_config_.vio_debug);

    opt_flow_ptr_ = basalt::OpticalFlowFactory::getOpticalFlow(
        vio_config_, calib_, mpUseProducerConsumerArchitecture, mpLogger);

    vio_estimator_ = basalt::VioEstimatorFactory::getVioEstimator<double>(
        vio_config_, calib_, basalt::constants::g, use_imu,
        mpUseProducerConsumerArchitecture, mpLogger);

    local_mapper_ = std::make_shared<basalt::LocalMapper>(calib_, vio_config_,
                                                          mpLogger);
```

`Controller::Stop` gains one line, `if (mpLogger) mpLogger->SaveAll();`, placed after `local_mapper_->Stop()` returns at `src/controller.cpp:92-94` and before `opt_flow_ptr_.reset()` at `:98`, so that every writer thread has finished before the files are closed.

#### Design rationale, why the directory is a parameter rather than a configuration key

`VioConfig::load` at `src/utils/vio_config.cpp:131-139` constructs a `cereal::JSONInputArchive` and calls `archive(*this)` with no exception handling, and [`../context/vio_config_json.md`](../context/vio_config_json.md) records that a missing key is fatal while an unknown key is inert. Adding a `log_directory` field to `struct VioConfig` would therefore abort every run that used any of the seven existing visual inertial configuration files until all of them, and the batch template at `data/iccv21/basalt_batch_config.toml`, were edited in the same commit. Passing the directory as a defaulted argument to `Controller::initialize` costs one parameter, breaks nothing, and leaves the ROS2 node free to expose it as a parameter alongside `calibration_file_path` and `configuration_file_path` at `ros_ws/src/slam/src/basalt/node.cpp:15-21` in a later change.

#### Backwards compatibility impact

Every new parameter is trailing and defaulted, so the five existing factory call sites at `src/opt_flow.cpp:197`, `src/vio.cpp:300` and `:311`, `src/rs_t265_vio.cpp:175` and `:179`, `src/vio_sim.cpp:923` and `src/mapper_sim_naive.cpp:752` compile and behave exactly as before, each receiving `Logger::Disabled()`. The two direct `NfrMapper` constructions at `src/mapper.cpp:566` and `src/mapper_sim.cpp:163` likewise. `BasaltSLAM::InitialiseSlam` at `ros_ws/src/slam/src/basalt/slam.cpp:48-56` passes seven arguments today and continues to compile against the eight parameter signature.

### Change 7, the call site refactors

#### Optical flow, one record per frame at one place

The requirement is that the two branches of `processFrame` stop logging separately and that a single record is written before the method returns. `struct OpticalFlowTrackStats` moves to `include/basalt/utils/logger.h` and grows the fields the site currently holds in locals, so that one object carries the whole frame.

```cpp
/// Per frame counters for the frame to frame tracker. The five atomics are
/// accumulated from inside a tbb::parallel_for in trackPoints, the rest are
/// written once on the calling thread.
struct OpticalFlowTrackStats {
    std::atomic<int> mAttempted{0};
    std::atomic<int> mForwardFailed{0};
    std::atomic<int> mBackwardFailed{0};
    std::atomic<int> mRecoveryRejected{0};
    std::atomic<int> mTracked{0};

    int mPointsBefore = 0;
    int mDetected = 0;
    int mEpipolarRejected = 0;
    int mOut = 0;
    double mFlowSum = 0.0;
    double mFlowMax = 0.0;
    double mPyramidSeconds = 0.0;
    double mTrackSeconds = 0.0;
    double mDetectSeconds = 0.0;
    double mTotalSeconds = 0.0;
    double mDtSeconds = 0.0;
};
```

The two branches then only fill the struct, and the tail of `processFrame` becomes the single emission point.

```cpp
        trackStats.mOut = int(transforms->observations.at(0).size());
        trackStats.mTotalSeconds = tTotal.elapsed();

        mpLogger->AddOpticalFlowFrame(
            curr_t_ns, int64_t(frame_counter), trackStats.mDtSeconds,
            trackStats.mPointsBefore, trackStats.mAttempted.load(),
            trackStats.mTracked.load(), trackStats.mForwardFailed.load(),
            trackStats.mBackwardFailed.load(),
            trackStats.mRecoveryRejected.load(), trackStats.mDetected,
            trackStats.mEpipolarRejected, trackStats.mOut,
            trackStats.mTracked.load()
                ? trackStats.mFlowSum / double(trackStats.mTracked.load())
                : 0.0,
            trackStats.mFlowMax);
        mpLogger->PrintOpticalFlowFrame();

        mpLogger->AddOpticalFlowTiming(
            curr_t_ns, trackStats.mPyramidSeconds, trackStats.mTrackSeconds,
            trackStats.mDetectSeconds, trackStats.mTotalSeconds,
            int(input_queue.size()));
        mpLogger->PrintOpticalFlowTiming();

        frame_counter++;
        return transforms;
```

On the first frame the tracking counters remain zero and `dt` remains zero, which is the correct reading and gives one uniform table rather than two shapes. The `trackPoints` signature keeps its trailing `OpticalFlowTrackStats*` defaulted to `nullptr`, unchanged, so the stereo call at `addPoints` still passes nothing. The pointer is now passed unconditionally from the tracking loop rather than gated on `config.vio_debug`, because the counters are five relaxed atomic increments over a loop that already performs two pyramidal Lucas Kanade solves per point.

#### Estimator, the legacy artefacts

The local `ExecutionStats stats` at `src/vi_estimator/sqrt_keypoint_vio.cpp:1337` and the two members `stats_all_` and `stats_sums_` at `include/basalt/vi_estimator/sqrt_keypoint_vio.h:275-276` are removed from the estimator and re established inside `Logger`, because `python/basalt/log.py:93-95` and `python/basalt/run.py:107-112` read the files they produce.

```cpp
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
```

Inside `optimize` every `stats.add(...)` becomes `mpLogger->SolverScratch().add(...)` with the key and format unchanged, and the pair at `:1760-1761` becomes `mpLogger->FinishVioOptimize()`. The same values additionally reach the `vio_solver_timing` aggregate through `Logger::AddVioSolverTiming`, so the comma separated file and the JSON carry the same numbers. The ten `stats_sums_` calls outside `optimize`, at `:452`, `:817`, `:830`, `:833`, `:835`, `:1049`, `:1213`, `:1304`, `:1313` and `:1320`, move to `mpLogger->SolverScratch()` likewise, preserving their key names exactly. Together with the sixteen `stats.add` calls inside `optimize` and the three `stats_all_` uses at `:1760`, `:1821` and `:1826`, that is the complete set of twenty nine.

`debug_finalize` then becomes the following, which reproduces the current output of `src/vio.cpp:634` and adds the comma separated files.

```cpp
template <class Scalar_>
void SqrtKeypointVioEstimator<Scalar_>::debug_finalize() {
    mpLogger->PrintSummary();
    mpLogger->SaveLegacyStats();
    mpLogger->SaveAll();
}
```

Because `src/vio.cpp` constructs its estimator through the factory with no logger, it would otherwise receive `Logger::Disabled()` and lose the batch artefacts entirely. The single required change is at `src/vio.cpp:311-312`, which constructs a logger and passes it, giving that tool the comma separated files as well.

```cpp
        auto logger =
            std::make_shared<basalt::Logger>(".", vio_config.vio_debug);
        vio = basalt::VioEstimatorFactory::getVioEstimator(
            vio_config, calib, basalt::constants::g, use_imu,
            use_double,  // defect, see below, this slot is now threading
            logger);
```

#### Defect found while validating this snippet, and deliberately left standing

`src/vio.cpp:312` passes `use_double` into the fifth parameter of `getVioEstimator`. That parameter is `useProducerConsumerArchitecture`, declared at `include/basalt/vi_estimator/vio_estimator.h:147-150`. The fifth slot carried the scalar type selector upstream and was repurposed in this fork, and the offline call site was never updated. Two consequences follow. The `--use-double` command line flag registered at `src/vio.cpp:249` now switches the offline tool between the synchronous and the producer consumer threading models rather than between the `float` and the `double` estimator, and the scalar type is in any case pinned to `double` because `VioEstimatorFactory::getVioEstimator<double>` is the only instantiation exported, at `src/vi_estimator/vio_estimator.cpp:177-181`.

The correction is to pass `false` in that slot and to retire or rename the flag. It is not made here, because this change is about logging and because anyone currently passing `--use-double` is relying, knowingly or not, on the threading model it actually selects. Appending the logger after that argument is safe regardless of how the defect is later resolved, since the new parameter is trailing and defaulted.

#### Local mapper and shared base, the predicate split

Every `if (mpVioDebugMode) std::cout << ...` becomes a pair, an unconditional `Add` and a `Print` that the logger itself gates. Two sites keep an explicit predicate because they perform work rather than formatting, the covisibility reference recomputation at `src/vi_estimator/local_mapper.cpp:938-939` and the observer histogram at `:1012-1017`, both of which walk the whole landmark set. They stay behind `mpVioDebugMode`, and the corresponding columns are recorded only when it is set.

### Change 8, the build

`CMakeLists.txt` gains two lines, the header immediately after `include/basalt/utils/keypoints.h` at `CMakeLists.txt:352` in the `PUBLIC` block, and the source immediately after `src/utils/keypoints.cpp` at `CMakeLists.txt:385` in the `PRIVATE` block.

```cmake
    ${CMAKE_CURRENT_SOURCE_DIR}/include/basalt/utils/logger.h
    ...
    ${CMAKE_CURRENT_SOURCE_DIR}/src/utils/logger.cpp
```

No link libraries change. `logger.cpp` needs only the standard library and `time_utils.hpp`, and the public header adds no dependency that consumers of `basalt` do not already have. The build uses `-Wall -Wextra -Werror`, recorded in [`../context/vio_optical_flow_instrumentation.md`](../context/vio_optical_flow_instrumentation.md), so unused parameters in the disabled paths must be consumed rather than left dangling.

### Change 9, the analysis scripts

`scripts/vio_log_extract.py` reconstructed records from interleaved console prose, and `scripts/vio_log_plot.py` drew one run's own PNGs from its output, neither with a way to select a subset of runs or produce a scalar comparison table. Both are deleted, superseded by `scripts/analysis/`, `plans/analysis_module.md`'s offline dashboard that reads the CSV files this change produces directly, computes every metric across any number of runs, and renders one static, interactive page. Correction, 2026-09-27. This paragraph previously kept both scripts alive indefinitely, planning a `--from-csv` mode for `vio_log_plot.py` and retiring `vio_log_extract.py` only once no capture predated the CSV writer, superseded once the dashboard reached every capture of interest with no reconstruction step needed.

The join that the extractor performs in Python, folding every record sharing a frame timestamp into one row, becomes a database style join across files keyed on the mandatory `t_ns` first column. Correction, 2026-09-26. This paragraph previously stated that `t_ns` is present in every per frame aggregate by construction, which an audit of `/ws/logs/2026-09-26_16-37-51/`'s own headers, prompted by the session that requested Change 19 through Change 23, showed was not true for seven records, `map_pair_match.csv`, `mapper_detect.csv`, `mapper_iter_summary.csv`, `mapper_linearise.csv`, `mapper_solver_iter.csv`, `mapper_stereo.csv` and `vio_solver_timing.csv`, each written with no invoking frame or keyframe timestamp at all, and one further record, `vio_marg_timing.csv`, that carries the right value under the inconsistent name `frame_id`. `t_ns` is the intended and, as of Change 19 through Change 23, the actual invariant across every record this plan produces, not merely the aspiration this paragraph originally described.

### Change 10, ground truth yaw alignment, `AddVioInit`

#### Need

`SqrtKeypointVioEstimator::ProcessFrame` corrects the gravity aligned initial yaw against `mpCurrentGTPose` at `src/vi_estimator/sqrt_keypoint_vio.cpp:274-284`, rotating `T_w_i_init` about Z by `yawOffset` so the estimator's world frame heading matches ground truth's heading at the seed frame. Nothing records that this correction happened, by how much, or what the two yaw readings were before it. A run without ground truth applies no correction and `T_w_i_init`'s yaw is whatever the accelerometer's `FromTwoVectors` solve happened to produce, an arbitrary angle about the gravity vector. `vio_init.csv` is silent on both cases today.

Corrected 2026-09-26 after review. The first draft packed the three angles into one `vec3` column, which would have rendered as the positional `yaw_align_0`, `yaw_align_1`, `yaw_align_2` in the header rather than named columns, and relied on `SetBlankEmptyVectors` to blank them, which only exists for vector columns. `vio_init.csv` is written at most twice in a run, so there is no benefit to the vector mechanism here, and the review asked for three named scalar columns instead, `-999.0` as the explicit sentinel for the unaligned case rather than a blank cell, and the offset itself wrapped to `(-pi, pi]` before it is logged, since the difference of two `atan2` results is not otherwise guaranteed to land in that range even though the rotation it produces is unaffected. The snippet below replaces the first draft in full.

#### Code snippet

The correction block gains three named locals, wrapped into `(-pi, pi]` before either the rotation or the log call uses them.

```cpp
// Turn the gravity aligned world about Z so body X takes the ground
// truth heading. Tilt stays with the accelerometer, which the initial
// bias state is consistent with.
const bool yawAligned = mpCurrentGTPose.has_value();
double gtYawRad = 0.0;
double slamYawRad = 0.0;
double yawOffsetRad = 0.0;
if (mpCurrentGTPose) {
    const Eigen::Matrix<Scalar, 3, 3> rAcc = T_w_i_init.so3().matrix();
    const Eigen::Matrix3d rGt = mpCurrentGTPose->so3().matrix();
    slamYawRad = std::atan2(double(rAcc(1, 0)), double(rAcc(0, 0)));
    gtYawRad = std::atan2(rGt(1, 0), rGt(0, 0));
    // atan2(sin(x), cos(x)) wraps x to (-pi, pi]. rotZ is periodic in its
    // argument modulo 2 pi, so the correction applied is unaffected, but
    // the logged value is only directly comparable against
    // gt_yaw_rad - slam_yaw_rad, both already atan2 results, if wrapped
    // the same way.
    yawOffsetRad = std::atan2(std::sin(gtYawRad - slamYawRad),
                              std::cos(gtYawRad - slamYawRad));
    T_w_i_init.so3() = Sophus::SO3<Scalar>::rotZ(Scalar(yawOffsetRad)) *
                       T_w_i_init.so3();
    T_w_i_init.translation() =
        mpCurrentGTPose->translation().template cast<Scalar>();
}
```

`Logger::AddVioInit` gains four trailing parameters and `vio_init`'s registry gains four named columns.

```cpp
// include/basalt/utils/logger.h
void AddVioInit(int64_t tNs, int64_t imuTNs, int64_t imuQueue,
                int64_t skippedBehind, const Eigen::Vector3d& accel,
                const Eigen::Vector3d& gyro, const Eigen::Vector3d& bg,
                const Eigen::Vector3d& ba, const Eigen::Vector3d& g,
                const Eigen::Vector4d& qWI, double tiltDeg,
                bool yawAligned, double yawOffsetRad, double gtYawRad,
                double slamYawRad);
```

```cpp
// src/utils/logger.cpp, constructor registry
Register("vio_init", "[VIO][init]",
         {{"t_ns", "ns"}, {"imu_t_ns", "ns"}, {"imu_minus_frame", "ms"},
          {"imu_queue", "count"}, {"skipped_behind", "count"},
          {"accel", "vec3"}, {"accel_norm", "f"}, {"gyro", "vec3"},
          {"gyro_norm", "f"}, {"bg", "vec3"}, {"ba", "vec3"},
          {"g", "vec3"}, {"q_w_i", "vec4"}, {"tilt_deg", "f"},
          {"yaw_aligned", "flag"}, {"yaw_offset_rad", "f"},
          {"gt_yaw_rad", "f"}, {"slam_yaw_rad", "f"}});
```

No `SetBlankEmptyVectors` call is needed for `vio_init`, unlike the first draft, since none of its columns are vector typed for the unaligned case.

```cpp
// src/utils/logger.cpp, Logger::AddVioInit
void Logger::AddVioInit(int64_t tNs, int64_t imuTNs, int64_t imuQueue,
                        int64_t skippedBehind, const Eigen::Vector3d& accel,
                        const Eigen::Vector3d& gyro,
                        const Eigen::Vector3d& bg, const Eigen::Vector3d& ba,
                        const Eigen::Vector3d& g, const Eigen::Vector4d& qWI,
                        double tiltDeg, bool yawAligned, double yawOffsetRad,
                        double gtYawRad, double slamYawRad) {
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
    s.add("yaw_aligned", yawAligned ? 1.0 : 0.0);
    s.add("yaw_offset_rad", yawAligned ? yawOffsetRad : -999.0);
    s.add("gt_yaw_rad", yawAligned ? gtYawRad : -999.0);
    s.add("slam_yaw_rad", yawAligned ? slamYawRad : -999.0);
    s.CommitRow();
}
```

The one call site gains the four arguments, immediately after `tiltDeg`.

```cpp
mpLogger->AddVioInit(
    curr_frame->t_ns, imuData->t_ns, imuQueueSizeOnEntry,
    numImuSkippedBehind, imuData->accel.template cast<double>(),
    imuData->gyro.template cast<double>(), mpBg.template cast<double>(),
    mpBa.template cast<double>(), g.template cast<double>(),
    T_w_i_init.unit_quaternion().coeffs().template cast<double>(),
    tiltDeg, yawAligned, yawOffsetRad, gtYawRad, slamYawRad);
```

`yaw_offset_rad == gt_yaw_rad - slam_yaw_rad`, up to the `(-pi, pi]` wrap, is a standing invariant a reader can check directly against the CSV whenever `yaw_aligned=1`. `Logger::AddVioInit` substitutes the sentinel `-999.0` into all three angle columns whenever `yawAligned` is false, following the same decide-inside-the-method pattern `AddVioTriang`'s own `depth_m_mean` already uses rather than asking every caller to remember to do so. `-999.0` falls well outside `(-pi, pi]`, the range every `atan2` result and hence every one of these three columns is otherwise confined to, so it cannot be mistaken for a genuine reading, and a downstream reader is expected to key off `yaw_aligned` to decide whether alignment ran at all, using the sentinel only as a defence against a reader that forgets to check it.

#### Design rationale

Three named scalar columns rather than one packed vector, because `vio_init.csv` is written at most twice per run and there is no benefit to the vector blanking mechanism here, a named column reads directly as `yaw_offset_rad`, `gt_yaw_rad` and `slam_yaw_rad` in the header rather than as the positional `yaw_align_0` through `yaw_align_2`, and a downstream script keys on name exactly as every other column in this file already does. The angle wrap matters for a reason beyond the header, `Sophus::SO3::rotZ` is periodic in its argument modulo 2 pi, so the correction it applies to `T_w_i_init` is identical whether `yawOffsetRad` is wrapped or not, but the logged value is only directly comparable against `gt_yaw_rad - slam_yaw_rad`, itself already confined to `(-pi, pi]` being a difference of two `atan2` results, if `yawOffsetRad` is wrapped into the same range before it is stored, so the wrap is applied once, to the one variable both the rotation and the log call use.

#### Backwards compatibility impact

`AddVioInit` has exactly one caller, `src/vi_estimator/sqrt_keypoint_vio.cpp:306-312`, updated in the same change. `vio_init.csv` gains four columns, `yaw_aligned`, `yaw_offset_rad`, `gt_yaw_rad` and `slam_yaw_rad`, appended after `tilt_deg`, so a reader keyed on column name rather than position is unaffected and any positional reader is already broken by every earlier change to this file's own column set.

### Change 11, front end track lifetime and death, `of_track_death`

#### Need

`FrameToFrameOpticalFlow` tracks every keypoint's pixel position frame to frame in `transforms->observations.at(0)`, keyed by `KeypointId` (`size_t`, `include/basalt/optical_flow/optical_flow.h:50`), but keeps no record of how long a track survives or why it eventually dies. `of_frame.csv` already counts `fwd_fail`, `bwd_fail` and `recov_rej` per frame, the three ways `trackPoints` (`frame_to_frame_optical_flow.h:259-326`) drops an id, but a count says nothing about which tracks these were nor how old they had grown, the two things a track length distribution and a survival curve need.

#### Code snippet

Two bookkeeping maps and a mutex guarded death list.

```cpp
// include/basalt/utils/logger.h, struct OpticalFlowTrackStats
struct OpticalFlowTrackStats {
    std::atomic<int> mAttempted{0};
    std::atomic<int> mForwardFailed{0};
    std::atomic<int> mBackwardFailed{0};
    std::atomic<int> mRecoveryRejected{0};
    std::atomic<int> mTracked{0};

    int mPointsBefore = 0;
    int mDetected = 0;
    int mEpipolarRejected = 0;
    int mOut = 0;
    double mFlowSum = 0.0;
    double mFlowMax = 0.0;
    double mPyramidSeconds = 0.0;
    double mTrackSeconds = 0.0;
    double mDetectSeconds = 0.0;
    double mTotalSeconds = 0.0;
    double mDtSeconds = 0.0;

    // (track id, reason), reason 0 forward fail, 1 backward fail,
    // 2 recovery rejected. trackPoints stays const, mutation happens
    // through this pointer member rather than through `this`.
    std::mutex mMtxDeaths;
    std::vector<std::pair<size_t, int>> mvDeaths;
};
```

`trackPoints`'s three failure branches each record the dying id alongside the atomic counter they already increment.

```cpp
// include/basalt/optical_flow/frame_to_frame_optical_flow.h, trackPoints
if (valid) {
    Eigen::AffineCompact2f transform_1_recovered = transform_2;

    valid = trackPoint(pyr_2, pyr_1, transform_2, transform_1_recovered);

    if (valid) {
        Scalar dist2 = (transform_1.translation() -
                        transform_1_recovered.translation())
                           .squaredNorm();

        if (dist2 < config.optical_flow_max_recovered_dist2) {
            result[id] = transform_2;
            if (stats) stats->mTracked++;
        } else if (stats) {
            stats->mRecoveryRejected++;
            std::lock_guard<std::mutex> lock(stats->mMtxDeaths);
            stats->mvDeaths.emplace_back(id, 2);
        }
    } else if (stats) {
        stats->mBackwardFailed++;
        std::lock_guard<std::mutex> lock(stats->mMtxDeaths);
        stats->mvDeaths.emplace_back(id, 1);
    }
} else if (stats) {
    stats->mForwardFailed++;
    std::lock_guard<std::mutex> lock(stats->mMtxDeaths);
    stats->mvDeaths.emplace_back(id, 0);
}
```

`FrameToFrameOpticalFlow` gains two members alongside the existing `mpLogger`, following the same `mp` naming this class already uses for its own post hoc additions.

```cpp
// include/basalt/optical_flow/frame_to_frame_optical_flow.h, class members
std::map<size_t, int64_t> mpTrackBirthTNs;
std::map<size_t, int> mpTrackAge;
```

Birth, in `addPoints`, right after the existing insertion.

```cpp
transforms->observations.at(0)[last_keypoint_id] = transform;
new_poses0[last_keypoint_id] = transform;
mpTrackBirthTNs[last_keypoint_id] = transforms->t_ns;
mpTrackAge[last_keypoint_id] = 0;

last_keypoint_id++;
```

Aging, folded into `processFrame`'s existing flow magnitude loop, which already walks every surviving id.

```cpp
for (const auto& kv : new_transforms->observations.at(0)) {
    auto itOld = transforms->observations.at(0).find(kv.first);
    if (itOld == transforms->observations.at(0).end()) continue;
    const double d = (kv.second.translation() -
                      itOld->second.translation())
                         .norm();
    trackStats.mFlowSum += d;
    if (d > trackStats.mFlowMax) trackStats.mFlowMax = d;
    mpTrackAge[kv.first]++;
}
```

Death emission, after the per camera `trackPoints` loop and before `transforms = new_transforms;`.

```cpp
for (const auto& [id, reason] : trackStats.mvDeaths) {
    const int64_t birthTNs =
        mpTrackBirthTNs.count(id) ? mpTrackBirthTNs.at(id) : curr_t_ns;
    const int ageFrames = mpTrackAge.count(id) ? mpTrackAge.at(id) : 0;
    mpLogger->AddOpticalFlowTrackDeath(curr_t_ns, int64_t(id), birthTNs,
                                      ageFrames, reason);
    mpLogger->PrintOpticalFlowTrackDeath();
    mpTrackBirthTNs.erase(id);
    mpTrackAge.erase(id);
}

transforms = new_transforms;
```

The registry and the two `Logger` methods.

```cpp
// src/utils/logger.cpp, constructor registry
Register("of_track_death", "[Optical Flow][track-death]",
         {{"t_ns", "ns"}, {"track_id", "count"}, {"birth_t_ns", "ns"},
          {"age_frames", "count"}, {"reason", "enum"}});
```

```cpp
// src/utils/logger.cpp
void Logger::AddOpticalFlowTrackDeath(int64_t tNs, int64_t trackId,
                                      int64_t birthTNs, int ageFrames,
                                      int reason) {
    if (!mpEnabled) return;
    AggregateStats& s = mpAggregates.at("of_track_death");
    s.add_int("t_ns", tNs);
    s.add_int("track_id", trackId);
    s.add_int("birth_t_ns", birthTNs);
    s.add("age_frames", double(ageFrames));
    s.add("reason", double(reason));
    s.CommitRow();
}

void Logger::PrintOpticalFlowTrackDeath() { Emit("of_track_death"); }
```

#### Design rationale

A per death event log rather than a per frame histogram, because the survival curve the front end diagnostics ask for needs the age at death for individual tracks, not a frame level aggregate. `map_tracks.csv`'s own `len_hist` bucketing is the right shape for the mapper's already retired tracks, but here the length distribution and the survival curve are two different reductions of the same underlying series, `age_frames` grouped into buckets is the distribution, the complementary empirical CDF of `age_frames` is the survival curve, and only a raw per track record supports computing both without recomputing the other from scratch. `trackPoints` stays `const`, mutation happens through the `stats` pointer exactly as the existing atomic counters already do, so no signature or call site outside this one method changes shape.

Only three reasons are recorded, forward failed, backward failed, recovery rejected, because those are the only three ways `trackPoints` drops an id, matching `of_frame.csv`'s own three counters exactly. Epipolar rejection happens in `filterPoints`, a separate pass that only runs when `calib.intrinsics.size() > 1`, and is inert on this stack's monocular configuration per `context/vio_optical_flow_instrumentation.md`, so it is not wired into this event log, a track rejected there would need a fourth reason added when the stack gains a stereo configuration.

#### Backwards compatibility impact

`OpticalFlowTrackStats` gains two members and no existing member changes type or meaning. `trackPoints`'s signature is unchanged, `stats` was already an optional pointer and every existing caller already passes it or `nullptr`. `addPoints`, `processFrame`'s aging loop and `FrameToFrameOpticalFlow`'s member list are the only touched surfaces, `PatchOpticalFlow` and `MultiscaleFrameToFrameOpticalFlow` are untouched, matching Change 5's existing exception.

### Change 12, reprojection error before optimisation, `vio_reproj_pre_opt`

#### Need

The front end has no three dimensional point, so a reprojection residual can only be formed once a track is matched to an existing landmark, which happens in the association loop at `src/vi_estimator/sqrt_keypoint_vio.cpp:550-584`. `BundleAdjustmentBase::computeProjections` (`include/basalt/vi_estimator/ba_base.h:91-93`, `src/vi_estimator/ba_base.cpp:361-401`) already projects every landmark observed in the current frame through whatever pose currently sits in `frame_states`, and `SqrtKeypointVioEstimator` already calls it once, for visualisation only, well after `optimize_and_marg` has corrected that pose. Calling it before optimisation instead, using the IMU propagated prior exactly as it stands at association time, gives the residual VINS-Mono's own inlier gate is built on, the gap between where a landmark should project and where the tracker actually placed it, before anything has had a chance to correct for that gap.

What this residual actually measures is the front end's own tracking together with the VIO thread's own triangulation, neither of which has been touched by the backend optimiser yet, as distinct from a residual computed once the solver has converged. The record is named `vio_reproj_pre_opt` rather than `vio_reproj` for exactly this reason, corrected 2026-09-26 after review, so that a later addition logging the post optimisation residual has an unambiguous name of its own to take, `vio_reproj_post_opt` or similar, without either record's name implying it is the only reprojection error this stack reports.

#### Code snippet

Inserted after the association loop, before `optimize_and_marg` is called.

```cpp
{
    std::vector<Eigen::aligned_vector<Eigen::Vector4d>> proj(
        opt_flow_meas->observations.size());
    this->computeProjections(proj, last_state_t_ns);

    std::vector<double> residualsPx;
    for (size_t cam = 0; cam < proj.size(); cam++) {
        for (const Eigen::Vector4d& p : proj[cam]) {
            const KeypointId kptId = KeypointId(p[3]);
            const auto it = opt_flow_meas->observations[cam].find(kptId);
            if (it == opt_flow_meas->observations[cam].end()) continue;
            residualsPx.push_back(
                (p.template head<2>() -
                 it->second.translation().template cast<double>())
                    .norm());
        }
    }

    if (!residualsPx.empty()) {
        std::sort(residualsPx.begin(), residualsPx.end());
        const double sum =
            std::accumulate(residualsPx.begin(), residualsPx.end(), 0.0);
        const double mean = sum / double(residualsPx.size());
        const double median = residualsPx[residualsPx.size() / 2];
        const size_t p95Idx =
            size_t(0.95 * double(residualsPx.size() - 1));
        mpLogger->AddVioReprojectionPreOpt(
            opt_flow_meas->t_ns, int(residualsPx.size()), mean, median,
            residualsPx[p95Idx], residualsPx.back());
    } else {
        mpLogger->AddVioReprojectionPreOpt(opt_flow_meas->t_ns, 0, 0.0, 0.0,
                                           0.0, 0.0);
    }
    mpLogger->PrintVioReprojectionPreOpt();
}
```

`<numeric>` is added to `src/vi_estimator/sqrt_keypoint_vio.cpp`'s includes for `std::accumulate`.

The header declaration, the registry entry and the two `Logger` methods.

```cpp
// include/basalt/utils/logger.h
void AddVioReprojectionPreOpt(int64_t tNs, int n, double meanPx,
                              double medianPx, double p95Px, double maxPx);
void PrintVioReprojectionPreOpt();
```

```cpp
// src/utils/logger.cpp, constructor registry
Register("vio_reproj_pre_opt", "[VIO][reproj-pre-opt]",
         {{"t_ns", "ns"}, {"n", "count"}, {"mean_px", "f"},
          {"median_px", "f"}, {"p95_px", "f"}, {"max_px", "f"}});
```

```cpp
// src/utils/logger.cpp
void Logger::AddVioReprojectionPreOpt(int64_t tNs, int n, double meanPx,
                                      double medianPx, double p95Px,
                                      double maxPx) {
    if (!mpEnabled) return;
    AggregateStats& s = mpAggregates.at("vio_reproj_pre_opt");
    s.add_int("t_ns", tNs);
    s.add("n", double(n));
    s.add("mean_px", meanPx);
    s.add("median_px", medianPx);
    s.add("p95_px", p95Px);
    s.add("max_px", maxPx);
    s.CommitRow();
}

void Logger::PrintVioReprojectionPreOpt() { Emit("vio_reproj_pre_opt"); }
```

#### Design rationale

`computeProjections` is called against `last_state_t_ns`, which the association loop has already populated in `frame_states` as the IMU propagated prior, the same state the association loop itself used to decide connectivity, so this reuses geometry the frame has already paid for rather than adding a second projection pass. Using the prior rather than the post optimisation pose is the entire point, VINS-Mono's own 1.5 pixel inlier gate is applied before the solve for exactly this reason, a residual computed after optimisation only shows how well the solver could explain the data given everything else it already believed, not whether the raw association was trustworthy in the first place.

#### Backwards compatibility impact

`computeProjections` is a `const` template method with one existing call site, at `sqrt_keypoint_vio.cpp:806`, for visualisation, and this change adds a second call site rather than touching the first, so the visualisation path is unaffected. Nothing about the association loop's own control flow changes, this block only reads `frame_states` and `opt_flow_meas`, both already fully populated at the point it runs.

### Change 13, VIO side parallax, extending `vio_triang`

#### Need

`vio_triang.csv` already reports the baseline distance at which each candidate was triangulated, but baseline alone does not capture parallax, a short baseline at a large depth and a long baseline at a shallow depth can produce the same linear separation while the underlying viewing geometry is very different. `LocalMapper`'s own `setup_opt` already computes the correct measure, the cosine of the angle between the two viewing rays, at `src/vi_estimator/local_mapper.cpp:691-696`, citing it explicitly as ORB-SLAM3's parallax gate, and gates on it through `mpMaxCosParallax`. The VIO side triangulation loop at `src/vi_estimator/sqrt_keypoint_vio.cpp:625-723` has the identical ingredients already in scope, the two bearing vectors `p0_3d`, `p1_3d` and the relative transform `T_0_1`, and computes neither the angle nor its distribution.

#### Code snippet

Four new accumulators alongside the existing baseline and depth ones.

```cpp
Scalar minBaseline = std::numeric_limits<Scalar>::max();
Scalar maxBaseline = 0;
Scalar minDepth = std::numeric_limits<Scalar>::max();
Scalar maxDepth = 0;
Scalar sumDepth = 0;
Scalar minCosParallax = std::numeric_limits<Scalar>::max();
Scalar maxCosParallax = -std::numeric_limits<Scalar>::max();
Scalar sumCosParallax = 0;
int numParallaxSamples = 0;
```

The parallax computation, inserted immediately after the baseline is measured and before the short baseline gate, so it runs for every candidate that reaches this point regardless of whether the baseline gate later rejects it, exactly as the baseline extremes themselves are already tracked before that gate.

```cpp
const Scalar baseline = T_0_1.translation().norm();
if (baseline < minBaseline) minBaseline = baseline;
if (baseline > maxBaseline) maxBaseline = baseline;

const Vec3 ray0 = p0_3d.template head<3>().normalized();
const Vec3 ray1 = (T_0_1.so3() * p1_3d.template head<3>()).normalized();
const Scalar cosParallax = ray0.dot(ray1);
numParallaxSamples++;
if (cosParallax < minCosParallax) minCosParallax = cosParallax;
if (cosParallax > maxCosParallax) maxCosParallax = cosParallax;
sumCosParallax += cosParallax;

if (T_0_1.translation().squaredNorm() < min_triang_distance2) {
    numShortBaseline++;
    continue;
}
```

`AddVioTriang` gains four trailing parameters, mirroring the existing sum then divide pattern `depth_m_mean` already uses.

```cpp
// include/basalt/utils/logger.h
void AddVioTriang(int64_t tNs, int candidates, int added, int noPriorObs,
                  int unprojectFail, int shortBaseline, int notFinite,
                  int behind, int tooClose, double minTriangDist,
                  double baselineMin, double baselineMax, double depthMin,
                  double depthSum, double depthMax, double cosParallaxMin,
                  double cosParallaxSum, double cosParallaxMax,
                  int numParallaxSamples);
```

```cpp
// src/utils/logger.cpp, constructor registry
Register("vio_triang", "[VIO][triang]",
         {{"t_ns", "ns"}, {"candidates", "count"}, {"added", "count"},
          {"no_prior_obs", "count"}, {"unproject_fail", "count"},
          {"short_baseline", "count"}, {"not_finite", "count"},
          {"behind", "count"}, {"too_close", "count"},
          {"min_triang_dist", "f"}, {"baseline_m_min", "f"},
          {"baseline_m_max", "f"}, {"depth_m_min", "f"},
          {"depth_m_mean", "f"}, {"depth_m_max", "f"},
          {"cos_parallax_min", "f"}, {"cos_parallax_mean", "f"},
          {"cos_parallax_max", "f"}});
```

```cpp
// src/utils/logger.cpp, Logger::AddVioTriang, appended before CommitRow
s.add("cos_parallax_min",
      numParallaxSamples ? cosParallaxMin : 0.0);
s.add("cos_parallax_mean",
      numParallaxSamples ? cosParallaxSum / double(numParallaxSamples) : 0.0);
s.add("cos_parallax_max",
      numParallaxSamples ? cosParallaxMax : 0.0);
```

The call site.

```cpp
mpLogger->AddVioTriang(
    opt_flow_meas->t_ns, int(unconnected_obs0.size()),
    num_points_added, numNoPriorObs, numUnprojectFail,
    numShortBaseline, numNotFinite, numBehind, numTooClose,
    config.vio_min_triangulation_dist,
    double(maxBaseline > 0 ? minBaseline : Scalar(0)),
    double(maxBaseline), double(num_points_added ? minDepth : Scalar(0)),
    double(sumDepth), double(maxDepth), double(minCosParallax),
    double(sumCosParallax), double(maxCosParallax), numParallaxSamples);
```

#### Design rationale

This is instrumentation only, no new rejection gate is added on the VIO side. `struct VioConfig` gains no field, matching Change 6's existing decision not to add configuration surface for a still unvalidated threshold, and cementing that decision here means the low parallax population is fully visible in `vio_triang.csv` before any gate is chosen, rather than a gate being picked first and its own rejections then hidden from the very data that would justify it.

#### Backwards compatibility impact

`AddVioTriang` has exactly one call site, updated in the same change. Every existing column keeps its name, format and position, the three new columns are appended after `depth_m_max`, and `AddVioTriang`'s own accumulation of `depth_m_mean` is untouched.

### Change 14, per frame IMU excitation summary, extending `vio_imu_frame`

#### Need

Checking whether a run gave the estimator enough acceleration diversity to make scale and bias observable, the practical proxy this session's literature survey settled on in `context/slam_analysis.md`, needs a per frame representative accelerometer and gyroscope reading, and nothing today logs raw IMU vectors at frame cadence. `vio_imu_ingest.csv` logs raw `accel`/`gyro` only once every 200 samples (`Controller::GrabIMU`), far coarser than the per frame granularity a sliding window excitation check needs, and `IntegratedImuMeasurement` (`thirdparty/basalt-headers/include/basalt/imu/preintegration.h`) retains no raw sample history at all, every calibrated sample is consumed by `integrate` and discarded.

#### Code snippet

Two accumulators declared alongside the existing per frame counters in `ProcessFrame`.

```cpp
int numImuIntegrated = 0;
int numImuSkippedBehind = 0;
int64_t firstIntegratedImuTNs = -1;
int64_t lastIntegratedImuTNs = -1;
bool stretchFallbackFired = false;
int64_t stretchSpanNs = 0;
Vec3 accelSum = Vec3::Zero();
Vec3 gyroSum = Vec3::Zero();
```

Accumulated only inside the main drain loop, deliberately not in the zero order hold stretch fallback below it, so the sum and `numImuIntegrated`, which the stretch fallback also does not increment, stay over the same set of samples.

```cpp
while (imuData->t_ns <= curr_frame->t_ns) {
    meas->integrate(*imuData, this->mpAccelCov, this->mpGyroCov);
    accelSum += imuData->accel;
    gyroSum += imuData->gyro;
    if (firstIntegratedImuTNs < 0)
        firstIntegratedImuTNs = imuData->t_ns;
    lastIntegratedImuTNs = imuData->t_ns;
    numImuIntegrated++;
    if (!popFromImuDataQueueNonBlocking(imuData)) break;
    if (!imuData) return nullptr;
    imuData->accel = this->calib.calib_accel_bias.getCalibrated(imuData->accel);
    imuData->gyro = this->calib.calib_gyro_bias.getCalibrated(imuData->gyro);
}
```

The mean, formed just before the existing `AddVioImuFrame` call.

```cpp
const Vec3 accelMean = numImuIntegrated > 0
    ? Vec3(accelSum / Scalar(numImuIntegrated))
    : Vec3::Zero();
const Vec3 gyroMean = numImuIntegrated > 0
    ? Vec3(gyroSum / Scalar(numImuIntegrated))
    : Vec3::Zero();
mpLogger->AddVioImuFrame(
    curr_frame->t_ns, double(frameDtNs) * 1e-9, numImuIntegrated,
    int(std::round(double(frameDtNs) * 1e-9 * this->calib.imu_update_rate)),
    numImuSkippedBehind, firstIntegratedImuTNs, lastIntegratedImuTNs,
    coverage, double(meas->get_dt_ns()) * 1e-9, stretchFallbackFired,
    double(stretchSpanNs) * 1e-9, int(imuQueueSizeOnEntry),
    int(this->imu_data_queue.size()),
    double(imuData ? imuData->t_ns - curr_frame->t_ns : 0) * 1e-9,
    accelMean.template cast<double>(), gyroMean.template cast<double>());
```

The registry and signature gain two `vec3` columns.

```cpp
// src/utils/logger.cpp, constructor registry
Register("vio_imu_frame", "[VIO][imu]",
         {{"t_ns", "ns"}, {"frame_dt", "ms"}, {"integrated", "count"},
          {"expected", "count"}, {"skipped_behind", "count"},
          {"first_imu_ns", "ns"}, {"last_imu_ns", "ns"},
          {"coverage", "ratio"}, {"meas_dt", "ms"}, {"stretch", "flag"},
          {"stretch_span", "ms"}, {"imu_queue_in", "count"},
          {"imu_queue_out", "count"}, {"head_minus_frame", "ms"},
          {"accel_mean", "vec3"}, {"gyro_mean", "vec3"}});
```

```cpp
// include/basalt/utils/logger.h
void AddVioImuFrame(int64_t tNs, double frameDt, int integrated,
                    int expected, int skippedBehind, int64_t firstImuNs,
                    int64_t lastImuNs, double coverage, double measDt,
                    bool stretch, double stretchSpan, int imuQueueIn,
                    int imuQueueOut, double headMinusFrame,
                    const Eigen::Vector3d& accelMean,
                    const Eigen::Vector3d& gyroMean);
```

`Logger::AddVioImuFrame` gains two `s.add(...)` calls for the new columns, before `s.CommitRow()`, in the same order the registry lists them.

#### Design rationale

This logs the raw material an observability check needs, not the check itself. The check, a sliding window sample covariance of the gravity compensated specific force direction, needs the per frame orientation already in `vio_state.csv`'s `q_w_i` and the fixed gravity vector already in `vio_init.csv`'s `g` to rotate `accel_mean` into the world frame, all of which is an offline join across three already existing files rather than new C++, and belongs in `context/slam_analysis.md` as a Python analysis method rather than here. Excluding the zero order hold stretch sample from the sum is deliberate, that sample is a repeated integration of the last real reading rather than a new direction, and including it would bias the mean toward a sample already represented once.

#### Backwards compatibility impact

`AddVioImuFrame` has exactly one call site, updated in the same change. The two new columns are appended after `head_minus_frame`, every existing column is unaffected, and `numImuIntegrated`'s own meaning, and every quantity computed from it, is untouched since the stretch fallback still does not increment it.

### Change 15, BoW candidate level logging, `map_bow_candidate`

#### Need

`map_bow_query.csv` already records the winning candidate's score, `best_score`, and how many candidates cleared `config.mapper_frames_to_match_threshold`, `above_thresh`, but not the score of any candidate that did not clear it. A precision recall curve needs to sweep the acceptance threshold across its full range, which requires the score of every candidate the database returned, not only the ones already above today's configured operating point. `LocalMapper::MatchLocal`'s own retrieval loop at `src/vi_estimator/local_mapper.cpp:404-424` already holds every `(candidate, score)` pair in `results` before filtering, the loop that currently discards the sub threshold ones with a bare `continue`.

#### Code snippet

```cpp
double best_score = 0.0;
size_t above = 0;
for (const auto& otcid_score : results) {
    if (otcid_score.first.frame_id == tcid.frame_id) continue;
    best_score = std::max(best_score, otcid_score.second);
    const bool aboveThresh =
        otcid_score.second > config.mapper_frames_to_match_threshold;
    mpLogger->AddMapBowCandidate(tcid.frame_id, tcid.frame_id,
                                 otcid_score.first.frame_id,
                                 otcid_score.second, aboveThresh);
    mpLogger->PrintMapBowCandidate();
    if (!aboveThresh) continue;
    const auto idx_it = id_to_key_idx.find(otcid_score.first);
    if (idx_it == id_to_key_idx.end()) continue;
    ++above;
    match_pair m;
    m.i = i;
    m.j = idx_it->second;
    m.score = otcid_score.second;
    ids_to_match.emplace_back(m);
}
```

`config.mapper_frames_to_match_threshold`'s comparison is restated as a named `aboveThresh` local rather than the original inline `if (... <= threshold) continue;`, the control flow this produces is identical, every candidate that previously reached `ids_to_match` still does, in the same order.

The registry and the two `Logger` methods.

```cpp
// src/utils/logger.cpp, constructor registry
Register("map_bow_candidate", "[Local Mapper][bow-cand]",
         {{"t_ns", "ns"}, {"query_kf", "ns"}, {"candidate_kf", "ns"},
          {"score", "f"}, {"above_thresh", "flag"}});
```

```cpp
// src/utils/logger.cpp
void Logger::AddMapBowCandidate(int64_t tNs, int64_t queryKf,
                                int64_t candidateKf, double score,
                                bool aboveThresh) {
    if (!mpEnabled) return;
    AggregateStats& s = mpAggregates.at("map_bow_candidate");
    s.add_int("t_ns", tNs);
    s.add_int("query_kf", queryKf);
    s.add_int("candidate_kf", candidateKf);
    s.add("score", score);
    s.add("above_thresh", aboveThresh ? 1.0 : 0.0);
    s.CommitRow();
}

void Logger::PrintMapBowCandidate() { Emit("map_bow_candidate"); }
```

#### Design rationale

Every candidate the database returns is now one row, so a downstream script can join `map_bow_candidate.csv`'s `query_kf`/`candidate_kf` pair against `gt_eval.csv` by nearest timestamp, label the pair a true or a false match by the ground truth distance and temporal gap rule documented in `context/slam_analysis.md`, and sweep `score` as the threshold, exactly the sweep `evo_res` and `rpg_trajectory_evaluation` already do for trajectory metrics. `map_bow_query.csv` is left exactly as it is, since it already serves the per query summary role, `map_bow_candidate.csv` is the new per candidate detail one level below it, matching the existing relationship between `map_match_summary.csv` and `map_pair_match.csv`.

#### Backwards compatibility impact

`MatchLocal`'s control flow is unchanged, `aboveThresh` computes the same boolean the inline comparison already did, and every existing branch that follows it, `id_to_key_idx` lookup, `ids_to_match` insertion, `above` counting, keeps its original condition and order. `AddMapBowQuery`'s own call, three lines below this loop, is untouched.

### Change 16, stationarity detection and logging, extending `vio_imu_frame`

Added 2026-09-26, closing a gap this session's cross reference of the logging catalogue against `plans/vio_drift_analysis.md`'s ablation study found.

#### Need

`plans/vio_drift_analysis.md` Phase 3b proposes constraining velocity while the vehicle is stationary, and its own interpretation instruction is explicit that a run cannot be trusted without first checking the fraction of frames on which the stationarity test fires and confirming it reads false after takeoff. Nothing in the estimator computes a per frame stationarity flag today, so there is nothing to check that instruction against. This change adds the detection Phase 3b already specifies, `accelDeviationMax`, `gyroNormMax` and the two thresholds they are compared against, reusing Phase 3b's own member names so its later constraint block needs to add nothing but itself. No zero velocity residual is added here, only the test and its log, the same scope boundary `Change 13` drew for the parallax angle.

#### Code snippet

Three new members on `SqrtKeypointVioEstimator`, named exactly as `plans/vio_drift_analysis.md` Phase 3b's own snippet already declares them.

```cpp
// include/basalt/vi_estimator/sqrt_keypoint_vio.h, private members
Scalar mpStationaryAccelTol = Scalar(0.5);
Scalar mpStationaryGyroTol = Scalar(0.05);
bool mpIsStationary = false;
```

Two more accumulators declared alongside `Change 14`'s `accelSum`/`gyroSum`, folded into the same drain loop rather than a second pass.

```cpp
Vec3 accelSum = Vec3::Zero();
Vec3 gyroSum = Vec3::Zero();
Scalar accelDeviationMax = Scalar(0);
Scalar gyroNormMax = Scalar(0);
```

```cpp
while (imuData->t_ns <= curr_frame->t_ns) {
    meas->integrate(*imuData, this->mpAccelCov, this->mpGyroCov);
    accelSum += imuData->accel;
    gyroSum += imuData->gyro;
    const Scalar accelDeviation =
        std::abs(imuData->accel.norm() - Scalar(9.81));
    if (accelDeviation > accelDeviationMax) accelDeviationMax = accelDeviation;
    const Scalar gyroNorm = imuData->gyro.norm();
    if (gyroNorm > gyroNormMax) gyroNormMax = gyroNorm;
    if (firstIntegratedImuTNs < 0)
        firstIntegratedImuTNs = imuData->t_ns;
    lastIntegratedImuTNs = imuData->t_ns;
    numImuIntegrated++;
    if (!popFromImuDataQueueNonBlocking(imuData)) break;
    if (!imuData) return nullptr;
    imuData->accel = this->calib.calib_accel_bias.getCalibrated(imuData->accel);
    imuData->gyro = this->calib.calib_gyro_bias.getCalibrated(imuData->gyro);
}
```

The test itself, formed alongside `accelMean`/`gyroMean` just before the `AddVioImuFrame` call.

```cpp
mpIsStationary = numImuIntegrated > 0 &&
                accelDeviationMax < mpStationaryAccelTol &&
                gyroNormMax < mpStationaryGyroTol;
```

The registry gains three columns, the signature three parameters, in the same order.

```cpp
// src/utils/logger.cpp, constructor registry
Register("vio_imu_frame", "[VIO][imu]",
         {{"t_ns", "ns"}, {"frame_dt", "ms"}, {"integrated", "count"},
          {"expected", "count"}, {"skipped_behind", "count"},
          {"first_imu_ns", "ns"}, {"last_imu_ns", "ns"},
          {"coverage", "ratio"}, {"meas_dt", "ms"}, {"stretch", "flag"},
          {"stretch_span", "ms"}, {"imu_queue_in", "count"},
          {"imu_queue_out", "count"}, {"head_minus_frame", "ms"},
          {"accel_mean", "vec3"}, {"gyro_mean", "vec3"},
          {"accel_deviation_max", "f"}, {"gyro_norm_max", "f"},
          {"is_stationary", "flag"}});
```

```cpp
// include/basalt/utils/logger.h
void AddVioImuFrame(int64_t tNs, double frameDt, int integrated,
                    int expected, int skippedBehind, int64_t firstImuNs,
                    int64_t lastImuNs, double coverage, double measDt,
                    bool stretch, double stretchSpan, int imuQueueIn,
                    int imuQueueOut, double headMinusFrame,
                    const Eigen::Vector3d& accelMean,
                    const Eigen::Vector3d& gyroMean,
                    double accelDeviationMax, double gyroNormMax,
                    bool isStationary);
```

`Logger::AddVioImuFrame` gains three more `s.add(...)` calls, in the same order as the registry, before `s.CommitRow()`. The call site gains three trailing arguments after `gyroMean.template cast<double>()`.

```cpp
mpLogger->AddVioImuFrame(
    curr_frame->t_ns, double(frameDtNs) * 1e-9, numImuIntegrated,
    int(std::round(double(frameDtNs) * 1e-9 * this->calib.imu_update_rate)),
    numImuSkippedBehind, firstIntegratedImuTNs, lastIntegratedImuTNs,
    coverage, double(meas->get_dt_ns()) * 1e-9, stretchFallbackFired,
    double(stretchSpanNs) * 1e-9, int(imuQueueSizeOnEntry),
    int(this->imu_data_queue.size()),
    double(imuData ? imuData->t_ns - curr_frame->t_ns : 0) * 1e-9,
    accelMean.template cast<double>(), gyroMean.template cast<double>(),
    double(accelDeviationMax), double(gyroNormMax), mpIsStationary);
```

#### Design rationale

`mpStationaryAccelTol`, `mpStationaryGyroTol` and `mpIsStationary` are named and typed exactly as `plans/vio_drift_analysis.md` Phase 3b's own snippet already declares them, so Phase 3b's own future addition, the zero velocity residual block in `optimize`, reads a member this change already computes rather than introducing its own detection a second time. `9.81` is the constant gravity magnitude already hard coded in this estimator's own initial attitude solve at `sqrt_keypoint_vio.cpp:268-269`, matched here rather than introducing a second gravity constant. Logging both the boolean and the two maxima it is thresholded against, rather than the boolean alone, is deliberate, Phase 3b's own justification section wants the thresholds themselves checked against real data, and a boolean cannot show how close a frame sat to the threshold, only whether it crossed it.

#### Backwards compatibility impact

`AddVioImuFrame` has exactly one call site, updated in the same change as `Change 14`'s own extension of it, so the two are naturally applied together. `mpIsStationary` defaults to `false` and is written by `ProcessFrame` alone, no other method reads or sets it, so nothing in the estimator's behaviour changes, this stays a detection and a log, not yet a constraint.

### Change 17, parallax rejection gate and its counter, extending `vio_triang` and `struct VioConfig`

Added 2026-09-26, closing a second gap this session's cross reference found.

#### Need

`Change 13` logs the cosine parallax distribution for every triangulation candidate but rejects none of them on it. `plans/vio_drift_analysis.md` Phase 2's own interpretation section needs the count a real gate would produce, `low_parallax`, alongside `Change 13`'s distribution, since a falling `tri_added` is only diagnosable against a rising rejection count on the same axis. Phase 2 already specifies the gate in full, a new `vio_min_triangulation_parallax_deg` configuration field defaulting to zero degrees, which reproduces today's behaviour exactly and is the same recipe `Change 6` already established for a trailing defaulted parameter. This change builds that gate and its counter, reusing `Change 13`'s own `ray0`/`ray1`/`cosParallax` locals rather than recomputing them. Whether to actually activate the gate on `data/sitl_config_vo.json`, at Phase 2's proposed 0.9 degrees, is Phase 2's own experiment to run, and is left at the inactive default here.

#### Code snippet

A new field on `struct VioConfig`, beside the existing `vio_min_triangulation_dist`.

```cpp
// include/basalt/utils/vio_config.h
double vio_min_triangulation_dist;
/// Minimum parallax angle in degrees required between the two bearings of
/// a triangulation candidate. Zero leaves the test inactive, reproducing
/// the behaviour of every configuration written before this field existed.
double vio_min_triangulation_parallax_deg;
```

```cpp
// src/utils/vio_config.cpp, VioConfig::VioConfig and serialize
vio_min_triangulation_dist = 0.05;
vio_min_triangulation_parallax_deg = 0.0;
```

```cpp
ar(CEREAL_NVP(config.vio_min_triangulation_dist));
ar(CEREAL_NVP(config.vio_min_triangulation_parallax_deg));
```

The gate itself, one more local beside `min_triang_distance2` and one more tally beside `numShortBaseline`.

```cpp
const Scalar min_triang_distance2 =
    Scalar(config.vio_min_triangulation_dist *
           config.vio_min_triangulation_dist);
const Scalar minParallaxCos =
    config.vio_min_triangulation_parallax_deg > 0
        ? std::cos(Scalar(config.vio_min_triangulation_parallax_deg) *
                   Scalar(M_PI / 180.0))
        : Scalar(1);
```

```cpp
int numNoPriorObs = 0, numUnprojectFail = 0, numShortBaseline = 0;
int numNotFinite = 0, numBehind = 0, numTooClose = 0, numLowParallax = 0;
```

`Change 13`'s own parallax block gains one gate, inserted after the short baseline gate it already sits beside.

```cpp
const Scalar baseline = T_0_1.translation().norm();
if (baseline < minBaseline) minBaseline = baseline;
if (baseline > maxBaseline) maxBaseline = baseline;

const Vec3 ray0 = p0_3d.template head<3>().normalized();
const Vec3 ray1 = (T_0_1.so3() * p1_3d.template head<3>()).normalized();
const Scalar cosParallax = ray0.dot(ray1);
numParallaxSamples++;
if (cosParallax < minCosParallax) minCosParallax = cosParallax;
if (cosParallax > maxCosParallax) maxCosParallax = cosParallax;
sumCosParallax += cosParallax;

if (T_0_1.translation().squaredNorm() < min_triang_distance2) {
    numShortBaseline++;
    continue;
}

if (minParallaxCos < Scalar(1) && cosParallax > minParallaxCos) {
    numLowParallax++;
    // Currently low parallax frames also added. This gate will be checked later. Currently only computation is done. Leaving this here for future implementation.
    // continue;
}
```

`AddVioTriang` gains one trailing parameter and `vio_triang`'s registry gains one column.

```cpp
// include/basalt/utils/logger.h
void AddVioTriang(int64_t tNs, int candidates, int added, int noPriorObs,
                  int unprojectFail, int shortBaseline, int notFinite,
                  int behind, int tooClose, double minTriangDist,
                  double baselineMin, double baselineMax, double depthMin,
                  double depthSum, double depthMax, double cosParallaxMin,
                  double cosParallaxSum, double cosParallaxMax,
                  int numParallaxSamples, int lowParallax);
```

```cpp
// src/utils/logger.cpp, constructor registry
Register("vio_triang", "[VIO][triang]",
         {{"t_ns", "ns"}, {"candidates", "count"}, {"added", "count"},
          {"no_prior_obs", "count"}, {"unproject_fail", "count"},
          {"short_baseline", "count"}, {"not_finite", "count"},
          {"behind", "count"}, {"too_close", "count"},
          {"min_triang_dist", "f"}, {"baseline_m_min", "f"},
          {"baseline_m_max", "f"}, {"depth_m_min", "f"},
          {"depth_m_mean", "f"}, {"depth_m_max", "f"},
          {"cos_parallax_min", "f"}, {"cos_parallax_mean", "f"},
          {"cos_parallax_max", "f"}, {"low_parallax", "count"}});
```

`Logger::AddVioTriang` gains `s.add("low_parallax", double(lowParallax));` before `s.CommitRow()`. The call site gains `numLowParallax` as its final argument.

```cpp
mpLogger->AddVioTriang(
    opt_flow_meas->t_ns, int(unconnected_obs0.size()),
    num_points_added, numNoPriorObs, numUnprojectFail,
    numShortBaseline, numNotFinite, numBehind, numTooClose,
    config.vio_min_triangulation_dist,
    double(maxBaseline > 0 ? minBaseline : Scalar(0)),
    double(maxBaseline), double(num_points_added ? minDepth : Scalar(0)),
    double(sumDepth), double(maxDepth), double(minCosParallax),
    double(sumCosParallax), double(maxCosParallax), numParallaxSamples,
    numLowParallax);
```

#### Design rationale

The gate sits after the short baseline gate rather than before it, matching Phase 2's own ordering and meaning a candidate that already failed on baseline is never double counted against the parallax tally. `minParallaxCos < Scalar(1)` is the inactive test, at the default of zero degrees `minParallaxCos` is exactly `Scalar(1)` and no `cosParallax`, itself bounded in `[-1, 1]`, can exceed it except by the same floating point graze that exactly parallel bearings would produce regardless, so a default configuration's `low_parallax` column reads zero throughout and every other column is what `Change 13` alone would have produced.

#### Backwards compatibility impact

Adding a field to `serialize` is a breaking change for every configuration file that predates it, `Change 6`'s own discussion and `../context/vio_config_json.md` both record that a key `serialize` reads and a file omits throws an uncaught `cereal::Exception` and aborts the process. All eight configuration sources, the seven files matching `data/*config*.json` and the `[value0]` table in `data/iccv21/basalt_batch_config.toml`, must gain `vio_min_triangulation_parallax_deg` at `0.0` in the same commit, which reproduces every existing run's behaviour exactly. Setting `data/sitl_config_vo.json` to `0.9`, Phase 2's own proposed value, is not done here, activating the experiment is Phase 2's own step in `plans/vio_drift_analysis.md`, not a logging change. `AddVioTriang` has exactly one call site, updated in the same change as `Change 13`.

### Change 18, run manifest, copying the calibration and configuration into the log directory

Added 2026-09-26, closing the third gap this session's cross reference found, per the direction to copy the files rather than log their contents.

#### Need

No record in this catalogue carries the configuration or calibration a run was flown under, and every phase of `plans/vio_drift_analysis.md`'s ablation study changes exactly one such value and scores the run against it. Today that pairing exists only in whichever operator remembers which JSON file was loaded for which log directory, which does not survive a run being revisited later or compared by a dashboard that only reads CSVs. `Controller::load_config` already reads `config_path_` and `calib_path_` at `src/controller.cpp:113-134`, and `Controller::initialize` already knows `logDirectory` at the point it constructs `mpLogger`, `src/controller.cpp:159-160`, so both halves of the pairing are already in scope at the one place a run's log directory is decided.

#### Code snippet

```cpp
// src/controller.cpp, added to the includes
#include <basalt/utils/filesystem.h>
```

```cpp
    mpLogger =
        std::make_shared<basalt::Logger>(logDirectory, vio_config_.vio_debug);

    // Every run's own calibration and configuration travel with its CSVs
    // under fixed names, so a later comparison never has to guess which
    // file on disk a given log directory was flown under.
    if (!logDirectory.empty()) {
        std::error_code ec;
        if (!config_path_.empty())
            basalt::fs::copy_file(
                config_path_, logDirectory + "/config.json",
                basalt::fs::copy_options::overwrite_existing, ec);
        if (!calib_path_.empty())
            basalt::fs::copy_file(
                calib_path_, logDirectory + "/calibration.json",
                basalt::fs::copy_options::overwrite_existing, ec);
    }
```

#### Design rationale

A file copy rather than a new CSV record, per the direction that settled this change, because the configuration and the calibration are already complete, versioned JSON documents, and restating their fields as logged columns would only create a second, lossier copy of something that already exists in full. Fixed destination names, `config.json` and `calibration.json`, rather than the source basename, are deliberate, a cross run dashboard can then open `<run>/config.json` for any run without first discovering what its author happened to name the source file. The `std::error_code` overload of `basalt::fs::copy_file` is used rather than the throwing one, a missing or unreadable source file is worth noting but must never abort a live SLAM session over a convenience copy. `basalt::fs`, the portable alias in `include/basalt/utils/filesystem.h`, is already used by `src/mapper.cpp`, `src/mapper_sim.cpp` and three other translation units in this codebase, so this introduces no new dependency.

#### Backwards compatibility impact

Note, 2026-10-23. The `mpLogger` construction shown in the snippet above is superseded by `Change 24`, which gates it on `vio_config_.vio_debug`, while the copy block is unchanged and stays ungated. `Controller::initialize`'s only new statement runs when `logDirectory` is non empty, the same guard the CSV sink itself already uses, so a caller that leaves `logDirectory` at its default empty string sees no new behaviour and no new file. `config_path_` and `calib_path_` are read, never written, so `load_config`'s own use of them is unaffected. `copy_options::overwrite_existing` means calling `initialize` twice on the same `logDirectory`, which the ground truth triggered reseed path in `Controller::TrackMonocular` does not do today but a future caller might, replaces rather than errors on the second copy.

### Change 19 through Change 23, the invoking timestamp gap

An audit of `/ws/logs/2026-09-26_16-37-51/`'s own headers, prompted directly by the session that requested this section, found seven records written with no column identifying the frame or keyframe whose processing produced the row, `map_pair_match.csv`, `mapper_detect.csv`, `mapper_iter_summary.csv`, `mapper_linearise.csv`, `mapper_solver_iter.csv`, `mapper_stereo.csv` and `vio_solver_timing.csv`, plus one further record, `vio_marg_timing.csv`, that already carries the right value under the name `frame_id` rather than the `t_ns` every other record uses. Local mapping in this codebase processes exactly one keyframe per `MapLocally` cycle, confirmed at `src/vi_estimator/local_mapper.cpp:99-113`, where the `try_pop` loop that would once have collected more than one keyframe per cycle is commented out and `mpKFInputQueue->pop(kf)` blocks for a single keyframe before the cycle proceeds, and the VIO estimator likewise processes exactly one frame per `ProcessFrame` call, so every one of these eight records has exactly one correct invoking timestamp to log, never an ambiguous choice between several. Two further callers of the mapper's own shared base, the offline batch tools `src/mapper.cpp` and `src/mapper_sim.cpp`, were checked against every fix below, since `class NfrMapper` is shared between the live `LocalMapper` and these two standalone executables, and neither tool has a single new keyframe concept, they optimise a whole batch of frames at once. Every fix below derives its timestamp from state the shared base already holds, `frame_poses`, `img_data` or the local keyframe list a method is already iterating, rather than from a `LocalMapper`-only member such as `mpNewKeyframesForTracking`, so `class NfrMapper` itself stays as valid a base for the batch tools as it is today, and both tools compile and behave exactly as before without being touched.

### Change 19, invoking timestamp on `vio_solver_timing`

#### Need

`AddVioSolverTiming` is called immediately before `AddVioSolverIter` at `src/vi_estimator/sqrt_keypoint_vio.cpp:1794`, the latter already passing `last_state_t_ns` as its own first argument. `vio_solver_timing.csv` is the one record in the VIO estimator's own per iteration cluster, `vio_linearise.csv`, `vio_solver_iter.csv`, `vio_solver_timing.csv`, that does not carry it, breaking the join across the three that a per iteration timing breakdown needs.

#### Code snippet

```cpp
// include/basalt/utils/logger.h
void AddVioSolverTiming(int64_t tNs, int numCams, int numLms, int numObs,
                       double allocateLmb, double linearizeProblem,
                       double performQr, double getDenseHB, double solve,
                       double backSubstitute, double computeError2,
                       double iteration, double residentMemory,
                       double residentMemoryPeak);
```

```cpp
// src/utils/logger.cpp, constructor registry
Register("vio_solver_timing", "[VIO][solver-timing]",
         {{"t_ns", "ns"}, {"num_cams", "count"}, {"num_lms", "count"},
          {"num_obs", "count"}, {"allocateLMB", "ms"},
          {"linearizeProblem", "ms"}, {"performQR", "ms"},
          {"get_dense_H_b", "ms"}, {"solve", "ms"},
          {"backSubstitute", "ms"}, {"computerError2", "ms"},
          {"iteration", "ms"}, {"resident_memory", "f"},
          {"resident_memory_peak", "f"}});
```

```cpp
// src/utils/logger.cpp
void Logger::AddVioSolverTiming(int64_t tNs, int numCams, int numLms,
                                int numObs, double allocateLmb,
                                double linearizeProblem, double performQr,
                                double getDenseHB, double solve,
                                double backSubstitute, double computeError2,
                                double iteration, double residentMemory,
                                double residentMemoryPeak) {
    if (!mpEnabled) return;
    AggregateStats& s = mpAggregates.at("vio_solver_timing");
    s.add_int("t_ns", tNs);
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
```

```cpp
// src/vi_estimator/sqrt_keypoint_vio.cpp:1794
mpLogger->AddVioSolverTiming(
    last_state_t_ns, numCams, numLms, numObs, allocateLmbSeconds,
    linearizeProblemSeconds, performQrSeconds, getDenseHBSeconds,
    solveSeconds, backSubstituteSeconds, computeError2Seconds,
    iteration_time, residentMemory, residentMemoryPeak);
```

#### Design rationale

`last_state_t_ns` is the same value `AddVioSolverIter` two lines below already logs for this exact iteration, so `vio_solver_timing.csv` and `vio_solver_iter.csv` become directly joinable on `t_ns` and `iteration`/`it`'s own position within a frame's solve, closing the one gap in the VIO estimator's own per iteration cluster.

#### Backwards compatibility impact

`AddVioSolverTiming` has exactly one call site, updated in this same change, so the signature change is not a compile hazard anywhere else. The eleven existing columns keep their name, order and unit, `t_ns` is a new leading column, so a reader of the old header schema fails obviously, a wrong column count, rather than silently reading a shifted value into the wrong name.

### Change 20, renaming `vio_marg_timing`'s `frame_id` to `t_ns`

#### Need

`AddVioMargTiming(last_state_t_ns, ...)`, called at `src/vi_estimator/sqrt_keypoint_vio.cpp:1425`, already receives the correct invoking timestamp, `vio_marg_timing.csv` is not missing data. The column is named `frame_id` rather than `t_ns`, the one record among the thirty nine this plan now produces that uses a different name for the same join key, which breaks a generic `t_ns` keyed join across every file rather than requiring a per file exception list.

#### Code snippet

```cpp
// include/basalt/utils/logger.h
void AddVioMargTiming(int64_t tNs, double measure, double margLinearize,
                     double margHelper, double marg, double margLog,
                     double marginalize);
```

```cpp
// src/utils/logger.cpp, constructor registry
Register("vio_marg_timing", "[VIO][marg-timing]",
         {{"t_ns", "ns"}, {"measure", "ms"},
          {"marg_linearize", "ms"}, {"marg_helper", "ms"},
          {"marg", "ms"}, {"marg_log", "ms"}, {"marginalize", "ms"}});
```

```cpp
// src/utils/logger.cpp
void Logger::AddVioMargTiming(int64_t tNs, double measure,
                              double margLinearize, double margHelper,
                              double marg, double margLog,
                              double marginalize) {
    if (!mpEnabled) return;
    AggregateStats& s = mpAggregates.at("vio_marg_timing");
    s.add_int("t_ns", tNs);
    s.add("measure", measure);
    s.add("marg_linearize", margLinearize);
    s.add("marg_helper", margHelper);
    s.add("marg", marg);
    s.add("marg_log", margLog);
    s.add("marginalize", marginalize);
    s.CommitRow();
}
```

The call site at `src/vi_estimator/sqrt_keypoint_vio.cpp:1425` passes the same `last_state_t_ns` it already does, only the parameter name changes, so the diff there is limited to the declaration this call resolves against.

#### Design rationale

A pure rename rather than a new column, since the value logged does not change, only the reader's ability to write one join expression, `on t_ns`, across every file rather than special casing this one record's own column name.

#### Backwards compatibility impact

`AddVioMargTiming` has exactly one call site, unchanged in argument order or value, only the parameter's local name changes at the declaration, which does not affect callers. No analysis tooling reads `vio_marg_timing.csv` today, `scripts/vio_log_extract.py` reads console text, not this CSV, so no existing consumer breaks.

### Change 21, invoking timestamp on `map_pair_match`

#### Need

`AddMapPairMatch`, called at `src/vi_estimator/local_mapper.cpp:514` inside `LocalMapper::MatchLocal`, logs `kf1` and `kf2`, the two keyframes being matched, but not which cycle's invocation produced the attempt. `kf1` happens to equal the invoking keyframe on every call today, both branches that populate `ids_to_match`, the Bag of Words retrieval loop at `:400-431` and the temporal neighbour loop at `:451-484`, only ever set `m.i` to a new keyframe's index, `id1 = keys[ids_to_match[j].i]` at `:495` is therefore always the invoking keyframe. This is an invariant a reader would have to discover by reading the loop bodies rather than one the file states, and it is exactly the kind of implicit convention the session that requested this change flagged as the problem, an explicit `t_ns` column removes the need to trust it.

#### Code snippet

```cpp
// include/basalt/utils/logger.h
void AddMapPairMatch(int64_t tNs, int64_t kf1, int64_t kf2,
                    int descriptorMatches, int minMatches,
                    int ransacInliers, bool stored);
```

```cpp
// src/utils/logger.cpp, constructor registry
Register("map_pair_match", "[Local Mapper][match]",
         {{"t_ns", "ns"}, {"kf1", "ns"}, {"kf2", "ns"},
          {"descriptor_matches", "count"}, {"min_matches", "count"},
          {"ransac_inliers", "count"}, {"stored", "flag"}});
```

```cpp
// src/utils/logger.cpp
void Logger::AddMapPairMatch(int64_t tNs, int64_t kf1, int64_t kf2,
                             int descriptorMatches, int minMatches,
                             int ransacInliers, bool stored) {
    if (!mpEnabled) return;
    AggregateStats& s = mpAggregates.at("map_pair_match");
    s.add_int("t_ns", tNs);
    s.add_int("kf1", kf1);
    s.add_int("kf2", kf2);
    s.add("descriptor_matches", double(descriptorMatches));
    s.add("min_matches", double(minMatches));
    s.add("ransac_inliers", double(ransacInliers));
    s.add("stored", stored ? 1.0 : 0.0);
    s.CommitRow();
}
```

```cpp
// src/vi_estimator/local_mapper.cpp, top of LocalMapper::MatchLocal,
// hoisted above every other local so the match_func lambda at :491, which
// captures by reference, sees it
void LocalMapper::MatchLocal() {
    int neighbourPairs = 0;
    const int64_t matchTNs = mpNewKeyframesForTracking.empty()
                                 ? 0
                                 : *mpNewKeyframesForTracking.rbegin();
    std::vector<TimeCamId> keys;
    ...
```

```cpp
// src/vi_estimator/local_mapper.cpp:514, inside match_func
mpLogger->AddMapPairMatch(
    matchTNs, id1.frame_id, id2.frame_id, int(md.matches.size()),
    config.mapper_min_matches, int(md.inliers.size()),
    !md.inliers.empty());
```

The pre-existing `const int64_t matchTNs = ...` declaration further down `MatchLocal`, at what was `:545` before this hoist, is deleted, its one remaining use, `AddMapMatchSummary`'s own call, reads the hoisted declaration instead.

#### Design rationale

`mpNewKeyframesForTracking.rbegin()` is the same idiom `AddMapMatchSummary` and `AddMapTracks` already use elsewhere in this file, so this is not a new convention, it is applying the existing one to the one call site in `MatchLocal` that had not yet adopted it. Hoisting the declaration to the top of the function rather than computing it twice keeps one definition rather than two that must agree, `mpNewKeyframesForTracking` does not change during `MatchLocal`'s own execution, it is populated once per cycle before `MatchLocal` is called and read only from within it, so moving the read earlier changes nothing about what value it observes.

#### Backwards compatibility impact

`AddMapPairMatch` has exactly one call site, updated in this same change. `kf1` and `kf2` keep their existing meaning and position relative to each other, only `t_ns` is newly prepended, so a downstream reader who was already using `kf1` as the invoking keyframe, relying on the implicit invariant described above, keeps getting the same value, now under an explicit name as well.

### Change 22, invoking timestamp on `mapper_detect` and `mapper_stereo`

#### Need

`NfrMapper::detect_keypoints()` and `NfrMapper::match_stereo()`, `src/vi_estimator/nfr_mapper.cpp:500-536` and `:538-579`, each process the frames currently held in `img_data`, filtered to the invoking cycle's own new keyframe before either is called when the caller is `LocalMapper::MapLocally`, and log a per call summary, `mapper_detect.csv`'s `frames` column in the audited run reads `1` on every row, a count rather than a timestamp, and `mapper_stereo.csv` logs no frame identifier of any kind.

#### Code snippet

```cpp
// include/basalt/utils/logger.h
void AddMapperDetect(int64_t tNs, int frames, double detection,
                    int cornersTotal);
void AddMapperStereo(int64_t tNs, int pairs, int matches, int inliers);
```

```cpp
// src/utils/logger.cpp, constructor registry
Register("mapper_detect", "[Mapper][detect]",
         {{"t_ns", "ns"}, {"frames", "count"}, {"detection", "ms"},
          {"corners_total", "count"}});

Register("mapper_stereo", "[Mapper][stereo]",
         {{"t_ns", "ns"}, {"pairs", "count"}, {"matches", "count"},
          {"inliers", "count"}});
```

```cpp
// src/utils/logger.cpp
void Logger::AddMapperDetect(int64_t tNs, int frames, double detection,
                             int cornersTotal) {
    if (!mpEnabled) return;
    AggregateStats& s = mpAggregates.at("mapper_detect");
    s.add_int("t_ns", tNs);
    s.add("frames", double(frames));
    s.add("detection", detection);
    s.add("corners_total", double(cornersTotal));
    s.CommitRow();
}

void Logger::AddMapperStereo(int64_t tNs, int pairs, int matches,
                             int inliers) {
    if (!mpEnabled) return;
    AggregateStats& s = mpAggregates.at("mapper_stereo");
    s.add_int("t_ns", tNs);
    s.add("pairs", double(pairs));
    s.add("matches", double(matches));
    s.add("inliers", double(inliers));
    s.CommitRow();
}
```

```cpp
// src/vi_estimator/nfr_mapper.cpp, NfrMapper::detect_keypoints, replacing
// the existing corners_total loop's bounds so the same pass also tracks
// the newest key, no new pass over keys is added
int cornersTotal = 0;
int64_t detectTNs = 0;
for (const int64_t key : keys) {
    if (key > detectTNs) detectTNs = key;
    for (size_t cam = 0; cam < calib.intrinsics.size(); cam++) {
        const auto it = feature_corners.find(TimeCamId(key, cam));
        if (it != feature_corners.end())
            cornersTotal += int(it->second.corners.size());
    }
}

mpLogger->AddMapperDetect(detectTNs, int(keys.size()),
                          elapsed1.count() * 1e-6, cornersTotal);
mpLogger->PrintMapperDetect();
```

```cpp
// src/vi_estimator/nfr_mapper.cpp, NfrMapper::match_stereo, computed once
// before the img_data loop since img_data is a std::unordered_map and so
// has no ordering to read the newest key from directly
int64_t stereoTNs = 0;
for (const auto& kv : img_data)
    if (kv.first > stereoTNs) stereoTNs = kv.first;

int num_matches = 0;
int num_inliers = 0;

for (const auto& kv : img_data) {
    ...  // unchanged body
}

mpLogger->AddMapperStereo(stereoTNs, int(img_data.size()), num_matches,
                          num_inliers);
mpLogger->PrintMapperStereo();
```

#### Design rationale

Both fixes derive the timestamp from state the function already owns, `keys` in `detect_keypoints`, `img_data` in `match_stereo`, taking the maximum rather than assuming a single element, since `class NfrMapper` itself, unlike `LocalMapper`, has no guarantee that only one frame is ever processed per call, the offline `src/mapper.cpp` and `src/mapper_sim.cpp` call both methods over a full batch. The newest frame in the set being processed is the same newest-wins convention `frame_poses.rbegin()->first` already uses for `cycleTNs` in `LocalMapper::MapLocally`, applied here to `NfrMapper`'s own state rather than `LocalMapper`'s, so no `LocalMapper`-only member is referenced from the shared base. `img_data` is `std::unordered_map<int64_t, OpticalFlowInput::Ptr>`, confirmed at `include/basalt/vi_estimator/nfr_mapper.h:207`, an explicit linear scan for the maximum key is used rather than `rbegin()`, which an unordered container does not order meaningfully.

#### Backwards compatibility impact

Both methods have call sites in `LocalMapper::MapLocally` and in the offline `src/mapper.cpp`, neither call passes the timestamp explicitly, both are internal to `detect_keypoints`/`match_stereo` and require no caller change at all. On the single new keyframe path this codebase actually runs, `keys.size() == 1` and `img_data.size() == 1` in the live stack, so the computed maximum equals that one frame's own timestamp exactly, the same value a caller-supplied parameter would have carried, with no signature change and therefore no risk to the two offline tools.

### Change 23, invoking timestamp and pass discriminator on the mapper's own optimiser

#### Need

`NfrMapper::optimize(int num_iterations)`, `src/vi_estimator/nfr_mapper.cpp:254-437`, logs `AddMapperLinearise`, `AddMapperSolverIter` and `AddMapperIterSummary` with only an `iter` column, the inner loop counter, and no timestamp at all. `LocalMapper::MapLocally` calls `optimize(mpOptIterations)` twice per cycle, once at `:204` before `filterOutliers`, once at `:227` after it, so adding only a timestamp would still leave every row from the first call indistinguishable from the matching `iter` row of the second, both would share the same `t_ns` and the same `iter` sequence starting at zero. This is a second, narrower gap than the other four changes in this section, closing it needs a discriminator between the two calls as well as a timestamp.

#### Code snippet

```cpp
// include/basalt/vi_estimator/nfr_mapper.h, optimize keeps its own default
// so src/mapper_sim.cpp:520's optimize() and src/mapper.cpp:634's
// optimize(num_opt_iter) both keep compiling unchanged
void optimize(int num_iterations = 10, int opt_pass = 0);
```

```cpp
// include/basalt/utils/logger.h
void AddMapperLinearise(int64_t tNs, int optPass, int iter, double vision,
                       double relError, double rollPitchError,
                       double total, int landmarks, int observations,
                       int relPoseFactors, int rollPitchFactors);

void AddMapperSolverIter(int64_t tNs, int optPass, int iter, int inner,
                        int status, double lambda, double fDiff,
                        double maxInc, double visionError,
                        double relError, double rollPitchError,
                        double total, bool converged, bool errorIncreased);

void AddMapperIterSummary(int64_t tNs, int optPass, int iter,
                         double iteration, int numStates, int numPoses,
                         int innerSteps);
```

```cpp
// src/utils/logger.cpp, constructor registry
Register("mapper_linearise", "[Mapper][linearize]",
         {{"t_ns", "ns"}, {"opt_pass", "enum"}, {"iter", "count"},
          {"vision", "e"}, {"rel_error", "e"}, {"roll_pitch_error", "e"},
          {"total", "e"}, {"landmarks", "count"}, {"observations", "count"},
          {"rel_pose_factors", "count"}, {"roll_pitch_factors", "count"}});

Register("mapper_solver_iter", "[Mapper][solver]",
         {{"t_ns", "ns"}, {"opt_pass", "enum"}, {"iter", "count"},
          {"inner", "count"}, {"status", "enum"}, {"lambda", "e"},
          {"f_diff", "e"}, {"max_inc", "e"}, {"vision_error", "e"},
          {"rel_error", "e"}, {"roll_pitch_error", "e"}, {"total", "e"},
          {"converged", "flag"}, {"error_increased", "flag"}});

Register("mapper_iter_summary", "[Mapper][iter]",
         {{"t_ns", "ns"}, {"opt_pass", "enum"}, {"iter", "count"},
          {"iteration", "ms"}, {"num_states", "count"},
          {"num_poses", "count"}, {"inner_steps", "count"}});
```

```cpp
// src/utils/logger.cpp
void Logger::AddMapperLinearise(int64_t tNs, int optPass, int iter,
                                double vision, double relError,
                                double rollPitchError, double total,
                                int landmarks, int observations,
                                int relPoseFactors, int rollPitchFactors) {
    if (!mpEnabled) return;
    AggregateStats& s = mpAggregates.at("mapper_linearise");
    s.add_int("t_ns", tNs);
    s.add("opt_pass", double(optPass));
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

void Logger::AddMapperSolverIter(int64_t tNs, int optPass, int iter,
                                 int inner, int status, double lambda,
                                 double fDiff, double maxInc,
                                 double visionError, double relError,
                                 double rollPitchError, double total,
                                 bool converged, bool errorIncreased) {
    if (!mpEnabled) return;
    AggregateStats& s = mpAggregates.at("mapper_solver_iter");
    s.add_int("t_ns", tNs);
    s.add("opt_pass", double(optPass));
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

void Logger::AddMapperIterSummary(int64_t tNs, int optPass, int iter,
                                  double iteration, int numStates,
                                  int numPoses, int innerSteps) {
    if (!mpEnabled) return;
    AggregateStats& s = mpAggregates.at("mapper_iter_summary");
    s.add_int("t_ns", tNs);
    s.add("opt_pass", double(optPass));
    s.add("iter", double(iter));
    s.add("iteration", iteration);
    s.add("num_states", double(numStates));
    s.add("num_poses", double(numPoses));
    s.add("inner_steps", double(innerSteps));
    s.CommitRow();
}
```

```cpp
// src/vi_estimator/nfr_mapper.cpp, NfrMapper::optimize, one new local
// before the existing iteration loop, threaded into all three calls
// already inside it
void NfrMapper::optimize(int num_iterations, int opt_pass) {
    AbsOrderMap aom;
    const int64_t invokingTNs =
        frame_poses.empty() ? 0 : frame_poses.rbegin()->first;

    for (const auto& kv : frame_poses) {
        ...  // unchanged
    }

    for (int iter = 0; iter < num_iterations; iter++) {
        ...
        mpLogger->AddMapperLinearise(
            invokingTNs, opt_pass, iter, rld_error, lopt.rel_error,
            lopt.roll_pitch_error, error_total, int(lmdb.numLandmarks()),
            int(lmdb.numObservations()), int(rel_pose_factors.size()),
            int(roll_pitch_factors.size()));
        ...
        mpLogger->AddMapperSolverIter(
            invokingTNs, opt_pass, iter, inner, 1, lambda, f_diff, max_inc,
            after_update_vision_error, after_rel_error,
            after_roll_pitch_error, after_error_total, converged,
            errorIncreased);
        // and the mirrored call three lines below with status 0
        ...
        mpLogger->AddMapperIterSummary(
            invokingTNs, opt_pass, iter, double(elapsed.count()) * 1e-6,
            int(frame_states.size()), int(frame_poses.size()), inner);
        ...
    }
}
```

```cpp
// src/vi_estimator/local_mapper.cpp, LocalMapper::MapLocally, the two
// existing calls gain a pass number, 0 before landmark filtering, 1 after
tStep = std::chrono::high_resolution_clock::now();
optimize(mpOptIterations, 0);
tEnd = std::chrono::high_resolution_clock::now();
const double optimize1Seconds =
    std::chrono::duration<double>(tEnd - tStep).count();

...  // filterOutliers unchanged

tStep = std::chrono::high_resolution_clock::now();
optimize(mpOptIterations, 1);
tEnd = std::chrono::high_resolution_clock::now();
const double optimize2Seconds =
    std::chrono::duration<double>(tEnd - tStep).count();
```

#### Design rationale

`invokingTNs` is computed exactly the way `cycleTNs` already is in `LocalMapper::MapLocally`, `frame_poses.rbegin()->first`, but read from inside `NfrMapper::optimize` itself rather than passed in, so the fix needs no change to `optimize`'s required parameters and generalises to the offline `src/mapper.cpp`/`src/mapper_sim.cpp` callers exactly as Change 22's does. `opt_pass` is the one genuinely new piece of information in this section, without it `t_ns` alone would still leave the two calls per cycle indistinguishable, since neither `frame_poses` nor the newest keyframe changes between them, only landmarks are filtered in between. It is a defaulted trailing parameter rather than a required one, per the project's own preference for an optional argument whose default reproduces existing behaviour over a second method, so `optimize()` and `optimize(num_opt_iter)`, the two batch tool call sites, keep compiling and keep logging `opt_pass = 0` on every row, a harmless label neither tool distinguishes today. The new parameter names inside `nfr_mapper.h`/`.cpp`, `num_iterations`, `opt_pass`, `invokingTNs`, follow that file's own established snake and mixed case style rather than this project's camelCase convention, matching the surrounding code in a file that is otherwise an largely unmodified upstream Basalt translation unit, while every `Logger::Add*` parameter, `tNs`, `optPass`, stays camelCase, consistent with every other method in `logger.h`/`.cpp`, itself entirely this project's own code.

#### Backwards compatibility impact

`optimize`'s new parameter is defaulted, so `src/mapper_sim.cpp:520` and `src/mapper.cpp:634` compile and behave exactly as before, both logging `opt_pass = 0`, a value neither tool's own output was ever going to be read against a pass number in the first place. `AddMapperLinearise`, `AddMapperSolverIter` and `AddMapperIterSummary` each have exactly one call site, all three inside `optimize` itself, updated in this same change, so the signature change is not a compile hazard anywhere else. The two existing `optimize(mpOptIterations)` calls in `LocalMapper::MapLocally` gain an explicit second argument, `0` and `1`, the only call sites in the live stack that now distinguish the two passes, matching exactly the order `filterOutliers` already imposes between them.

### Blast radius, Change 10 to Change 18

| File | Change | Compatibility |
|---|---|---|
| `include/basalt/utils/logger.h` | `AddVioInit`, `AddVioTriang` and `AddVioImuFrame` each gain trailing parameters, three new `Add`/`Print` pairs, two new members on `OpticalFlowTrackStats` | every one of the three extended methods has exactly one call site, updated in the same change |
| `src/utils/logger.cpp` | three registries extended, three registries added, `SetBlankEmptyVectors(true)` added for `vio_init` | new columns append after existing ones, no column renamed or reordered |
| `include/basalt/optical_flow/frame_to_frame_optical_flow.h` | `mpTrackBirthTNs`, `mpTrackAge` members, death recording in `trackPoints`, aging folded into the existing flow magnitude loop, death emission in `processFrame` | `trackPoints`'s signature and its `const`ness are unchanged |
| `src/vi_estimator/sqrt_keypoint_vio.cpp` | yaw alignment locals named and logged, a new reprojection block, parallax accumulators and gate in the triangulation loop, `accelSum`/`gyroSum` and the stationarity test in the IMU drain loop | `<numeric>` added to the includes, no existing local is renamed |
| `src/vi_estimator/local_mapper.cpp` | `MatchLocal`'s retrieval loop gains a named `aboveThresh` and a logging call before the existing `continue` | the loop's control flow and its output, `ids_to_match`, are unchanged |
| `include/basalt/vi_estimator/sqrt_keypoint_vio.h` | three new members, `mpStationaryAccelTol`, `mpStationaryGyroTol`, `mpIsStationary` | defaulted, no existing member renamed |
| `include/basalt/utils/vio_config.h` | one new field, `vio_min_triangulation_parallax_deg` | defaults to `0.0`, every existing field unchanged |
| `src/utils/vio_config.cpp` | default and `serialize` entry for the new field | breaking for a config file that omits it, see Change 17's own compatibility note |
| `src/controller.cpp` | `#include <basalt/utils/filesystem.h>`, a file copy in `Controller::initialize` | guarded by the same `logDirectory` emptiness check the logger itself already uses |

### Blast radius

| File | Change | Compatibility |
|---|---|---|
| `include/basalt/utils/time_utils.hpp` | virtual destructor, four virtual `add` methods, third variant arm, `stats_` and `order_` made protected, `overload` helper relocated here | source compatible, ABI changes, whole tree is rebuilt together |
| `src/utils/time_utils.cpp` | `add_int` added, four `std::visit` sites gain a third arm, anonymous namespace `overload` removed | behaviour of existing keys unchanged |
| `include/basalt/utils/logger.h` | new | none |
| `src/utils/logger.cpp` | new | none |
| `include/basalt/optical_flow/frame_to_frame_optical_flow.h` | trailing `Logger::Ptr` parameter, `mpLogger` member, `OpticalFlowTrackStats` moved out, both branches merged into one emission | `PatchOpticalFlow` and `MultiscaleFrameToFrameOpticalFlow` untouched |
| `include/basalt/optical_flow/optical_flow.h` | factory gains trailing defaulted parameter | `src/opt_flow.cpp:197`, `src/vio.cpp:300`, `src/rs_t265_vio.cpp:175` unchanged |
| `src/optical_flow/optical_flow.cpp` | forwards the logger in the four `frame_to_frame` branches only | the four `patch` branches unchanged |
| `include/basalt/vi_estimator/vio_estimator.h` | factory gains trailing defaulted parameter | `src/vio_sim.cpp:923`, `src/mapper_sim_naive.cpp:752`, `src/rs_t265_vio.cpp:179` unchanged |
| `src/vi_estimator/vio_estimator.cpp` | `factory_helper` forwards the logger | explicit instantiation at `:178-181` gains the parameter |
| `include/basalt/vi_estimator/sqrt_keypoint_vio.h` | `mpLogger` member, `stats_all_` and `stats_sums_` removed | `debug_finalize` keeps its signature and its artefacts |
| `src/vi_estimator/sqrt_keypoint_vio.cpp` | twenty nine debug predicates replaced, twenty nine `ExecutionStats` calls rerouted, ten new stage timers | output keys of `stats_all.json` and `stats_sums.json` unchanged |
| `include/basalt/vi_estimator/sqrt_keypoint_vo.h` and `.cpp` | trailing constructor parameter accepted and stored, nothing else | `stats_all_` and `stats_sums_` remain, VO artefacts unchanged |
| `include/basalt/vi_estimator/nfr_mapper.h` | trailing `Logger::Ptr` parameter, `mpLogger` member | `src/mapper.cpp:566` and `src/mapper_sim.cpp:163` unchanged |
| `src/vi_estimator/nfr_mapper.cpp` | five aggregates, `[Mapper]` prefixes, four new column groups | `match_all` and `addMargData` untouched |
| `include/basalt/vi_estimator/local_mapper.h` | trailing `Logger::Ptr` parameter forwarded to `NfrMapper` | no other consumer constructs it |
| `src/vi_estimator/local_mapper.cpp` | forty two debug predicates replaced by twelve aggregates | two expensive reference computations stay behind `mpVioDebugMode` |
| `include/basalt/controller.h` | `logDirectory` parameter on `initialize`, `mpLogger` member | `ros_ws/src/slam/src/basalt/slam.cpp:48-56` unchanged |
| `src/controller.cpp` | constructs the logger, forwards it three ways, saves in `Stop`, `GrabIMU` print replaced | inertial ingest counters keep their meaning |
| `src/vio.cpp` | constructs a logger at `:311` and passes it | required to retain the batch artefacts, one line |
| `CMakeLists.txt` | two source list entries | none |

Nothing outside `ext/basalt` needs to change. `ros_ws/src/slam/src/basalt/slam.cpp`, `node.cpp` and `driver.cpp` compile unmodified, and the ROS2 node can adopt the log directory parameter whenever it is wanted.

### Validation strategy

The development container has no Eigen at `/usr/include/eigen3`, recorded in [`../context/vio_optical_flow_instrumentation.md`](../context/vio_optical_flow_instrumentation.md), so nothing here compiles locally. Verification proceeds in the following order.

A host build with `colcon build --packages-select slam` under `-Wall -Wextra -Werror` is the first gate, and it is a meaningful one for this change, because a missed arm in any of the four `std::visit` calls is a compile error rather than a silent fault, and because an ambiguous `add` call would fail there too.

The offline batch path is the second gate. Running `src/vio.cpp` on EuRoC MH_01_easy and confirming that `stats_all.ubjson` and `stats_sums.ubjson` still load through `python/basalt/log.py` establishes that the legacy artefacts survived. Comparing the key sets before and after is sufficient, the values will differ run to run.

The live path is the third gate. A SITL flight with a log directory set should produce thirty five files, and three invariants are checked on them. Every per frame file has one row per frame and the same `t_ns` set, which the previous extractor could only approximate. No row is truncated or contains a field from another record, which is the failure the extractor was built to detect and which should now be structurally impossible. The console output with `vio_debug` set carries the same information as before, with no line split across two writers.

A regression on ORB-SLAM3 is not required, since nothing in this change touches the shared lifecycle policy.

### Validation strategy, Change 10 to Change 18

The same host build gate applies, `AddVioInit`, `AddVioTriang` and `AddVioImuFrame` all change signature, so a stale call site anywhere is a compile error rather than a silent fault. Beyond the build, `vio_init.csv`'s invariant `yaw_offset_rad == gt_yaw_rad - slam_yaw_rad`, up to the `(-pi, pi]` wrap, on a ground truth seeded run, and `yaw_aligned=0` with all three angle columns reading the `-999.0` sentinel on a run without ground truth, is checked directly against the produced file. `of_track_death.csv`'s row count should equal `of_frame.csv`'s summed `fwd_fail + bwd_fail + recov_rej` across the run, since every one of those counted rejections is now also one emitted row, a direct cross file consistency check that needs no new tooling. `vio_reproj_pre_opt.csv`'s `n` column should track `connected0` from `vio_assoc.csv` on the same frame, since both count observations matched to an existing landmark this frame. `map_bow_candidate.csv`'s `above_thresh` count per query, summed, should equal `map_bow_query.csv`'s own `above_thresh` column on the same `query_kf`, which is the two logging paths agreeing on the same threshold decision from two independent code paths.

`vio_imu_frame.csv`'s `is_stationary` should read true through the whole ground phase of a SITL flight and false from the first moment of climb onward, checked against the same mission `plans/vio_drift_analysis.md` measures, and a run where it never reads true at all means `mpStationaryAccelTol`/`mpStationaryGyroTol` are miscalibrated against this configuration's actual noise floor rather than that the vehicle was never stationary. `vio_triang.csv`'s `low_parallax` column should read zero on every existing configuration file, none of which sets `vio_min_triangulation_parallax_deg` above its `0.0` default, and this is the direct proof that `Change 17` changed no run's behaviour before Phase 2 activates it. `<run log directory>/config.json` and `calibration.json` should exist after any run started with a non empty log directory and should be byte identical to whichever file `config_path_`/`calib_path_` named, which a checksum against the source is enough to confirm.

### Deliberately not done

`SqrtKeypointVoEstimator` keeps its own `stats_all_` and `stats_sums_` and its own print sites. The requirement names the optical flow, the estimator and the local mapper, and the visual only estimator is a separate live path reached by `slam_type` `monocular-only`. Mirroring this work there is a follow up of the same shape, and leaving it untouched means the VO artefacts cannot regress while the VIO ones are being moved.

`NfrMapper::match_all` and the offline `src/mapper.cpp` and `src/mapper_sim.cpp` paths keep their untagged prints. They construct `NfrMapper` directly, receive the disabled logger, and are not part of the live stack.

`PatchOpticalFlow` and `MultiscaleFrameToFrameOpticalFlow` gain no instrumentation, because they have none today and are not selected by any configuration in `data/`.

No ROS2 publisher is written. `AggregateStats::SetRowCallback` is the hook it will attach to, and it is left unset so that the cost today is one branch per committed row.

`struct VioConfig` gains no field from Change 1 through Change 15, for the reason given under Change 6. `Change 17` later adds `vio_min_triangulation_parallax_deg`, corrected here 2026-09-26, because that field is a rejection gate the ablation study in `plans/vio_drift_analysis.md` needed rather than a piece of logging plumbing, and defaults to `0.0`, reproducing the behaviour this section originally described for every configuration file that predates it.

The comma separated writer performs no rotation and no compression. A three hundred second flight at the measured 19.6 Hz writes roughly ninety columns per frame across the eight per frame aggregates, which is on the order of ten megabytes once the per iteration solver files are included, and `SetRetainedRows` bounds memory rather than disk. A run long enough to matter should set the directory to a path the operator is prepared to manage.

### Deliberately not done, Change 10 to Change 18

`Change 13` on its own added no rejection gate on the VIO side's parallax, instrumentation only, per its own design rationale. `Change 17` adds the gate, corrected here 2026-09-26 since the two changes are now both part of this plan, but leaves it at its inactive default in every configuration file, so activating it on `data/sitl_config_vo.json` at Phase 2's proposed 0.9 degrees remains `plans/vio_drift_analysis.md` Phase 2's own step rather than something this logging plan does on its behalf. No temporal consistency grouping or minimum inlier count is added to the local mapper's loop matching, `map_bow_candidate.csv` records what the existing single stage pipeline already decides, it does not change how that decision is made, a three stage pipeline resembling ORB-SLAM3's is a separate, larger change should the precision recall curve this data enables show the current single stage operating point is inadequate. No sliding window covariance or rank check is computed in C++, `accelSum`/`gyroSum` are raw material, the check itself is Python side and lives in `context/slam_analysis.md`. No zero velocity residual is added to `optimize`, `Change 16` builds only the detection and its log that residual will need, per Phase 3b's own scope. No manifest CSV is added for the third gap, `Change 18` copies the source files themselves rather than restating their fields as columns, per the direction that settled it. `SqrtKeypointVoEstimator` gains none of these nine changes, consistent with Change 7's own exception, it keeps its own `stats_all_`/`stats_sums_` path untouched.

### Blast radius, Change 19 to Change 23

| File | Change | Compatibility |
|---|---|---|
| `include/basalt/utils/logger.h` | `AddVioSolverTiming`, `AddVioMargTiming`, `AddMapPairMatch`, `AddMapperDetect`, `AddMapperStereo`, `AddMapperLinearise`, `AddMapperSolverIter` and `AddMapperIterSummary` each gain a leading `int64_t tNs`, the last three also gain `int optPass` | every one of the eight methods has exactly one call site, updated in the same change |
| `src/utils/logger.cpp` | eight registries gain a leading `{"t_ns", "ns"}`, three of them also gain `{"opt_pass", "enum"}`, `vio_marg_timing`'s `frame_id` key renamed to `t_ns` | every other column keeps its name, order and unit |
| `include/basalt/vi_estimator/nfr_mapper.h` | `optimize` gains a second defaulted parameter, `int opt_pass = 0` | `src/mapper_sim.cpp:520` and `src/mapper.cpp:634` compile and behave unchanged |
| `src/vi_estimator/nfr_mapper.cpp` | `detect_keypoints`, `match_stereo` and `optimize` each gain a small local computing the invoking timestamp from state already owned, `keys`, `img_data` or `frame_poses` | no signature change to `detect_keypoints` or `match_stereo`, no behavioural change to any of the three methods' own computation |
| `src/vi_estimator/local_mapper.cpp` | `MatchLocal`'s `matchTNs` declaration hoisted to the top of the function and reused at its `AddMapPairMatch` call, `MapLocally`'s two `optimize` calls gain an explicit `0`/`1` pass argument | `AddMapMatchSummary`'s own call, the declaration's only other use, reads the hoisted value unchanged |
| `src/vi_estimator/sqrt_keypoint_vio.cpp` | `AddVioSolverTiming`'s call gains `last_state_t_ns` as its first argument, `AddVioMargTiming`'s call is otherwise untouched | both call sites already had the right value in scope, only the callee's declaration changed |

### Validation strategy, Change 19 to Change 23

The host build gate applies as before, eight methods change signature, so a stale call site anywhere is a compile error rather than a silent fault, and `optimize`'s new defaulted parameter is a compile time check that `src/mapper.cpp` and `src/mapper_sim.cpp` still build unmodified. Beyond the build, every one of the eight files this section touches should show a populated `t_ns` column with no blank or zero value once the run has produced at least one keyframe or one solver iteration, checked directly against a fresh run's own CSVs the way `Change 10` through `Change 18` were checked against the second audited run. `map_pair_match.csv`'s new `t_ns` column should equal its own `kf1` column on every row in the live `LocalMapper` path, the direct confirmation that the implicit invariant this change made explicit was correctly understood, a mismatch on any row would mean the hoisted `matchTNs` was read at the wrong point in `MatchLocal`'s own control flow. `mapper_linearise.csv`, `mapper_solver_iter.csv` and `mapper_iter_summary.csv` should each show exactly two distinct `opt_pass` values, `0` and `1`, per `t_ns`, and the `iter` column should restart at `0` within each `opt_pass` group, confirming the two `optimize` calls per cycle are now cleanly separable rather than interleaved as they were before this change.

### Deliberately not done, Change 19 to Change 23

`vio_linearise.csv`, `vio_marg.csv`, `vio_marg_nullspace.csv` and every other record already carrying `t_ns` before this section are untouched, this section closes the eight gaps the audit found, it does not revisit records already correct. `src/mapper.cpp` and `src/mapper_sim.cpp` gain no explicit pass or timestamp argument of their own, `optimize`'s default and the internal derivation in `detect_keypoints`/`match_stereo` are chosen specifically so these two offline tools need no change at all, extending them to pass a meaningful `opt_pass` of their own, should a future session want per call discrimination in a batch mapping run too, is a separate, smaller change left for whenever that need arises. No new record is added, every fix in this section extends a record that already exists rather than introducing a new CSV, unlike `Change 11`, `Change 12` and `Change 15` earlier in this plan.

### Change 24, retiring the console sink and gating collection on `vio_debug`

Added 2026-10-23, ahead of an investigation into unprecedented drift and a changed scale in recent experiments, at the direction that the CSV files alone should carry the diagnostic record.

#### Need

Every `Logger::Print*` method renders the latest row of its `AggregateStats` table to `std::cout` through `Logger::Emit` at `src/utils/logger.cpp:601-608`. Each of those rows is already committed to its CSV by `AggregateStats::CommitRow`, so the console line duplicates the file and interleaves with the pipeline's other output on every thread. Separately, the logger was constructed unconditionally in `Controller::initialize`, so every ROS run collected and wrote the full CSV set whatever its configuration said. The direction was to remove the console rendering, to make both collection and log file creation follow the configuration's debug flag, and to leave the copy of `config.json` and `calibration.json` from `Change 18` ungated, since those two files are needed for every run whether or not diagnostics are recorded.

#### Code snippet

The superseded construction in `Controller::initialize`, `src/controller.cpp:161-167`, is commented out in place and replaced with a gated one. `Logger::Disabled()` is the existing null object at `src/utils/logger.cpp:582-590`, whose `mpEnabled` is false and whose empty output directory means `Register` opens no CSV stream.

```cpp
    // Superseded 2026-10-23, collection and CSV creation now follow vio_debug.
    // mpLogger =
    //     std::make_shared<basalt::Logger>(logDirectory, vio_config_.vio_debug);
    mpLogger = vio_config_.vio_debug
                   ? std::make_shared<basalt::Logger>(logDirectory,
                                                      vio_config_.vio_debug)
                   : basalt::Logger::Disabled();
```

`Logger::FinishVioOptimize` discards the solver scratch when disabled rather than retaining it.

```cpp
void Logger::FinishVioOptimize() {
    // Callers fill the scratch unconditionally, so a disabled logger must
    // still discard it or it grows for the life of the estimator.
    if (!mpEnabled) {
        mpSolverScratch = ExecutionStats();
        return;
    }
    mpStatsAll.merge_all(mpSolverScratch);
    mpStatsSums.merge_sums(mpSolverScratch);
    mpSolverScratch = ExecutionStats();
}
```

Every one of the fifty `mpLogger->Print*` invocations is commented out in place, with no accompanying note, at the direction that settled this change.

```cpp
        mpLogger->AddVioState(...);
        // mpLogger->PrintVioState();
```

The fifty sites are three in `include/basalt/optical_flow/frame_to_frame_optical_flow.h`, one in `src/controller.cpp`, nineteen in `src/vi_estimator/sqrt_keypoint_vio.cpp` including `PrintSummary` inside `debug_finalize`, one in `src/vi_estimator/sqrt_keypoint_vo.cpp`, twenty in `src/vi_estimator/local_mapper.cpp` and six in `src/vi_estimator/nfr_mapper.cpp`. Each was a standalone statement directly after its `Add*` call, and none was the sole body of an unbraced `if`, `for` or `while`, which was verified by scanning the line before each site, so commenting it out leaves no control flow attached to the next statement. The header comment on `Controller::initialize` at `include/basalt/controller.h:35-42` was corrected in place to describe the new gate.

#### Design rationale

A single gate at the one construction site rather than a flag threaded through every component. Every consumer, namely the optical flow front end, the VIO and VO estimators, the local mapper and its `NfrMapper` base, `Controller::AddGtEval` and the controller's own inertial ingest record, receives the same `mpLogger` pointer, and every component already substitutes `Logger::Disabled()` for a null logger, so handing the null object to all of them suppresses every `Add*`, every CSV stream, `SaveLegacyStats` and `SaveAll` with no new mechanism. No component reads logger rows in memory, since `AggregateStats::SetRowCallback`, `Logger::Aggregate` and `AggregateStats::RowCount` have no callers, so no live consumer can be starved by the gate.

The gate covers the ROS path in `Controller::initialize` alone. The offline `basalt_vio` tool constructs its own logger at `src/vio.cpp:311`, and `python/basalt/log.py:93-95` and `python/basalt/run.py:107-112` read the `stats_all.json` and `stats_sums.json` it writes, while `data/iccv21/basalt_batch_config.toml` sets `vio_debug = false`, so gating that site would have silently broken every batch run launched from the default template.

The `FinishVioOptimize` change closes a defect the gate would otherwise have made live. `SqrtKeypointVioEstimator::optimize` and `marginalize` call `mpLogger->SolverScratch().add(...)` unconditionally, for example at `src/vi_estimator/sqrt_keypoint_vio.cpp:577`, `:998` and `:1018-1019`, and `ExecutionStats::add` appends to a vector at `src/utils/time_utils.cpp:35-72`. The previous `FinishVioOptimize` returned before clearing the scratch whenever `mpEnabled` was false, so on the null object the scratch grew by roughly twenty entries per frame for the life of the estimator. That leak already existed for any caller constructing an estimator without a logger, and the gate would have imposed it on every ROS run with `vio_debug = false`.

The logger's console API, `Print*`, `Emit`, `PrintSummary`, `AggregateStats::PrintLatest` and the `consoleEnabled` constructor argument, is left in place unused, so the call sites can be restored line by line and no signature changes.

#### Backwards compatibility impact

With `vio_debug = true`, which `data/sitl_config_vo.json:17` sets, the logger is constructed exactly as before and every CSV is written unchanged. With `vio_debug = false`, the run directory holds only `config.json` and `calibration.json`, because the `Change 18` copy block at `src/controller.cpp:169-182` keys only on `logDirectory` being non-empty and is untouched, and the directory itself is still created unconditionally by the ROS wrapper at `/ws/ros_ws/src/slam/src/basalt/slam.cpp:57-58`. The console sink was already silent whenever `vio_debug` was false, because the constructor received `vio_debug` as `consoleEnabled`, so the visible change with debug on is that the `[VIO]`, `[Optical Flow]`, `[Map]` and `[Mapper]` record lines no longer appear. For `basalt_vio`, the `=== stats all ===` and `=== stats sums ===` dump no longer reaches stdout, while `stats_all.json`, `stats_sums.json` and every CSV are still written. The enabled path of `FinishVioOptimize` is byte identical, and its disabled path still writes nothing.

#### Blast radius

| File | Change | Compatibility |
|---|---|---|
| `src/controller.cpp` | logger construction gated on `vio_config_.vio_debug`, one `Print*` commented out | copy block untouched, debug on reproduces the previous construction exactly |
| `include/basalt/controller.h` | comment on `initialize` corrected | no code change |
| `src/utils/logger.cpp` | `FinishVioOptimize` clears the scratch when disabled | enabled path unchanged |
| `include/basalt/optical_flow/frame_to_frame_optical_flow.h`, `src/vi_estimator/sqrt_keypoint_vio.cpp`, `src/vi_estimator/sqrt_keypoint_vo.cpp`, `src/vi_estimator/local_mapper.cpp`, `src/vi_estimator/nfr_mapper.cpp` | forty nine `Print*` invocations commented out | `Add*` calls and CSV content unchanged |
| `src/vio.cpp` | untouched | batch `stats_*.json` and CSVs unchanged |

#### Validation strategy

Run in the dev container on 2026-10-23. `grep -rnE "^\s*mpLogger->Print" src include` returns nothing, and the fifty commented sites sum per file to three, one, nineteen, one, twenty and six. A `-fsyntax-only` pass with the flags of `/ws/ros_ws/build/slam/compile_commands.json` over `src/controller.cpp`, `src/utils/logger.cpp`, `src/vi_estimator/sqrt_keypoint_vio.cpp`, `src/vi_estimator/sqrt_keypoint_vo.cpp`, `src/vi_estimator/local_mapper.cpp`, `src/vi_estimator/nfr_mapper.cpp` and `src/optical_flow/optical_flow.cpp`, the last of which includes `frame_to_frame_optical_flow.h`, compiles all seven cleanly. Two environmental substitutions were needed, because the container had been rebuilt on Debian 12 with GCC 12.2 and without `/usr/include/eigen3`. Eigen 3.4.0 vendored at `/ws/sitl_ws/src/ue5/ProjectAirSim/unreal/Blocks/Plugins/ProjectAirSim/SimLibs/eigen/include` stood in for the system copy, and `-Wno-missing-template-keyword` silenced a GCC 12 diagnostic that `-Werror` promotes on `thirdparty/basalt-headers/include/basalt/imu/imu_types.h:240-242`, an untouched header last changed in 2021. The only remaining diagnostics are the pre-existing `unused parameter` warnings in the untouched optical flow headers.

Outstanding, on the build host after `colcon build --packages-select slam`. A SITL run with `vio_debug = false` should leave a run directory holding only `config.json` and `calibration.json`, print no logger record lines, and keep resident memory flat over a long run, which confirms the `FinishVioOptimize` fix. A SITL run with `vio_debug = true` should produce the full CSV set with headers and row counts comparable to an earlier run and print no logger record lines. `basalt_vio` on a EuRoC sequence should still write its CSVs together with `stats_all.json` and `stats_sums.json`. The host build should also confirm whether the GCC 12 `missing-template-keyword` diagnostic affects it, since it would fail a `-Werror` build independently of this change.

#### Deliberately not done

Roughly ninety five direct `std::cout` lines that do not pass through `mpLogger` are left, many of them under `if (config.vio_debug)`, at the direction that settled this change, namely forty one in `src/vi_estimator/sqrt_keypoint_vo.cpp`, eighteen in `src/vi_estimator/sqrt_keypoint_vio.cpp`, fifteen in `src/vi_estimator/nfr_mapper.cpp`, eleven in `src/controller.cpp` and ten in `src/vi_estimator/local_mapper.cpp`. Because CSV collection now also requires `vio_debug = true`, those lines still print during every logged run. `src/vio.cpp` is not gated, for the batch pipeline reason given above. The console API in `class Logger` is not deleted.

### Change 25, gating the mapping thread's per-cycle console diagnostics on `vio_debug`

Added 2026-10-24, at the direction to identify the unguarded `std::cout` lines in the local mapper and gate them under the debug flag on the mapping thread.

#### Need

`Change 24` left the direct `std::cout` lines that do not pass through `mpLogger` untouched. The mapping thread is the one most likely to emit per-cycle text, since it runs a full match, track and optimise cycle for every keyframe, so its remaining console output was surveyed and classified before anything was gated.

#### Inventory and classification

The mapping thread runs `LocalMapper::Run` and, through it, `MapLocally`, `processMargData`, `detect_keypoints`, `match_stereo`, `MatchLocal`, the shadowed `build_tracks` and `setup_opt`, `optimize`, `filterOutliers`, keyframe culling and the `hash_bow_database` calls. Every live console write reachable from that path is listed below. The rule applied is that lifecycle messages and any message preceding `std::abort()` stay ungated, and only per-cycle diagnostic output is gated, a preference the user stated on 2026-10-24 after rejecting an edit that had gated the shutdown sentinel.

| Site | Message | Thread | Class | Action |
|---|---|---|---|---|
| `src/vi_estimator/local_mapper.cpp:101` | `[Local Mapper] got shutdown sentinel` | mapping | lifecycle | left ungated |
| `src/vi_estimator/local_mapper.cpp:40-48` | four `LocalMapper::Stop` progress lines | caller of `Stop`, the controller | lifecycle | left ungated |
| `src/vi_estimator/local_mapper.cpp:888-933` | `[Local Mapper][cull-select]` lines | mapping | diagnostic | already under `if (mpVioDebugMode)` |
| `src/vi_estimator/nfr_mapper.cpp:123` | `std::cerr` "Unknown size" followed by `std::abort()` | mapping, via `processMargData` | fatal | left ungated |
| `src/vi_estimator/ba_base.cpp:248` | "Unknown frame id" followed by `std::abort()` | mapping, via the base class | fatal | left ungated |
| `include/basalt/hash_bow/hash_bow.h:82` | `HashBowBase::RemoveKeyframes not implemented` | not reached | none | none, since `LocalMapper` installs `HashBowStl<256>`, which overrides it, at `src/vi_estimator/local_mapper.cpp:20-23` |
| `src/utils/keypoints.cpp:410-414` | RANSAC inlier count reduced by non-linear refinement | mapping, via `findInliersRansac` in `LocalMapper::MatchLocal` | per-match diagnostic | gated |

`src/vi_estimator/nfr_mapper.cpp:63`, `:647`, `:702`, `:706` and `:732` sit in `NfrMapper::addMargData`, `NfrMapper::match_all` and `NfrMapper::build_tracks`. None of them runs on the mapping thread, because `LocalMapper` calls `processMargData` directly at `src/vi_estimator/local_mapper.cpp:311`, shadows `build_tracks` and never calls `match_all`. They belong to the offline `src/mapper.cpp` and `src/mapper_sim.cpp` tools and are left alone.

#### Code snippet

`findInliersRansac` is a shared helper whose only other caller is `NfrMapper::match_all` at `src/vi_estimator/nfr_mapper.cpp:671`, so the gate rides on a trailing defaulted argument whose default reproduces the previous output exactly.

```cpp
// include/basalt/utils/keypoints.h
void findInliersRansac(const KeypointsData& kd1, const KeypointsData& kd2,
                       const double ransac_thresh, const int ransac_min_inliers,
                       MatchData& md, bool verbose = true);
```

```cpp
// src/utils/keypoints.cpp
void findInliersRansac(const KeypointsData& kd1, const KeypointsData& kd2,
                       const double ransac_thresh, const int ransac_min_inliers,
                       MatchData& md, bool verbose) {
  ...
  if (verbose && ransac.inliers_.size() + 2 < num_inliers_ransac) {
    std::cout << "Warning: non-linear refinement reduced the relative pose "
```

```cpp
// src/vi_estimator/local_mapper.cpp:513-515, LocalMapper::MatchLocal
                findInliersRansac(f1, f2, config.mapper_ransac_threshold,
                                  config.mapper_min_matches, md,
                                  mpVioDebugMode);
```

`mpVioDebugMode` is the existing `NfrMapper` member set from `config.vio_debug` at `src/vi_estimator/nfr_mapper.cpp:53` and declared at `include/basalt/vi_estimator/nfr_mapper.h:225`, which `LocalMapper` already reads at `:842`, `:888`, `:901` and `:930`. Only the print is gated, and the inlier refinement and its result are unconditional.

#### Backwards compatibility impact

`NfrMapper::match_all` passes no sixth argument, so the offline mapper tools print exactly as before. `LocalMapper` with `vio_debug = true`, the setting in `data/sitl_config_vo.json:17`, prints exactly as before. `LocalMapper` with `vio_debug = false` no longer prints the warning, and every match result is unchanged. The new parameter is trailing and defaulted, so no other call site changes.

#### Validation strategy

Run in the dev container on 2026-10-24. A grep confirms that `findInliersRansac` has exactly two callers and that only `src/vi_estimator/local_mapper.cpp` passes a sixth argument. The syntax check of `/ws/skills/cpp-syntax-check-tooling.md` compiles `src/vi_estimator/local_mapper.cpp` and `src/vi_estimator/nfr_mapper.cpp` cleanly. Both include `include/basalt/utils/keypoints.h`, so the new declaration and both calls, one explicit and one defaulted, are type checked. `src/utils/keypoints.cpp` itself cannot be compiled here, because OpenCV is absent from the container, so its two token body edit was verified by inspection of its diff. Outstanding, on the build host after `colcon build --packages-select slam`, a SITL run with `vio_debug = false` should still show the shutdown sentinel and the `Stop` lines at shutdown and no RANSAC warning, and a run with `vio_debug = true` should match the previous console output.

#### Deliberately not done

The shutdown sentinel, the `LocalMapper::Stop` progress lines and the two messages preceding `std::abort()` are not gated, per the rule above. The offline-only prints in `NfrMapper` are not gated, since they never run on the mapping thread and gating them would change the offline tools' output. Direct console output on the VIO, VO and controller threads remains outside this change.

## Conclusion

The instrumentation this repository added during the divergence and drift investigations is the most valuable diagnostic asset it has, and it is currently held in the least durable form available, interleaved console text parsed by regular expression. This change does not add instrumentation so much as give the existing instrumentation a structure. A print statement is recognised as a row, a row is declared once through a registry, and the two things a row is wanted for, a line a human reads and a column a script reads, are produced from one recording rather than from two independent code paths.

The design builds on `class ExecutionStats` rather than beside it, which keeps the merging and JSON serialisation that the batch evaluation pipeline depends on and confines the extension to a third variant arm and four virtualised methods. Injection is by trailing defaulted parameter throughout, so every offline tool, every factory caller and the ROS2 node compile and behave exactly as they do today, and a component constructed without a logger receives a null object rather than a null pointer.

Three properties follow that the current arrangement cannot offer. The CSV files are the single diagnostic sink, so no record is any longer parsed from console text, and the corruption that forced four separate defences into the extraction script and once misreported a nine millisecond figure as fifty two seconds cannot recur. Note, 2026-10-23. This sentence previously read that console output is serialised against one mutex, which held until `Change 24` commented out every `Print*` invocation and left the CSV files as the only sink. Rows reach disk as they are produced, so a run terminated by a failing `BASALT_ASSERT` still yields its record. And the per frame state of the whole pipeline exists as an object at a single point, which is what a ROS2 health topic needs and what no amount of further parsing could have supplied.
