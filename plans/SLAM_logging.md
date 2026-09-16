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

`scripts/vio_log_extract.py` exists to reconstruct records from interleaved console prose. Once the comma separated files are produced directly it has no input to reconstruct, and `scripts/vio_log_plot.py` can read them with no intermediate step. Neither script is deleted by this change, because every capture recorded before it lands is still console text and must remain analysable. The correct sequence is to add a `--from-csv` mode to `vio_log_plot.py` that reads the per aggregate files and joins them on `t_ns`, leaving the existing path intact, and to retire `vio_log_extract.py` only once no capture of interest predates the new writer.

The join that the extractor performs in Python, folding every record sharing a frame timestamp into one row, becomes a database style join across files keyed on the mandatory `t_ns` first column. That column is present in every per frame aggregate by construction, which is the reason the registry declares it first everywhere.

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

### Deliberately not done

`SqrtKeypointVoEstimator` keeps its own `stats_all_` and `stats_sums_` and its own print sites. The requirement names the optical flow, the estimator and the local mapper, and the visual only estimator is a separate live path reached by `slam_type` `monocular-only`. Mirroring this work there is a follow up of the same shape, and leaving it untouched means the VO artefacts cannot regress while the VIO ones are being moved.

`NfrMapper::match_all` and the offline `src/mapper.cpp` and `src/mapper_sim.cpp` paths keep their untagged prints. They construct `NfrMapper` directly, receive the disabled logger, and are not part of the live stack.

`PatchOpticalFlow` and `MultiscaleFrameToFrameOpticalFlow` gain no instrumentation, because they have none today and are not selected by any configuration in `data/`.

No ROS2 publisher is written. `AggregateStats::SetRowCallback` is the hook it will attach to, and it is left unset so that the cost today is one branch per committed row.

`struct VioConfig` gains no field, for the reason given under Change 6.

The comma separated writer performs no rotation and no compression. A three hundred second flight at the measured 19.6 Hz writes roughly ninety columns per frame across the eight per frame aggregates, which is on the order of ten megabytes once the per iteration solver files are included, and `SetRetainedRows` bounds memory rather than disk. A run long enough to matter should set the directory to a path the operator is prepared to manage.

## Conclusion

The instrumentation this repository added during the divergence and drift investigations is the most valuable diagnostic asset it has, and it is currently held in the least durable form available, interleaved console text parsed by regular expression. This change does not add instrumentation so much as give the existing instrumentation a structure. A print statement is recognised as a row, a row is declared once through a registry, and the two things a row is wanted for, a line a human reads and a column a script reads, are produced from one recording rather than from two independent code paths.

The design builds on `class ExecutionStats` rather than beside it, which keeps the merging and JSON serialisation that the batch evaluation pipeline depends on and confines the extension to a third variant arm and four virtualised methods. Injection is by trailing defaulted parameter throughout, so every offline tool, every factory caller and the ROS2 node compile and behave exactly as they do today, and a component constructed without a logger receives a null object rather than a null pointer.

Three properties follow that the current arrangement cannot offer. Console output is serialised against one mutex, which removes the corruption that forced four separate defences into the extraction script and once misreported a nine millisecond figure as fifty two seconds. Rows reach disk as they are produced, so a run terminated by a failing `BASALT_ASSERT` still yields its record. And the per frame state of the whole pipeline exists as an object at a single point, which is what a ROS2 health topic needs and what no amount of further parsing could have supplied.
