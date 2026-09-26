# `VioConfig` and the cereal JSON configuration path

This file records how a configuration file under `data/` becomes a populated `basalt::VioConfig`, what the cereal JSON archive does and does not tolerate, and the consequences of those tolerances for anyone adding a configuration item. It was written on 2026-09-12 while exposing the local mapper's eight hard-coded parameters as configuration, and every claim below was verified by compiling and running `src/utils/vio_config.cpp` standalone against the repository's own configuration files.

## The call chain

`VioConfig::load(filename)` at `src/utils/vio_config.cpp:122-130` opens an `std::ifstream`, constructs a `cereal::JSONInputArchive` over it, and calls `archive(*this)`. Cereal resolves that call by argument-dependent lookup to the free function `serialize(Archive&, basalt::VioConfig&)` defined in `namespace cereal` at `src/utils/vio_config.cpp:160-221`, which is a flat sequence of `ar(CEREAL_NVP(config.field))` statements. `VioConfig::save` at `:112-120` is the exact mirror through `cereal::JSONOutputArchive`, and the same `serialize` serves both directions.

Six call sites invoke `load`, namely `src/vio.cpp:268`, `src/vio_sim.cpp:919`, `src/opt_flow.cpp:178`, `src/rs_t265_vio.cpp:157`, `src/mapper.cpp:177` and `src/controller.cpp:114`. The last is the one the ROS driver reaches, with the path supplied by the `config_file_path` parameter of `controllers/slam/basalt_driver_node.py:14` and threaded from `controllers/launch/basalt_slam_test.launch.py:52-58`. That node's compiled-in default names `data/sitl_config.json`, a file which does not exist in the tree, so the launch argument is not optional in practice.

## Where the key names come from

`CEREAL_NVP(x)` expands to `cereal::make_nvp("x", x)`, taking the name from the stringified expression rather than from the member. Because `serialize` writes `CEREAL_NVP(config.vio_max_kfs)` on a parameter named `config`, the emitted name is the literal `"config.vio_max_kfs"`, prefix included. This is the whole explanation for the `config.` prefix on every key in `data/*.json`, and it means a key name is changed only by changing the name of the `serialize` parameter or the member itself. The enclosing `"value0"` object is cereal's own wrapper for the single root object handed to `archive()`.

## What the archive tolerates

Lookup is by name and is order-independent. `JSONInputArchive::search()` at `thirdparty/basalt-headers/thirdparty/cereal/include/cereal/archives/json.hpp:565-579` first tests whether the node under the cursor already carries the requested name, and only on a mismatch calls `Iterator::search(name)` at `:531-547`, which scans every member of the current object linearly. Two properties follow.

A key present in the file but absent from `serialize` is inert. It is never visited, costs nothing, and produces no diagnostic. The repository demonstrates this by accident, since every configuration file carries `config.vio_outlier_threshold`, `config.vio_filter_iteration`, `config.vio_lm_landmark_damping_variant` and `config.vio_lm_pose_damping_variant`, none of which `serialize` reads, the first two because they are commented out at `:180-181` and the last two because no such member exists. All seven files load without complaint.

A key read by `serialize` and absent from the file is fatal. The scan falls off the end of the member list and throws at `json.hpp:546` with the message `JSON Parsing failed - provided NVP (<name>) not found`. Neither `load` nor any caller catches `cereal::Exception`, so the process aborts. Verified on 2026-09-12 by deleting one key from a copy of `data/sitl_config_vo.json`, which produced exactly that message followed by `terminate called after throwing an instance of 'cereal::Exception'`.

The practical consequence is that adding a field to `serialize` is a breaking change for every configuration file that predates it, and there is no optional-NVP facility in this vendored cereal to soften it. Every configuration source in the tree must be updated in the same commit.

## Every configuration source in the tree

Seven JSON files match `data/*config*.json`, namely `euroc_config.json`, `euroc_config_no_factors.json`, `euroc_config_no_weights.json`, `euroc_config_vo.json`, `kitti_config.json`, `sitl_config_vo.json` and `tumvi_512_config.json`. There is an eighth source that a search for JSON files misses. `data/iccv21/basalt_batch_config.toml` carries a `[value0]` table of the same keys, which `scripts/batch/generate-batch-configs.py:53-61` deep-copies, merges the per-experiment overrides into, and writes out as `basalt_config_<combination>.json` for each batch job. A field added to `serialize` without a matching entry in that table breaks every generated batch run rather than any file visible under `data/`.

## Numeric types

`VioConfig` carried no `size_t` member before 2026-09-12. `size_t` is nevertheless safe through this archive. On x86-64 Linux with GCC, `size_t`, `unsigned long` and `std::uint64_t` are the same type, so the exact overload `loadValue(uint64_t&)` at `json.hpp:652` and `saveValue(uint64_t)` at `:254` are selected, and the `unsigned long` fallback templates at `:694-700` are correctly disabled by their `!std::is_same<T, std::uint64_t>` guard. On a platform where `uint64_t` is `unsigned long long` those fallbacks take over through `loadLong`. Either way the value round-trips. A negative literal in the file would trip rapidjson's `GetUint64` assertion, so a `size_t` configuration item must never be given a negative default.

Note also that `VioConfig` types several counts as `double`, for instance `mapper_min_matches`, `mapper_min_track_length` and `mapper_max_hamming_distance`. That is inherited from upstream Basalt and is not a convention worth propagating.

## Naming convention for new fields

Members of `struct VioConfig` are snake_case and carry a subsystem prefix, `optical_flow_`, `vio_` or `mapper_`. The workspace C++ convention in `CLAUDE.md` section 9 does not apply here, both because `VioConfig` is upstream Basalt code with an entrenched convention and because these names are user-facing JSON keys rather than internal identifiers. The local mapper's items therefore take a `local_mapper_` prefix. The textual collision with the `mapper_` prefix is harmless, since `Iterator::search` compares with `strncmp` over the search length and then requires the found name to have exactly that length at `json.hpp:540-541`, so no prefix can shadow another.

## Local mapper parameters, added 2026-09-12

Eight parameters that `class LocalMapper` had declared with in-class initialisers are now supplied by configuration. They are `local_mapper_max_local_map_size` at 30, `local_mapper_min_local_map_size` at 8, `local_mapper_min_redundant_observers` at 3, `local_mapper_cull_redundancy_thresh` at 0.9, `local_mapper_min_observed_for_cull` at 30, `local_mapper_max_cull_per_pass` at 2, `local_mapper_opt_iterations` at 5 and `local_mapper_filter_outlier_threshold` at 3.0. The defaults in `VioConfig::VioConfig` reproduce the values the declarations carried, so a default-constructed `VioConfig` that is never loaded drives the mapper exactly as before.

`LocalMapper::LocalMapper` at `src/vi_estimator/local_mapper.cpp:17-35` assigns each member from the corresponding field. The members keep their names, their types and their public access, so no consumer changed, including the external read of `mpMaxLocalMapSize` at `src/basalt_slam.cpp:260`. Their types are matched exactly in `VioConfig` rather than widened to `int`, because `src/vi_estimator/local_mapper.cpp:948` passes `mpMaxCullPerPass` to `std::min` alongside a `size_t`, which fails to compile if the two arguments differ in type.

The in-class initialisers were removed rather than left in place. Retaining them would have put the same default in two files with only one of them read, so editing the header value would have had no effect, which is a silent trap. `VioConfig::VioConfig` is now the sole home of each default.

Four further parameters remain hard-coded in `include/basalt/vi_estimator/local_mapper.h`, namely `mpFilterMinObs`, `mpMaxInvDist`, `mpMaxCosParallax` and `mpLocalMatchNeighbours`. They were outside the range the 2026-09-12 request named and are exposed by the same recipe whenever wanted.
