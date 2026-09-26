# Visualiser Defects Found in the 2026-09-08 SITL Recording

## 1. Motivation

A ten minute screen recording of a SITL survey flight, `visualisation_test.webm`, was captured with the Pangolin GUI of `basalt_slam` running against `data/sitl_config_vo.json` and `data/sitl_calib.json`. Three anomalies were reported from it. The plotter opens showing ten anonymous numbered traces rather than the position series, and only a click on `show_est_pos` produces the intended plot. The local map appears late, then flickers in and out for the rest of the flight. Two saturated red frusta are drawn where a single current frame is expected.

All three were reproduced from extracted frames and traced to concrete code. Two are defects of the visualisation pipeline itself. The third is the visualisation pipeline faithfully displaying a defect that lives in `LocalMapper`, which matters because the cure is not in the GUI and a GUI level workaround would hide a real mapping failure.

## 2. Evidence

Frames were decoded with GStreamer, since the container has no `ffmpeg`. The method is now recorded in [`/ws/skills/media-analysis-tooling.md`](/ws/skills/media-analysis-tooling.md). The whole clip was sampled at 0.5 Hz into 304 PNG frames, giving a 608 second duration, and the 3D panel of every frame was classified by exact colour match against the four palette entries in use.

The plotter legend at t equals 16 s reads `$0` through `$9`, and by t equals 20 s it reads `position x`, `position y`, `position z`. This brackets the user click and confirms the reported ordering.

Local map point pixels are zero over t in [16, 64], [224, 274] and [380, 458], and rise monotonically from near zero at t equals 66, 324 and 470. The transition into the longest gap was examined frame by frame. Between t equals 378 and t equals 380 the orange cloud disappears in place while the red trajectory, the blue sliding window landmarks and the camera pose all hold still. The map is therefore not drifting out of view, the published snapshot is going empty.

The magnified 3D panel at t equals 590, where the operator had zoomed in, shows two saturated red frusta at `{250, 0, 26}` alongside two pale red frusta at `{255, 128, 128}` and two blue frusta at `{0, 50, 255}`.

## 3. High level design

Two of the three fixes are confined to `src/visualisation/visualiser.cpp` and `include/basalt/visualisation/utils.h`, both owned exclusively by the GUI. The third fix belongs in `src/vi_estimator/local_mapper.cpp` and changes mapping behaviour rather than display, so it is recorded separately below and its consequences for run comparability are stated explicitly.

The unifying insight across the two GUI defects is that the visualiser inherited assumptions from the offline `src/vio.cpp` viewer without inheriting the context that made them safe. The offline viewer rebuilt its plot series from a replay slider, so it never depended on an initial call. The offline viewer also drew only one estimator, so a palette in which several semantic layers share a colour was tolerable. A live viewer that overlays a VIO sliding window on a local map has more layers than the palette has distinguishable colours.

## 4. Defect D1, the plotter opens with ten unnamed series

### Cause

`SlamVisualiser::SetupLayout` constructs the plotter with a non null log at `src/visualisation/visualiser.cpp:103`.

```cpp
mpPlotter = new pangolin::Plotter(&mpVioDataLog, 0.0, 100, -10.0, 10.0, 0.01f, 0.01f);
```

Pangolin's constructor installs ten default series whenever that log pointer is non null, at `thirdparty/Pangolin/src/plot/plotter.cpp:283-288`.

```cpp
plotseries.reserve(RESERVED_SIZE);
for(unsigned int i=0; i< 10; ++i) {
    std::ostringstream ss;
    ss << "$" << i;
    if(log) AddSeries("$i", ss.str());
}
```

Each series takes the default title `"$y"`, so `AddSeries` routes it through `PlotTitleFromExpr` at `plotter.cpp:1116`. That function substitutes a human label only when the index is within `default_log->Labels()`, at `plotter.cpp:1121-1143`, and falls back to emitting the raw `$id` otherwise. `mpVioDataLog` never receives a `SetLabels` call, so its label vector is empty and all ten titles degrade to the literal strings `$0` through `$9`.

`SlamVisualiser::DrawPlots` at `visualiser.cpp:406` is the only code that clears those defaults and installs the intended labelled series, and its only call site is inside the render loop at `visualiser.cpp:234-236`.

```cpp
if (mpShowEstPos->GuiChanged() || mpShowEstVel->GuiChanged() ||
    mpShowEstBg->GuiChanged() || mpShowEstBa->GuiChanged())
    DrawPlots();
```

`pangolin::Var<bool>::GuiChanged` at `thirdparty/Pangolin/include/pangolin/var/var.h:276-283` returns true only after a GUI interaction has raised `gui_changed`, and clears the flag as it reports. Construction never raises it. The Pangolin defaults therefore stand until the operator clicks a checkbox, which is exactly the reported behaviour.

### Change

Call `DrawPlots` once at the end of `SetupLayout`, after every toggle has been constructed, so the plotter's series always match the toggle state.

```cpp
    main_display.AddDisplay(img_view_display);
    main_display.AddDisplay(display3D);

    // Pangolin's Plotter constructor installs ten default "$0".."$9" series
    // whenever it is handed a log. DrawPlots clears them and installs the
    // labelled series matching the toggles, and nothing else calls it until
    // the first GuiChanged, so the defaults would otherwise stand on screen.
    DrawPlots();
}
```

### Backwards compatibility

`DrawPlots` begins with `ClearSeries` and `ClearMarkers`, so it is idempotent and order independent. It reads only the four toggles, all constructed earlier in the same function, and writes only `mpPlotter`. It has no other call site and no other caller depends on the series being absent before the first click. The offline viewers in `src/vio.cpp` and the rest of the `vis_utils.h` family are untouched. Risk is nil.

## 5. Defect D2, the local map flickers

### What the visualiser does

`SlamVisualiser::ConsumeLocalMapQueue` at `visualiser.cpp:501-509` latches whatever snapshot arrives.

```cpp
mvpLocalMapVisQueue.pop(data);
if (!data) break;
std::lock_guard<std::mutex> lock(mpMtxLocalMap);
mpLatestLocalMap = data;  // keep only the newest map
```

`DrawScene` at `visualiser.cpp:310-321` then renders that cache unconditionally. The cache is a `shared_ptr` that is never reset, so a snapshot persists on screen until the next one replaces it. The queue holds four entries and the mapper publishes with `try_push`, but a dedicated consumer thread drains it immediately, so no drop mechanism inside the GUI can account for a gap of 78 seconds. The frame by frame evidence at t equals 378 to 380 already excludes the camera moving the map out of view. The only remaining explanation is that the arriving snapshot carries an empty `points` vector, and the visualiser is showing it correctly.

### Where the emptiness comes from

`LocalMapper::MapLocally` builds the snapshot at `src/vi_estimator/local_mapper.cpp:277-287`, immediately after `CullRedundantKeyframes` and two optimisation passes, and fills the points through `get_current_points`. That function walks `lmdb.getHostKfs()` at `src/vi_estimator/ba_base.cpp:230-266`, so an empty landmark database yields an empty point set.

`LocalMapper::SelectKeyframesToCull` at `local_mapper.cpp:667-734` can empty it in a single pass, for three compounding reasons.

The minimum map guard is disabled. Line 668 carries `// if (frame_poses.size() <= 5) return false;  // keep minimum map` as a comment, so there is no floor below which culling stops.

Recent keyframes are not protected. Lines 676 to 679 comment out `keep_recent` and set `const size_t eligible_end = ordered.size();`, so the newest keyframe is as eligible for culling as the oldest.

Criterion 1 does not exclude a keyframe that is already serving as another's redundancy partner. The loop at lines 684 to 716 marks `a` as soon as any other `b` covers at least `mpCullCovisibilityThresh` of `a`'s observations, and `a` remains in the candidate set for every subsequent `i`. A mutually redundant pair therefore marks both members, and a run of high overlap keyframes is marked wholesale.

`mpCullCovisibilityThresh` is 0.5 at `include/basalt/vi_estimator/local_mapper.h:60`. A nadir looking survey flight over homogeneous terrain produces exactly the high pairwise covisibility that drives this ratio past a half for long runs of consecutive keyframes.

`CullRedundantKeyframes` then executes the removal. `lmdb.removeFrame(culled)` at line 920 strips the culled keyframe's observations, and Step 2b at lines 923 to 934 drops every landmark left with fewer than two observations. When a run is culled together the rehost search at `FindBestRehostKf` has few surviving candidates, so landmarks are removed rather than rehosted. The database empties, the next snapshot carries no points, and the map visibly rebuilds from scratch as fresh keyframes arrive. The measured monotone growth from near zero at t equals 66, 324 and 470 is that rebuild.

### Change D2a, make the local map state legible in the GUI

This belongs in the visualisation pipeline and is worth making regardless of the mapper fix, because at present an operator cannot distinguish a mapper that is idle between keyframes from one that published an empty map from one that has died. All three render as an unchanging screen.

Add a status string to the panel and the arrival time to the cache, in `include/basalt/visualisation/visualiser.h`.

```cpp
    basalt::LocalMapperVisualizationData::Ptr mpLatestLocalMap;
    // Arrival wall time of mpLatestLocalMap, written under mpMtxLocalMap, so
    // the panel can report snapshot age and distinguish an idle mapper from
    // one publishing empty maps.
    std::chrono::steady_clock::time_point mpLocalMapArrival;
    bool mpHasLocalMap = false;
```

and beside the existing toggles,

```cpp
    // Read-only panel readout of the latest local-map snapshot. Pangolin's
    // TextInput draws title and value on one row and never wraps, so one Var
    // per field is the only way to give each its own line.
    std::unique_ptr<pangolin::Var<std::string>> mpLocalMapPts, mpLocalMapKfs,
        mpLocalMapAge;
```

Register it in `SetupLayout` after the toggles.

```cpp
    mpLocalMapPts = std::make_unique<pangolin::Var<std::string>>(
        "ui.map_pts", std::string("-"), pangolin::META_FLAG_READONLY);
    mpLocalMapKfs = std::make_unique<pangolin::Var<std::string>>(
        "ui.map_kfs", std::string("-"), pangolin::META_FLAG_READONLY);
    mpLocalMapAge = std::make_unique<pangolin::Var<std::string>>(
        "ui.map_age", std::string("-"), pangolin::META_FLAG_READONLY);
```

Three fields rather than one, because `TextInput::Render` at `thirdparty/Pangolin/src/display/widgets/widgets.cpp:653-677` draws the title left-aligned at `raster[0]` and the value right-aligned at `v.l + v.w - width - 2`, both on the same baseline `raster[1]`, and the widget occupies exactly one row of height `tab_h()`. There is no wrapping and no multi-line widget in the library, so the only way to place a value on its own line is to give it its own `Var`. A single field carrying `4213 pts 37 kf 0.3s` overlapped its own `local_map` title inside the 200 pixel panel.

A `std::string` variable falls through the widget dispatch at `thirdparty/Pangolin/src/display/widgets/widgets.cpp:146-172` to a `TextInput`, whose `Render` at `:653-655` re-reads the variable every frame unless the operator is mid edit. `META_FLAG_READONLY` at `thirdparty/Pangolin/include/pangolin/var/varvaluegeneric.h:36` clears `can_edit` at `widgets.cpp:503`, so the field displays without ever becoming editable.

Stamp the arrival in `ConsumeLocalMapQueue`.

```cpp
        std::lock_guard<std::mutex> lock(mpMtxLocalMap);
        mpLatestLocalMap = data;  // keep only the newest map
        mpLocalMapArrival = std::chrono::steady_clock::now();
        mpHasLocalMap = true;
```

Refresh the fields once per frame from the GL thread, through a `UpdateLocalMapStatus` helper called in `Run` before `FinishFrame`. It snapshots the cache under the existing mutex and does no GL work, so it cannot block the consumer thread.

```cpp
void SlamVisualiser::UpdateLocalMapStatus() {
    basalt::LocalMapperVisualizationData::Ptr lm;
    std::chrono::steady_clock::time_point arrival;
    bool has_lm;
    {
        std::lock_guard<std::mutex> lock(mpMtxLocalMap);
        lm = mpLatestLocalMap;
        arrival = mpLocalMapArrival;
        has_lm = mpHasLocalMap;
    }
    if (!has_lm || !lm) return;

    const double age = std::chrono::duration<double>(
                           std::chrono::steady_clock::now() - arrival)
                           .count();
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%zu", lm->points.size());
    *mpLocalMapPts = std::string(buf);
    std::snprintf(buf, sizeof(buf), "%zu", lm->keyframes.size());
    *mpLocalMapKfs = std::string(buf);
    std::snprintf(buf, sizeof(buf), "%.1fs", age);
    *mpLocalMapAge = std::string(buf);
}
```

A snapshot carrying zero points now reads as `map_pts 0` beside a `map_age` that keeps resetting, and a mapper that has stopped publishing reads as counts that stop changing while the age climbs. The two failure modes become distinguishable at a glance.

### Change D2b, stop the mapper emptying its own map

This is the cure and it lives outside the visualisation pipeline. It changes what the local mapper retains, which changes bundle adjustment inputs and therefore the pose corrections fed back to the VIO.

Three protections are restored or added in `LocalMapper::SelectKeyframesToCull`. A floor stops culling once the map is small. Recent keyframes are made ineligible again. A keyframe that has served as another's redundancy partner is excluded from culling in the same pass, so a mutually redundant pair loses one member rather than both, and a hard `max_cull` backstop prevents any single pass from crossing the floor.

```cpp
bool LocalMapper::SelectKeyframesToCull(std::vector<int64_t>& keyframesToCull) {
    // Floor restored 2026-09-08. The 2026-09-08 SITL recording showed the map
    // emptying wholesale and rebuilding from scratch three times over a ten
    // minute flight, because every protection below was disabled.
    if (frame_poses.size() <= mpMinLocalMapSize) return false;

    std::vector<int64_t> ordered;
    ordered.reserve(frame_poses.size());
    for (const auto& kv : frame_poses) {
        ordered.push_back(kv.first);
    }
    std::sort(ordered.begin(), ordered.end());

    // Superseded 2026-09-08. Treating the newest keyframes as eligible let a
    // keyframe be culled before it had accumulated the observations that would
    // justify keeping it.
    // const size_t eligible_end = ordered.size();  // - keep_recent;
    const size_t keep_recent =
        std::min<size_t>(mpNewKeyframesForTracking.size(), ordered.size());
    const size_t eligible_end = ordered.size() - keep_recent;

    // A single pass must not take the map below the floor.
    const size_t max_cull = frame_poses.size() - mpMinLocalMapSize;

    const auto& obs = lmdb.getObservations();

    // Partners that justified culling somebody else. They must survive to host
    // the rehosted landmarks, so they are not themselves eligible this pass.
    std::set<int64_t> claimed;

    // Criterion 1 — redundancy.
    for (size_t i = 0; i < eligible_end; ++i) {
        if (keyframesToCull.size() >= max_cull) break;
        const int64_t a = ordered[i];
        if (claimed.count(a)) continue;
        size_t total_a = 0;
        ...
            if (static_cast<double>(covis) / static_cast<double>(total_a) >=
                mpCullCovisibilityThresh) {
                keyframesToCull.push_back(a);
                claimed.insert(b);
                break;
            }
```

with a new bound beside the existing ones in `include/basalt/vi_estimator/local_mapper.h`.

```cpp
    size_t mpMaxLocalMapSize = 150;
    // Floor below which culling stops. Without it a single pass could empty the
    // map outright, which is what the 2026-09-08 SITL recording showed.
    size_t mpMinLocalMapSize = 8;
    double mpCullCovisibilityThresh = 0.5;
```

`mpMaxLocalMapSize` reads 150 rather than the 50 the recording ran with, because it was raised on disk while this work was in progress. The floor is independent of it and holds at either value.

Criterion 2, the capacity rule at lines 719 to 727, is unchanged and still trims the oldest keyframe when the map exceeds `mpMaxLocalMapSize` and Criterion 1 found nothing, so the map cannot grow without bound.

### Backwards compatibility

D2a adds members and one panel entry. It reads the existing cache under the existing mutex and writes only a new `Var`. No existing behaviour changes and no other translation unit is affected.

D2b changes `LocalMapper::SelectKeyframesToCull`, whose only caller is `LocalMapper::CullRedundantKeyframes` at `local_mapper.cpp:230`, whose only caller is `LocalMapper::MapLocally`. The method is not virtual and shadows nothing in `NfrMapper`, so the offline mapper paths in `src/mapper.cpp` and `src/mapper_sim.cpp` are untouched. The change is nonetheless behavioural for every `basalt_slam` run, so historical trajectories produced before it, including those behind commit `b4de7aa`, are not comparable with those after it. The superseded `eligible_end` line is left commented in place beside its replacement per the workspace's code change discipline, for removal at commit time.

## 6. Defect D3, two red frusta where one current frame is expected

### Cause

The palette assigns the same colour to different meanings. `include/basalt/utils/vis_utils.h:43-44` defines two constants with an identical triple.

```cpp
const uint8_t cam_color[3]{250, 0, 26};
const uint8_t state_color[3]{250, 0, 26};
```

`DrawScene` at `visualiser.cpp:296-299` renders every sliding window state in `state_color`, with nothing marking the newest.

```cpp
for (const auto& p : vio->states)
    for (size_t i = 0; i < mpCalib.T_i_c.size(); i++)
        render_camera((p * mpCalib.T_i_c[i]).matrix(), 2.0f, state_color, 0.1f);
```

`VioVisualizationData::states` is filled from `frame_states` at `src/vi_estimator/sqrt_keypoint_vio.cpp:620-623`, and `config.vio_max_states` is 3 in `data/sitl_config_vo.json`. Up to three identically red frusta are therefore drawn, and the operator has no way to tell which one is the live frame. The recording shows two, because the window held two states at that instant.

`data/sitl_calib.json` declares a single camera, so the inner loop over `mpCalib.T_i_c` contributes exactly one frustum per pose. Duplication from a stereo rig is excluded.

A fourth red family layer compounds it. `include/basalt/visualisation/utils.h:42` defines the local map keyframes as `{255, 128, 128}`, a desaturated red, while the trailing comment calls it amber. Amber is near `{255, 191, 0}`. At the magnified zoom level of the recording a pale red local map frustum beside a saturated red state frustum reads as two red keyframes.

For completeness, the offline viewer at `src/vio.cpp:812-821` draws the newest pose a second time in `cam_color` before running the states loop, so with `cam_color` equal to `state_color` it double draws one frustum in one colour. `SlamVisualiser` never inherited that block, so the live viewer does not double draw. The palette collision that would have hidden it is inherited, though, and is what D3 corrects.

### Change

The shared header `include/basalt/utils/vis_utils.h` is consumed by `src/vio.cpp`, `src/mapper.cpp`, `src/vio_sim.cpp`, `src/mapper_sim.cpp`, `src/mapper_sim_naive.cpp` and `src/rs_t265_vio.cpp` in addition to the visualiser. Editing those constants in place would silently restyle six other executables. New constants go into the visualiser owned `include/basalt/visualisation/utils.h` instead, and the shared header is left untouched.

```cpp
// Live viewer palette. Kept here rather than in utils/vis_utils.h because that
// header is shared with six offline viewers whose appearance must not change.
// cam_color and state_color there are the same triple, which is why the newest
// state was previously indistinguishable from the rest of the window.
const uint8_t current_frame_color[3]{250, 0, 26};    // newest state, live edge
const uint8_t window_state_color[3]{120, 120, 120};  // older window states
const uint8_t local_map_point_color[3]{255, 128, 0};  // orange
const uint8_t local_map_kf_color[3]{255, 191, 0};     // amber, was {255,128,128}
```

`DrawScene` splits the states loop so the newest state is drawn last, heavier and in the live colour.

```cpp
    if (vio) {
        // Older window states are dimmed, so the newest state reads as the
        // single current frame. states.back() is the live edge.
        for (size_t s = 0; s + 1 < vio->states.size(); s++)
            for (size_t i = 0; i < mpCalib.T_i_c.size(); i++)
                render_camera((vio->states[s] * mpCalib.T_i_c[i]).matrix(), 2.0f,
                              window_state_color, 0.1f);
        if (!vio->states.empty())
            for (size_t i = 0; i < mpCalib.T_i_c.size(); i++)
                render_camera((vio->states.back() * mpCalib.T_i_c[i]).matrix(),
                              3.0f, current_frame_color, 0.15f);
        for (const auto& p : vio->frames)
            for (size_t i = 0; i < mpCalib.T_i_c.size(); i++)
                render_camera((p * mpCalib.T_i_c[i]).matrix(), 2.0f, pose_color,
                              0.1f);
        glPointSize(3);
        glColor3ubv(pose_color);
        pangolin::glDrawPoints(vio->points);
    }
```

After the change the scene carries exactly one saturated red frustum, drawn larger than the rest, and four visually separable layers, namely the red current frame, grey older window states, blue VIO keyframes and amber local map keyframes.

### Backwards compatibility

`include/basalt/utils/vis_utils.h` is not edited, so all six offline viewers keep their present appearance byte for byte. `local_map_kf_color` and `local_map_point_color` live in `include/basalt/visualisation/utils.h`, which is also included by `include/basalt/controller.h` for `basalt::GtPose` and by `include/basalt/vi_estimator/local_mapper.h` for `LocalMapperVisualizationData`. Neither reads a colour constant, and `src/visualisation/visualiser.cpp` is the sole consumer of the palette. The two new constants are additions. The trajectory polyline at `visualiser.cpp:283` continues to use the shared `cam_color` and is unchanged, which is correct because it is a different mark type and does not compete with a frustum for identity.

## 7. Blast radius

| Symbol or file | Consumers | Effect of the change |
|---|---|---|
| `SlamVisualiser::SetupLayout` | `SlamVisualiser::Start` only | One added call to an idempotent private method |
| `SlamVisualiser::DrawPlots` | `Run`, and now `SetupLayout` | None, it clears before it builds |
| `SlamVisualiser::DrawScene` | Bound into `display3D.extern_draw_function` | Rendering only |
| `SlamVisualiser::ConsumeLocalMapQueue` | `mpLocalMapConsumerThread` only | Two added writes under the existing mutex |
| `include/basalt/visualisation/utils.h` | `src/visualisation/utils.cpp`, `include/basalt/controller.h`, `include/basalt/vi_estimator/local_mapper.h`, `include/basalt/visualisation/visualiser.h` | Additions only. The three non GUI consumers include it for `LocalMapperVisualizationData` and `GtPose` and read no colour, and the constants are `const` at namespace scope so they carry internal linkage and cannot collide |
| `include/basalt/utils/vis_utils.h` | Six offline viewers plus the visualiser | Not edited |
| `LocalMapper::SelectKeyframesToCull` | `CullRedundantKeyframes` only | Behavioural, gated on the user's decision |

## 8. Validation

What was run in the development container, which has neither `cmake` nor the TBB headers as recorded in [`../context/keyframe_driven_local_mapping.md`](../context/keyframe_driven_local_mapping.md), so the library itself cannot be built here.

The selection logic of `SelectKeyframesToCull` and the body of `UpdateLocalMapStatus` were transcribed verbatim onto stub types and compiled with `g++ -std=c++17 -Wall -Wextra`. Under the pathological input of twenty keyframes all mutually redundant, two of them new and a floor of eight, the pre-change logic culls all eighteen eligible keyframes while the new logic culls nine and leaves eleven standing. The `claimed` set accounts for the halving, since alternate keyframes survive as hosts, and `max_cull` is the backstop that would have capped it at twelve had the pairing been less even. A map already at or below the floor returns false and culls nothing. The status formatter produces `4213 pts 37 kf 0.0s` for a populated snapshot and `no snapshot` before the first arrival.

All five edited files were checked for bracket balance with comments and string literals stripped, and all balance.

Every claim the plan makes about Pangolin was read from `thirdparty/Pangolin` rather than recalled. The default series loop at `src/plot/plotter.cpp:283-288`, the title fallback at `:1121-1143`, the consume-on-read behaviour of `GuiChanged` at `include/pangolin/var/var.h:276-283`, the widget dispatch that sends a `std::string` variable to a `TextInput` at `src/display/widgets/widgets.cpp:146-172`, that widget re-reading its variable every frame at `:653-655`, and `META_FLAG_READONLY` clearing `can_edit` at `:503`.

What remains outstanding, requiring a build and a flight on a proper host.

D1 is confirmed by launching `basalt_slam --show-gui` and reading the plotter legend before touching any control. It must read `position x`, `position y`, `position z` from the first rendered frame, and toggling `show_est_vel` must add the three velocity traces without disturbing them.

D2a is confirmed by reading the `map_pts`, `map_kfs` and `map_age` panel fields across a flight. A healthy mapper shows a point count that changes and an age that resets. The former defect would reproduce as a point count of zero with a fresh age, which is the discriminating observation the recording could not supply.

D2b is confirmed by that same field never reaching zero points once the map is established, and by the local map point cloud persisting across the intervals that were empty in the recording, namely t in [224, 274] and t in [380, 458] of an equivalent flight.

D3 is confirmed by zooming into the sliding window in the 3D panel and finding exactly one saturated red frustum, drawn at the leading edge of the red trajectory and larger than the grey frusta trailing it, with the local map keyframes now clearly amber.

## 9. Deliberately not done

The plotter's x axis origin `mpStartTns` is set from the first state sample and the plotter is constructed with a fixed x range of 0 to 100, so a flight longer than 100 seconds scrolls out of the initial view. The recording shows the operator panning it manually. This is a usability limitation rather than a defect and is out of scope here.

`mpVioDataLog` still receives no `SetLabels` call. It is unnecessary once `DrawPlots` runs at layout time, because every series is added with an explicit title, and adding labels would duplicate the naming in two places.

The stale snapshot question is left as it stands. When the mapper is idle between keyframes the visualiser keeps drawing the last map, which is the correct behaviour for a live viewer, and D2a makes the staleness visible rather than hiding it.
