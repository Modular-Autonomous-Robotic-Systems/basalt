# The Live Visualiser Pipeline

Reference for `class SlamVisualiser`, written 2026-09-08 while diagnosing three defects from the `visualisation_test.webm` SITL recording. The change record with full code is [`../plans/visualiser_defects.md`](../plans/visualiser_defects.md). Related context lives in [`keyframe_driven_local_mapping.md`](keyframe_driven_local_mapping.md) and [`gt_slam_alignment.md`](gt_slam_alignment.md).

## The two palettes and why there are two

`include/basalt/utils/vis_utils.h` is shared with six offline viewers, namely `src/vio.cpp`, `src/mapper.cpp`, `src/vio_sim.cpp`, `src/mapper_sim.cpp`, `src/mapper_sim_naive.cpp` and `src/rs_t265_vio.cpp`. Editing a constant there restyles all of them, so the live viewer's own colours live in `include/basalt/visualisation/utils.h` instead. That header is also included by `include/basalt/controller.h` for `GtPose` and by `include/basalt/vi_estimator/local_mapper.h` for `LocalMapperVisualizationData`, neither of which reads a colour. All the constants are `const` at namespace scope, so they carry internal linkage and multiple inclusion raises no one-definition rule violation.

`cam_color` and `state_color` in the shared header are the same triple `{250, 0, 26}`. This is the root of a trap. Any scheme that tries to mark the current frame by drawing it in `cam_color` over states drawn in `state_color` is invisible. Stock `src/vio.cpp:812-821` does exactly that, so it double draws one frustum in one colour and nobody can see it. `SlamVisualiser` never inherited that block.

The live palette as of 2026-09-08 is `current_frame_color{250, 0, 26}` for `states.back()`, `window_state_color{120, 120, 120}` for the older sliding-window states, `pose_color{0, 50, 255}` from the shared header for VIO keyframes, `local_map_point_color{255, 128, 0}` for mapped landmarks and `local_map_kf_color{255, 191, 0}` for local map keyframes. That last one was `{255, 128, 128}` until 2026-09-08, a desaturated red whose comment called it amber. It read as a second red frustum beside a VIO state frustum and was the direct cause of the two red keyframes report.

`VioVisualizationData::states` is filled from `frame_states` at `src/vi_estimator/sqrt_keypoint_vio.cpp:620-623`. `Eigen::aligned_map` is `std::map` with `std::less` at `thirdparty/basalt-headers/include/basalt/utils/eigen_utils.hpp:57`, so iteration is in ascending timestamp order and `states.back()` is the newest state. `config.vio_max_states` is 3 in `data/sitl_config_vo.json`, so up to three state frusta are drawn at once. In visual-only mode `frame_states` is asserted empty at `sqrt_keypoint_vo.cpp:475`, so `states` is empty and only blue keyframe frusta appear.

`data/sitl_calib.json` declares one camera, so the inner loop over `mpCalib.T_i_c` contributes exactly one frustum per pose. A stereo rig would double every frustum, which is worth remembering before reading a doubled frustum as a defect.

## Pangolin plotter traps

`pangolin::Plotter`'s constructor installs ten default series whenever it is handed a non-null log, at `thirdparty/Pangolin/src/plot/plotter.cpp:283-288`. Each takes the default title `"$y"`, which routes through `PlotTitleFromExpr` at `:1116`. That function substitutes a human label only when the series index falls inside `default_log->Labels()`, at `:1121-1143`, and otherwise emits the raw `$id`. `mpVioDataLog` never receives `SetLabels`, so the ten titles degrade to the literal strings `$0` through `$9`. This is what the operator sees if nothing replaces them.

`SlamVisualiser::DrawPlots` is the only code that clears them. Until 2026-09-08 its sole call site was gated on `GuiChanged()`, so the Pangolin defaults stood on screen until the first checkbox click. `pangolin::Var<T>::GuiChanged` at `thirdparty/Pangolin/include/pangolin/var/var.h:276-283` is a consume-on-read edge detector, not a state query. It clears `gui_changed` as it reports and can only ever describe a transition the operator caused, so it can never establish an initial state. `DrawPlots` is now called once at the end of `SetupLayout`, and it is safe to call anywhere because it begins with `ClearSeries` and `ClearMarkers`.

The plotter is constructed with a fixed x range of 0 to 100 seconds at `src/visualisation/visualiser.cpp:103`, so a longer flight scrolls out of the initial view and must be panned manually. This is a known limitation, not a defect.

## Panel widgets

`Panel::AddVariable` at `thirdparty/Pangolin/src/display/widgets/widgets.cpp:146-172` dispatches by type. A `bool` becomes a `Checkbox` when `META_FLAG_TOGGLE` is set and a `Button` otherwise, the arithmetic types become a `Slider`, a `std::function<void(void)>` becomes a `FunctionButton`, and everything else including `std::string` falls through to a `TextInput`. `TextInput::Render` at `:653-655` re-reads its variable every frame unless the operator is mid edit, so a `Var<std::string>` written from the render loop is a live readout. `META_FLAG_READONLY`, defined at `thirdparty/Pangolin/include/pangolin/var/varvaluegeneric.h:36`, clears `can_edit` at `:503` and makes it display-only. The three-argument `Var` constructor at `var.h:169` takes the flags directly.

A widget is one row and never wraps. `TextInput::Render` at `:653-677` draws the title left-aligned at `raster[0]`, which `ResizeChildren` sets to `v.l + 2`, and the value right-aligned at `v.l + v.w - gledit.Width() - 2`, both on the same baseline `raster[1]`. When title and value together exceed the panel width they overlap, and the library offers no multi-line widget. The only way to put a value on its own line is a separate `Var`, so a readout with several fields becomes several narrow `Var`s rather than one wide one. The panel is 200 pixels wide, set by `UI_WIDTH` in `SlamVisualiser::SetupLayout`.

The widget title comes from `Meta().friendly`, which `InitialiseNewVarMeta` at `thirdparty/Pangolin/include/pangolin/var/var.h:68` sets to the last dot-separated part of the variable name. It is captured into the widget's `gltext` when the widget is constructed at registration, so mutating `friendly` afterwards does not change what is drawn. Keep the leaf name short.

## Cache semantics and what intermittency actually means

The three consumer threads keep latest-only caches. `mpLatestVio`, `mpLatestLocalMap` and the appended `mvpVioTrajectory`. The two pointer caches are `shared_ptr`s that are never reset, so whatever arrived last stays on screen indefinitely. A queue drop therefore cannot make anything disappear, it can only make it stale.

This matters for diagnosis. If a layer vanishes from the 3D panel, the arriving payload was empty, it was not dropped. The local map queue holds four entries and `LocalMapper` publishes with `try_push` at `src/vi_estimator/local_mapper.cpp:286`, but a dedicated consumer drains it immediately, so drops cannot account for a multi-second gap.

Because `LocalMapper` is keyframe-driven, an idle mapper and a mapper publishing empty maps and a dead mapper all render identically, as an unchanging screen. The `ui.map_pts`, `ui.map_kfs` and `ui.map_age` readouts added 2026-09-08 separate them, so a point count of zero with a fresh age is an empty publish while an unchanging count with a climbing age is silence. They were briefly a single `ui.local_map` field carrying all three values, which overlapped its own title, for the reason given under panel widgets above.

## How the recording was analysed

The container has no `ffmpeg`. See [`/ws/skills/media-analysis-tooling.md`](/ws/skills/media-analysis-tooling.md) for the GStreamer recipe. The technique that made the diagnosis tractable was to decode the whole clip at 0.5 Hz and then classify the 3D panel of every frame by exact match against the palette triples, with a tolerance of about 18 per channel and a white-pixel count used to reject frames where the panel was not on screen. That converts a qualitative flicker report into a timeline of per-layer pixel counts, and the shape of that timeline is diagnostic. Monotone growth from near zero means a layer being rebuilt from scratch, whereas a smooth decay means it leaving the viewport. A layer that vanishes between two adjacent frames while the trajectory and the other layers hold still is a data change and never a camera move.

Measured on `visualisation_test.webm`, 608 seconds long, local map points were absent over t in [16, 64], [224, 274] and [380, 458] and rebuilt monotonically from near zero at t equal to 66, 324 and 470.

## The world-origin triad is the only coordinate axes drawn, and only its blue (Z) axis is meaningful (2026-09-19)

`DrawScene` (`src/visualisation/visualiser.cpp:344`) calls `pangolin::glDrawAxis(Sophus::SE3d().matrix(), 1.0)`, identity, no transform. This is the only place the visualiser draws a multi-axis triad, `render_camera` (`include/basalt/utils/vis_utils.h:48-77`) draws every state/keyframe/GT frustum as a single flat colour with no axis lines. So what's on screen as "the coordinate system" is literally `T_w_i`'s own SLAM-world reference frame, coloured by Pangolin's fixed convention X=red, Y=green, Z=blue (`thirdparty/Pangolin/include/pangolin/gl/gldraw.h:142-147`).

Only the blue (Z) axis carries a reliable meaning: Basalt gravity-aligns its world at filter init, so Z is up by design (see `gt_slam_alignment.md` §5c). The red/green (X/Y) axes do not, this platform's standard NED/FRD IMU mounting sits almost exactly on the degenerate, noise-dominated branch of the `FromTwoVectors` call that sets the initial pose, and four real captured `T_w_i` matrices show the horizontal heading swinging across roughly 160° between runs with no discernible pattern. Full derivation and the four-sample table are in `gt_slam_alignment.md` §5e. Practical upshot for reading this visualiser: describing what it plots as "front-right-up" over-claims, treat only "Z is up" as a guaranteed reading of the triad, not any horizontal-axis meaning.

The follow-camera (`visualiser.cpp:217-227`) zeroes the pose's rotation before calling `mpCamera.Follow`, so it re-centres the view on the vehicle's position only and never rotates the viewport to track heading; the fixed `pangolin::AxisZ` up-vector set at construction (`visualiser.cpp:167`) is consistent with, and does not compensate for, the same Z-up/arbitrary-yaw world convention.
