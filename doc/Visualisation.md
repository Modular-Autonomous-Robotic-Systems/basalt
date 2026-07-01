# Visualisation in Basalt SLAM

## 1. Introduction

Basalt implements integration tests through executables for individual compoenents. The code-base ships with three offline integration tests: `basalt_vio` (`src/vio.cpp`), `basalt_mapper` (`src/mapper.cpp`), and `basalt_vio_sim` (`src/vio_sim.cpp`), each of which embeds a graphical user interface (GUI) built on top of the Pangolin OpenGL toolkit (`thirdparty/Pangolin`). The integration tests serve a dual purpose. They invoke the live pipelines using sensor input to estimate VIO and mapping outputs on benchmark or recorded datasets, and provide a diagnostic tool that render the geometric state of the SLAM system in a form that a human can inspect and investigate.

A new local-mapping subsystem is being introduced into the Basalt as desigined in `doc/LocalMapper.md`. The local mapper runs concurrent to the Visual Inertial Odometry (VIO) thread, ingesting `MargData` packets emitted by the VIO thread, and pushes refined keyframe poses into a queue for the VIO thread to consume. Neither of the two existing visualisations, however, currently renders both the VIO state and the local map simultaneously: `basalt_vio` only performs VIO and renders the VIO output, and `basalt_mapper` is an offline mapping tool that loads serialised `MargData` from disk and only visualises map built offline. To validate the integrated SLAM pipeline end-to-end, a unified visualisation that simultaneously renders the live VIO trajectory and the live local map is required. `ORB_SLAM3` implements a similar pipeline which may be used as a reference for implementation (`ORB_SLAM3` to be used as reference and not as a code source due to GPLv3 license it carries.).

This document collects the working knowledge required to design, implement, and reason about the unified visualisation interface required for `basalt_slam` integration test. This document is divided into the following 5 sections. **§2** surveys the Pangolin toolkit at the level of conceptual preliminaries, software design, software architecture, and worked sample usage drawn from the existing Basalt code. **§3** dissects the existing per-executable visualisation pipelines for VIO, mapping, and simulation, identifying the patterns that recur across them and the points at which they diverge. **§4** outlines the proposed unified SLAM visualisation system and its abstract class boundary. Section **§4** will be revised in subsequent iterations as the implementation matures. **§5** consolidates every Pangolin and Basalt visualisation type, method, and field referenced in the preceding sections into a single tabular reference. **§6** lists references to source files, third-party headers, and adjacent Basalt documentation.

---

## 2. Pangolin

Pangolin is a C++ library for rapid prototyping of OpenGL-based computer-vision and robotics user interfaces, originally developed by Steven Lovegrove and bundled in Basalt under `thirdparty/Pangolin`. The library does not aim to be a fully retained-mode scene graph nor a general-purpose game engine and thus, is intentionally thin. Its purpose is restricted to four orthogonal concerns: windowing, viewport layout, OpenGL state management, and lightweight GUI controls, all composed in a way that makes it natural to express the read-only video viewer, 3D viewer and control knobs pattern that dominates research code in SLAM, photogrammetry, and computational geometry visualisation requirements.

Subsections 2.1–2.4 disccuss in detail the following:
1. graphics preliminaries the library assumes the reader knows
2. the design idioms used in the library
3. the software architecture that ties the idioms into a runnable application
4. a catalogue of concrete usage patterns drawn from the existing Basalt executables.

### 2.1 Visualisation and 3D Rendering Preliminary

This section builds the conceptual foundation needed to understand how Pangolin and Basalt transform SLAM state estimates into pixels on a screen. It is structured as a learning resource. It begins with an introduction to OpenGL and the graphics pipeline, then progressively deepens into each mechanism that Basalt's visualisation code relies on. Subsections 2.1.1–2.1.7 each examines one specific mechanism, relating it back to the pipeline overview and anchoring it in Basalt and Pangolin source code. Together these concepts are sufficient to read and extend any of the three existing Basalt GUIs, and they form the basis for designing the unified SLAM visualisation described in §4.

#### 2.1.1 OpenGL and the Graphics Pipeline

OpenGL (Open Graphics Library) is a cross-platform C API that exposes a hardware-accelerated graphics pipeline. It is imperative rather than declarative, which implies that an OpenGL application issues a sequence of state-setting calls (`glEnable`, `glColor3ubv`, `glMatrixMode`) and draw commands (`glDrawArrays`) which the GPU driver executes in issuance order. Pangolin is a thin layer on top of OpenGL which handles window creation, viewport layout, and convenience draw helpers, but every pixel ultimately results from an OpenGL call issued by Pangolin or Basalt itself.

In OpenGL all data is expressed as vertices. A vertex in OpenGL is simply a 3D position, a triplet of floating-point coordinates in world space, optionally decorated with per-point attributes such as a colour or surface normal. It carries no semantic meaning at the GPU level. Thus, the GPU does not know whether a vertex is a camera pose or a landmark. In Basalt's SLAM visualisation, two conceptually distinct kinds of data are ultimately expressed as OpenGL vertices: map points and camera poses.

At its simplest, rendering a 3D object requires three things:
1. providing the GPU with a set of vertex positions (and optionally colours and normals)
2. telling OpenGL how to assemble those vertices into primitives such as triangles, lines, or points
3. letting the GPU to execute the rendering pipeline that transforms those vertices into pixels.

The pipeline is a fixed sequence of stages through which vertex data passes on its way to the screen, as described in the following flowchart.

```
  CPU (application code)
       │
  ┌────▼───────────────────────────────────────────────────────────┐
  │  Vertex Data (CPU RAM)                                         │
  │  e.g. std::vector<Eigen::Vector3f> of landmark positions       │
  └────┬───────────────────────────────────────────────────────────┘
       │  glDraw* call transfers data and triggers GPU execution
  ┌────▼───────────────────────────────────────────────────────────┐
  │  Vertex Processing                                             │
  │  Apply Model–View–Projection matrices to each vertex:          │
  │    x_clip = P · V · M · X_local                               │
  │  Output: clip-space (homogeneous) coordinates per vertex       │
  └────┬───────────────────────────────────────────────────────────┘
       │
  ┌────▼───────────────────────────────────────────────────────────┐
  │  Primitive Assembly                                            │
  │  Group vertices into geometric primitives:                     │
  │    GL_POINTS    → one point per vertex                         │
  │    GL_LINES     → pairs of vertices → discrete line segments   │
  │    GL_LINE_STRIP → connected polyline through all vertices      │
  └────┬───────────────────────────────────────────────────────────┘
       │
  ┌────▼───────────────────────────────────────────────────────────┐
  │  Rasterisation                                                 │
  │  Convert continuous primitives to discrete pixel fragments.    │
  │  For each covered pixel: interpolate depth z and colour.       │
  │  Clip (discard) fragments outside the active viewport.         │
  └────┬───────────────────────────────────────────────────────────┘
       │
  ┌────▼───────────────────────────────────────────────────────────┐
  │  Fragment Processing                                           │
  │  Per-pixel operations:                                         │
  │    Depth test: compare fragment z to depth buffer; discard     │
  │                occluded fragments (§2.1.4)                     │
  │    Colour: apply glColor / texture sample                      │
  └────┬───────────────────────────────────────────────────────────┘
       │
  ┌────▼───────────────────────────────────────────────────────────┐
  │  Framebuffer Output                                            │
  │  Write surviving fragments to the colour buffer.               │
  │  Update depth buffer with new minimum depth values.            │
  │  Double buffering: FinishFrame() swaps back→front (§2.1.3).    │
  └────────────────────────────────────────────────────────────────┘
```

In current basalt visualisation implementation, each tracked feature or marginalised map point is a `Eigen::Vector3f` in world coordinates that is passed to `glVertex3f` (or packed into a `pangolin::GlBuffer`) and rendered as a `GL_POINTS` primitive, creating one OpenGL vertex per map point. Each camera pose is an `SE3d` rigid-body transform. Since a vertex is only a 3D point, its translation component (the camera-centre position in world coordinates) is extracted and used as an OpenGL vertex. A sequence of such translations is typically rendered as a `GL_LINE_STRIP` to draw the estimated trajectory, or as individual `GL_POINTS` to highlight keyframe locations. In both cases the value passed to the GPU is a plain `vec3` and what differs between the two SLAM data types is only which geometric primitive OpenGL uses to assemble those vertices and how they are coloured.

Rasterisation bridges the gap between the continuous world of 3D geometry and the discrete pixel grid of the display. Given a primitive, for instance, a line segment from projected vertex $A$ to projected vertex $B$ in screen space, the rasteriser determines which pixels the segment covers and generates one fragment per covered pixel. A fragment in OpenGL is essentially a candidate pixel which is not yet a final pixel because it still needs to pass depth testing and other fragment processing stages before rendering. Each fragment carries its screen-space $(x, y)$ position, its interpolated depth $z$ (used for depth testing), and any per-vertex colour. In Basalt's 3D views, the trajectory line strips, landmark point clouds, and camera frustum wireframes all pass through rasterisation on every rendered frame. Given a primitive, for instance, a line segment from projected vertex $A$ to projected vertex $B$ in screen space, the rasteriser determines which pixels the segment covers and generates one fragment per covered pixel.

However, before determining pixel coverage the GPU performs a perspective divide over the clip space. OpenGL utilises a homogenous coordinate system to process and render 3D points. caled the clip space. Clip space is the 4D homogeneous coordinate space that results from applying the full MVP matrix to a vertex: $\mathbf{x}_{clip} = P \cdot V \cdot M \cdot \tilde{X}_{local}$, yielding a 4-vector $(x_c, y_c, z_c, w_c)$ in which $w_c$ encodes camera-space depth and scales all other components. Without normalisation the coordinates in clip space are not yet true 3D positions. The MVP transformation matrix is discussed in more details in §2.1.3. The perspective divide resolves this by dividing each clip-space coordinate $(x_c, y_c, z_c, w_c)$ by $w_c$ to produce Normalised Device Coordinates (NDC). NDC is a resolution-independent unit cube in which all visible geometry lies within $[-1, 1]^3$. A viewport transform maps NDC $x/y$ linearly onto the pixel grid of the active viewport, which is why Basalt's draw calls produce geometrically correct output at any window size. The rasteriser also uses NDC to clip and discard any primitive parts that fall outside $[-1, 1]^3$ before stepping across the pixel grid. 

The central question for 3D rendering is: given a 3D point expressed in the robot's world frame, which screen pixel should it map to? The Model–View–Projection (MVP) transformation applied during vertex processing converts 3D points to pixel locations for rendering. The MVP pipeline is the mathematical core that §2.1.1 examines in detail, and it is the mechanism that Pangolin's `OpenGlRenderState` is built around.

#### 2.1.2 Viewport, Framebuffer, and Double Buffering

The framebuffer is a GPU-resident block of memory that stores the in-progress image for a window. It is the final destination in the rendering pipeline diagram above. The framebuffer is not a single flat array and has multiple attachments. The colour buffer stores one RGBA value per pixel, written by the fragment processing stage. The depth buffer stores one floating-point depth value per pixel, used by the depth test (§2.1.4). Both attachments are part of the same framebuffer object and are cleared together at the start of each frame.

The viewport is a rectangular sub-region of the framebuffer to which the rasteriser maps NDC coordinates. The viewport transform maps the $[-1,1]^2$ NDC square linearly to a pixel rectangle of width $w$ and height $h$. Pangolin's `View::Activate()` computes the concrete pixel rectangle from the `View`'s fractional or pixel-unit bounds and issues the corresponding `glViewport(x, y, width, height)` call — the application never needs to compute pixel offsets manually. This is why calling `display3D.Activate(camera)` and `img_view_display.Activate()` in the render loop correctly confines each draw callback's output to its own region of the screen.

The window framebuffer is double-buffered* by default. The GPU maintains two complete sets of colour and depth attachments: a front buffer (the currently displayed image) and a back buffer (the image being constructed). Without double buffering, drawing commands would modify the buffer that the display controller is actively scanning out to the monitor, causing screen tearing (a visible horizontal band where content from two different frames appears simultaneously). With double buffering:
1. All `glDraw*` calls during a frame write to the back buffer, which is not yet visible to the user.
2. `pangolin::FinishFrame()` (`display.h:90`) issues a buffer swap. It is an atomic OS-level operation that makes the completed back buffer the new front buffer and recycles the old front buffer for the next frame.

The per-frame structure in `src/vio.cpp:486–...` follows this pattern directly:
```cpp
while (!pangolin::ShouldQuit()) {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);  // clear back buffer
    // ... activate views, issue glDraw* calls to back buffer ...
    pangolin::FinishFrame();   // swap back→front; poll window events
}
```
`glClear` at the top of each frame resets the back buffer's colour attachment and resets the depth buffer to 1.0 (the maximum, representing "nothing drawn yet"). `FinishFrame` both performs the buffer swap and drives Pangolin's internal event loop. This dispatches keyboard and mouse events to registered `Handler` objects (§2.2 D4). Because the front buffer is only updated at `FinishFrame`, every frame the user sees is atomically complete: intermediate drawing states are never visible. The same double-buffered render loop structure applies directly to the unified `basalt_slam` executable. Because intermediate map states (e.g., a partially-drawn trajectory) are written only to the invisible back buffer, frames are always complete and consistent from the user's perspective regardless of how many draw calls compose a frame.

#### 2.1.3 Coordinate Frames and the Model–View–Projection (MVP) Pipeline

The vertex processing stage described in section 2.1.1 applies three sequential matrix transformations to every input vertex, taking it from a local object frame all the way to normalised device coordinates (NDC) ready for the rasteriser. This chain executes on the GPU for every vertex in every `glDraw*` call that Basalt makes. ** A vertex $\mathbf{X}_\text{local}$ in object-local coordinates is transformed to clip coordinates by:
$$
\mathbf{x}_\text{clip} = \mathbf{P}\,\mathbf{V}\,\mathbf{M}\,\tilde{\mathbf{X}}_\text{local}
$$
where $\tilde{\mathbf{X}}_\text{local} = [X, Y, Z, 1]^\top$ is the homogeneous representation of a 3D point. After the GPU performs perspective division, the resulting normalised device coordinate lies in the $[-1, 1]^3$ cube:
$$
\mathbf{x}_\text{ndc} = \mathbf{x}_\text{clip}\,/\,x_{\text{clip},w}
$$

The rasteriser maps this cube linearly to integer pixel coordinates within the active viewport. The three matrices $\mathbf{M}$, $\mathbf{V}$, and $\mathbf{P}$ are the three stages of the MVP pipeline. $\mathbf{M} \in SE(3)$** transforms from the object-local frame to the world frame. When rendering a camera frustum at a known SLAM pose, the model matrix is that camera's world-frame transform. Basalt realises this with the matrix stack idiom in `render_camera` (`include/basalt/utils/vis_utils.h:71–76`):
```cpp
glPushMatrix();
glMultMatrixd(T_w_c.data());   // M = T_w_c: camera pose in world frame
// draw frustum lines in the camera's local frame ...
glPopMatrix();                  // restore the previous model matrix
```
`glMultMatrixd` right-multiplies the current top of the `GL_MODELVIEW` stack by `T_w_c`, composing the model transform with the view matrix already loaded by `View::Activate()`. `glPushMatrix`/`glPopMatrix` save and restore the stack so that each frustum is drawn independently without contaminating the matrix state for subsequent draw calls. The View matrix $\mathbf{V} \in SE(3)$ transforms from world frame to camera (eye) frame. It encodes the virtual observer's position and orientation into the rendering pipeline.

The initial position of the view in the display window, `cam_p` is constructed by rotating a canonical offset $(-0.5, -3, -5)$ into the world frame using the first estimated IMU pose and the IMU-to-camera-0 calibration, placing the virtual camera behind and above the robot's starting position. The `pangolin::AxisZ` argument to `pangolin::ModelViewLookAt` declares world $+Z$ as the "up" direction, consistent with Basalt's Z-forward world convention (§2.1.2 and §2.1.4). The Projection matrix $\mathbf{P} \in \mathbb{R}^{4 \times 4}$ is then used to map camera-frame coordinates to clip coordinates, encoding perspective foreshortening and the near/far clipping planes. `ProjectionMatrix(w, h, fu, fv, u0, v0, zNear, zFar)` constructs a perspective projection matrix using focal lengths `fu`, `fv` and principal point `u0`, `v0` in pixels. Vertices closer than `zNear` or farther than `zFar` in camera space are clipped by the rasteriser. More details regarding this can be found later in the document in sections §2.1.4 and 44.

Pangolin's `OpenGlRenderState` (`opengl_render_state.h:173`) bundles $\mathbf{P}$ and $\mathbf{V}$ together. Calling `display3D.Activate(camera)` (`src/vio.cpp:507`) uploads both matrices to the GPU's matrix stack before the draw callback fires. The view matrix $\mathbf{V}$ is stored in `OpenGlRenderState::modelview` and the projection matrix in `OpenGlRenderState::projection` (see §2.1.4 for the full member variable reference). The model matrix $\mathbf{M}$ remains the application's responsibility and is set to whatever the application pushes via `glPushMatrix`/`glMultMatrixd` inside its draw callback. Every time Basalt's `draw_scene` callback fires (`src/vio.cpp:481`), it inherits the view and projection already set by the `OpenGlRenderState`. The callback then pushes individual camera poses via `render_camera`, and calls `pangolin::glDrawPoints`/`pangolin::glDrawLineStrip` for landmarks and trajectories. Each of those calls traverses the full vertex processing → primitive assembly → rasterisation → fragment processing pipeline described in §2.1.1

#### 2.1.4 Camera Convention

The following two axis conventions coexist in the visualisation code:
1. OpenGL's canonical convention which is X-right, Y-up, Z-back (RUB). The camera looks down the $-Z$ axis of its local frame, with $+Y$ pointing toward the top of the screen.
2. SLAM literature and Basalt's internal state machines use the X-right, Y-down, Z-forward (RDF) computer-vision convention. The camera looks down $+Z$, with $+Y$ downward. This is consistent with image-plane coordinate system where the origin is at the top-left.
This matters because Basalt's estimated transforms (`T_w_i`, `T_i_c`) are all expressed in RDF, yet OpenGL's fixed-function pipeline assumes RUB. Thus, appropriate coordinate transformation must be applied.

Pangolin provides two families of projection constructors to handle the aforementioned two conventions:
1. `ProjectionMatrix(w, h, fu, fv, u0, v0, zNear, zFar)` (`opengl_render_state.h:240–242`) is an alias for `ProjectionMatrixRUB_BottomLeft`. It implementes the OpenGL-standard RUB form, with the image origin at the bottom-left of the viewport.
2. `ProjectionMatrixRDF_TopLeft(w, h, fu, fv, u0, v0, zNear, zFar)` (`opengl_render_state.h:228–229`). It implements the computer-vision form with the origin at the top-left, matching the RDF image convention.

All three Basalt integration test executables use `ProjectionMatrix` (RUB). SLAM-frame transforms are passed directly into `glMultMatrixd` without axis remapping. The mismatch is compensated at the `ModelViewLookAt` level with `pangolin::AxisZ`, which designates world $+Z$ as the viewer's "up" direction (`src/vio.cpp:473`), aligning the rendered scene orientation with Basalt's Z-forward world frame.

The protected members of `OpenGlRenderState` (`opengl_render_state.h:214–219`) each serve a specific role in tracking the virtual camera and projection state. The following table describes all the relevant member variables.

| Member | Type | Role |
|---|---|---|
| `modelview` | `OpenGlMatrix` | The current view matrix $\mathbf{V}$. Set initially by `ModelViewLookAt`; mutated live by `Handler3D` mouse interactions and by `Follow()`. |
| `projection` | `std::vector<OpenGlMatrix>` | One (or more, for stereo) projection matrices $\mathbf{P}$. In all Basalt executables this vector has exactly one element. |
| `modelview_premult` | `std::vector<OpenGlMatrix>` | Optional transforms pre-multiplied into the view before GPU upload — used for stereo eye offsets; always empty in Basalt's monocular setup. |
| `T_cw` | `OpenGlMatrix` | The camera-to-world transform snapshot taken at the last `Follow()` call. Used to compute the incremental view update that keeps the scene following the robot while preserving user-interactive rotations. |
| `follow` | `bool` | Flag set by `Follow()` and cleared by `Unfollow()`. When true, `Apply()` incorporates the `T_cw`-based offset into the uploaded model-view. |


The "follow camera" behaviour in `src/vio.cpp:489–507` demonstrates how the aforementioned members interact during a rendered frame. Refer to the following code snippet:
```cpp
// src/vio.cpp:489–507 (simplified)
if (follow) {
    Sophus::SE3d T_w_i = it->second->states.back();  // latest IMU world pose
    T_w_i.so3() = Sophus::SO3d();   // zero rotation: track translation only
    camera.Follow(T_w_i.matrix());  // stores T_w_i into T_cw; updates modelview
}
display3D.Activate(camera);         // uploads P + V (with Follow offset) to GPU
```
`camera.Follow(T_wc)` stores the new pose in `T_cw` and recomputes `modelview` so the virtual camera translates with the robot's IMU trajectory while still allowing the user to rotate the view interactively via mouse. The rotation component of `T_w_i` is zeroed so the viewport remains axis-aligned as the robot translates. After `display3D.Activate(camera)` uploads the matrices, the draw callback (`draw_scene`) pushes individual camera-frame poses via `render_camera`, which right-multiplies each `T_w_c` into the already-loaded view matrix — the composed result being the full MVP chain.

#### 2.1.5 Depth Testing

When observing overlapping primitive shapes, the rendering pipeline does not inherently know which of two overlapping primitives is closer to the camera. Without additional information, the colour buffer at each pixel simply reflects the last `glDraw*` call that wrote a fragment there according the painter's algorithm(rendering from background to foreground). Correctness under the painter's algorithm requires drawing all objects in strict back-to-front order. For a live SLAM visualisation where the user orbits the virtual camera interactively via `Handler3D`, computing correct back-to-front order for trajectories, landmark clouds, and frustum wireframes is intractable as depth relationships between all primitives change continuously as the camera moves, depending on the camera pose that is itself being mutated in real time by mouse input. This essentially leads to a major problem when dealing with occuled geometry while rendering.

The solution to the aforementioned overlap problem is the depth buffer. The depth buffer is the depth attachment of the framebuffer described in §2.1.3. For every pixel, the depth buffer stores the $z$-value (mapped to the range $[0, 1]$, where 0 is the near plane and 1 is the far plane) of the closest fragment written so far. When `glEnable(GL_DEPTH_TEST)` is active, the GPU performs the following per-fragment comparison before writing to the colour buffer:
```
if fragment.z < depth_buffer[pixel]:
    colour_buffer[pixel] ← fragment.colour    // new closest: write colour
    depth_buffer[pixel]  ← fragment.z         // update stored minimum depth
else:
    discard fragment                           // occluded: do not overwrite
```
This comparison is performed entirely in hardware at the fragment processing stage, with no CPU involvement and negligible cost.

Using depth testing described above, for a segment of the VIO trajectory passing behind a cluster of point-cloud landmarks, the trajectory's fragments project to the same screen pixels as the closer landmark points, leading to occlusion. The depth test detects that the trajectory fragments have larger $z$-values (they are farther away) than the values already written by the landmarks, and discards them — the trajectory is occluded by the landmark cloud and correctly does not appear in front of it. Without depth testing, whichever primitive was drawn last by `glDraw*` would win, producing visually incorrect and non-deterministic ordering that changes with draw call sequence.

Each Basalt main routine enables depth testing exactly once, immediately after window creation:
```cpp
// src/vio.cpp:432–434  (same pattern at mapper.cpp:209–211, vio_sim.cpp:340–342)
pangolin::CreateWindowAndBind("Main", 1800, 1000);
glEnable(GL_DEPTH_TEST);
```
`glEnable(GL_DEPTH_TEST)` modifies per-context OpenGL state. The per-context OpenGL state bounds to the OpenGL context created by `CreateWindowAndBind` and persists until explicitly changed with `glDisable`. Despite its significance, it must however be noted that OpenGL contexts are initialised with depth testing disabled by default, a historical legacy from when depth buffers were optional hardware. Because depth testing is needed for all draw calls throughout the entire application lifetime, enabling it once immediately after context creation (and never disabling it) is both correct and efficient. The per-frame `glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)` resets the depth buffer to 1.0 at the start of each frame so each frame begins fresh with no residual depth information. With depth testing enabled, Basalt's `draw_scene` callback can issue draw calls for trajectories, landmarks, and frustums in any order. The GPU's depth buffer ensures the correct depth ordering in the final image regardless of draw-call sequence. There is no need to sort primitives by distance from the camera before drawing.

#### 2.1.6 Immediate Mode vs. Retained Mode

Basalt's visualisation code uses a specific subset of the OpenGL API that looks different from both the oldest and the newest OpenGL programming styles. Understanding which generation of API is in use, and why, is necessary to read the existing draw functions correctly, to extend them for the unified `basalt_slam` visualisation, and to correctly interpret what Pangolin's helpers do under the hood.

The following two OpenGL programming generations exist:
1. Immediate mode (OpenGL 1.x; deprecated since 3.0): The application calls `glBegin(mode)`, issues per-vertex commands (`glVertex3f`, `glColor3ubv`) in a loop, and calls `glEnd()`. Geometry state is manipulated by calls like `glPushMatrix`, `glColor3ubv`. The GPU receives vertices one at a time, inline with the CPU instruction stream. The approach is simple but maximally inefficient.
2. Modern programmable pipeline (OpenGL 3.3+ core profile): Vertex data is uploaded once into a Vertex Buffer Object (VBO), a GPU-resident memory allocation. The GPU reads from its own memory at draw time, eliminating CPU-GPU transfer overhead. Fragment and vertex shaders in GLSL replace the fixed-function pipeline. This scales to millions of vertices but requires substantial boilerplate.

Pangolin's drawing helpers in `gl/gldraw.h` use neither pure immediate mode nor full retained mode. They use the client-side vertex array path from OpenGL 1.1. Using `glVertexPointer` to set a pointer into CPU RAM, `glEnableClientState(GL_VERTEX_ARRAY)` to activate it, and `glDrawArrays` to submit all vertices in one batched call as follows:
```cpp
// thirdparty/Pangolin/include/pangolin/gl/gldraw.h:82–98
template<typename T>
inline void glDrawVertices(size_t num_vertices, const T* vertex_ptr, GLenum mode, ...) {
    glVertexPointer(..., vertex_ptr);          // pointer into CPU memory
    glEnableClientState(GL_VERTEX_ARRAY);      // activate client-state (deprecated 3.0+)
    glDrawArrays(mode, 0, num_vertices);       // one batched draw call from CPU pointer
    glDisableClientState(GL_VERTEX_ARRAY);
}
```
This is the implementation underlying `pangolin::glDrawLines`, `pangolin::glDrawPoints`, and `pangolin::glDrawLineStrip` — the helpers used in Basalt's draw callbacks. The vertex data (e.g., a `std::vector<Eigen::Vector3f>` of landmark positions) lives in CPU RAM and is transferred to the GPU on every `glDrawArrays` call. When the vector is updated next frame with new landmark positions, the next `glDrawArrays` transfers the entire new dataset from scratch.

Thus, it is correct to characterise Pangolin as using a mix of deprecated and modern API simultaneously. The deprecated API in use includes: `glPushMatrix`/`glPopMatrix` (matrix stack, deprecated 3.0), `glColor3ubv` (per-vertex colour via fixed-function, deprecated 3.0), `glVertexPointer`/`glEnableClientState` (client-side vertex arrays, deprecated 3.0), and `glMatrixMode` (deprecated 3.0). The simultaneously used modern API includes: `glDrawArrays` (core since 1.1), `glClear`/`glClearColor`, `glEnable`/`glDisable`, and `glViewport`. Pangolin requests an OpenGL compatibility profile context (by not specifying core profile in `CreateWindowAndBind`), which makes the deprecated calls functional on desktop OpenGL drivers. These deprecated calls are unavailable in core profile, WebGL, and OpenGL ES — contexts targeting embedded systems or browsers. Thus, Pangolin may not work on edge devices.

#### 2.1.7 Column-Major Convention

OpenGL specifies that $4 \times 4$ transformation matrices are stored in column-major order: the 16 elements fill memory column by column, so element at row $r$, column $c$ is stored at linear index $4c + r$. This is visible in Pangolin's `OpenGlMatrix` accessor:
```cpp
// opengl_render_state.h:148–153
GLprecision& operator()(int r, int c) {
    return m[4*c + r];   // column c starts at index 4c; row r is the offset
}
// raw storage: GLprecision m[16]; (opengl_render_state.h:157)
```
When `glMultMatrixd(ptr)` reads 16 doubles from `ptr`, it interprets them in this column-major order. Passing a matrix whose memory is laid out row-by-row would silently apply the transpose — a sign error with no compile-time diagnostic.

Eigen's default storage order for fixed-size matrices is also column-major (the `Options` template parameter defaults to `ColMajor`). Basalt represents all SE(3) poses as `Sophus::SE3d`, whose `.matrix()` method returns an `Eigen::Matrix4d` in column-major layout. Consequently, `Eigen::Matrix4d::data()` returns a pointer to a column-major 16-element double array — exactly what `glMultMatrixd` expects. The direct pass in `render_camera` (`vis_utils.h:72`):
```cpp
glMultMatrixd(T_w_c.data());   // column-major Eigen → column-major OpenGL: correct
```
is correct without any transposition.

It must be noted that Pangolin's conversion constructor (`opengl_render_state.h:305–313`) converts an `Eigen::Matrix<P,4,4>` to `OpenGlMatrix` via an element-wise loop over $(r, c)$ pairs, storing each at `m[c*4+r]`. The element-wise loop is necessary to handle mixed precision (e.g., `float` Eigen matrix to `double` `OpenGlMatrix`), where a raw `memcpy` would be incorrect. For `Matrix4d` (matching precision), the loop and a direct copy produce identical results, but Pangolin uses the general path for correctness across all instantiations.

---

With the aforementioned foundation established, §2.2 Software Design and Fundamentals describes the five orthogonal abstractions that Pangolin provides on top of this OpenGL substrate, i.e. window and context management, the hierarchical view tree, the render state object, the input handler system, and the variable and panel binding mechanism. All of the aforementioned abstractions together define the application-level structure that all Basalt GUIs are built from.

### 2.2 Software Design & Fundamentals
<!-- TODO seems good but will have to check formatting and writing. -->
Pangolin's design can be summarised as five orthogonal abstractions, none of which depends on the others except through well-defined narrow interfaces.

**(D1) Window-and-context.** `pangolin::CreateWindowAndBind(title, w, h)` (`display.h:72`) opens a platform-native window and binds an OpenGL context to the calling thread. The function returns a `WindowInterface&` but most applications discard it: subsequent calls to free functions in the `pangolin` namespace operate on the implicit current context tracked by `pangolin::ContextManager`. There is exactly one such implicit context per thread, and it is the only thread permitted to issue GL calls. This single design choice is the root cause of the producer–consumer architecture of every Basalt visualisation (cf. §3.2).

**(D2) The View tree.** A `pangolin::View` (`display/view.h:63`) is a recursive, hierarchical screen rectangle. Each view holds:
1. `Attach top, left, right, bottom`: bounds expressed in mixed coordinates: a `Pix(n)` attach is in pixels, a fractional attach is in $[0,1]$ relative to the parent. This dual mode lets layouts mix a fixed-width side-panel (the `UI_WIDTH = 200` pixel column in every Basalt GUI, e.g. `src/vio.cpp:86`) with proportional sub-displays.
2. `aspect`: an optional locked aspect ratio.
3. `Layout layout`: one of `LayoutOverlay` (children stack, top-most wins), `LayoutVertical`, `LayoutHorizontal`, or the equal-spacing variants. `LayoutEqual` is what `src/vio.cpp:442` and `src/mapper.cpp:216` use to tile a variable number of camera image views.
4. `std::vector<View*> views`: the children. `AddDisplay(View&)` appends.
5. `Handler* handler`: the input handler (see (D4)).
6. `std::function<void(View&)> extern_draw_function`: an optional draw callback (`view.h:226`); if non-null, `View::Render` invokes it instead of the default which simply recurses into children. This is the hook that every Basalt 3D scene uses to inject its own `draw_scene` (`src/vio.cpp:481`, `src/mapper.cpp` is structured slightly differently. See §3.3.2).

The single global `pangolin::DisplayBase()` (`display.h:187`) is the implicit root view returned by the first call. `pangolin::CreateDisplay()` (`display.h:195`) appends a new anonymous child to it.

**(D3) Render state.** `pangolin::OpenGlRenderState` (`opengl_render_state.h:173`) owns the *projection* and *model-view* matrices independently of any view. The application typically constructs one with `ProjectionMatrix(...)` and `ModelViewLookAt(...)` (e.g. `src/vio.cpp:469`–`473`) and then passes it to `View::SetHandler(new Handler3D(camera))` so the handler can mutate the matrices in response to mouse drags. A `Follow(T_wc)` method (`opengl_render_state.h:200`) re-anchors the model-view to track a moving rigid-body pose. This is the mechanism behind the VIO GUI's "follow" button (`src/vio.cpp:489`–`504`).

**(D4) Input handlers.** `pangolin::Handler` (`handler/handler.h:55`) is an abstract base with virtual `Keyboard`, `Mouse`, `MouseMotion`, `PassiveMouseMotion`, `Special` methods. Two concrete subclasses are commonly used: `Handler3D` (`handler.h:71`): orbit/pan/zoom of a `OpenGlRenderState` driven by mouse drags; and `ImageViewHandler` (`handler/handler_image.h`, used as a base of `ImageView`): pan/zoom of an image with mouse-coordinate read-back. Other handlers are plug-and-play; an application can subclass `Handler3D` and override `Mouse` to implement, e.g., point-picking. Basalt does not currently customise the handler.

**(D5) Variables and panels.** `pangolin::Var<T>` (`var/var.h:84`) is a templated wrapper that registers a *named* variable in the global `VarState` singleton. The wrapped value participates in two-way data binding:
1. The application reads the value via `operator const T&()`. `if (show_obs)` in `src/vio.cpp:710` is a read.
2. A panel widget renders the variable as a checkbox (for `bool`), slider (for numeric), button (for `std::function<void()>`), or text box (for `std::string`). The widget mutates the underlying value via `operator=`.
3. The application detects user input via `Var<T>::GuiChanged()` (`var.h:276`), a one-shot edge detector that returns `true` once after the GUI has changed the value and false thereafter.

`pangolin::CreatePanel("ui")` (defined in `pangolin.h`, called e.g. at `src/vio.cpp:451`) creates a panel view that auto-renders every `Var` whose name begins with `"ui."`. The naming convention is the only contract: declaring `Var<bool> show_obs("ui.show_obs", true, false, true)` causes the panel to display a toggleable checkbox labelled "show_obs", without the application needing to touch the panel's internals.

The convenient alias
```cpp
using Button = pangolin::Var<std::function<void(void)>>;
```
(`src/vio.cpp:88`, `src/mapper.cpp:130`, `src/vio_sim.cpp:150`) turns a `std::function`-typed `Var` into a clickable button whose effect is to invoke the stored functor. It is used by every Basalt GUI to bind buttons to free functions like `alignButton`, `saveTrajectoryButton`, etc.

**(D6) Specialised views.** Two `View` subclasses provide turnkey functionality.
1. `pangolin::ImageView` (`display/image_view.h:15`) inherits both `View` and `ImageViewHandler` and provides a `SetImage(ptr, w, h, pitch, GlPixFormat)` method that uploads pixel data to a `GlTexture` and renders it as a screen-aligned quad. Basalt creates one `ImageView` per camera, indexed by `cam_id`, and overlays feature visualisations through `extern_draw_function` (`src/vio.cpp:462`–`463`).
2. `pangolin::Plotter` (`plot/plotter.h:101`) inherits `View` and `Handler` and renders a 2D Cartesian plot driven by a `pangolin::DataLog` (`plot/datalog.h:180`). The application logs floating-point sample tuples via `DataLog::Log(...)`; `Plotter::AddSeries(x_expr, y_expr, mode, colour, title, log)` then declares which dimensions to plot using a tiny GLSL-compatible expression grammar (`$0`, `$1`, ..., `$N` index series). This is the bottom panel of every VIO and simulation GUI.

The five abstractions are deliberately decoupled: a `View` knows nothing about a `Var`, a `Var` knows nothing about a `Handler`, and an `OpenGlRenderState` knows nothing about a `View`. They are composed at the call-site of the application, not by inheritance.

### 2.3 Software Architecture

A canonical Pangolin application unfolds in five strictly-ordered phases. Lines 432–587 of `src/vio.cpp` realise each phase exactly once.

**(A1) Window and context.** `pangolin::CreateWindowAndBind("Main", 1800, 1000)` (`src/vio.cpp:432`) creates the GL context and attaches it to the calling (main) thread. `glEnable(GL_DEPTH_TEST)` (`src/vio.cpp:434`) is issued immediately afterwards because it is a per-context state and is needed as discussed earlier.

**(A2) Static layout.** Top-level `View`s are constructed by `pangolin::CreateDisplay().SetBounds(...)` chains. `src/vio.cpp:436`–`445` creates three top-level views: `main_display` (the right-hand 3D + image area), `img_view_display` (top-left, the camera grid), and `plot_display` (bottom strip). Each `SetBounds(bottom, top, left, right)` call mixes pixel and fractional `Attach`s so the layout reflows when the window resizes. The fixed-width `ui` panel is created last by `pangolin::CreatePanel("ui").SetBounds(...)` (`src/vio.cpp:451`), which intentionally overlaps the left margin. A panel is rendered on top of any view that occupies the same screen rectangle.

**(A3) Variable declarations.** All `Var<T>` instances are declared at file scope (`src/vio.cpp:93`–`120`). Because `Var<T>::Var(const std::string& name, ...)` registers itself with the global `VarState` during static initialisation, by the time the panel is created every "ui."-prefixed variable is already known to it and the panel displays them in declaration order. Variable declaration internally notifies the current context of the variables declared and the through a prefix unique variable sets may be identified. This is why the side panels of all three Basalt executables are visually consistent: changes only require editing one file. The same property makes the variables freely readable from any free function: `show_obs`, `show_ids`, etc. are global names, accessible from `draw_image_overlay` without having to plumb a context pointer through every callback (`src/vio.cpp:710`).

**(A4) Dynamic content and callbacks.** Geometry that depends on per-camera or per-frame state is wired up after the layout is fixed. Each `ImageView` is constructed and added to its parent view inside a loop over `calib.intrinsics` (`src/vio.cpp:454`–`464`); each one binds an `extern_draw_function = std::bind(&draw_image_overlay, _1, idx)` so the per-camera draw call has access to the camera id. The 3D `display3D` view is constructed with `SetHandler(new Handler3D(camera))` (`src/vio.cpp:479`). Note the `new` without a matching `delete`: this is an intentional leak because Pangolin owns the handler for the application's lifetime. The view is then given `extern_draw_function = draw_scene` (`src/vio.cpp:481`).

**(A5) Main loop.** A `while (!pangolin::ShouldQuit())` loop (`src/vio.cpp:486`) runs at the application's maximum rate. Within one iteration:

1. `glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)` clears the framebuffer.
2. Application logic runs. For VIO, this is the follow-camera update (`src/vio.cpp:489`–`504`), the `GuiChanged` polling for variable changes (`src/vio.cpp:512`–`558`), and the "step / continue" driver (`src/vio.cpp:569`–`586`).
3. `pangolin::FinishFrame()` (`src/vio.cpp:567`) processes pending GL events, dispatches input to the active handler, recursively renders the view tree (which calls each view's `extern_draw_function`), and swaps buffers.

A small but important consequence of this architecture: no rendering happens outside the main loop. Producer threads (the VIO estimator, the IMU feeder, the optical-flow frontend) cannot draw anything; they must hand off through concurrent queues. This is the architectural seam at which §4's proposed combined visualiser will plug in.

### 2.4 Sample Usage

This subsection demonstrates each Pangolin idiom through a worked excerpt from the existing Basalt source. Every pattern is presented in a uniform shape: a name, a citation, the literal code, and a one-paragraph commentary that ties the snippet back to the abstractions of §2.2 and the canonical phases of §2.3. The intent is that a developer reading §3 already has a known-working template to copy.

**(U1) Window creation and event loop** (`src/vio.cpp:432`–`486`, `src/mapper.cpp:209`–`249`, `src/vio_sim.cpp:340`–`383`)

```cpp
// src/vio.cpp:432
pangolin::CreateWindowAndBind("Main", 1800, 1000);
glEnable(GL_DEPTH_TEST);
// ... layout, panels, views constructed here ...

while (!pangolin::ShouldQuit()) {                                // src/vio.cpp:486
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    // per-frame application logic
    pangolin::FinishFrame();                                     // src/vio.cpp:567
}
```

All three executables guard the GUI path on a `--show-gui` CLI option and use the same `CreateWindowAndBind` → `while (!ShouldQuit())` → `FinishFrame()` skeleton. The `glEnable(GL_DEPTH_TEST)` must follow `CreateWindowAndBind` because the depth buffer is per-context state.

**(U2) Mixed pixel / fractional layout** (`src/vio.cpp:436`–`445`)

```cpp
// src/vio.cpp:436
pangolin::View &main_display = pangolin::CreateDisplay().SetBounds(
    0.0, 1.0, pangolin::Attach::Pix(UI_WIDTH), 1.0);

pangolin::View &img_view_display =
    pangolin::CreateDisplay()
        .SetBounds(0.4, 1.0, 0.0, 0.4)
        .SetLayout(pangolin::LayoutEqual);

pangolin::View &plot_display = pangolin::CreateDisplay().SetBounds(
    0.0, 0.4, pangolin::Attach::Pix(UI_WIDTH), 1.0);
```

The `main_display` is bounded by `(bottom=0.0, top=1.0, left=Pix(UI_WIDTH), right=1.0)`: full vertical extent, a $200$-pixel left margin reserved for the side panel, and a fractional right edge. The fluent `.SetBounds(...).SetLayout(...)` chain reflects D2's design intent: a `View` is a self-describing rectangle, and the resize logic is encapsulated by the `Attach` mixed coordinate system.

**(U3) Side panel of toggles** (`src/vio.cpp:93`–`120`, `src/mapper.cpp:114`–`144`, `src/vio_sim.cpp:130`–`156`)

```cpp
// src/vio.cpp:93
pangolin::Var<int>  show_frame  ("ui.show_frame",   0,     0,     1500);
pangolin::Var<bool> show_flow   ("ui.show_flow",    false, false, true);
pangolin::Var<bool> show_obs    ("ui.show_obs",     true,  false, true);
pangolin::Var<bool> show_ids    ("ui.show_ids",     false, false, true);
// ... 13 more toggles ...
pangolin::Var<bool> follow      ("ui.follow",       true,  false, true);

// src/vio.cpp:451
pangolin::CreatePanel("ui").SetBounds(0.0, 1.0, 0.0,
                                      pangolin::Attach::Pix(UI_WIDTH));
```

Each `Var<bool>` declared with `(min=false, max=false, toggle=true)` renders as a checkbox; an `int` declared with `(value=0, min=0, max=N)` renders as a horizontal slider; a `double` with similar arguments renders as a scrolling spinner. Because the `Var<T>` constructor registers the variable into the global `VarState` during static initialisation (before `main` runs), the panel created at line 451 already finds them; no manual wiring is needed.

**(U4) Buttons** (`src/vio.cpp:88`, `:106`, `:107`, `:112`, `:118`)

```cpp
// src/vio.cpp:88
using Button = pangolin::Var<std::function<void(void)>>;

// src/vio.cpp:106 onwards — each Button is bound to a free function
Button next_step_btn ("ui.next_step",  &next_step);
Button prev_step_btn ("ui.prev_step",  &prev_step);
Button align_se3_btn ("ui.align_se3",  &alignButton);
Button save_traj_btn ("ui.save_traj",  &saveTrajectoryButton);
```

The alias collapses the verbose `Var<std::function<void(void)>>` into a single token. When the user clicks the rendered button, Pangolin invokes the stored functor with no arguments. This is the simplest possible action API: every Basalt GUI binds buttons to existing free functions like `alignButton` (`src/vio.cpp:935`) without subclassing or callback registration plumbing.

**(U5) Image grid with one view per camera** (`src/vio.cpp:454`–`464`)

```cpp
// src/vio.cpp:454
std::vector<std::shared_ptr<pangolin::ImageView>> img_view;
while (img_view.size() < calib.intrinsics.size()) {
    std::shared_ptr<pangolin::ImageView> iv(new pangolin::ImageView);

    size_t idx = img_view.size();
    img_view.push_back(iv);

    img_view_display.AddDisplay(*iv);
    iv->extern_draw_function =
        std::bind(&draw_image_overlay, std::placeholders::_1, idx);
}
```

This is the canonical pattern for an N-camera display. Each `ImageView` is heap-allocated to outlive the loop iteration, registered as a child of `img_view_display`, and bound to `draw_image_overlay` with the camera index captured by value. The outer view's `LayoutEqual` then handles the tiling automatically, so adding a third or fourth camera is a configuration-only change.

**(U6) 3D scene with orbit/pan/zoom** (`src/vio.cpp:466`–`481`)

```cpp
// src/vio.cpp:466
Eigen::Vector3d cam_p(-0.5, -3, -5);
cam_p = vio->getT_w_i_init().so3() * calib.T_i_c[0].so3() * cam_p;

camera = pangolin::OpenGlRenderState(
    pangolin::ProjectionMatrix(640, 480, 400, 400, 320, 240, 0.001, 10000),
    pangolin::ModelViewLookAt(cam_p[0], cam_p[1], cam_p[2], 0, 0, 0,
                              pangolin::AxisZ));

pangolin::View &display3D =
    pangolin::CreateDisplay()
        .SetAspect(-640 / 480.0)
        .SetBounds(0.4, 1.0, 0.4, 1.0)
        .SetHandler(new pangolin::Handler3D(camera));

display3D.extern_draw_function = draw_scene;
```

The `OpenGlRenderState camera` is declared at file scope (`src/vio.cpp:124`) so that `draw_scene` (which receives only a `pangolin::View&`) can read it without a captured context. `Handler3D(camera)` wires mouse drags into `camera`'s mutable model-view matrix; the `new` is intentionally not paired with a `delete` because Pangolin retains ownership for the application lifetime. The negative aspect `-640/480.0` flips the y-axis to match the image-plane convention.

**(U7) Plotter and data log** (`src/vio.cpp:90`, `:447`–`449`, `:399`, `:878`–`932`)

```cpp
// src/vio.cpp:90
pangolin::DataLog imu_data_log, vio_data_log, error_data_log;
pangolin::Plotter *plotter;

// src/vio.cpp:447
plotter = new pangolin::Plotter(&imu_data_log, 0.0, 100, -10.0, 10.0,
                                0.01f, 0.01f);
plot_display.AddDisplay(*plotter);

// src/vio.cpp:386 — append a row each time a state is consumed
std::vector<float> vals;
vals.push_back((t_ns - start_t_ns) * 1e-9);    // $0  : timestamp seconds
for (int i = 0; i < 3; i++) vals.push_back(vel_w_i[i]);          // $1..$3
for (int i = 0; i < 3; i++) vals.push_back(T_w_i.translation()[i]); // $4..$6
for (int i = 0; i < 3; i++) vals.push_back(bg[i]);               // $7..$9
for (int i = 0; i < 3; i++) vals.push_back(ba[i]);               // $10..$12
vio_data_log.Log(vals);

// src/vio.cpp:882 — declare which columns to render
plotter->AddSeries("$0", "$4", pangolin::DrawingModeLine,
                   pangolin::Colour::Red(), "position x", &vio_data_log);
// ... eleven more AddSeries ...
double t = vio_dataset->get_image_timestamps()[show_frame] * 1e-9;
plotter->AddMarker(pangolin::Marker::Vertical, t, pangolin::Marker::Equal,
                   pangolin::Colour::White());
```

A `DataLog` is a column-oriented `float` buffer; the row layout is chosen by the application. The `Plotter`'s `AddSeries(x_expr, y_expr, mode, colour, title, &log)` then declares which columns to render with a tiny GLSL-compatible expression grammar: `$0` is column 0 (timestamp, the natural x-axis), `$1`..`$3` are velocity, `$4`..`$6` position, `$7`..`$9` gyro bias, `$10`..`$12` accel bias. The vertical marker visually couples the plot to the active `show_frame` timestamp.

**(U8) GUI-driven action via `GuiChanged()`** (`src/vio.cpp:512`–`558`)

```cpp
// src/vio.cpp:512
if (show_frame.GuiChanged()) {
    for (size_t cam_id = 0; cam_id < calib.intrinsics.size(); cam_id++) {
        size_t frame_id = static_cast<size_t>(show_frame);
        int64_t timestamp = vio_dataset->get_image_timestamps()[frame_id];

        std::vector<basalt::ImageData> img_vec =
            vio_dataset->get_image_data(timestamp);

        pangolin::GlPixFormat fmt;
        fmt.glformat = GL_LUMINANCE;
        fmt.gltype = GL_UNSIGNED_SHORT;
        fmt.scalable_internal_format = GL_LUMINANCE16;

        if (img_vec[cam_id].img.get())
            img_view[cam_id]->SetImage(
                img_vec[cam_id].img->ptr, img_vec[cam_id].img->w,
                img_vec[cam_id].img->h, img_vec[cam_id].img->pitch, fmt);
    }
    draw_plots();
}

if (show_est_vel.GuiChanged() || show_est_pos.GuiChanged() ||
    show_est_ba.GuiChanged()  || show_est_bg.GuiChanged()) {
    draw_plots();
}
```

`GuiChanged()` is a one-shot edge detector. It returns `true` exactly once after the user mutates the widget and `false` thereafter. The pattern decouples the side-panel from the rendering: the panel simply edits a value, the loop polls `GuiChanged()`, and only the relevant downstream effect (image upload, plotter rebuild) runs. This avoids re-uploading the camera image every frame, which would otherwise cost megabytes per second of redundant GPU traffic.

**(U9) Follow-camera mode** (`src/vio.cpp:489`–`504`)

```cpp
// src/vio.cpp:489
if (follow) {
    size_t frame_id = show_frame;
    int64_t t_ns = vio_dataset->get_image_timestamps()[frame_id];
    auto it = vis_map.find(t_ns);

    if (it != vis_map.end()) {
        Sophus::SE3d T_w_i;
        if (!it->second->states.empty()) {
            T_w_i = it->second->states.back();
        } else if (!it->second->frames.empty()) {
            T_w_i = it->second->frames.back();
        }
        T_w_i.so3() = Sophus::SO3d();   // zero the rotation: keep world upright

        camera.Follow(T_w_i.matrix());
    }
}
```

`OpenGlRenderState::Follow(T_wc)` re-anchors the model-view to track a moving rigid-body pose so the orbit handler's drags act relative to the latest body frame. Zeroing `T_w_i.so3()` is a deliberate UX choice: rather than tumble the world as the camera rotates, the world remains upright while the camera origin tracks the body. Without this trick the orbit interaction would feel disorienting on a robot that pitches and rolls.

**(U10) Stepping and continuous-play driver** (`src/vio.cpp:569`–`586`)

```cpp
// src/vio.cpp:569
if (continue_btn) {
    if (!next_step())
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
} else {
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
}

if (continue_fast) {
    int64_t t_ns = vio->last_processed_t_ns;
    if (timestamp_to_id.count(t_ns)) {
        show_frame = timestamp_to_id[t_ns];
        show_frame.Meta().gui_changed = true;
    }

    if (vio->finished) {
        continue_fast = false;
    }
}
```

Two independent semantics coexist. `continue_btn` advances `show_frame` by one tick per render iteration. Every tick involves a wait of 50ms and thus, this implements a play at 20 fps mode. `continue_fast` snaps `show_frame` to whatever the estimator just processed (`last_processed_t_ns`): a "follow the live edge" mode where rendering frequency decouples from estimator frequency. Setting `show_frame.Meta().gui_changed = true` programmatically triggers the same `GuiChanged()` codepath as a user drag, reusing the U8 mechanism for free.

**(U11) Producer-thread to GL-thread handover** (`src/vio.cpp:127`, `:129`, `:349`–`363`)

```cpp
// src/vio.cpp:127
std::unordered_map<int64_t, basalt::VioVisualizationData::Ptr> vis_map;
tbb::concurrent_bounded_queue<basalt::VioVisualizationData::Ptr> out_vis_queue;

// src/vio.cpp:317 — connect the queue to the estimator
if (show_gui) vio->out_vis_queue = &out_vis_queue;

// src/vio.cpp:349 — consumer thread
t3.reset(new std::thread([&]() {
    basalt::VioVisualizationData::Ptr data;
    while (true) {
        out_vis_queue.pop(data);                  // blocks until producer pushes
        if (data.get()) {
            vis_map[data->t_ns] = data;
        } else {
            break;                                // nullptr is the sentinel
        }
    }
}));
```

The estimator (running on its own thread) calls `out_vis_queue.push(payload)` whenever a new keyframe is processed. The consumer thread `t3` blocks on `pop`, inserts the payload into `vis_map` keyed by timestamp, and loops. The main GL thread later reads `vis_map` from `draw_scene` and `draw_image_overlay`. Because the only writer to `vis_map` is `t3` and the only reader is the GL thread, no mutex is needed — the queue is the synchronisation primitive.

**(U12) Per-image drawing of overlays** (`src/vio.cpp:699`–`784`)

```cpp
// src/vio.cpp:699
void draw_image_overlay(pangolin::View &v, size_t cam_id) {
    UNUSED(v);
    size_t frame_id = show_frame;
    auto it = vis_map.find(vio_dataset->get_image_timestamps()[frame_id]);

    if (show_obs) {
        glLineWidth(1.0);
        glColor3f(1.0, 0.0, 0.0);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

        if (it != vis_map.end() && cam_id < it->second->projections.size()) {
            const auto &points = it->second->projections[cam_id];

            // Compute depth range for hue mapping
            double min_id = points[0][2], max_id = points[0][2];
            for (const auto &points2 : it->second->projections)
                for (const auto &p : points2) {
                    min_id = std::min(min_id, p[2]);
                    max_id = std::max(max_id, p[2]);
                }

            for (const auto &c : points) {
                float r, g, b;
                getcolor(c[2] - min_id, max_id - min_id, b, g, r);
                glColor3f(r, g, b);

                pangolin::glDrawCirclePerimeter(c[0], c[1], 6.5f);

                if (show_ids)
                    pangolin::GlFont::I().Text("%d", int(c[3])).Draw(c[0], c[1]);
            }

            glColor3f(1.0, 0.0, 0.0);
            pangolin::GlFont::I()
                .Text("Tracked %d points", points.size()).Draw(5, 20);
        }
    }
}
```

`draw_image_overlay(view, cam_id)` is invoked by Pangolin once per `ImageView` per frame, after the underlying image has been blitted. The function draws into the same pixel coordinate system as the image (origin at top-left, units in pixels). The depth-encoded colour wheel via `getcolor` is the visual cue that lets a human read landmark depth at a glance. Close points are red, far points are blue. The `GlFont::I()` returns Pangolin's default singleton font; `.Text(fmt, ...).Draw(x, y)` is a fluent builder.

**(U13) 3D scene composition** (`src/vio.cpp:786`–`839`)

```cpp
// src/vio.cpp:786
void draw_scene(pangolin::View &view) {
    UNUSED(view);
    view.Activate(camera);                       // glViewport + matrices
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

    glPointSize(3);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    // 1. Estimated trajectory truncated at show_frame
    glColor3ubv(cam_color);
    if (!vio_t_w_i.empty()) {
        size_t end = std::min(vio_t_w_i.size(), size_t(show_frame + 1));
        Eigen::aligned_vector<Eigen::Vector3d> sub_gt(
            vio_t_w_i.begin(), vio_t_w_i.begin() + end);
        pangolin::glDrawLineStrip(sub_gt);
    }

    // 2. Ground-truth trajectory
    glColor3ubv(gt_color);
    if (show_gt) pangolin::glDrawLineStrip(gt_t_w_i);

    // 3. Recent keyframe frusta and landmarks
    auto it = vis_map.find(vio_dataset->get_image_timestamps()[show_frame]);
    if (it != vis_map.end()) {
        for (const auto &p : it->second->states)
            for (size_t i = 0; i < calib.T_i_c.size(); i++)
                render_camera((p * calib.T_i_c[i]).matrix(), 2.0f, state_color, 0.1f);

        for (const auto &p : it->second->frames)
            for (size_t i = 0; i < calib.T_i_c.size(); i++)
                render_camera((p * calib.T_i_c[i]).matrix(), 2.0f, pose_color, 0.1f);

        glColor3ubv(pose_color);
        pangolin::glDrawPoints(it->second->points);
    }

    pangolin::glDrawAxis(Sophus::SE3d().matrix(), 1.0);
}
```

`view.Activate(camera)` simultaneously sets the viewport rectangle (so subsequent `glDraw*` calls go into the correct part of the framebuffer) and applies the projection + model-view matrices held by `camera`. The body of the function is then a sequence of `glColor3ubv` + `pangolin::glDraw*` pairs. The colour state is "sticky" across draw calls, which is why every group is preceded by an explicit colour set. `render_camera` (defined in `include/basalt/utils/vis_utils.h:48`) wraps the frustum-drawing details and encapsulates a `glPushMatrix`/`glMultMatrixd`/`glDrawLines`/`glPopMatrix` quartet.

**(U14) Sentinel-driven shutdown cascade** (`src/vio.cpp:186`, `:602`–`619`)

```cpp
// src/vio.cpp:186 — feed_images
opt_flow_ptr->input_queue.push(nullptr);          // sentinel into front-end

// src/vio.cpp:601
vio->maybe_join();                                // estimator drains queues, exits
vio->drain_input_queues();                        // empty any half-pushed input

t1.join(); t2.join();                             // join feeders
terminate = true;
if (t3) t3->join();                               // visualisation consumer
t4.join();                                        // state consumer
```

Each input queue and each output queue uses `nullptr` as an end-of-stream sentinel. The cascade propagates: feeder pushes `nullptr`, optical-flow front-end forwards `nullptr` to VIO, VIO forwards `nullptr` to `out_vis_queue` and `out_state_queue`, and the consumer threads break out of their `while (true)` loops. The pattern is documented in detail in `doc/LocalMapper.md §2.4`; §4 of the present document extends the cascade to include a third tier (the local mapper).

These fourteen patterns collectively cover every Pangolin construct exercised by the existing GUIs. §3 will treat each of `src/vio.cpp`, `mapper.cpp`, and `vio_sim.cpp` as compositions of these patterns.

---

## 3. Basalt VIO & Mapper Visualisation Pipeline

The three Basalt visualisation executables (`basalt_vio`, `basalt_mapper`, and `basalt_vio_sim`) exercise three superficially-different but structurally-similar pipelines. All three are integration tests composing the full processing chain (frontend → estimator → output) and each renders the resulting state. The reader is referred to `doc/VIO.md` for the VIO algorithmic content and `doc/Mapping.md` for the mapper's NFR algorithmic content. This section is concerned only with the visualisation layer that wraps those algorithms. §3.1 describes the data path that feeds the GUIs, §3.2 the recurring design decisions, §3.3 walks each executable in turn, §3.4 contrasts them, and §3.5 critiques the design from a software-engineering perspective.

### 3.1 Pipeline Architecture

The offline visualisation layer is the consumer end of a producer–consumer pipeline that begins at the dataset (or simulator) and traverses the front-end and estimator before reaching the GUI. The estimator base class `VioEstimatorBase<Scalar>` (`include/basalt/vi_estimator/vio_estimator.h:64`) exposes four queues. Two are input queues filled by feeder threads (`vision_data_queue` and `imu_data_queue`), and three are output queue pointers that the executable populates only if it wants to consume them: `out_state_queue`, `out_marg_queue`, and `out_vis_queue`. When the GUI is enabled, `out_vis_queue` is set to a local `tbb::concurrent_bounded_queue<VioVisualizationData::Ptr>` (`src/vio.cpp:317`). When `out_vis_queue` is null the estimator skips the visualisation-payload assembly entirely (`src/vi_estimator/sqrt_keypoint_vio.cpp:583`).


After processing every frame, the estimator emits one `VioVisualizationData::Ptr` (`include/basalt/vi_estimator/vio_estimator.h:46`) containing the timestamp, the active `frame_states` poses, the marginalised `frame_poses`, the current 3D landmark positions, the corresponding ids, the unmodified `OpticalFlowResult::Ptr` (so the GUI can recompute patch overlays from the original image), and the per-camera 2D projections of every active landmark `Eigen::Vector4d (u, v, depth, id)`. The struct is intentionally lightweight: the heavy `OpticalFlowResult::Ptr` is shared, the pose vectors are flat `Sophus::SE3d`, and there are no shared-state pointers back into the estimator. Independently, every state update emits a `PoseVelBiasState<double>::Ptr` (`vio_estimator.h:84`) — a cheap 15-DoF vector. Two queues are used because the visualisation payload is geometrically rich (poses, landmarks, projections) while the state payload is numerical (a state vector to be plotted as a time series). Splitting the payloads avoids forcing the plotter to discard ¾ of the data it receives.

Each GUI executable spawns one or more consumer `std::thread`s that pop from these queues and translate the payloads into rendering-ready data structures. In `src/vio.cpp` there are four such consumers: feeder threads `t1`/`t2` (image and IMU input, `src/vio.cpp:343`–`344`), visualisation consumer `t3` (`src/vio.cpp:349`–`363`), state consumer `t4` (`src/vio.cpp:365`–`404`). The consumers are simple `while (true) { queue.pop(data); if (!data) break; ... }` loops and `nullptr` is the sentinel that signals end-of-data.

Visualisation data is held by the consumer thread in a thread-local `std::unordered_map<int64_t, VioVisualizationData::Ptr> vis_map` keyed by timestamp (`src/vio.cpp:127`). Frame poses and thus the trajectory is stored in `Pangoling::DataLog vio_data_log`. The renderer (main thread) reads `vis_map` and `vio_data_log` directly. There is no mutex. The only writer is the consumer thread, the only reader is the main thread, and `unordered_map::find` and reading of `vio_data_log` in the plotter is safe against concurrent insertions of new items. This is a fragile property in general, but acceptable here because the visualisation only ever shows one timestamp at a time and the existing race window is much smaller than the human-perception threshold. The intended threading model is therefore:
```
                feed_images ┐                                ┌── t3 consumer ──► vis_map
                feed_imu    ├► OF in_q ► OF out_q ► VIO ►── ┤                       │
                            │             (vision_data_queue)│                       │
                                                            └── t4 consumer ──► vio_data_log
                                                                  out_state_queue
```
Reading the renderer-side cache out of `vis_map` happens from `draw_scene` and `draw_image_overlay`, both of which are invoked by Pangolin during `FinishFrame()`. Reading of frame poses from `vio_data_log` is performed in `draw_plots` which is invoked in the main thread.

### 3.2 Design Decisions and Patterns

The following handful of recurring decisions shape every Basalt GUI implemented in the offline integration tests.

**(P1) Producer–consumer with TBB queues.** `tbb::concurrent_bounded_queue<T>::push` blocks if the queue is full and `pop` blocks if empty, providing automatic back-pressure. Setting the capacity small (10–300) means that a slow consumer cannot let the producer's working set grow without bound. The capacity-300 IMU queue (`vio_estimator.h:72`) is sized for ~1.5 s of 200 Hz data, comfortably above the 50 ms estimator latency.

**(P2) File-scope Pangolin globals.** Every `Var<T>`, `OpenGlRenderState`, `DataLog`, `Plotter*`, and the consumer-side `vis_map` are declared at file scope (`src/vio.cpp:90`–`158`, `src/mapper.cpp:76`–`148`, `src/vio_sim.cpp:90`–`158`). The motivation is direct: Pangolin's `extern_draw_function` is `std::function<void(View&)>` — only a `View&` is plumbed to the callback. To access anything else the callback must read globals. The pattern is simple to write and easy to read but defeats reuse: an attempt to instantiate two visualisations in the same process would collide on every name. This is the principal reason §4 proposes encapsulating state on a `class`.

**(P3) Sentinel `nullptr` shutdown.** Every input queue (`opt_flow_ptr->input_queue`, `vio->imu_data_queue`, `vio->vision_data_queue`) and every output queue uses `nullptr` as an end-of-stream sentinel. `feed_images` (`src/vio.cpp:186`) pushes a final `nullptr` after the loop; the optical-flow thread then pushes `nullptr` to the VIO; the VIO pushes `nullptr` to `out_vis_queue` and `out_state_queue` (`sqrt_keypoint_vio.cpp:190`–`192`); the consumer threads break out of their `while (true)` loops on receiving `nullptr` and terminate. The cascade pattern is described in detail in `doc/LocalMapper.md §2.4`. The unified visualiser must integrate into this cascade.

**(P4) Double-buffered visualisation.** The producer queues *immutable* `VioVisualizationData::Ptr` instances; the consumer caches them in `vis_map`; the renderer reads only from `vis_map`. Producer and renderer never share a writable object — there is no need for a mutex. The pattern is identical to Pangolin's own internal double buffering at the framebuffer level.

**(P5) Separation of plot data and 3D data.** The two output queues from VIO carry semantically-different data. `out_state_queue` is consumed by `t4`, which appends to `vio_data_log` for the plotter; `out_vis_queue` is consumed by `t3`, which fills `vis_map` for the 3D renderer. Although both are derived from the same VIO state, decoupling them lets the plotter receive *every* VIO update at the estimator's full rate while the 3D renderer can drop frames freely without losing trajectory points.

**(P6) GUI-flag gating.** Every executable accepts a `--show-gui` (default `true`) flag. When false, the queue pointers are not connected, the consumer threads are not started, and the executable becomes a pure batch-mode integration test. The flag is the simplest possible mechanism for keeping CI green on headless build agents.

### 3.3 Reference Implementations

Basalt provides integration tests for optical flow, vio and mapper units, integrating and running these over EuRoC, TUM or ROS-bag-formatted datasets. It must however, be noted that all of the current implementations are offline and can only run and visualise data offline. We discuss a realtime visualisation module in later sections that is to be developed for SLAM integration test.

#### 3.3.1 `src/vio.cpp`: Live VIO

`basalt_vio` is the canonical live VIO integration test (`doc/VioMapping.md`). It consumes a EuRoC- or ROS-bag-formatted dataset, drives the optical-flow frontend at full speed, and visualises every estimator output.

The visualisation window is laid out as four nested displays plus the side panel (`src/vio.cpp:436`–`451`). The image below shows this layout for the EuRoC `MH_05` dataset: the two left ImageViews show stereo with red feature circles, the top-right area renders the live trajectory in red against the green ground-truth path, and the bottom strip plots position vs. time with the active timestamp marked by a vertical white line.

![basalt_vio running on EuRoC MH_05](img/MH_05_VIO.png)

`draw_scene` implements rendering of the 3D interactive scene. The method (`src/vio.cpp:786`–`839`) renders items as follows:
1. The estimated trajectory `vio_t_w_i` truncated at frame indexed currently by `show_frame` (`src/vio.cpp:797`–`802`): `pangolin::glDrawLineStrip` in red. `show_frame` is updated in `next_step()` (`src/vio.cpp:857:867`) and within the main rendering loop if `continue_fast` is true.
2. The full ground-truth trajectory `gt_t_w_i` (`src/vio.cpp:804`–`806`): `glDrawLineStrip` in green.
3. The most recent state's camera frusta: one per `calib.T_i_c[i]` rendered with `render_camera(...)` (`src/vio.cpp:813`–`822`, `include/basalt/utils/vis_utils.h:48`).
4. The active states' frusta in `state_color` (`src/vio.cpp:824`–`827`).
5. The active keyframes' frusta in `pose_color` (`src/vio.cpp:829`–`832`).
6. The current landmarks as `glDrawPoints` (`src/vio.cpp:835`).
7. The world-frame axis triad (`src/vio.cpp:838`).

Colors rendering decisions are fixed within the code and the hard-coded RGB constants `cam_color`, `state_color`, `pose_color` and `gt_color` are declared in `vis_utils.h:43`–`46`. These constants determine the colors of all rendered items. The trajectory truncation at `show_frame` lets the user scrub through history with the slider. Both `vio_t_w_i` and `gt_t_w_i` are pre-loaded into buffers so scrubbing is O(1).

The `render_camera` helper that draws every frustum is worth dissecting because it is the only Basalt-specific wrapper around immediate-mode OpenGL state in the rendering pipeline:
```cpp
// include/basalt/utils/vis_utils.h:48
inline void render_camera(const Eigen::Matrix4d& T_w_c, float lineWidth,
                          const uint8_t* color, float sizeFactor) {
  const float sz = sizeFactor;
  const float width = 640, height = 480, fx = 500, fy = 500, cx = 320, cy = 240;

  const Eigen::aligned_vector<Eigen::Vector3f> lines = {
      {0, 0, 0}, {sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz},
      // ... eight line segments forming a frustum cage ...
  };

  glPushMatrix();
  glMultMatrixd(T_w_c.data());        // column-major Eigen → GL_MODELVIEW
  glColor3ubv(color);
  glLineWidth(lineWidth);
  pangolin::glDrawLines(lines);
  glPopMatrix();
}
```
The function deliberately encapsulates the `glPushMatrix`/`glPopMatrix` pair so that the caller never leaks the modified `GL_MODELVIEW` stack. The world-to-camera transform `T_w_c` is pushed onto the stack via `glMultMatrixd`, which exploits the column-major equivalence between Eigen's `Matrix4d` storage and OpenGL's expected layout (cf. §2.1's "column-major convention" paragraph). No transpose or copy is performed.

Image overlays are performed by the method `draw_image_overlay`. `draw_image_overlay` (`src/vio.cpp:699`–`784`) is invoked once per `ImageView` per frame. It looks up `vis_map` by the displayed timestamp. If entries are found for a displayed timestamp, it iterates over `it->second->projections[cam_id]` and renders a circle plus optional id label per landmark, with hue interpolated by depth via `getcolor` (`vis_utils.h:79`). A separate branch under `if (show_flow)` reads `it->second->opt_flow_res->observations[cam_id]` and renders the patch coordinates produced by `opt_flow_ptr->patch_coord`. This is a debug-only visualisation that exposes the affine optical-flow transformation per keypoint.

Additional debug plots for pose, velocity and IMU bias estimates are also provided through the method `draw_plots`. `draw_plots` (`src/vio.cpp:878`–`933`) is called whenever any `show_est_*` toggle changes. It clears all series and adds back only those that the user has enabled, indexing into `vio_data_log` columns by name (`$0` is timestamp, `$1`–`$3` are velocity, `$4`–`$6` are position, `$7`–`$9` are gyro bias, `$10`–`$12` are accel bias). A vertical marker is drawn at the current timestamp (`src/vio.cpp:931`).

The VIO integration test support three visualisation modes:
1. Step by step visualisation
2. 20 FPS playback
3. Fast playback
Thus, three CLI variables drive the playback. `step_by_step` blocks the feeder threads on a `std::condition_variable` named `cv`, (`src/vio.cpp:170`), implementing a step by step VIO test. `next_step()` and `prev_step()` notify the conditional variable to control playback. The feeder thread suspends on `cv.wait(lk)` (`src/vio.cpp:172`) before pushing each frame, so processing cannot outpace the render loop. Only `next_step()` issues `cv.notify_one()` (`src/vio.cpp:861`), releasing exactly one frame into the optical-flow input queue per call. `prev_step()` rewinds only the `show_frame` display slider and does not wake the feeder, because processed frames are already committed to `vis_map` and cannot be re-driven through the estimator. When `continue_btn` is true, `next_step()` is called every loop iteration whereas when `continue_fast` is true, the displayed `show_frame` snaps to `vio->last_processed_t_ns`. `continue_fast` clears itself automatically once `vio->finished` becomes true (`src/vio.cpp:583`–`585`), making fast-forward playback self-terminating.

In `src/vio.cpp`, five threads run in parallel with the main render loop:

| Thread | Source | Role |
|---|---|---|
| `t1` (`src/vio.cpp:343`) | `feed_images` | Pushes `OpticalFlowInput::Ptr` from the dataset into the optical-flow input queue |
| `t2` (`src/vio.cpp:344`) | `feed_imu` | Pushes `ImuData<double>::Ptr` into `vio->imu_data_queue` |
| `t3` (`src/vio.cpp:349`) | (lambda) | Pops `out_vis_queue` and writes into `vis_map` |
| `t4` (`src/vio.cpp:365`) | (lambda) | Pops `out_state_queue` and appends to `vio_data_log` (and aux trajectories) |
| `t5` (`src/vio.cpp:418`, optional) | (lambda) | Periodically prints queue sizes for debugging |

Once vio is complete over the input sequence, thread joining is staged. `vio->maybe_join()` is awaited first (`src/vio.cpp:601`), `vio->drain_input_queues()` empties any half-pushed input on abort (`src/vio.cpp:605`) and finally `t1`–`t5` are joined at completion of the process.

#### 3.3.2 `src/mapper.cpp`: Offline Map Inspection

`basalt_mapper` is an offline tool. It loads serialised `MargData` from a directory written previously by `basalt_vio --marg-data ...` and runs the NFR mapping pipeline interactively, with each stage triggered by a button.

The basalt mapping visualisation window is two-paned. It consists of a pair of left ImageViews stacked vertically and a single right 3D view (`mapper.cpp:213`–`247`). The image below shows the resulting interface: top-left and bottom-left ImageViews showing stereo features for two different timestamps, with detected/matched/inlier overlays in red/blue/green; the 3D view shows the full keyframe trajectory with relative-pose-factor edges and triangulated points.

![basalt_mapper on TUM-VI magistrale1](img/magistrale1_mapping.png)

Two `ImageView`s are dynamically created in `mapper.cpp:222`–`231`; each is bound to `draw_image_overlay` with a different `view_id` (0 or 1). The two views display two independently selected keyframes (`show_frame1` and `show_frame2`) chosen by separate sliders. The dual-slider pattern lets the user inspect feature correspondences between two arbitrarily-chosen keyframes. When `lock_frames` is true (`mapper.cpp:118`, default), changing one slider drags the other (`mapper.cpp:252`–`263`); this turns the dual view into a synchronised stereo display. When unlocked, the two sliders are independent. The user picks any two frames to inspect their feature-matching relationship in `nrf_mapper->feature_matches`.

**3D scene.** `draw_scene` (`mapper.cpp:502`–`530`) draws:
1. All triangulated landmark positions as red points (`mapper.cpp:509`). The sparse map of triangulated landmark positions is incrementally grown through the mapping process.
2. The ground-truth trajectory in green (`mapper.cpp:512`).
3. Co-visibility edges: every observation pair `(host, target)` from `nrf_mapper->lmdb.getObservations()` rendered as a line (`mapper.cpp:592`–`606`, populated into `edges_vis` by `computeEdgeVis`).
4. Roll-pitch factor visualisations: for each `nrf_mapper->roll_pitch_factors`, a short purple line in the direction of the factor's measured gravity (`mapper.cpp:609`–`619`, `mapper.cpp:518`–`520`).
5. Relative-pose factors: every `nrf_mapper->rel_pose_factors` rendered as a red line connecting the two frames (`mapper.cpp:621`–`630`, `mapper.cpp:522`–`523`).
6. A small SE(3) axis triad at every keyframe pose (`mapper.cpp:525`–`527`).

The body of `draw_scene` is concise because the heavy lifting of materialising the line endpoints is performed by `computeEdgeVis`, which traverses the mapper's data structures and produces flat `std::vector` payloads ready for immediate-mode rendering:
```cpp
// src/mapper.cpp:502
void draw_scene() {
  glPointSize(3); glColor3f(1.0, 0.0, 0.0);
  glEnable(GL_BLEND); glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  glColor3ubv(pose_color);
  if (show_points) pangolin::glDrawPoints(mapper_points);

  glColor3ubv(gt_color);
  if (show_gt) pangolin::glDrawLineStrip(gt_frame_t_w_i);

  glColor3f(0.0, 1.0, 0.0);
  if (show_edges) pangolin::glDrawLines(edges_vis);     // co-visibility (green)

  glLineWidth(2);
  glColor3f(1.0, 0.0, 1.0);
  if (show_edges) pangolin::glDrawLines(roll_pitch_vis); // roll-pitch (magenta)
  glLineWidth(1);

  glColor3f(1.0, 0.0, 0.0);
  if (show_edges) pangolin::glDrawLines(rel_edges_vis);  // rel-pose (red)

  for (const auto& kv : nrf_mapper->getFramePoses()) {
    pangolin::glDrawAxis(kv.second.getPose().matrix(), 0.1);
  }
  pangolin::glDrawAxis(Sophus::SE3d().matrix(), 1.0);
}

// src/mapper.cpp:590 — recomputed after each pipeline stage
void computeEdgeVis() {
  edges_vis.clear();
  for (const auto& kv1 : nrf_mapper->lmdb.getObservations()) {
    for (const auto& kv2 : kv1.second) {
      Eigen::Vector3d p1 = nrf_mapper->getFramePoses()
                               .at(kv1.first.frame_id).getPose().translation();
      Eigen::Vector3d p2 = nrf_mapper->getFramePoses()
                               .at(kv2.first.frame_id).getPose().translation();
      edges_vis.emplace_back(p1);
      edges_vis.emplace_back(p2);
    }
  }
  // ... similar loops populate roll_pitch_vis, rel_edges_vis ...
}
```
It must be noted that `computeEdgeVis` iterates over all the keypoint observations at once. `src/mapper.cpp` iterates over the entire dataset at once to perform offline mapping. The mapper visualisation pipeline has much scope for improvement especially with respect to memory management and efficient computation.

Because the mapper is the source of truth (there is no estimator running in a separate thread), all the geometric data is held directly on `nrf_mapper`. There is no `vis_map` cache, no `out_vis_queue`, no consumer thread. The simplification is enabled by the offline nature of the executable. Note however that `computeEdgeVis` re-traverses the entire `lmdb` every time it is called; for the live local mapper this O(N) cost would be unacceptable at 1–5 Hz, motivating the delta-update design discussed in §3.5 (O2) and §4.

**Action panel.** The side panel exposes seven buttons (`mapper.cpp:132`–`140`) that drive the stages of the offline mapping pipeline:

| Button | Wired function | Effect |
|---|---|---|
| `detect` | `detect()` (`mapper.cpp:653`) | Runs `nrf_mapper->detect_keypoints()` |
| `match` | `match()` (`mapper.cpp:659`) | Runs stereo and full-database matching |
| `tracks` | `tracks()` (`mapper.cpp:665`) | Builds multi-view tracks, sets up optimisation |
| `optimize` | `optimize()` (`mapper.cpp:633`) | Runs `num_opt_iter` LM iterations |
| `filter` | `filter()` (`mapper.cpp:675`) | Drops outlier observations |
| `aling_se3` | `alignButton()` (`mapper.cpp:640`) | Computes ATE against ground truth |
| `save_traj` | `saveTrajectoryButton()` (`mapper.cpp:680`) | Dumps poses in TUM/EuRoC format |

The sequencing above is up to the user though some steps require others to have been executes as a prerequisite. The headless path (`mapper.cpp:336`–`362`) hard-codes `detect → match → tracks → optimize → filter → optimize` as the canonical batch flow. The mapper runs entirely on the main thread. The render loop is therefore an idle loop punctuated by 50 ms sleeps (`mapper.cpp:334`) and the heavy work happens inside the button callbacks, which block the main thread until the requested stage finishes. This is acceptable for an offline tool but unworkable for a live mapper. The combined visualisation cannot follow this template.

#### 3.3.3 `src/vio_sim.cpp`: Simulation

`basalt_vio_sim` synthesises sensor data from a continuous-time `Se3Spline<5>` ground-truth trajectory (`vio_sim.cpp:98`), feeds it through the same `VioEstimatorBase` pipeline as `basalt_vio`, and overlays ground-truth, noisy, and estimated quantities in the GUI. It is the most pedagogically transparent of the three visualisations because every value rendered has a known ground-truth counterpart.

The simulation visualisation pipeline includes three displays plus the side panel, mirroring `basalt_vio` but without the third (right) image area. The image below shows the layout: stereo image views with green/yellow/blue dots for ground-truth, noisy, and VIO-estimated 2D observations, the 3D scene with the synthetic landmark sphere and the ground-truth + estimated trajectory, and the bottom plotter with ground-truth (dashed) and estimated (solid) position curves overlaid.

![basalt_vio_sim layout](img/SIM_VIO.png)

`draw_image_overlay` (`vio_sim.cpp:437`–`527`) draws three sets of 2D points: ground-truth (`gt_observations`), noisy (`noisy_observations`), and VIO-projected (`vis_map[t_ns]->projections[cam_id]`). Each set is colour-coded so the user can see, simultaneously, the noise injected at the input and the residual error at the output. The base "image" is a constant grey 8-bit image (`vio_sim.cpp:633`–`640`) — there is no real image, only the geometry overlays.

`draw_scene` (`vio_sim.cpp:529`–`564`) draws the simulated landmark cloud (a uniform sphere of `NUM_POINTS` points generated at `vio_sim.cpp:739`), the ground-truth trajectory in green, the estimated trajectory in red, and the VIO frusta from `vis_map`. The current ground-truth pose is rendered as an axis triad so the user can compare it to the estimated camera frustum.

The quantity plotter at the bottom of the window has an identical pattern to that in `basalt_vio` but with two `DataLog`s — `imu_data_log` (logged inside `gen_data` at simulation time, with all ground-truth state) and `vio_data_log` (logged by the consumer thread). The plotter (`draw_plots` at `vio_sim.cpp:790`–`906`) overlays `DrawingModeDashed` ground-truth series with `DrawingModeLine` estimated series. The eye reads the difference as the estimation error.

The vio simulation pipeline consitutes the following four consumer threads (`vio_sim.cpp:213`–`308`):
1. `t0` feeds noisy IMU
2. `t1` feeds noisy 2D observations directly into `vio->vision_data_queue` (bypassing the optical-flow frontend, since the spline already provides per-frame correspondences)
3. `t2` consumes `out_vis_queue`
4. `t3` consumes `out_state_queue`
The structure is identical to `src/vio.cpp`'s `t1`–`t4` modulo the source of the input.

### 3.4 Common Code and Divergences

The three executables exhibit a high degree of structural similarity along most axes and divergence along a few specific ones. The table summarises:

| Aspect | `basalt_vio` | `basalt_mapper` | `basalt_vio_sim` |
|---|---|---|---|
| Window title | `"Main"` | `"Main"` | `"Main"` |
| Window size | `1800 × 1000` | `1800 × 1000` | `1800 × 1000` |
| `UI_WIDTH` | 200 px | 200 px | 200 px |
| Image-view multiplicity | One per `calib.intrinsics` | Two (frame1, frame2) | One per `calib.intrinsics` |
| Plotter present | Yes | No | Yes |
| 3D view present | Yes | Yes | Yes |
| Consumer threads | 4 (`t1`..`t4` + optional `t5`) | 0 (synchronous) | 4 (`t0`..`t3`) |
| Output queues used | `out_vis_queue`, `out_state_queue`, `out_marg_queue` | None (direct access to `nrf_mapper`) | Same as VIO |
| `OpenGlRenderState` follow | Yes (`follow` toggle) | No | No |
| Step-by-step driver | Yes (`step_by_step`, `continue_btn`, `continue_fast`) | N/A (button-driven) | Yes (`continue_btn`, `next_step`) |
| Source of geometry | `vis_map` (live) | `nrf_mapper` (static) | `vis_map` (live) + `gt_*` (static) |
| Save-trajectory button | Yes | Yes | No |

The high overlap of layout, window dimensions, panel idioms, and `OpenGlRenderState` setup makes it natural to factor out a base class. The differences that must be preserved are: number of `ImageView`s, presence of a plotter, and whether the geometry source is a live cache or a static structure.

The colour conventions in `vis_utils.h:43`–`46` are shared verbatim across all three (`cam_color`, `state_color`, `pose_color`, `gt_color`) — confirming that the rendering primitives are already factored into the common header. What is missing is the *layout and lifecycle* abstraction.

### 3.5 Analysis

Several observations follow from the preceding analysis of the offline visualisation pipeline implementation.

**(O1) The file-scope global pattern is convenient but defeats reuse.** Every `Var<T>`, `OpenGlRenderState`, `DataLog`, and `vis_map` is at file scope. This makes the executables short and easy to read but means none of the layout code can be reused without copy-paste. A second `basalt_vio` window in the same process is impossible from the same thread because the `Var<bool> show_obs("ui.show_obs", ...)` static initialiser would either collide or be silently shared, depending on Pangolin's `VarState` implementation. Multi-threaded implementation have been attempted and can be referred to if required. The proposed combined visualiser in §4 must, therefore, be a class with member variables. Every `Var<T>` becomes a member, every callback becomes a method, and every `OpenGlRenderState` becomes owned state.

**(O2) The mapper renders a snapshot, not a stream.** `basalt_mapper` is built around the assumption that the entire map is loaded once, optimised, and rendered as a still image. There is no mechanism for incremental updates: `computeEdgeVis()` (`mapper.cpp:590`) rebuilds the entire edge list every time it is called, and the renderer reads it in full each frame. A live local mapper, in contrast, will emit a small delta per cycle (new keyframe, new factor, dropped landmark) at ~1–5 Hz. The combined visualiser must accept this delta stream and avoid the O(N) re-build. Currently, however, the requirements are limited to visualisation of the local map map points and VIO trajectory. Thus, the combined visualer class must only accept the latest set of map points from the the local mapper and render the same in the current VIO visualisation implementation.

**(O3) The VIO and simulator share their visualisation code by copy-paste.** The patterns of §3.3.1 and §3.3.3 differ only in the source of the input (real dataset vs. spline) and in whether ground-truth-noisy overlays are drawn. The `draw_image_overlay`, `draw_scene`, `draw_plots`, `next_step`, `alignButton`, and `saveTrajectoryButton` functions are near-identical. A single `VioVisualizer` class with a hook for "extra image overlays" would eliminate this duplication.

**(O4) The threading model is correct but ad-hoc.** The producer-consumer queue pattern is sound, but each executable manages its consumer threads by hand and relies on careful nullptr propagation. A combined visualiser should encapsulate the consumer-thread lifecycle — a single `start()` / `stop()` interface that internally manages whichever queues it is consuming similar to the lifecycle management interfaces implemented for `class LocalMapper` (`LocalMapper::Stop`) and `class SqrtKeypointVioEstimator` (`SqrtKeypointVioEstimator::drain_input_queues`).

**(O5) The frame slider is the only common GUI affordance for time travel.** Every offline executable lets the user move backwards in time via `show_frame`. This affordance is essential for debugging when the estimator diverges. The operator may wants to scrub to the moment of divergence and inspect the inputs. The combined visualiser should keep this affordance and extend it to the local map: scrubbing back should restore the local map snapshot at that timestamp, not merely the VIO trajectory. This would require extensive map management and the implementation of this step has, thus been deferred to later stages.

### 3.6 Conclusion

The three existing visualisations are best understood as three points on the same design surface. They share the layout grammar (left side panel, image grid, 3D view, optional plotter), the colour palette, the producer–consumer queue pattern, the `nullptr`-sentinel shutdown cascade, and the immediate-mode rendering style. However, they differ in the source of geometry (live VIO vs. static `nrf_mapper` vs. spline simulator), the multiplicity of `ImageView`s (one-per-camera vs. two-frame inspection), and whether a plotter is present. The differences are real and intentional, but they are also small enough that a class refactor (the topic of §4) can collapse the layout, lifecycle, and panel scaffolding into a single reusable base while retaining the freedom to vary the geometry-source and image-view policies per instance. The patterns enumerated in §3.2 (P1–P6) and the observations of §3.5 (O1–O5) together form the design requirements that the unified visualiser must respect and the limitations it must overcome.

These observations motivate the class design proposed in §4.

---

## 4. Basalt SLAM Visualisation System

`basalt::Controller` is has been extended with a real-time local mapper as described in `doc/LocalMapper.md`. The system will run three asynchronous estimation pipelines, optical flow, VIO, local mapping which constitute the SLAM stack of the planned integration test executable. To validate this stack end-to-end and to support continuing development, a unified visualisation that renders both the VIO trajectory at full estimator rate and the local-map snapshot at its (lower) update rate is required. This section performs a deep dive into the design and implementation of the required visualisation pipeline.

The design described in this section is implemented in the codebase. The status note below records the realised state. The remaining subsections retain the design discussion that produced the implementation, so the prospective phrasing in places documents the rationale rather than pending work.

#### 4.0 Implementation Status

The unified visualisation pipeline is implemented across the following files:
- `include/basalt/visualisation/utils.h` declares the Pangolin-free payloads `struct LocalMapperVisualizationData` and `struct GtPose` and the distinct colour constants `local_map_point_color` and `local_map_kf_color`.
- `src/visualisation/utils.cpp` is the Pangolin-free compilation anchor for the utilities and is compiled only into the `basalt_slam` target.
- `include/basalt/visualisation/visualiser.h` and `src/visualisation/visualiser.cpp` declare and define `class SlamVisualiser`.
- `class LocalMapper` gained the null-gated hook `out_vis_queue` and publishes a snapshot from `LocalMapper::MapLocally`.
- `class Controller` gained the `enableVisualisation` argument on `initialize`, the optional `gtcw` argument on `TrackMonocular`, and the methods `IsVisualisationEnabled`, `SetGroundTruthVisualisationQueue`, and `GetOpticalFlow`.
- `src/basalt_slam.cpp` gained the `--show-gui` flag, the ground-truth forwarding in `feed_data`, and the GUI lifecycle in `main`.
- `CMakeLists.txt` adds the two visualisation sources to the `basalt_slam` target and links Pangolin there only.

Two refinements were made relative to the draft snippets in this section. The shipped `class SlamVisualiser` holds an additional member `mpOpticalFlow` so the optical-flow overlay can read `patch_coord` through `Controller::GetOpticalFlow`. The shipped `SlamVisualiser::DrawScene` copies each cache under its own mutex rather than under one shared lock, which removes a data race on the `mpLatestVio` and `mpLatestLocalMap` shared pointers.

### 4.1 Goals and Constraints

The unified visualiser must satisfy the following four classes of constraints in addition to building on top of the current offline implmentation as discussed in the previous section:

**(G1) Functional goals.**
- Render the live VIO trajectory continuously, in the same form as `basalt_vio`. Stereo image grid with tracked points, recent landmarks and keyframes from VIO pipeline and plotter for state evolution are to be visualised.
- Render the live local map. Mapped keyframes and triangulated landmarks  with a marker distinct from the VIO sliding-window landmarks need to be plotted.
- Unify VIO and mapping visualisation
- Provide buttons to toggle estimated quantities
- Implement data flow between the visualisation pipeline and the VIO and Mapping Pipeline implemented in `class Controller`, through queues in `class SlamVisualiser` and providing appropriate references to other members of `class Controller`
- Manage Data pushed to the queues within `class SlamVisualiser` for visualisation
- Update `Controller::TrackMonocular` to accept optional pose information `gtcw`, only to be passed onto the visualiser if the member is provided (Use `std::optional` here)

**(G2) Non-functional goals.**
- Add zero overhead to the estimator threads when the GUI is disabled.
- Combine the VIO and Mapper visualisation for real-time usage in one single window
- Implement a visualisation data structure for the mapper similar to the existing VIO visualisation API (For example, `basalt::VioVisualizationData`. We need to implement similar methods for the local mapping thread visualisation)
- Implement a layer of abstraction implementing the visualisation pipeline for the combined VIO and local mapping integration test executable named `class SlamVisualiser`
- Ensure the current offline implementation is reused to implment the unified visualisation interface (Let the current implementation as is for now. Implement all methods, classes and interfaces at `src/visualisation/visualiser.cpp`, `src/visualisation/utils.cpp`, `include/basalt/visualisation/visualiser.h` and `include/basalt/visualisation/utils.h`)

**(G3) Concurrency constraints.**
- OpenGL context affinity must be respected. Every `glDraw*` and Pangolin layout call must execute on the main (GL-bound) thread (cf. §2.3 (D1)).
- The VIO emits at the estimator's keyframe rate (~10–20 Hz); the local mapper at the marginalisation rate (~1–5 Hz). The visualiser must accept/read both without back-pressuring either.
- The local mapper publishes refined poses back into the VIO state via a callback `mpVioPoseUpdateCallback` (`doc/LocalMapper.md §3`). Similarly, the visualiser must not reach into the local mapper's internal state synchronously as the local mapper holds locks on `lmdb` and `frame_poses` during optimisation. This would lead to delayed local map updates and freezing of the GUI. The visualiser must instead consume a copy of the latest map points published by the local mapper.

**(G4) Compatibility constraints.**
- Gate the visualisation on a single CLI flag identical to the existing `--show-gui` switches.
- Continue to support headless mode for CI: when `--show-gui false`, no GL context is created, no consumer threads are started, no queue pointers are connected.

### 4.2 API Design

This section gives a high-level overview of the unified visualisation API. The API boundary, compilation targerts, and the lifecycle of `class SlamVisualiser` and the core types it consumes. The detailed data plumbing is deferred to §4.3 and the rendering internals to §4.4.

#### 4.2.1 Class Boundary and Build Target

The single most important architectural decision is the placement of the Pangolin-dependent code. The core `basalt` shared library does not link Pangolin. Only the executables explicitly link to Pangolin for visualisation (`CMakeLists.txt:393`–`446`). We do not want to add Pangolin to the core basalt library. To preserve headless CI (G4) and add zero overhead to the estimator threads when the GUI is off (G2), the visualisation pipeline is compiled only into the `basalt_slam` executable.

The four files requested by §4.1 split along the Pangolin boundary:

| File | Compiled into | Pangolin | Contents |
|---|---|---|---|
| `include/basalt/visualisation/utils.h` | both (header) | no | `LocalMapperVisualizationData`, `GtPose`, distinct colour constants, Pangolin-free helpers. |
| `src/visualisation/utils.cpp` | `basalt_slam` only | no | out-of-line definitions of the Pangolin-free helpers declared in `utils.h`. |
| `include/basalt/visualisation/visualiser.h` | `basalt_slam` only | yes | `class SlamVisualiser`. |
| `src/visualisation/visualiser.cpp` | `basalt_slam` only | yes | layout, the `glDraw*` callbacks, the consumer threads. |

`include/basalt/visualisation/utils.h` and `src/visualisation/utils.cpp` are deliberately kept Pangolin-free. They include only Eigen, Sophus, and the basalt aligned-container aliases only so that `LocalMapper`, which lives in `libbasalt.so`, can include it to initialise its publish queue. A header include adds no link dependency, and because `utils.cpp` is never added to the `libbasalt.so` target, no Pangolin symbol ever enters the core library. This is the mechanism that satisfies the constraint that the SLAM visualisation pipeline is only ever compiled with the final executable.


#### 4.2.2 Local Mapping and Ground Truth Visualisation Data Structures

The mapper publishes a snapshot(`LocalMapperVisualizationData`) type that mirrors `VioVisualizationData` (G2). It is a flat, immutable, self-contained value. It only holds copies of visualised local map and never pointers back into the mapper's live state (G3). `GtPose` is used to store, manage and share the ground truth pose received at `Controller::TrackMonocular(frame, tcw, gtcw)` invocation when a frame is received with ground truth data. Declared in `include/basalt/visualisation/utils.h`:
```cpp
#pragma once

#include <Eigen/Dense>
#include <sophus/se3.hpp>
#include <basalt/utils/sophus_utils.hpp>   // Eigen::aligned_vector
#include <memory>
#include <vector>

namespace basalt {

// Immutable per-cycle snapshot of the local map, published by LocalMapper for
// the GUI. Mirrors VioVisualizationData; carries only value copies so the
// consumer never touches the mapper's live lmdb/frame_poses (cf. §4.1 G3).
struct LocalMapperVisualizationData {
    typedef std::shared_ptr<LocalMapperVisualizationData> Ptr;

    int64_t t_ns;

    // World-frame keyframe poses, copied from LocalMapper::frame_poses.
    Eigen::aligned_vector<Sophus::SE3d> keyframes;

    // World-frame triangulated landmarks, materialised by get_current_points().
    Eigen::aligned_vector<Eigen::Vector3d> points;
    std::vector<int> point_ids;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

// Ground-truth pose sample forwarded by Controller::TrackMonocular when the
// caller supplies it. Pangolin-free so Controller (libbasalt.so) can use it.
struct GtPose {
    int64_t t_ns;
    Sophus::SE3d T_w_i;
};

// Distinct rendering colours for the local map, kept separate from the VIO
// palette in vis_utils.h so map points read as a different layer (G1).
const uint8_t local_map_point_color[3]{255, 128, 0};  // orange
const uint8_t local_map_kf_color[3]{255, 200, 0};     // amber

}  // namespace basalt
```
The colour constants are `const` at namespace scope, which gives them internal linkage, so including the header in several translation units raises no one definition rule violation issues. They are intentionally distinct from `pose_color = {0,50,255}` (the VIO landmark blue) so the operator can tell the sliding-window landmarks apart from the local map at a glance.

#### 4.2.3 Core Updates for Integration

Three small, additive changes to existing core types (controller and local mapper currently) feed the visualiser. Each is null-gated so it costs nothing when the GUI is off.
`class LocalMapper` (`include/basalt/vi_estimator/local_mapper.h`) mirrors the VIO publish hook exactly:
```cpp
#include "basalt/visualisation/utils.h"   // LocalMapperVisualizationData (Pangolin-free)

// public, alongside the other queue wiring:
//   A null pointer means the mapper assembles no snapshot — zero overhead,
//   identical contract to VioEstimatorBase::out_vis_queue.
tbb::concurrent_bounded_queue<LocalMapperVisualizationData::Ptr>* out_vis_queue =
    nullptr;
```

`class Controller` (`include/basalt/controller.h`) implements a visualisation flag, the optional ground-truth argument to determine if visualisation is enabled, the ingestion pathway for optional ground truth data and a ground truth queue setter method:
```cpp
#include <optional>
#include "basalt/visualisation/utils.h"   // basalt::GtPose

// Initialise mid-flight with specific state. New trailing flag arms the
// visualisation taps; it is the single source of truth for "GUI required".
void initialize(int64_t t_ns, const Sophus::SE3d& T_w_i,
                const Eigen::Vector3d& vel_w_i, const Eigen::Vector3d& bg,
                const Eigen::Vector3d& ba,
                bool useProducerConsumerArchitecture = false,
                bool enableVisualisation = false);

// gtcw is forwarded to the GUI only when visualisation is enabled and a
// ground-truth queue has been registered; existing callers are unaffected.
void TrackMonocular(OpticalFlowInput::Ptr& frame, Sophus::SE3f& tcw,
                    std::optional<Sophus::SE3d> gtcw = std::nullopt);

bool IsVisualisationEnabled() const;
void SetGroundTruthVisualisationQueue(
    tbb::concurrent_bounded_queue<basalt::GtPose>* queue);

// private members:
bool mpEnableVisualisation = false;                              // mirrors mpUseProducerConsumerArchitecture style
tbb::concurrent_bounded_queue<basalt::GtPose>* mvpGroundTruthQueue = nullptr;
```
The VIO half needs no struct or method change as `VioEstimatorBase::out_vis_queue` (`vio_estimator.h:86`) and `out_state_queue` (`:84`) already exist and are already null-gated by the estimator.

#### 4.2.4 `class SlamVisualiser` API

`class SlamVisualiser` encapsulates everything that `src/vio.cpp` keeps at file scope (O1). The Pangolin `Var` toggles, the `OpenGlRenderState`, the `DataLog`/`Plotter`, the per-frame caches, the consumer threads, and the draw callbacks are all declared as members of `class SlamVisualiser`. The class owns its queues (G1) and connects them to the estimators through references obtained from the `Controller` (G1: "references to other members of `class Controller`"). The public API intentionally tiny, consisting only  of `SlamVisualiser::Start`, `SlamVisualiser::Run`, `SlamVisualiser::Stop`. Refer to the following code snippet for the class declaration:
```cpp
// include/basalt/visualisation/visualiser.h  (basalt_slam target only)
#pragma once

#include <pangolin/pangolin.h>
#include <basalt/controller.h>
#include <basalt/vi_estimator/vio_estimator.h>     // VioVisualizationData
#include <basalt/visualisation/utils.h>            // LocalMapperVisualizationData, GtPose
#include <mutex>
#include <thread>
#include <unordered_map>

namespace basalt {

class SlamVisualiser {
public:
    // Captures references to the Controller's VIO, local mapper and calibration.
    // No GL work happens in the constructor (it may run off the main thread).
    explicit SlamVisualiser(basalt::Controller& controller);
    ~SlamVisualiser();

    // Create the GL window + layout, wire the queues to the estimators, and
    // start the consumer threads. MUST be called on the main (GL) thread.
    void Start();

    // Blocking main-thread render loop; returns when the window is closed.
    void Run();

    // Sentinel-nullptr shutdown of the consumer threads; idempotent.
    void Stop();

private:
    // ── draw callbacks (bound into Pangolin extern_draw_function) ──────
    void DrawScene(pangolin::View& view);
    void DrawImageOverlay(pangolin::View& view, size_t cam_id);
    void DrawPlots();
    void SetupLayout();

    // ── consumer-thread bodies (no GL calls) ──────────────────────────
    void ConsumeVioVisQueue();      // → mpLatestVio
    void ConsumeVioStateQueue();    // → mpVioDataLog + mvpVioTrajectory
    void ConsumeLocalMapQueue();    // → mpLatestLocalMap

    // ── references into the Controller ────────────────────────────────
    basalt::Controller& mpController;
    basalt::Calibration<double>& mpCalib;
    basalt::VioEstimatorBase<double>::Ptr mpVio;
    std::shared_ptr<basalt::LocalMapper> mpLocalMapper;
    // Held only to read patch_coord for the optical-flow overlay (G1).
    basalt::OpticalFlowBase::Ptr mpOpticalFlow;

    // ── queues owned here, connected to the estimators in Start() ─────
    tbb::concurrent_bounded_queue<basalt::VioVisualizationData::Ptr>
        mvpVioVisQueue;
    tbb::concurrent_bounded_queue<basalt::PoseVelBiasState<double>::Ptr>
        mvpVioStateQueue;
    tbb::concurrent_bounded_queue<basalt::LocalMapperVisualizationData::Ptr>
        mvpLocalMapVisQueue;
    tbb::concurrent_bounded_queue<basalt::GtPose> mvpGroundTruthQueue;

    // ── latest-only caches (live edge; no history, no image cache) ────
    basalt::VioVisualizationData::Ptr mpLatestVio;
    basalt::LocalMapperVisualizationData::Ptr mpLatestLocalMap;
    std::mutex mpMtxVioVis;  // guards mpLatestVio
    std::mutex mpMtxVioState; // guards mvpVioTrajectory
    std::mutex mpMtxLocalMap;  // guards mpLatestLocalMap

    // ── trajectories: positions only, cheap to retain ────────────────
    Eigen::aligned_vector<Eigen::Vector3d> mvpVioTrajectory;  // consumer-written
    Eigen::aligned_vector<Eigen::Vector3d> mvpGroundTruthTrajectory;  // main-thread only

    // ── Pangolin objects ──────────────────────────────────────────────
    pangolin::OpenGlRenderState mpCamera;
    pangolin::DataLog mpVioDataLog;
    pangolin::Plotter* mpPlotter = nullptr;
    std::vector<std::shared_ptr<pangolin::ImageView>> mpImgViews;
    int64_t mpStartTns = -1;        // x-axis origin for the plotter
    int64_t mpLastImageTns = -1;    // dedupes image-grid uploads

    // ── toggles (registered into the "ui." panel) ─────────────────────
    std::unique_ptr<pangolin::Var<bool>> mpShowObs, mpShowFlow, mpShowIds,
        mpShowGt, mpShowEstPos, mpShowEstVel, mpShowEstBg, mpShowEstBa,
        mpFollow, mpShowLocalMapPoints, mpShowLocalMapKfs;

    // ── consumer threads + lifecycle flags ────────────────────────────
    std::thread mpVioVisConsumerThread, mpVioStateConsumerThread,
        mpLocalMapConsumerThread;
    std::atomic<bool> mpRunning{false};
};

}  // namespace basalt
```
The `Var<bool>` toggles are held by `std::unique_ptr` rather than declared inline because, unlike `src/vio.cpp` where they are file-scope statics auto-registered before `main`, here they must be constructed after `pangolin::CreateWindowAndBind` has created the context and the `ui` panel. Wrapping them in pointers lets the constructor leave them empty and `SetupLayout()` create them at the right time.

The three-phase lifecycle of `class SlamVisualiser` is realised through the following methods:
- `SlamVisualiser::Start()` (main thread): create the window, build the layout and panel, set `mpVio->out_vis_queue`, `mpVio->out_state_queue`, `mpLocalMapper->out_vis_queue`, and register `mvpGroundTruthQueue` with the controller; then launch the three consumer threads.
- `SlamVisualiser::Run()` (main thread): the `while (!pangolin::ShouldQuit())` loop that drains the ground-truth queue, uploads the current images, runs the follow-camera update, and calls `FinishFrame()`.
- `SlamVisualiser::Stop()`: push a `nullptr` sentinel into each consumer queue and join the threads; safe to call more than once via `mpRunning`.

### 4.3 Data Flow Design

This section specifies the queues, who owns them, how they are wired, and how data moves from the three producer contexts (the VIO estimator, the local-mapper thread, and the synchronous `TrackMonocular` call) to the single GL thread without back-pressuring any producer (G3).

#### 4.3.1 Queue Inventory and Ownership

All four queues are members of `SlamVisualiser` (G1). They are connected to the producers in `Start()` and disconnected by the cascade shutdown.

| Queue (member) | Element | Producer | Consumer Thread Body| Cache populated |
|---|---|---|---|---|
| `mvpVioVisQueue` | `VioVisualizationData::Ptr` | VIO `measure()` via `out_vis_queue` | `ConsumeVioVisQueue` | `mpLatestVio` |
| `mvpVioStateQueue` | `PoseVelBiasState<double>::Ptr` | VIO `measure()` via `out_state_queue` | `ConsumeVioStateQueue` | `mpVioDataLog`, `mvpVioTrajectory` |
| `mvpLocalMapVisQueue` | `LocalMapperVisualizationData::Ptr` | `LocalMapper::MapLocally` via `out_vis_queue` | `ConsumeLocalMapQueue` | `mpLatestLocalMap` |
| `mvpGroundTruthQueue` | `GtPose` | `Controller::TrackMonocular` | main thread (`Run`) | `mvpGroundTruthTrajectory` |

The VIO half reuses the two output queues the estimator already exposes. Tapping `out_state_queue` is what supplies the plotter with velocity and bias time-series (these are absent from `VioVisualizationData`, which carries only `SE3d` poses). This tap is valid because `basalt_slam` runs in the synchronous configuration (`useProducerConsumerArchitecture == false`), where the `Controller` does not wire `out_state_queue` and does not start its own pose thread (`controller.cpp:159`–`171`). Thus, the pointer is free for the visualiser to claim.

#### 4.3.2 Wiring in `Start()`

```cpp
void SlamVisualiser::Start() {
    if (!mpController.IsVisualisationEnabled()) return;  // honour the single gate (G4)

    // Bounded capacities; producers use try_push (below) so a full queue drops
    // the newest frame instead of blocking the estimator (G3).
    mvpVioVisQueue.set_capacity(20);
    mvpVioStateQueue.set_capacity(100);
    mvpLocalMapVisQueue.set_capacity(4);
    mvpGroundTruthQueue.set_capacity(200);

    pangolin::CreateWindowAndBind("Basalt SLAM", 1800, 1000);
    glEnable(GL_DEPTH_TEST);   // per-context state; must follow window creation
    SetupLayout();

    // Connect the visualiser's queues to the estimator output hooks. These are
    // raw-pointer taps, identical to how src/vio.cpp connects out_vis_queue.
    mpVio->out_vis_queue = &mvpVioVisQueue;
    mpVio->out_state_queue = &mvpVioStateQueue;
    mpLocalMapper->out_vis_queue = &mvpLocalMapVisQueue;
    mpController.SetGroundTruthVisualisationQueue(&mvpGroundTruthQueue);

    mpRunning = true;
    mpVioVisConsumerThread = std::thread(&SlamVisualiser::ConsumeVioVisQueue, this);
    mpVioStateConsumerThread = std::thread(&SlamVisualiser::ConsumeVioStateQueue, this);
    mpLocalMapConsumerThread = std::thread(&SlamVisualiser::ConsumeLocalMapQueue, this);
}
```
Connecting the queues after `mpVio->initialize()` has run is safe. The estimator re-reads the `out_*` pointers each frame, so as long as the wiring happens before the dataset feed starts (which the integration test guarantees, §4.5), no frame is missed.

#### 4.3.3 Producer Side: the Local-Map Snapshot

The mapper publishes from inside `MapLocally()`, immediately after the final `optimize()` and beside the existing pose callback. This is the only point in the cycle where `frame_poses` and `lmdb` are quiescent on the mapping thread, so the snapshot is internally consistent and race-free (G3). The new lines (additions marked) extend `src/vi_estimator/local_mapper.cpp`:
```cpp
        optimize(mpOptIterations);
        filterOutliers(mpFilterOutlierThreshold, 4);
        optimize(mpOptIterations);

        // ── publish an immutable local-map snapshot for the GUI ──────────
        if (out_vis_queue) {
            LocalMapperVisualizationData::Ptr vis_data =
                std::make_shared<LocalMapperVisualizationData>();
            vis_data->t_ns = frame_poses.empty() ? 0 : frame_poses.rbegin()->first;
            get_current_points(vis_data->points, vis_data->point_ids);
            vis_data->keyframes.reserve(frame_poses.size());
            for (const auto& kv : frame_poses)
                vis_data->keyframes.emplace_back(kv.second.getPose());
            out_vis_queue->try_push(std::move(vis_data));  // never block the mapper
        }

        if (mpVioPoseUpdateCallback) mpVioPoseUpdateCallback(frame_poses);
```
`get_current_points` is the inherited `class BundleAdjustmentBase` method (`ba_base.cpp:228`) that materialises world-frame landmark positions from the inverse-depth keypoints. It is the same method `src/mapper.cpp:635` uses to build `mapper_points`. `frame_poses` is an ordered `Eigen::aligned_map`, so `rbegin()->first` is the most recent keyframe timestamp. `try_push` is the back-pressure guard required by G3. At ≤5 Hz with a capacity-4 queue and a fast consumer the queue never actually fills, but `try_push` guarantees the mapper thread is never blocked even if the GL thread stalls.

#### 4.3.4 Producer Side: the Ground-Truth Ingestion 

`Controller::TrackMonocular` contains the optional argument to ingest ground truth pose per frame and forwards it only when both the flag and the queue are present:
```cpp
void Controller::TrackMonocular(OpticalFlowInput::Ptr& frame, Sophus::SE3f& tcw,
                                std::optional<Sophus::SE3d> gtcw) {
    OpticalFlowResult::Ptr res = opt_flow_ptr_->processFrame(frame->t_ns, frame);
    current_latest_pose_ = vio_estimator_->ProcessFrame(res);
    tcw = current_latest_pose_->T_w_i.cast<float>();

    // Forward the ground-truth pose to the GUI only when asked to (G1/G4).
    if (mpEnableVisualisation && mvpGroundTruthQueue && gtcw) {
        mvpGroundTruthQueue->try_push(basalt::GtPose{frame->t_ns, *gtcw});
    }
}
```

`Controller::SetGroundTruthVisualisationQueue` simply stores the pointer, and `Controller::initialize` records the flag:
```cpp
void Controller::SetGroundTruthVisualisationQueue(
    tbb::concurrent_bounded_queue<basalt::GtPose>* queue) {
    mvpGroundTruthQueue = queue;
}
bool Controller::IsVisualisationEnabled() const { return mpEnableVisualisation; }
// inside initialize(...): mpEnableVisualisation = enableVisualisation;
```

Routing ground truth through the `class Controller` (rather than letting the feeder push straight to the visualiser) keeps the integration test's feeder agnostic of the GUI: it always calls `Controller::TrackMonocular(frame, tcw, gtcw)`, and whether the pose is rendered is decided entirely by the single `Controller::enableVisualisation` flag.

#### 4.3.5 Consumer Side

The two VIO consumers mirror `src/vio.cpp`'s `t3` (vis) and `t4` (state) verbatim, differing only in that they write members instead of file globals and keep just the latest 3D payload (no `vis_map` history, no image cache):
```cpp
void SlamVisualiser::ConsumeVioVisQueue() {
    basalt::VioVisualizationData::Ptr data;
    while (true) {
        mvpVioVisQueue.pop(data);            // blocking
        if (!data) break;                    // nullptr sentinel from Stop()
        std::lock_guard<std::mutex> lock(mpMtxVioVis);
        mpLatestVio = data;                  // live edge only
    }
}

void SlamVisualiser::ConsumeVioStateQueue() {
    basalt::PoseVelBiasState<double>::Ptr data;
    while (true) {
        mvpVioStateQueue.pop(data);
        if (!data) break;
        if (mpStartTns < 0) mpStartTns = data->t_ns;

        {
            std::lock_guard<std::mutex> lock(mpMtxVioState);
            mvpVioTrajectory.emplace_back(data->T_w_i.translation());
        }

        // Column layout matches src/vio.cpp draw_plots(): $0 time, $1-$3 vel,
        // $4-$6 pos, $7-$9 gyro bias, $10-$12 accel bias. DataLog is internally
        // mutex-protected, so logging here while the plotter reads is safe.
        std::vector<float> vals;
        vals.push_back((data->t_ns - mpStartTns) * 1e-9);
        for (int i = 0; i < 3; i++) vals.push_back(data->vel_w_i[i]);
        for (int i = 0; i < 3; i++) vals.push_back(data->T_w_i.translation()[i]);
        for (int i = 0; i < 3; i++) vals.push_back(data->bias_gyro[i]);
        for (int i = 0; i < 3; i++) vals.push_back(data->bias_accel[i]);
        mpVioDataLog.Log(vals);
    }
}

void SlamVisualiser::ConsumeLocalMapQueue() {
    basalt::LocalMapperVisualizationData::Ptr data;
    while (true) {
        mvpLocalMapVisQueue.pop(data);
        if (!data) break;
        std::lock_guard<std::mutex> lock(mpMtxLocalMap);
        mpLatestLocalMap = data;             // keep only the newest map
    }
}
```
The ground-truth queue is drained on the main thread at the top of the render loop (§4.4), so `mvpGroundTruthTrajectory` is touched by exactly one thread and needs no lock. A mutex for each thread is used. `SlamVisualiser::mpMtxLocalMap`, `SlamVisualiser::mpMtxVioState` and `mpMtxVioVis` are the three locks covering the three caches that the GL thread reads in `DrawScene`. At latencies between 1–20 Hz the contention is negligible and the locks are held only for a `shared_ptr` copy or a single `emplace_back`.

#### 4.3.6 Shutdown Integration

`SlamVisualiser` plugs into the existing cascade sentinel pattern (P3) without disturbing it. The estimator-side `nullptr` sentinels are owned by the VIO and the mapper, exactly as today (`controller.cpp:55`–`87`. The visualiser only needs to release its own consumer threads:
```cpp
void SlamVisualiser::Stop() {
    if (!mpRunning.exchange(false)) return;   // idempotent
    mvpVioVisQueue.push(nullptr);
    mvpVioStateQueue.push(nullptr);
    mvpLocalMapVisQueue.push(nullptr);
    if (mpVioVisConsumerThread.joinable()) mpVioVisConsumerThread.join();
    if (mpVioStateConsumerThread.joinable()) mpVioStateConsumerThread.join();
    if (mpLocalMapConsumerThread.joinable()) mpLocalMapConsumerThread.join();
}
```
The recommended teardown order in `basalt_slam` (§4.5) is: window closes → `Run()` returns → set the feeder's `terminate` flag → join the feeder → `controller->Stop()` (drains VIO and mapper, which pushes their own `nullptr`s) → `visualiser.Stop()`. Because the visualiser's queues are distinct from the estimator queues, the order of `controller->Stop()` versus `visualiser.Stop()` is not delicate; the visualiser's blocking `pop`s are released by its own sentinels.

### 4.4 Visualisation

This section is the code-level specification of the rendering layer. It opens with the requirements that shape `class SlamVisualiser`, then walks the layout, the render-state, each draw callback, and the local-map additions, citing the §2 Pangolin abstractions and the §3 reference patterns being reused.

#### 4.4.1 The Unified Window Design Requirements

The design of `SlamVisualiser` and its draw callbacks is driven by the following requirements, gathered from §4.1 and the §3.5 analysis:
1. One window must show, simultaneously, the live VIO trajectory and the live local map (G1, G2). This collapses what were two separate executables (`basalt_vio`, `basalt_mapper`) into one layout.
2. The VIO view must be rendered in the same form as `basalt_vio`: a stereo/multi-cam image grid with tracked-point overlays, the estimated trajectory, recent landmarks and keyframe frusta, and a state plotter (G1).
3. The local map must be rendered with a marker distinct from the VIO sliding-window landmarks (G1) — a different colour and point size for landmarks, and keyframe frusta in a separate colour.
4. Every `glDraw*` and Pangolin layout call must run on the main GL thread (G3, D1). Consumer threads only fill caches; the draw callbacks only read them.
5. Estimated quantities must be individually toggleable via panel buttons (G1).
6. The renderer reads only the latest cached payload — no timestamp-keyed history, no cached image stream (feedback: long runs must not accumulate image memory; scrubbing is deferred per O5).
7. The window, panel width, colour palette, `render_camera` frustum helper, and `getcolor` depth ramp are reused verbatim from `src/vio.cpp` and `vis_utils.h` (O3, G2: reuse the offline implementation).

#### 4.4.2 Layout Construction

`SetupLayout()` reproduces the four-region layout of `src/vio.cpp:436`–`484` (U2, U5, U6, U7) as a member function. The only structural difference is that the `Var` toggles are constructed here (after the context exists) instead of at file scope:
```cpp
void SlamVisualiser::SetupLayout() {
    constexpr int UI_WIDTH = 200;

    pangolin::View& main_display = pangolin::CreateDisplay().SetBounds(
        0.0, 1.0, pangolin::Attach::Pix(UI_WIDTH), 1.0);

    pangolin::View& img_view_display = pangolin::CreateDisplay()
        .SetBounds(0.4, 1.0, 0.0, 0.4)
        .SetLayout(pangolin::LayoutEqual);

    pangolin::View& plot_display = pangolin::CreateDisplay().SetBounds(
        0.0, 0.4, pangolin::Attach::Pix(UI_WIDTH), 1.0);

    mpPlotter = new pangolin::Plotter(&mpVioDataLog, 0.0, 100, -10.0, 10.0,
                                      0.01f, 0.01f);
    plot_display.AddDisplay(*mpPlotter);

    pangolin::CreatePanel("ui").SetBounds(0.0, 1.0, 0.0,
                                          pangolin::Attach::Pix(UI_WIDTH));

    // Toggles. Each Var name carries the "ui." prefix so the panel auto-renders
    // it (D5). The two trailing flags drive the local-map layer (G1).
    mpShowObs = std::make_unique<pangolin::Var<bool>>("ui.show_obs", true, false, true);
    mpShowFlow = std::make_unique<pangolin::Var<bool>>("ui.show_flow", false, false, true);
    mpShowIds = std::make_unique<pangolin::Var<bool>>("ui.show_ids", false, false, true);
    mpShowGt = std::make_unique<pangolin::Var<bool>>("ui.show_gt", true, false, true);
    mpShowEstPos = std::make_unique<pangolin::Var<bool>>("ui.show_est_pos", true, false, true);
    mpShowEstVel = std::make_unique<pangolin::Var<bool>>("ui.show_est_vel", false, false, true);
    mpShowEstBg = std::make_unique<pangolin::Var<bool>>("ui.show_est_bg", false, false, true);
    mpShowEstBa = std::make_unique<pangolin::Var<bool>>("ui.show_est_ba", false, false, true);
    mpFollow = std::make_unique<pangolin::Var<bool>>("ui.follow", true, false, true);
    mpShowLocalMapPoints = std::make_unique<pangolin::Var<bool>>("ui.show_local_map_points", true, false, true);
    mpShowLocalMapKfs = std::make_unique<pangolin::Var<bool>>("ui.show_local_map_kfs", true, false, true);

    // One ImageView per camera (U5). Bound to DrawImageOverlay with the camera
    // index captured, via a member-function bind on this.
    while (mpImgViews.size() < mpCalib.intrinsics.size()) {
        auto iv = std::make_shared<pangolin::ImageView>();
        size_t idx = mpImgViews.size();
        mpImgViews.push_back(iv);
        img_view_display.AddDisplay(*iv);
        iv->extern_draw_function =
            std::bind(&SlamVisualiser::DrawImageOverlay, this,
                      std::placeholders::_1, idx);
    }

    // Initial virtual-camera placement reuses the vio.cpp recipe: a canonical
    // offset rotated into the world frame by the VIO's initial pose (U6).
    Eigen::Vector3d cam_p(-0.5, -3, -5);
    cam_p = mpVio->getT_w_i_init().so3() * mpCalib.T_i_c[0].so3() * cam_p;
    mpCamera = pangolin::OpenGlRenderState(
        pangolin::ProjectionMatrix(640, 480, 400, 400, 320, 240, 0.001, 10000),
        pangolin::ModelViewLookAt(cam_p[0], cam_p[1], cam_p[2], 0, 0, 0,
                                  pangolin::AxisZ));

    pangolin::View& display3D = pangolin::CreateDisplay()
        .SetAspect(-640 / 480.0)
        .SetBounds(0.4, 1.0, 0.4, 1.0)
        .SetHandler(new pangolin::Handler3D(mpCamera));   // Pangolin owns the handler
    display3D.extern_draw_function =
        std::bind(&SlamVisualiser::DrawScene, this, std::placeholders::_1);

    main_display.AddDisplay(img_view_display);
    main_display.AddDisplay(display3D);
}
```
In `src/vio.cpp` the callbacks are free functions assigned directly to `extern_draw_function`. Here they are member functions, so they are wrapped with `std::bind(&SlamVisualiser::DrawScene, this, _1)` to carry the instance. This is the standard adaptation when migrating Pangolin file-scope callbacks into a class (O1) and is the reason the file-scope-global pattern is replaced cleanly: every datum the callbacks need is now a member reachable through `this`.

#### 4.4.3 The Render Loop

`Run()` is the canonical `while (!ShouldQuit())` loop (A5), augmented with the ground-truth drain and the image upload. It is the only place OpenGL is used:
```cpp
void SlamVisualiser::Run() {
    while (!pangolin::ShouldQuit()) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // Drain ground truth on the GL thread → single-writer, no lock needed.
        basalt::GtPose gt;
        while (mvpGroundTruthQueue.try_pop(gt))
            mvpGroundTruthTrajectory.emplace_back(gt.T_w_i.translation());

        // Follow-camera: re-anchor to the latest body pose, rotation zeroed so
        // the world stays upright (U9).
        basalt::VioVisualizationData::Ptr latest;
        {
            std::lock_guard<std::mutex> lock(mpMtxVioVis);
            latest = mpLatestVio;
        }
        if (*mpFollow && latest) {
            Sophus::SE3d T_w_i;
            if (!latest->states.empty()) T_w_i = latest->states.back();
            else if (!latest->frames.empty()) T_w_i = latest->frames.back();
            T_w_i.so3() = Sophus::SO3d();
            mpCamera.Follow(T_w_i.matrix());
        }

        // Upload the current frame's images once per new timestamp (U8 logic,
        // but keyed on the live edge rather than a slider).
        if (latest && latest->opt_flow_res && latest->t_ns != mpLastImageTns) {
            const auto& imgs = latest->opt_flow_res->input_images->img_data;
            pangolin::GlPixFormat fmt;
            fmt.glformat = GL_LUMINANCE;
            fmt.gltype = GL_UNSIGNED_SHORT;
            fmt.scalable_internal_format = GL_LUMINANCE16;
            for (size_t cam_id = 0;
                 cam_id < mpImgViews.size() && cam_id < imgs.size(); cam_id++) {
                if (imgs[cam_id].img)
                    mpImgViews[cam_id]->SetImage(
                        imgs[cam_id].img->ptr, imgs[cam_id].img->w,
                        imgs[cam_id].img->h, imgs[cam_id].img->pitch, fmt);
            }
            mpLastImageTns = latest->t_ns;
        }

        if (mpShowEstPos->GuiChanged() || mpShowEstVel->GuiChanged() ||
            mpShowEstBg->GuiChanged() || mpShowEstBa->GuiChanged())
            DrawPlots();

        pangolin::FinishFrame();   // renders the view tree (fires the callbacks), swaps
    }
}
```
The image source is `latest->opt_flow_res->input_images->img_data`, the optical-flow result carried inside the VIO visualisation payload which holds the source images, so the visualiser is decoupled from any dataset object and no separate image cache exists. Uploading is gated on `mpLastImageTns` so the GPU texture is refreshed only when a new frame actually arrives (the same redundant-upload avoidance as U8, keyed on the live edge instead of a slider).

#### 4.4.4 `DrawScene`: VIO and Local Map in One View

`DrawScene` composes the VIO geometry exactly as `src/vio.cpp:786`–`839` (U13) and then overlays the local map. The two layers are visually separated by colour and point size (requirement 3 of §4.4.1):
```cpp
void SlamVisualiser::DrawScene(pangolin::View& view) {
    view.Activate(mpCamera);                 // glViewport + upload P,V (U13)
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    // Each cache is copied under its own mutex. The shipped code does not share
    // one lock across all three reads, which removes a data race on the
    // mpLatestVio and mpLatestLocalMap shared pointers.
    basalt::VioVisualizationData::Ptr vio;
    basalt::LocalMapperVisualizationData::Ptr lm;
    {
        std::lock_guard<std::mutex> lock(mpMtxVioVis);
        vio = mpLatestVio;
    }
    {
        std::lock_guard<std::mutex> lock(mpMtxLocalMap);
        lm = mpLatestLocalMap;
    }

    // ── 1. VIO estimated trajectory (red) ─────────────────────────────
    // The state lock is held during the draw because the consumer may
    // emplace_back and reallocate the buffer while the GL thread iterates it.
    {
        std::lock_guard<std::mutex> lock(mpMtxVioState);
        glColor3ubv(cam_color);
        if (!mvpVioTrajectory.empty()) pangolin::glDrawLineStrip(mvpVioTrajectory);
    }

    // ── 2. Ground-truth trajectory (green) ────────────────────────────
    if (*mpShowGt) {
        glColor3ubv(gt_color);
        pangolin::glDrawLineStrip(mvpGroundTruthTrajectory);
    }

    // ── 3. Latest VIO frusta + sliding-window landmarks ───────────────
    if (vio) {
        for (const auto& p : vio->states)
            for (size_t i = 0; i < mpCalib.T_i_c.size(); i++)
                render_camera((p * mpCalib.T_i_c[i]).matrix(), 2.0f, state_color, 0.1f);
        for (const auto& p : vio->frames)
            for (size_t i = 0; i < mpCalib.T_i_c.size(); i++)
                render_camera((p * mpCalib.T_i_c[i]).matrix(), 2.0f, pose_color, 0.1f);
        glPointSize(3);
        glColor3ubv(pose_color);
        pangolin::glDrawPoints(vio->points);
    }

    // ── 4. Local map: distinct marker (orange, larger) + amber KF frusta ─
    if (lm) {
        if (*mpShowLocalMapPoints) {
            glPointSize(5);                  // larger than the VIO 3px points
            glColor3ubv(local_map_point_color);
            pangolin::glDrawPoints(lm->points);
        }
        if (*mpShowLocalMapKfs)
            for (const auto& kf : lm->keyframes)
                for (size_t i = 0; i < mpCalib.T_i_c.size(); i++)
                    render_camera((kf * mpCalib.T_i_c[i]).matrix(), 2.0f,
                                  local_map_kf_color, 0.1f);
    }

    pangolin::glDrawAxis(Sophus::SE3d().matrix(), 1.0);   // world origin triad
}
```

Three rendering decisions follow directly from §2:
- Depth ordering is handled entirely by the depth buffer (§2.1.5); the VIO points, local-map points, frusta, and trajectories are issued in any order and the GPU resolves occlusion. No CPU-side sorting is required even though the camera orbits interactively.
- `render_camera` is reused verbatim from `vis_utils.h:48`. Passing `(kf * T_i_c).matrix()` to `render_camera` relies on the column-major Eigen↔OpenGL equivalence (§2.1.7) so no transpose is needed.
- The local-map landmarks use `glPointSize(5)` and `local_map_point_color` (orange) against the VIO's `glPointSize(3)` and `pose_color` (blue). Because `glPointSize` and `glColor` are sticky GL state (§2.1.6), each group sets its own before drawing, exactly as the existing callbacks do.

#### 4.4.5 `DrawImageOverlay` and `DrawPlots`

`SlamVisualiser::DrawImageOverlay` is used from `src/vio.cpp:699`–`784` (U12) with the `vis_map` lookup replaced by the live cache. It draws the per-camera tracked-point circles (hue-coded by depth via `getcolor`, §2.1) and, under `show_flow`, the optical-flow patch coordinates:
```cpp
void SlamVisualiser::DrawImageOverlay(pangolin::View& v, size_t cam_id) {
    (void)v;  // the view is unused; the cam_id selects the overlay source
    basalt::VioVisualizationData::Ptr vio;
    {
        std::lock_guard<std::mutex> lock(mpMtxVioVis);
        vio = mpLatestVio;
    }
    if (!vio) return;

    if (*mpShowObs && cam_id < vio->projections.size()) {
        // ... identical depth-ramp circle drawing as src/vio.cpp:716-748,
        //     reading vio->projections[cam_id] and (optionally under show_ids)
        //     the per-point id label.
    }
    if (*mpShowFlow && vio->opt_flow_res && mpOpticalFlow) {
        // ... identical patch-coordinate drawing as src/vio.cpp:752-783,
        //     reading vio->opt_flow_res->observations[cam_id] and the
        //     optical-flow frontend's patch_coord through mpOpticalFlow,
        //     obtained from Controller::GetOpticalFlow().
    }
}
```

`SlamVisualiser::DrawPlots` is used from `src/vio.cpp:878`–`933` (U7) unchanged except for reading the member toggles and `SlamVisualiser::mpVioDataLog`. The column indices ($0 time, $1–$3 velocity, $4–$6 position, $7–$9 gyro bias, $10–$12 accel bias) are exactly those logged by `SlamVisualiser::ConsumeVioStateQueue` (§4.3.5), so the series expressions are copied verbatim. It is invoked whenever a `show_est_*` toggle changes (the `GuiChanged()` edge in `Run()`), reproducing the U8 decoupling so the plotter is rebuilt only on demand.

#### 4.4.6 Why the File-Scope Pattern Is Replaced

The migration from `src/vio.cpp`'s file-scope globals (P2, O1) to members is what makes a single combined window possible. Every `Var`, the `OpenGlRenderState`, the `DataLog`, the `Plotter*`, the per-frame caches, and the trajectory buffers are now owned by one `SlamVisualiser` instance. The callbacks reach them through `this` rather than through globals, so the VIO layer and the local-map layer coexist without name collisions, and the consumer-thread lifecycle is encapsulated behind `Start`/`Stop` (O4) instead of being managed by hand in `main`.

### 4.5 Usage

This section shows how `class SlamVisualiser` is included into the combined VIO + local-mapping integration test (`src/basalt_slam.cpp`), the CMake change that keeps it executable-only, and how to run the test with the GUI enabled.

#### 4.5.1 Integration into `basalt_slam`

The headless integration test (`src/basalt_slam.cpp`) is extended with a `--show-gui` flag (default `false`, so existing CI invocations are unchanged) and the GUI lifecycle. The feeder thread is unchanged except that it now passes the dataset ground-truth pose to `Controller::TrackMonocular` as an `std::optional`:
```cpp
// CLI (alongside the existing options):
bool show_gui = false;
app.add_option("--show-gui", show_gui, "Show the SLAM GUI");

// Controller initialisation passes the GUI flag through to the new parameter:
controller->initialize(0, Sophus::SE3d(), Eigen::Vector3d::Zero(),
                       bg.cast<double>(), ba.cast<double>(),
                       /*useProducerConsumerArchitecture=*/false,
                       /*enableVisualisation=*/show_gui);
```

`feed_data` resolves the ground-truth pose for each image timestamp (the dataset exposes `get_gt_timestamps()` and `get_gt_pose_data()`, `dataset_io.h:123`–`125`) and forwards it. The shipped `gt_pose_for` lambda performs a nearest-neighbour lookup with `std::lower_bound`, since the mocap samples rarely coincide exactly with an image timestamp:
```cpp
// inside feed_data, replacing the existing TrackMonocular call:
Sophus::SE3f tcw;
std::optional<Sophus::SE3d> gtcw = gt_pose_for(image_timestamps[i]);
controller->TrackMonocular(data, tcw, gtcw);
```

`main` starts the visualiser before the feeder so the queues are wired before any frame is pushed, runs the render loop on the main thread while the feeder runs on `t1`, then tears down in the order specified in §4.3.6:
```cpp
if (show_gui) {
    basalt::SlamVisualiser visualiser(*controller);
    visualiser.Start();   // creates the window and wires the queues (main thread)

    std::thread t1(feed_data, controller, vio_dataset);
    visualiser.Run();     // blocks until the window is closed

    terminate = true;     // tell the feeder to stop early if the user quit first
    t1.join();
    controller->Stop();   // drains VIO + local mapper (their own nullptr sentinels)
    visualiser.Stop();    // releases the visualiser's consumer threads
} else {
    std::thread t1(feed_data, controller, vio_dataset);
    t1.join();
    controller->Stop();   // unchanged headless path
}

// ... existing post-run assertions on local_mapper->frame_poses ...
```
Starting the feeder after `SlamVisualiser::Start()` is a small improvement over a draft that started `t1` first. It guarantees `mpVio->out_vis_queue`, `mpVio->out_state_queue`, and `mpLocalMapper->out_vis_queue` are connected before the first `TrackMonocular` call, so no early frame is dropped from the display.

Note the constraint from §2.3 (D1): `SlamVisualiser::Start()` and `Run()` must execute on the thread that becomes the GL context owner — here, `main`. The dataset feeder, the VIO `measure()` (called synchronously from `TrackMonocular` on the feeder thread), and the local-mapper thread are all off the GL thread and reach the GUI only through the queues of §4.3.

#### 4.5.2 CMake

The two visualisation source files are added to the `basalt_slam` target only, and Pangolin is linked there. `libbasalt.so` is untouched, so the headless build and every other consumer of `libbasalt.so` remain Pangolin-free:
```cmake
add_executable(basalt_slam src/basalt_slam.cpp
  src/visualisation/visualiser.cpp
  src/visualisation/utils.cpp)
target_link_libraries(basalt_slam basalt pangolin basalt::cli11)
```

Because `LocalMapper` (in `libbasalt.so`) includes only the Pangolin-free `include/basalt/visualisation/utils.h`, this is the only edit required to the build; no Pangolin include directory or link is added to the core library target.

#### 4.5.3 Running the Test

To run the `basalt_slam` executable with the GUI run:
```
basalt_slam \
  --dataset-path /path/to/euroc/MH_05 \
  --dataset-type euroc \
  --cam-calib data/euroc_ds_calib.json \
  --config-path data/euroc_config.json \
  --show-gui true
```

The window opens with the `basalt_vio`-style layout: the camera image grid (top-left) with tracked-point overlays, the 3D view (right) showing the red VIO trajectory and green ground truth, the VIO sliding-window keyframe frusta and blue landmarks, and the local map drawn on top as orange points and amber keyframe frusta. The bottom strip plots the estimated state. The side panel exposes the toggles `show_obs`, `show_flow`, `show_ids`, `show_gt`, `show_est_*`, `follow`, `show_local_map_points`, and `show_local_map_kfs`.

Without `--show-gui` (or with `--show-gui false`), the executable is byte-for-byte the existing headless integration test: no window, no consumer threads, no queue pointers connected, and the post-run assertions on `local_mapper->frame_poses` run exactly as before (G2 zero overhead, G4 headless CI).

## 5. Key Classes & Interfaces

The references below collate every Pangolin and Basalt symbol referenced by §2–§4 into a single tabular reference. Each row records the symbol's name, the file/line at which it is declared (paths relative to the repository root `/ws/ros_ws/src/slam/ext/basalt/`), the kind, and a one-sentence role description. The grouping mirrors the reading order of the document.

### 5.1 Pangolin Core

| Symbol | File:Line | Kind | Role |
|---|---|---|---|
| `pangolin::View` | `thirdparty/Pangolin/include/pangolin/display/view.h:63` | struct | Hierarchical screen rectangle with bounds, layout, optional handler, and `extern_draw_function` callback. |
| `pangolin::View::SetBounds` | `view.h:126` (and overloads `:128`, `:131`) | method | Configures the bottom/top/left/right of the view in mixed pixel / fractional coordinates. |
| `pangolin::View::SetLayout` | `view.h:149` | method | Selects `LayoutOverlay`/`Vertical`/`Horizontal`/`Equal*`. Used for image grids. |
| `pangolin::View::SetHandler` | `view.h:135` | method | Assigns mouse/keyboard handler; takes ownership convention is non-deleting raw pointer. |
| `pangolin::View::SetDrawFunction` | `view.h:138` | method | Sets `extern_draw_function`; alternative is direct field assignment (`view.h:226`). |
| `pangolin::View::AddDisplay` | `view.h:152` | method | Appends a child view. |
| `pangolin::View::Activate` | `view.h:72`, `:75` | method | Sets `glViewport` and optionally applies an `OpenGlRenderState`'s matrices. |
| `pangolin::Layout` | `view.h:39` | enum | `LayoutOverlay`, `LayoutVertical`, `LayoutHorizontal`, `LayoutEqual`, `LayoutEqualVertical`, `LayoutEqualHorizontal`. |
| `pangolin::Attach` | `display/attach.h` | struct | Mixed-coordinate boundary; `Pix(n)` is pixel, fractional is `[0,1]`. |
| `pangolin::CreateWindowAndBind` | `display/display.h:72` | free fn | Creates platform window and binds OpenGL context to current thread. |
| `pangolin::CreateDisplay` | `display.h:195` | free fn | Creates an anonymous child of `DisplayBase()`. |
| `pangolin::CreatePanel` | `thirdparty/Pangolin/include/pangolin/display/widgets/widgets.h:41` | free fn | Creates a panel view that auto-renders `Var`s sharing its name prefix. |
| `pangolin::DisplayBase` | `display.h:187` | free fn | Returns the implicit root view. |
| `pangolin::FinishFrame` | `display.h:90` | free fn | Process events, render the view tree, swap buffers. |
| `pangolin::ShouldQuit` | `display.h:102` | free fn | True iff user requested window close. |
| `pangolin::OpenGlMatrix` | `display/opengl_render_state.h:98` | struct | Column-major 4×4 with Eigen interop (`:113`), `Load`/`Multiply`/`Inverse`. |
| `pangolin::OpenGlRenderState` | `opengl_render_state.h:173` | class | Owns projection + model-view matrices, `Apply()` to bind, `Follow(T_wc)` to track a moving frame. |
| `pangolin::ProjectionMatrix` | `opengl_render_state.h:241` | free fn | Pinhole projection in OpenGL's RUB convention. |
| `pangolin::ProjectionMatrixOrthographic` | `opengl_render_state.h:244` | free fn | Orthographic projection. |
| `pangolin::ModelViewLookAt` | `opengl_render_state.h:260`, `:263` | free fn | gluLookAt-style; the `AxisDirection` overload selects an up vector. |
| `pangolin::AxisDirection` | `opengl_render_state.h:69` | enum | `AxisX`/`AxisY`/`AxisZ` and negated variants. |
| `pangolin::Handler` | `handler/handler.h:55` | struct | Abstract input handler; `Keyboard`, `Mouse`, `MouseMotion`, `PassiveMouseMotion`, `Special` virtuals. |
| `pangolin::Handler3D` | `handler/handler.h:71` | struct | Orbit/pan/zoom handler bound to an `OpenGlRenderState`. |
| `pangolin::Var<T>` | `var/var.h:84` | class template | Named, GUI-bindable variable; auto-registered into `VarState` by ctor. |
| `pangolin::Var<T>::operator const T&` | `var.h:245` | method | Read the current value. |
| `pangolin::Var<T>::operator=` | `var.h:261` | method | Write the value programmatically. |
| `pangolin::Var<T>::GuiChanged` | `var.h:276` | method | One-shot edge detector: returns true exactly once after a GUI-driven change. |
| `pangolin::Var<T>::Meta` | `var.h:271` | method | Returns mutable `VarMeta` (range, step, gui_changed flag). |
| `Button` (alias) | `src/vio.cpp:88`, `src/mapper.cpp:130`, `src/vio_sim.cpp:150` | type alias | `pangolin::Var<std::function<void(void)>>`; renders as a clickable button. |

### 5.2 Pangolin Image and Plotting

| Symbol | File:Line | Kind | Role |
|---|---|---|---|
| `pangolin::ImageView` | `display/image_view.h:15` | class | `View` + `ImageViewHandler` that renders a `GlTexture` and supports pan/zoom. |
| `pangolin::ImageView::SetImage` | `image_view.h:30`, `:32`, `:34`, `:40`, `:42` | overloads | Upload pixel data with explicit `GlPixFormat`, `Image<T>`, `TypedImage`, or existing `GlTexture`. |
| `pangolin::ImageView::Clear` | `image_view.h:46` | method | Empties the texture. |
| `pangolin::GlPixFormat` | `gl/glpixformat.h` | struct | OpenGL-side pixel format descriptor (`glformat`, `gltype`, `scalable_internal_format`). |
| `pangolin::GlTexture` | `gl/gl.h` | class | OpenGL texture wrapper. |
| `pangolin::Plotter` | `plot/plotter.h:101` | class | 2D plot view backed by a `DataLog`; has its own `Handler`. |
| `pangolin::Plotter::Plotter` | `plotter.h:104` | ctor | `(default_log, left, right, bottom, top, tickx, ticky, ...)`. |
| `pangolin::Plotter::AddSeries` | `plotter.h:161` | method | Adds an X/Y series using `$0`,`$1`,...-indexed expressions. |
| `pangolin::Plotter::AddMarker` | `plotter.h:173` | method | Vertical/horizontal line marker. |
| `pangolin::Plotter::ClearSeries` / `ClearMarkers` | `plotter.h:153`, `:169` | methods | Reset the chart. |
| `pangolin::DrawingMode` | `plotter.h:47` | enum | `DrawingModePoints`, `DrawingModeDashed`, `DrawingModeLine`, `DrawingModeNone`. |
| `pangolin::Marker` | `plotter.h:55` | struct | Marker descriptor with direction, value, equality variant, colour. |
| `pangolin::DataLog` | `plot/datalog.h:180` | class | Block-allocated float sample log; thread-safe via `access_mutex`. |
| `pangolin::DataLog::Log` | `datalog.h:193`–`204` | overloads | Append one sample; variadic and `std::vector<float>` overloads. |
| `pangolin::DataLog::Clear` | `datalog.h:214` | method | Reset all blocks. |
| `pangolin::DataLogBlock` | `datalog.h:84` | class | Single contiguous block of samples; chained for unbounded growth. |

### 5.3 Pangolin Drawing Helpers

| Symbol | File:Line | Kind | Role |
|---|---|---|---|
| `pangolin::glDrawVertices<T>` | `gl/gldraw.h:83` | template free fn | Bind vertex pointer + `glDrawArrays` with `mode` (e.g. `GL_LINES`). |
| `pangolin::glDrawColoredVertices` | `gldraw.h:101` | template free fn | Same with per-vertex colour array. |
| `pangolin::glDrawLine` | `gldraw.h:118` | free fn | Single 2D line segment. |
| `pangolin::glDrawLines` | `gldraw.h:356` | free fn template | `GL_LINES` over a `std::vector<Eigen::Matrix<P,N,1>>`. |
| `pangolin::glDrawLineStrip` | `gldraw.h:364` | free fn template | `GL_LINE_STRIP` over a `std::vector<Eigen::Matrix<P,N,1>>`. |
| `pangolin::glDrawPoints` | `gldraw.h:346` | free fn template | `GL_POINTS` over a `std::vector<Eigen::Matrix<P,N,1>>`. |
| `pangolin::glDrawCirclePerimeter` | `gldraw.h:160`, `:392` | free fn / overload | Outline circle in 2D, used for keypoint markers. |
| `pangolin::glDrawAxis` | `gldraw.h:142`, `:429` | free fn / overload | Axis triad of given size; the `(T_wf, scale)` overload accepts a 4×4 transform. |
| `pangolin::GlFont::I` | `gl/glfont.h` | static accessor | Singleton default font. |
| `pangolin::GlFont::Text` / `.Draw` | `gl/glfont.h` | builder | Format-stringed text, drawn at a 2D location. |

### 5.4 Basalt Visualisation Utilities

| Symbol | File:Line | Kind | Role |
|---|---|---|---|
| `cam_color`, `state_color`, `pose_color`, `gt_color` | `include/basalt/utils/vis_utils.h:43`–`46` | global | Shared RGB tuples for cameras / states / keyframes / ground-truth. |
| `render_camera` | `vis_utils.h:48` | free fn | Draw a camera frustum at `T_w_c` with given line width, colour, scale. Wraps `glPushMatrix`/`glMultMatrixd`/`pangolin::glDrawLines`. |
| `getcolor` | `vis_utils.h:79` | free fn | Map a scalar `(p, np)` to an RGB hue, used to colour-code by depth. |

### 5.5 Basalt Visualisation Payloads and Queues

| Symbol | File:Line | Kind | Role |
|---|---|---|---|
| `basalt::VioVisualizationData` | `include/basalt/vi_estimator/vio_estimator.h:46` | struct | Per-frame visualisation snapshot from VIO: states, keyframes, landmarks, projections, raw OF result. |
| `basalt::VioVisualizationData::Ptr` | `vio_estimator.h:47` | typedef | `std::shared_ptr<VioVisualizationData>`. |
| `basalt::OpticalFlowResult` | `include/basalt/optical_flow/optical_flow.h:61` | struct | `t_ns`, `observations` (per-camera `aligned_map<KeypointId, AffineCompact2f>`), patch metadata. |
| `basalt::PoseVelBiasState<Scalar>` | `include/basalt/utils/imu_types.h` | struct | 15-DoF navigation state. Pushed to `out_state_queue` once per measurement. |
| `basalt::MargData` | `include/basalt/utils/imu_types.h` | struct | Off-thread payload to the local mapper; emitted via `out_marg_queue`. |
| `basalt::VioEstimatorBase<Scalar>::vision_data_queue` | `vio_estimator.h:80` | member | Capacity-10 input queue for `OpticalFlowResult::Ptr`. |
| `basalt::VioEstimatorBase<Scalar>::imu_data_queue` | `vio_estimator.h:81` | member | Capacity-300 input queue for `ImuData<double>::Ptr`. |
| `basalt::VioEstimatorBase<Scalar>::out_state_queue` | `vio_estimator.h:84` | pointer | Output state queue; null if disabled. |
| `basalt::VioEstimatorBase<Scalar>::out_marg_queue` | `vio_estimator.h:85` | pointer | Output marginalisation queue; null if disabled. |
| `basalt::VioEstimatorBase<Scalar>::out_vis_queue` | `vio_estimator.h:86` | pointer | Output visualisation queue; null if disabled. |
| `basalt::VioEstimatorBase<Scalar>::last_processed_t_ns` | `vio_estimator.h:77` | atomic | Most recent timestamp processed; used by `continue_fast` slider snap. |
| `basalt::VioEstimatorBase<Scalar>::finished` | `vio_estimator.h:78` | atomic | True once the estimator exits its main loop; the GUI uses this to stop the feeder threads. |

### 5.6 Existing Executable Entry Points

| Symbol | File:Line | Kind | Role |
|---|---|---|---|
| `main` (basalt_vio) | `src/vio.cpp:208` | free fn | Argument parsing, dataset/calibration load, queue wiring, GUI lifecycle. |
| `feed_images` | `src/vio.cpp:161` | free fn | Producer thread: pushes `OpticalFlowInput::Ptr` into `opt_flow_ptr->input_queue`. |
| `feed_imu` | `src/vio.cpp:191` | free fn | Producer thread: pushes `ImuData<double>::Ptr` into `vio->imu_data_queue`. |
| `draw_image_overlay` | `src/vio.cpp:699` | free fn | `extern_draw_function` for each `ImageView`. |
| `draw_scene` | `src/vio.cpp:786` | free fn | `extern_draw_function` for the 3D `display3D` view. |
| `draw_plots` | `src/vio.cpp:878` | free fn | Re-binds plotter series to `vio_data_log` based on `show_est_*` toggles. |
| `next_step`, `prev_step` | `src/vio.cpp:857`, `:868` | free fn | Step the `show_frame` slider. |
| `alignButton`, `saveTrajectoryButton` | `src/vio.cpp:935`, `:937` | free fn | Bound to GUI buttons. |
| `main` (basalt_mapper) | `src/mapper.cpp:150` | free fn | Loads `MargData` from disk, builds `nrf_mapper`, runs GUI. |
| `computeEdgeVis` | `src/mapper.cpp:590` | free fn | Rebuilds `edges_vis`, `roll_pitch_vis`, `rel_edges_vis` from `nrf_mapper`. |
| `detect`/`match`/`tracks`/`optimize`/`filter`/`alignButton`/`saveTrajectoryButton` | `src/mapper.cpp:653`, `:659`, `:665`, `:633`, `:675`, `:640`, `:680` | free fn | Bound to action buttons in the side panel. |
| `main` (basalt_vio_sim) | `src/vio_sim.cpp:161` | free fn | Generates synthetic data, runs the same VIO pipeline, displays both ground truth and estimate. |
| `gen_data` | `src/vio_sim.cpp:624` | free fn | Generates spline trajectory, IMU samples, and 2D observations. |
| `compute_projections` | `src/vio_sim.cpp:582` | free fn | Projects synthetic 3D points into each camera at each ground-truth pose. |


### 5.7 Unified SLAM Visualisation API

The symbols below are the new or modified types and members introduced by the unified visualisation system specified in §4.2–§4.5. They are grouped by file and annotated with the prefix convention used (`m` public member, `mp` private member, `mvp` private queue/vector). Paths are relative to the repository root.

New Pangolin-free visualisation payloads (`include/basalt/visualisation/utils.h`, §4.2.2):

| Symbol | Kind | Role |
|---|---|---|
| `basalt::LocalMapperVisualizationData` | struct | Immutable per-cycle local-map snapshot: `t_ns`, `keyframes` (`Eigen::aligned_vector<Sophus::SE3d>`), `points` (`Eigen::aligned_vector<Eigen::Vector3d>`), `point_ids`. Mirrors `VioVisualizationData`. |
| `basalt::LocalMapperVisualizationData::Ptr` | typedef | `std::shared_ptr<LocalMapperVisualizationData>`. |
| `basalt::GtPose` | struct | Ground-truth sample `{int64_t t_ns; Sophus::SE3d T_w_i;}` forwarded by `TrackMonocular`. |
| `basalt::local_map_point_color` / `local_map_kf_color` | const uint8_t[3] | Distinct local-map colours (orange / amber), separate from the VIO palette in `vis_utils.h`. |

`class SlamVisualiser` (`include/basalt/visualisation/visualiser.h`, `src/visualisation/visualiser.cpp`; `basalt_slam` target only, §4.2.4):

| Symbol | Kind | Role |
|---|---|---|
| `SlamVisualiser(basalt::Controller&)` | constructor | Captures references to the controller's VIO, local mapper, and calibration. No GL work. |
| `SlamVisualiser::Start` | method | Main-thread: create window + layout, wire the four queues, launch consumer threads. Honours `Controller::IsVisualisationEnabled()`. |
| `SlamVisualiser::Run` | method | Main-thread render loop until `pangolin::ShouldQuit()`; drains GT, uploads images, follows camera, `FinishFrame`. |
| `SlamVisualiser::Stop` | method | Idempotent sentinel-`nullptr` shutdown of the consumer threads. |
| `SlamVisualiser::DrawScene` / `DrawImageOverlay` / `DrawPlots` / `SetupLayout` | private methods | GL-thread draw callbacks and layout, ported from `src/vio.cpp` (U2, U5–U13). |
| `SlamVisualiser::ConsumeVioVisQueue` / `ConsumeVioStateQueue` / `ConsumeLocalMapQueue` | private methods | Consumer-thread bodies; fill caches only, no GL. |
| `mvpVioVisQueue`, `mvpVioStateQueue`, `mvpLocalMapVisQueue`, `mvpGroundTruthQueue` | private queues | Owned vis queues, connected to the estimators in `Start()` (§4.3.1). |
| `mpLatestVio`, `mpLatestLocalMap`, `mvpVioTrajectory`, `mvpGroundTruthTrajectory` | private caches | Latest-only payloads + positions-only trajectories |

Modified core types (additive, null-gated):

| Symbol | File:Line | Change |
|---|---|---|
| `LocalMapper::out_vis_queue` | `include/basalt/vi_estimator/local_mapper.h` | New public `tbb::concurrent_bounded_queue<LocalMapperVisualizationData::Ptr>*` publish hook, mirroring `VioEstimatorBase::out_vis_queue`. Filled by a `try_push` snapshot at the end of `MapLocally()` (§4.3.3). |
| `Controller::initialize(...)` | `include/basalt/controller.h:33`–`36` | New trailing `bool enableVisualisation = false` argument; stored in `mpEnableVisualisation`. |
| `Controller::TrackMonocular(...)` | `include/basalt/controller.h:47` | New trailing `std::optional<Sophus::SE3d> gtcw = std::nullopt`; pushed to the GT queue when visualisation is enabled (§4.3.4). |
| `Controller::SetGroundTruthVisualisationQueue` / `IsVisualisationEnabled` | `include/basalt/controller.h` | New methods registering `mvpGroundTruthQueue` and exposing `mpEnableVisualisation`. |
| `VioEstimatorBase::out_vis_queue` / `out_state_queue` | `vio_estimator.h:86`, `:84` | Unchanged; reused as the VIO vis and state taps (§4.3.1). |

### 5.8 Concluding Remarks

The classes catalogued above span three layers: the Pangolin layer (§§5.1–5.3) provides primitives that are reused verbatim across all three existing Basalt GUIs; the Basalt visualisation utilities and payloads (§§5.4–5.5) provide the data contract between the estimator threads and the GL thread; the executable entry points (§5.6) compose the two into runnable integration tests. The proposed additions in §5.7 do not introduce a new abstraction so much as they encapsulate the existing patterns in a class boundary that the live local mapper can plug into without disturbing the older code paths. By the time §5.7 has been implemented, every numeric and geometric quantity flowing through the SLAM stack has a single owning thread and a single rendering path. This is the property that makes the integration tests of §3.3 trustable and that the unified `SlamVisualiser` of §4 is designed to preserve.

---

## 6. References

The references below have been cross-checked against the repository state at the commit on branch `shreyas/local_mapper`. Line numbers cited throughout the document resolve against the same snapshot.

### 6.1 Basalt Source

1. `src/vio.cpp` — `basalt_vio` integration test (live VIO with GUI). Sections 3.3.1, 5.6.
2. `src/mapper.cpp` — `basalt_mapper` offline NFR mapping integration test with GUI. Sections 3.3.2, 5.6.
3. `src/vio_sim.cpp` — `basalt_vio_sim` simulation-driven VIO integration test. Sections 3.3.3, 5.6.
4. `src/vi_estimator/sqrt_keypoint_vio.cpp` — implementation of `SqrtKeypointVioEstimator`; lines 583–607 assemble the `VioVisualizationData::Ptr` payload, lines 190–192 issue the end-of-stream sentinel pushes. Section 3.1.
5. `include/basalt/vi_estimator/vio_estimator.h` — declaration of `VioVisualizationData` (line 46), `VioEstimatorBase` (line 64), and the four queue interfaces (lines 80–86). Sections 3.1, 5.5.
6. `include/basalt/utils/vis_utils.h` — shared rendering helpers `render_camera`, `getcolor`, and the four colour constants. Section 5.4.
7. `include/basalt/optical_flow/optical_flow.h` — declaration of `OpticalFlowResult` carried inside `VioVisualizationData`. Section 5.5.

### 6.2 Pangolin Source (`thirdparty/Pangolin/`)

8. `include/pangolin/display/view.h` — `View` class, `Layout` enum, `extern_draw_function`. Sections 2.2 (D2), 5.1.
9. `include/pangolin/display/display.h` — global window/context API: `CreateWindowAndBind`, `CreateDisplay`, `DisplayBase`, `FinishFrame`, `ShouldQuit`. Sections 2.3 (A1, A5), 5.1.
10. `include/pangolin/display/opengl_render_state.h` — `OpenGlMatrix`, `OpenGlRenderState`, projection / lookAt helpers. Sections 2.1, 2.2 (D3), 5.1.
11. `include/pangolin/display/image_view.h` — `ImageView` class. Sections 2.2 (D6), 5.2.
12. `include/pangolin/display/widgets/widgets.h` — `CreatePanel` declaration. Section 5.1.
13. `include/pangolin/handler/handler.h` — `Handler`, `Handler3D`. Sections 2.2 (D4), 5.1.
14. `include/pangolin/var/var.h` — `Var<T>`, GUI-bound named variable with `GuiChanged` edge detector. Sections 2.2 (D5), 5.1.
15. `include/pangolin/plot/plotter.h` — `Plotter` and `Marker`. Sections 2.2 (D6), 5.2.
16. `include/pangolin/plot/datalog.h` — `DataLog`, `DataLogBlock`. Sections 2.2 (D6), 5.2.
17. `include/pangolin/gl/gldraw.h` — immediate-mode drawing helpers (`glDrawPoints`, `glDrawLines`, `glDrawLineStrip`, `glDrawCirclePerimeter`, `glDrawAxis`, `glColorHSV`, `glDrawColoredVertices`). Sections 2.1, 5.3.

### 6.3 Companion Basalt Documentation

18. `doc/VIO.md` — algorithmic specification of the VIO estimator; written in the same academic structure mirrored by this document.
19. `doc/Mapping.md` — algorithmic specification of the offline NFR mapper; structural sibling of `doc/VIO.md`.
20. `doc/LocalMapper.md` — implementation specification of the live local mapper; defines the threading model, `MargData` ingestion, and the VIO pose-feedback channel that §4 of this document depends on.
21. `doc/LocalMapper2.md` — iteration plan for the local mapper, mirroring this document's iterative writing approach.
22. `doc/Simulation.md` — user-facing description of `basalt_vio_sim` and `basalt_mapper_sim`, including the screenshot captions used by §3.3.3.
23. `doc/VioMapping.md` — combined VIO + offline mapper user guide.
24. `doc/Marginalisation.md` — Schur-complement marginalisation derivation; the mathematical underpinning of the `MargData` packets the local mapper consumes.
25. `doc/Calibration.md`, `doc/Realsense.md`, `doc/DevSetup.md` — peripheral guides; cited only in passing.

### 6.4 Reference Screenshots (in `doc/img/`)

26. `MH_05_VIO.png` — `basalt_vio` rendering the EuRoC `MH_05` sequence. §3.3.1.
27. `MH_05_MAPPING.png` — `basalt_mapper` rendering the same sequence's serialised marginalisation output.
28. `MH_05_OPT_FLOW.png` — optical-flow stand-alone rendering.
29. `SIM_VIO.png` — `basalt_vio_sim` rendering. §3.3.3.
30. `SIM_MAPPER.png` — `basalt_mapper_sim` rendering.
31. `magistrale1_vio.png`, `magistrale1_mapping.png` — TUM-VI `magistrale1` runs of `basalt_vio` and `basalt_mapper`. §3.3.2.
32. `kitti.png`, `kitti_video.png`, `t265_vio.png`, `teaser.png` — additional runs across datasets and platforms.

### 6.5 External Background

33. *Pangolin* — Steven Lovegrove et al. The upstream project at `github.com/stevenlovegrove/Pangolin` hosts the canonical examples used as cross-references during the §2 walk-through. The version vendored under `thirdparty/Pangolin` is internally consistent with the API shape described here.
34. *OpenGL Programming Guide* (the "Red Book") — the canonical reference for the rasterisation pipeline, viewport coordinates, depth testing, and the immediate vs. retained-mode distinction summarised in §2.1. Used as a vocabulary source rather than as a tutorial.
35. Usenko, V., Demmel, N., Schubert, D., Stückler, J., & Cremers, D. (2020). *Visual-Inertial Mapping with Non-Linear Factor Recovery*. RA-L. Source for the algorithmic claims in §3.3.2 (the offline mapper renders `RelPoseFactor` and `RollPitchFactor` graphs). Cross-referenced from `doc/Mapping.md`.
