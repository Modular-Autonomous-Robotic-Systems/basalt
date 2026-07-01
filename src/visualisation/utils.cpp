#include <basalt/visualisation/utils.h>

// The Pangolin-free visualisation payloads (LocalMapperVisualizationData,
// GtPose) and the distinct colour constants are defined entirely in the
// header. This translation unit exists so the build target has a concrete
// compilation anchor for the visualisation utilities and so future out-of-line
// helpers can be added here without re-touching the Pangolin boundary.

namespace basalt {

}  // namespace basalt
