# AGENTS.md

## Build system

This is a **colcon** ROS2 workspace. Never run `cmake` directly on individual packages — always use colcon.

```bash
# Build everything
colcon build --symlink-install

# Build a single package
colcon build --symlink-install --packages-select tagslam

# After any build
source install/setup.bash
```

Build artifacts live in `build/` and `install/`. Both are colcon-managed; do not edit by hand.

## Dependencies

- ROS2 Rolling or Jazzy (older versions lack rosbag2 features)
- Required system libs: **GTSAM**, OpenCV, Eigen3, Boost (graph component), yaml-cpp
- Most ROS2 deps can be installed as apt packages:
  ```
  apt install ros-${ROS_DISTRO}-apriltag-detector ros-${ROS_DISTRO}-apriltag-draw \
              ros-${ROS_DISTRO}-apriltag-detector-umich ros-${ROS_DISTRO}-apriltag-detector-mit \
              ros-${ROS_DISTRO}-flex-sync
  ```

## Package layout

```
src/
├── tagslam/              # Main SLAM + sync_and_detect executables
├── pid_control/          # PID control + matplotlib UI for robot navigation
├── apriltag_detector/    # Multi-package subdir (detector, draw, umich, mit, tools)
├── apriltag_mit/         # MIT AprilTag C library (non-ROS2, plain cmake)
├── apriltag_msgs/        # ROS2 .msg definitions (AprilTagDetection, etc.)
└── flex_sync/            # Header-only sync library (like message_filters but dynamic)
```

## Test and lint

```bash
# Run tests for a single package
colcon test --packages-select tagslam

# Lint checks are part of `colcon test` when BUILD_TESTING=ON
# Linters used: ament_copyright, ament_cppcheck, ament_cpplint (.clang-format), ament_lint_cmake, ament_xmllint
```

## Code style

- C++17, based on Google style with custom `.clang-format` file (80 columns, brace wrapping, pointer alignment middle)
- Compile flags: `-Wall -Wextra -Wpedantic -Werror`
- Header include path pattern: `#include "tagslam/foo.hpp"` (project-scoped)

## Runtime conventions

- **Always** pass `use_sim_time:=True` when playing from rosbags, and use `ros2 bag play --clock-topics-all`
- 3 YAML config files are required: `cameras.yaml`, `tagslam.yaml`, `camera_poses.yaml` (default location: `config/` at workspace root)
- `sync_and_detect` reads sensor topics from `cameras.yaml`, detects AprilTags, and emits synchronized tag messages consumed by tagslam
- If a rosbag has images but no tags, `tagslam_from_bag` automatically invokes `sync_and_detect` internally
- Camera time stamps must be hardware-synced (exact match on `header.stamp`), or enable `use_approximate_sync`
- Executables install to `lib/tagslam/` (not `bin/`) — launch files resolve them via package name

## Key executables

| Command | Purpose |
|---|---|
| `ros2 run tagslam tagslam_node` | Online SLAM node |
| `ros2 run tagslam tagslam_from_bag` | Offline SLAM from rosbag |
| `ros2 run tagslam sync_and_detect_node` | Online tag detection + sync |
| `ros2 run tagslam sync_and_detect_from_bag` | Offline tag detection from rosbag |
| `ros2 run pid_control pid_node.py` | PID controller with matplotlib UI |

Equivalent launch files exist in each package's `launch/` subdirectory.
