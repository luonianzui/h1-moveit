# Workspace-local Nav2 localization, planning, and control runtime

This directory contains an unmodified, non-root extraction of official ROS 2
Jazzy Ubuntu packages. It does not replace or write into `/opt/ros/jazzy`.

Packages:

- `ros-jazzy-nav2-amcl` 1.3.12;
- `ros-jazzy-nav2-lifecycle-manager` 1.3.12;
- `ros-jazzy-nav2-planner`, `ros-jazzy-nav2-navfn-planner`, and
  `ros-jazzy-nav2-costmap-2d` 1.3.12;
- `ros-jazzy-nav2-controller`, `ros-jazzy-nav2-bt-navigator`,
  `ros-jazzy-nav2-behaviors`, `ros-jazzy-nav2-velocity-smoother`, and
  `ros-jazzy-nav2-regulated-pure-pursuit-controller` 1.3.12;
- the missing ROS package dependencies reported by `apt-get -s` for this
  localization, planning, and control closure.

Load the runtime together with the workspace overlay:

```bash
source runtime_dependencies/nav2_jazzy_1.3.12/setup.bash
```

The original `.deb` files are retained under `debs/`; `SHA256SUMS` records
their byte identities.
