# Unitree Dex1-1 model provenance

The Dex1-1 STL meshes and original URDF in this directory/package were copied
from Unitree Robotics' `dex1_1_service` repository:

- Repository: https://github.com/unitreerobotics/dex1_1_service
- Commit: `cdd9fc5a78d51521eb262a56e0c5c19770700932`
- License: BSD-3-Clause (see `LICENSE`)

`urdf/right_dex1_1_gripper.urdf.xacro` prefixes the original link and joint
names, adds an H1-2 right-wrist simulation adapter and TCP, and restores the
second contact pad from the supplied `Link2_3.STL` mesh.
