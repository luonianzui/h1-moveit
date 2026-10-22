# H1-2 AprilTag map 826-b1

All poses use `T_object_tag`: the tag frame is expressed in the named Gazebo
object link.  Tag `+Z` points out of the labelled surface, `+X` points toward
the marker image right edge, and `+Y` points toward its top edge.  Pose
estimation uses the black-border edge in `size_m`; the SDF plane is 1.25 times
larger because it includes a one-module white quiet zone.

The cabinet tags are separated on the horizontal body top to survive door and
arm occlusion.  The door tag is on the original outer door plane away from the
handle.  Cube ID 20 is the primary top tag; ID 23 is a 30 mm far-edge top tag
that keeps the neutral-pose gripper from hiding every cube observation.  The
two 20 mm side tags are reacquisition aids.  During all-tag occlusion the
estimator must use the certified `T_gripper_cube` and robot FK, not extrapolate
stale vision.

Every tag is a visual-only plane offset 0.5 mm from its source surface.  No
collision, inertial, friction, joint, mesh, latch, or support property is
changed by this map.

## Optical pose correction

`pose_correction_in_object` is a post-bundle, object-frame correction measured
from 50 stable Gazebo D435 frames on 2026-08-26.  The raw static mean errors
were 3.646 mm (cabinet), 1.099 mm (door), and 0.852 mm (cube).  The correction
is versioned inside `tag_map.yaml`, participates in the configuration hash,
and does not move any SDF visual or physical geometry.  A camera, resolution,
or renderer change invalidates this calibration and requires a fresh Stage-E
dataset.
