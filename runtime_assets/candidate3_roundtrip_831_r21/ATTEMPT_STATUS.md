# Candidate-3 r21 recording attempt status

Execution status: **PASS**

Video delivery status: **REJECTED_TRUNCATED**

The live v9/r17-equivalent flow reached its terminal PASS, but this attempt
took `171.442393068 s` from flow invocation to terminal completion.  The video
recorder's `169 s` safety cap therefore ended about `2.44 s` before terminal
completion.  Do not use `v9_r17_user_set_angle_roundtrip_r21.mp4` as the final
complete demonstration video.

The exact user-set GUI camera pose was captured in
`CAMERA_POSE_USER_SET.json` so a fresh non-overwriting replay can reproduce the
view without manual adjustment.
