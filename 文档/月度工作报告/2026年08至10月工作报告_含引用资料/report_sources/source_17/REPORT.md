# Unitree H1-2 Full-Scene Navigation Report

Date: 2026-09-11 CST  
Scope: Gazebo only

## Result

The user-approved official Unitree H1-2 recurrent gait is integrated into the
existing combined Gazebo scene. The visible run contained the square enclosure,
H1-2 with Mid360 and both grippers, table, green cube, and the original
incubator. Mid360 localization and Nav2 planned and executed the fixed startup
pose to incubator staging goal without upright assistance.

The automatic state sequence was:

```text
WAITING_FOR_HEALTH
PLANNING:HEALTH_GATES_PASS
NAVIGATING:PLAN_PASS
WAITING_FOR_STABLE_HOLD:NAV2_GOAL_SUCCEEDED
PASS:INCUBATOR_STAGING_GOAL_STABLE_HOLD
```

## Gait Contract

```text
policy source       /home/liu/robot/gym_sim/unitree_rl_gym-main/deploy/pre_train/h1_2/motion.pt
policy source SHA   ea3003268d5d2e4b5004eb26d0969fbfdedced265ceeec7bf0b335309259cc80
NumPy export SHA    68a1f2dff967fdc464451956eb6b4c60288cf375e38629092edab23b2adb014c
observation/action  47 / 12
policy rate         50 Hz
actuation           direct leg position targets through official PD gains
zero command        recurrent station-keeping gait
station transition  one LSTM-memory reset; phase and previous action preserved
upright assistance  false
```

The former V12 frequency scaling, integrated target limiter, finite motion
bursts, braking capture, and restart dwell are not used by this profile.

## Navigation Evidence

```text
startup pose                         (-0.60, 0.00, 0.00 rad)
goal pose                            (-0.45, 0.00, 0.00 rad)
planned path length                  0.14999996274711844 m
terminal physical goal error         0.06026098874092244 m
terminal localized goal error        0.0565685424949237 m
terminal up_z                        0.999793612045874
terminal height                      1.0139880725188575 m
post-PASS capture                    19.99177793500712 s / 495 samples
post-PASS minimum up_z               0.999519508590246
post-PASS minimum height             1.002879954146454 m
post-PASS minimum total support      329.7430277165396 N
locomotion faults                    none
```

The robot remained upright and in `STATION_KEEPING_GAIT` for the complete
post-PASS capture. This is a dynamic gait hold rather than a frozen pose: the
post-PASS physical goal error peaked at `0.1088907041009881 m`, corridor error
at `0.09743514023804267 m`, and station-anchor error at
`0.10125528177090068 m`. Therefore this report proves bounded short-distance
localization and walking in the full scene, but does not promote the run as a
strict continuous 0.08 m long-hold authority.

## Validation

The final source was rebuilt with the required Miniconda six-library isolation
procedure. Package tests passed:

```text
112 tests / 0 errors / 0 failures / 0 skipped
```

Relevant final source identities:

```text
unitree_h12_gazebo_controller.py  3c2767e4bcc4d264d43a3d1a47cdcb6a617a7d724827b56080747791bcd799cb
unitree_h12_locomotion.yaml       cb7e9797df04692c504c765e2ef348a93ae79674b39cf8919c8a40046a269731
unitree_h12_contract.py           5a71e90107a0877c9500ed518657d21256a15265fab939fda70ce8ec97e0f978
incubator_auto_navigator.py       92607c0a243404149d67dd3c607ac630d9721e902464a70e0469f094cd50427d
incubator navigation launch       83a0349e379fc5bc8c0ee6031392e3422f208a9ce2125b846c728be12dfde864
```

## Protection Audit

Before and after the run, the protected v9/r17 and v19/r29 incubator assets,
the source incubator model, and backup 909 matched their recorded hashes. No
incubator trajectory, door, handle, gripper, support, reset, detach, or recovery
command was issued. Shutdown used normal `SIGINT` after evidence capture.

The rejected `r18` lateral-limit experiment is retained separately and was
reverted byte-for-byte; it is not part of this result.
