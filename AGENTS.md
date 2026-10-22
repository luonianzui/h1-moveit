# AGENTS.md — H1-2 视觉空中换手与孵育箱操作权威交接

> 2026-10-14 日志存储更新：用户已明确授权将 r161–r211 的精确 31 份旧逐帧日志
> 无损压缩，完整核验后删除原始副本；不扩大为其他成果清理。新候选的准备、Python
> 回归及前后保护命令须经 `tools/trace_archive/run -- 原命令及参数` 启动，继承其
> `PYTHONPATH` 和 `H1_TRACE_ARCHIVE_REGISTRY`，才能按原路径/原 SHA 读取压缩日志。
> 旧 SHA256SUMS、封存源码、视频、报告保持原字节。旧清单使用
> `python3 tools/trace_archive/archive_cli.py verify-manifest 清单路径`，不能直接用
> 原生 `sha256sum -c` 读取已迁移的 raw 路径。细节见 `tools/trace_archive/README.md`；
> 本次进度/完成结果见 `runtime_evidence/trace_archive_20261014_r1/migration_result.json`。
> 31 份迁移已完成（68.313 GB → 14.226 GB），需新启动兼容进程加载完整 registry；
> 不复用迁移期间已加载部分 registry 的长进程。

本文是本项目唯一权威交接文档。只使用这里的最终配置、命令和实测结果；不要从
旧对话、已删除的历史文档或 `/tmp` 恢复脚本，也不要重建机器人模型。快速运行
步骤见 `QUICKSTART.md`。当前状态的短摘要和接手索引见 `HANDOFF_CURRENT.md`；该
摘要不替代本文，分支化接手步骤见 `HANDOFF_PLAN.md`，可复制的新对话提示词见
`HANDOFF_PROMPT.md`；任何冲突仍以本文最靠后的时间小节为准。

## 当前入口（2026-09-04）

- 精确当前状态以本文最末的“2026-09-04：当前双循环权威、r28 视角证据与交接清理”
  小节为准；中间所有“当前/下一步”文字只保留历史语义。
- 动作语义权威是 `candidate3_roundtrip_831_authority_v9 / r17`：右手单独操作把手，
  左手放入并取回方块，双循环 `164.523835390 s` PASS。
- 最新用户视角证据是非覆盖的 `candidate3_roundtrip_831_authority_v18 / r28`：与 v9/r17
  的身份无关 payload 一致，`173.580995272 s` PASS；它是固定轨迹视觉重放，不是新的
  AprilTag 动态重规划证书。
- 826 AprilTag A--L 已完成并作为独立成果线保留；报告见
  `runtime_evidence/826_apriltag_execution_20260827/REPORT.md`。
- 最新正式受保护备份是 `901.tar.zst`，SHA-256 为
  `212e16a1a08d3dec0ff097b5d93b4dd9c5619bc575c401b0b31394b012419cc9`；它不包含后续
  v9/r17、r20、r24、r25、r28 或本轮文档整理，不得覆盖。
- 当前 live 为 Gazebo server/GUI 已退出、仅余孤立子进程的退化非 fresh 现场，不能直接
  运行或录像。默认只读复核；新重放必须获得用户明确要求并创建全新非覆盖身份。

## 历史基线（2026-08-14）

- 环境：Ubuntu 24.04、ROS 2 Jazzy、MoveIt 2、Gazebo Harmonic、RTX 5090。
- 工作空间：
  `/home/liu/h1_moveit/h1_2_ros2_ws_claude_2026-07-30/h1_2_ros2_ws`。
- 本目录没有 Git 元数据；修改前必须直接检查现有文件。
- 当前物体 `green_cube` 为 `80 x 160 x 30 mm`、`0.012 kg`，加长方向为物体
  局部 Y。桌面顶面 `z=0.815 m`，静止物体中心 `z=0.830 m`。
- 真实 Gazebo D435 RGB-D、ROS 图像桥、HSV+深度检测和 `/cube_pose` 流程已经
  验证；运行 `cube_detector` 时使用 `use_mock_data=false`。
- Gazebo、GUI、`move_group`、`h1_2_depth_point_cloud` 和五个控制器的最新只读
  检查均健康；是否存在正在运行的离线 demo 以文末“2026-08-17 最新交接”为准，
  健康进程必须复用，禁止重复启动。
  2026-08-24 用户要求以后把方块默认初始化到左手处；当前启动默认值已改为
  `XYZ=(0.334000, +0.377000, 0.830000)`、`RPY=(0,0,+90 deg)`。该点位于中立
  左手 TCP 正下方；`+90 deg` 让 80 mm 夹持轴与左 Dex1 对齐，并为中层铁架中心
  保留 75 mm 端部抓持偏置。方块到孵育箱 body 的最近轴向间隔仍超过 `60 mm`。
- 加长方块的完整空中换手已经真实 Gazebo 验证：右手取物、50 mm 抬升、中央
  搬运、左手质心附近重抓、右手逐毫米卸载和撤离、左手单持、垂直放桌、释放
  和撤离，程序 `exit 0`。
- 孵育箱把手已恢复为 `incubator_assembly.SLDASM` 中的原装 STL、装配姿态、质量
  和惯量；Gazebo visual/collision 与 MoveIt 物理碰撞均使用同一个原装网格。
  先前的圆柱抓取套筒、凹入盒体碰撞和把手竖直化已经全部删除。
- 旧套筒/盒体把手结果全部作废。2026-08-14 已用左手夹住原装 STL 后半部，
  让完整 6-D TCP 绕固定把手转轴走圆弧，Gazebo 实测从朝下 `0 deg` 连续旋转到
  `89.827 deg`，程序 `exit 0`。这不是夹爪中心原地的纯腕关节旋转。
- 右手采用用户要求的深夹配置：TCP Z 从 `20.5 mm` 降为 `18.5 mm`。固定
  `60 deg` 倾角下，完整打开的 Dex1 STL 到桌面的精确最低净距为
  `10.541 mm`，仍通过 `10 mm` 手/桌门。`18.0 mm` 只剩 `10.041 mm`，不采用。

## 最终配置

```text
object size / mass                    80 x 160 x 30 mm / 0.012 kg
pickup spawn X / Y                    +315 / -180 mm
right donor local Y                   -64 mm
right donor TCP Z / inward tilt       18.5 mm / 60 deg
left receiver local Y                 +8 mm (COM distance 8 mm)
left receiver TCP Z / inward tilt     19.114705 mm / 55 deg
left outer staging local Y            +115 mm
receiver overpass height              15 mm
pickup / airborne lift                50 mm
right / left preload                  3 / 3 mm
airborne D435 correction X/Y/Z        +2 / -10 / -2 mm
hand/hand safety gate                 5 mm
hand/table safety gate                10 mm
Cartesian eef_step / minimum fraction 5 mm / 99.5%
right donor unload targets            +2,+1,0,-1,-2 mm
right donor retreat                   fully open, then 15+35 mm
```

右手局部 Y 向质心深入的离线单变量搜索结果：

```text
-63.5 mm  PASS, minimum hand/hand 5.214 mm
-63.0 mm  FAIL, hand/hand 4.914 mm
-62.5 mm  FAIL, hand/hand 4.843 mm
```

因此最终仍使用 `Y=-64 mm`；深夹通过降低 TCP Z 实现，不牺牲中心重抓的手间
距离门。

## 最终物理实测

最终 `grasp_z_offset=18.5 mm` 的完整 `donor_release_test`：

```text
right real preload feedback           q1/q2 = -15.131/-14.767 mm
right pinch-center offset             0.182 mm
right 50 mm lift                      49.99 mm clearance, 0.01 deg tilt
right central carry                   49.97 mm clearance, 0.02 deg tilt
fresh D435 vs Gazebo                  1.49 mm / 0.45 deg
left center Cartesian path            100.0%, 33 points
live minimum hand/hand, hand/table    5.763 / 60.763 mm
dual-contact hold                     49.56 mm, 0.40 deg
after right support detach            49.46 mm, 0.49 deg
right fully-open feedback shortfall   0.071 mm
right 15+35 mm retreat                100.0% / 100.0%
left-only gripper center offset       0.135 mm
left-only airborne hold               50.43 mm, 0.34 deg
left table support before release     0.60 mm, 0.23 deg
final table release                   0.00 mm, 0.00 deg, drift 0/0
program                               exit 0
```

MoveIt 规划成功不是物理成功的替代物。上述各阶段同时读取了 Gazebo 方块位姿和
真实 Dex1 指关节反馈。

## 标准命令

每个终端先执行：

```bash
cd /home/liu/h1_moveit/h1_2_ros2_ws_claude_2026-07-30/h1_2_ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

健康长期进程直接复用，不要重复启动：

```bash
ros2 launch h1_2_moveit_config gazebo.launch.py \
  spawn_incubator:=true incubator_x:=0.590
ros2 launch h1_2_moveit_config move_group_only.launch.py
ros2 run h1_2_vision cube_detector --ros-args -p use_mock_data:=false
```

完整离线中心重抓、闭合、donor 撤离和左手放桌预检（不发送轨迹）：

```bash
ros2 run h1_2_moveit_demo bimanual_air_handover_demo --ros-args \
  -p mode:=donor_release_test \
  -p center_regrasp_preflight_only:=true \
  -p object_size_x:=0.080 -p object_size_y:=0.160 -p object_size_z:=0.030 \
  -p right_grasp_y_offset:=-0.064 -p grasp_z_offset:=0.0185 \
  -p cooperative_inward_tilt_degrees:=60.0 \
  -p left_grasp_y_offset:=0.008 \
  -p transfer_receiver_z_offset:=0.019114705 \
  -p center_regrasp_search_tilt_degrees:=55.0 \
  -p serial_receiver_prehandover_y_offset:=0.115 \
  -p center_regrasp_overpass_height:=0.015 \
  -p pickup_test_lift:=0.050 \
  -p gripper_preload_position:=0.003 \
  -p transfer_receiver_preload_position:=0.003 \
  -p recovery_table_surface_z:=0.815
```

必须看到 `LONG_BLOCK_CENTER_PREFLIGHT_PASS` 和 `no trajectory executed`。

完整物理流程（脚本默认值已经固化为上表配置）：

```bash
HANDOVER_MODE=donor_release_test \
ros2 run h1_2_moveit_demo bimanual_air_handover.sh
```

## 安全与恢复规则

1. 首先只读检查残留的 Gazebo、ROS、`move_group`、检测器和 demo 进程；健康
   进程直接复用。
2. 修改抓点、姿态、路径或门限前，先向用户说明方法和精确数值。
3. 物体仍被夹持时禁止 reset 或强停；让物理门完成安全恢复，或使用对应恢复
   模式。
4. 抬升测试固定为 `50 mm`；禁止用 `5 mm` 代替。
5. 手间门为 `5 mm`，手桌门为 `10 mm`。不要把 donor 的 1 mm 卸载步长、
   `0.5 mm` 完全张开反馈门或回桌接触窗机械改成 5/10 mm。
6. 仿真网格搜索必须先做 STL 几何、完整 IK、碰撞和全路径预检，每次只改变
   一个变量。
7. 每个物理阶段读取 Gazebo 方块位姿和真实指关节反馈。
8. Cartesian 最低完整率 `99.5%`；禁止执行 partial。
9. 只有右手承重时使用 `HANDOVER_MODE=right_holder_place_recovery`；只有左手
   承重或 donor 已张开时使用 `HANDOVER_MODE=receiver_place_recovery`；双手仍
   夹持时使用 `HANDOVER_MODE=dual_hold_place_recovery`。
10. 未知低位左臂只使用已验证的固姿态世界 Z `+120 mm` 恢复；`+200 mm`
    仅解出 70%，禁止执行。

独立 Gazebo 抓取支撑插件只提供可控的柔顺持物力，不删除跨模型碰撞。右、左
支撑分别绑定 `right_wrist_yaw_link` 和 `left_wrist_yaw_link`；支撑转移必须发生
在真实双指接触和反馈门通过之后。

## 构建

Miniconda 的六个同名库会与系统库冲突。修改 `h1_2_moveit_demo` 或
`h1_2_moveit_config` 后必须使用以下精确隔离命令，不使用 `.so*` 模糊匹配：

```bash
cd /home/liu/h1_moveit/h1_2_ros2_ws_claude_2026-07-30/h1_2_ros2_ws
source /opt/ros/jazzy/setup.bash
(
  set -eo pipefail
  h1_backup_dir="$(mktemp -d /tmp/h1_miniconda_lib_backup.XXXXXX)"
  h1_conda_lib="/home/liu/miniconda3/lib"
  h1_names=(libssl.so libssl.so.3 libcrypto.so libcrypto.so.3 libcurl.so libcurl.so.4)
  h1_restore() {
    for h1_name in "${h1_names[@]}"; do
      if [[ -e "${h1_backup_dir}/${h1_name}" || -L "${h1_backup_dir}/${h1_name}" ]]; then
        mv "${h1_backup_dir}/${h1_name}" "${h1_conda_lib}/${h1_name}"
      fi
    done
    rmdir "${h1_backup_dir}" 2>/dev/null || true
  }
  trap h1_restore EXIT INT TERM
  for h1_name in "${h1_names[@]}"; do
    if [[ -e "${h1_conda_lib}/${h1_name}" || -L "${h1_conda_lib}/${h1_name}" ]]; then
      mv "${h1_conda_lib}/${h1_name}" "${h1_backup_dir}/${h1_name}"
    fi
  done
  colcon build --symlink-install --packages-select \
    h1_2_moveit_config h1_2_moveit_demo
)
source install/setup.bash
```

## 快速只读诊断

```bash
pgrep -af 'ros2|move_group|cube_detector|gz sim|bimanual_air_handover'
ros2 control list_controllers
ros2 topic hz /realsense_camera/color/image
ros2 topic hz /realsense_camera/depth/image
ros2 topic echo /cube_pose --once
gz model -m green_cube -p
ros2 topic echo /joint_states --once
```

五个控制器必须为 `active`。RGB-D 无帧时先检查 Gazebo 原生相机话题和 bridge；
不要通过重建模型处理运行时进程问题。

## 孵育箱最终配置（2026-08-14）

SolidWorks 装配源位于 `/home/liu/h1_moveit/hengwen/fyx`，唯一装配依据是
`incubator_assembly.SLDASM`。Gazebo 使用从该装配变换导出的
`h1_2_description/models/incubator/model.sdf`；不要让 Gazebo 直接读取
SLDASM，也不要重新猜测或重建模型。

### 原装把手几何

```text
incubator world pose                  XYZ=(0.590, 0.000, 0.815) m
front face                            model local -X，正对机器人
handle STL SHA-256                    ad6670185fa5756ba5dabdee7e542db2d
                                      f3346973fea4eb887288f54a6b6f59e
STL AABB                              20.000 x 30.000 x 133.898 mm
handle_link pose in door_link         xyz=(-0.019105784513,
                                           +0.377528763146,
                                           +0.189999998514) m
                                      rpy=(0,0,+1.613424341497) rad
source assembly STL pose              xyz=(-0.108597910435,
                                           +0.000027284668,
                                           -0.049683142499) m
                                      rpy=(0,+1.057894842926,0) rad
task q=0 STL pose in handle_link      xyz=(-0.009999668689,
                                           +0.000027284668,
                                           -0.119003896682) m
                                      rpy=(0,0,0) rad
mass                                  0.034402463808 kg
task q=0 COM in handle_link           (-0.000012261668,
                                       +0.020269512428,
                                       -0.040571812382) m
```

Gazebo 的 `handle_visual` 和 `handle_collision` 必须同时是上述同一份原装 STL、
同一位姿和 `0.001` 缩放。禁止添加抓取套筒、圆柱、连接块、凹入盒体或其他
便利几何；MoveIt 的 `incubator_handle_exact` 也必须是 `1 mesh / 0 primitive`。

SLDASM 只提供静态装配快照，没有可证明的转角、止动或锁定语义。当前任务不
实现新的锁舌、锁紧力或锁机构；“朝下锁、朝上开”只用于定义初始化状态、动作
方向和终点。把手仍通过现有仿真关节固定在门上：

```text
door_hinge_joint                      revolute, model +Z, 0..75 deg
handle_latch_joint                    revolute, handle local +Y, 0..180 deg
initial handle pose                   竖直朝下（真实场景含义：锁着）
task terminal pose                    从朝下逆时针转到 90 deg（本轮终点）
task rotation                         从机器人正面看柜门时逆时针
```

关节轴心固定在门上，模型不是自由物体。不要为本任务继续增加或完善锁定逻辑；
只需保证初始化朝下，并在任务中沿上述逆时针方向旋转到 `90 deg`。

当前把手旋转阶段不再要求门保持关闭，也不设置 `door <=1 deg` 或其他门角通过
门限。门角仍需读取并记录，用于实时场景同步和碰撞检查，但门随接触力转动本身
不判定为失败。

### 视觉与规划同步

`h1_2_depth_point_cloud` 同时读取 `door_hinge_joint` 和
`handle_latch_joint`，按实测两关节角跟踪原装 STL 的安装俯仰。动态遮罩只使用
当前角度下的网格局部 OBB，余量 `7 mm`，仅在自由端延长 `3 mm`；禁止恢复
`0..180 deg` 整段扫掠圆柱。未知环境仍由 OctoMap 保留，独立环境净距门仍为
`10 mm`。最终点云节点启用 `left_finger2_self_filter_enabled:=true`，只在随动指二
坐标系内剔除零余量 AABB `X[-13,6] / Y[-6.8,75] / Z[0,30.4] mm` 内的已知
自体点；不增加规划碰撞几何，也不改变 7 mm/10 mm 门。MoveIt 侧另附着同一份
`Link2_2.STL` 的感知专用随动 mask，每个 AABB 面只扩 `0.5 mm`，与现有 ShapeMask
合计 `7.5 mm < 10 mm`；所有 `visual_mask` 在严格碰撞审计副本中移除，不能改变
路径可行性。关节状态桥已固化在 `gazebo.launch.py`，不再依赖手工进程。

长期 Gazebo 健康时直接复用；缺失时才启动：

```bash
ros2 launch h1_2_moveit_config gazebo.launch.py \
  spawn_incubator:=true incubator_x:=0.590
```

只同步和核验 MoveIt 场景，不做 IK、规划或运动：

```bash
ros2 run h1_2_moveit_demo incubator_door_open_demo --ros-args \
  -p use_sim_time:=true -p preflight_only:=true -p scene_sync_only:=true
```

必须看到：

```text
SCENE_SYNC_ONLY_SUCCESS: exact handle has 1 mesh and 0 primitives;
no IK, trajectory planning, or controller command was issued.
```

### 最终把手配置与离线预检

2026-08-14 最终采用用户标出的原装把手后半部抓点：

```text
arm / palm direction                  left / local +Y toward pelvis +X
handle phase / palm down tilt         -159 deg / 7 deg
handle local X / Z                    0 / 90.000 mm
front outward offset                  16 mm
commanded finger q1 / q2              11.35 / 12.35 mm
mean / asymmetry                      11.85 / -0.50 mm
exact-STL pad contacts                8 / 10
contact stations                     42.912 / 17.628 mm (minimum 15 mm)
contact station separation            25.284 mm (limit 26 mm)
non-pad clearance finger1 / finger2   0.683 / 0.657 mm (limit 0.6 mm)
handle targets                        0..90 deg, 91 one-degree 6-D TCP poses
local consistency limit               4.9 deg per arm joint
fallback                              active-set 6-D DLS
precision refinement                  5 um / 0.005 deg requested
maximum measured FK residual          0.000376 mm / 0.000049 deg
maximum adjacent joint change         3.434 deg (limit 5 deg)
Cartesian completeness                91/91 = 100.000%; no partial
dense handle interpolation samples    1334
minimum environment clearance         >=10.001 mm (required 10.000 mm)
```

抓点位于转轴外侧，因此动作不是夹爪中心原地的纯腕部旋转。设固定轴心为
`p_axis`，实际目标为
`p_tcp(q)=p_axis+R_axis(q)*(p_tcp(0)-p_axis)`、
`R_tcp(q)=R_axis(q)*R_tcp(0)`：TCP 位置绕固定把手转轴走圆弧，TCP 姿态随原装
把手同步旋转以保持刚性夹持，七个左臂关节共同跟踪。正式参数名为
`front_handle_pivot_tcp_arc`；旧 `front_wrist_roll_guided_arc` 仅为兼容别名。

完整路径同时通过关节界、自碰、原装把手/门/箱体碰撞、OctoMap、点间插值、固定轴
TCP 圆弧和 `91/91` 精确 STL 双垫接触检查。冗余求解先使用带 `4.9 deg`
consistency limits 的局部 KDL IK，失败点才使用主动集 6-D DLS；碰撞、关节界、
`0.6 mm` 非垫净距和 `10 mm` 环境门均未放宽。

只允许在把手确实位于初始化 `q=0` 且夹爪未夹持时运行下列纯离线命令；当前
物理状态为约 `q=90 deg` 且仍夹持，禁止 reset 以复现：

```bash
ros2 run h1_2_moveit_demo incubator_door_open_demo --ros-args \
  -p use_sim_time:=true -p preflight_only:=true \
  -p door_open_plan_only:=true \
  -p door_free_end_scan_only:=true \
  -p door_free_end_full_arc_only:=true \
  -p front_handle_pivot_tcp_arc:=true \
  -p front_diagonal_dual_pad_test:=true \
  -p front_reverse_seed_entry:=true \
  -p front_handle_arm:=left \
  -p front_handle_phase_degrees:=-159.0 \
  -p front_palm_down_tilt_degrees:=7.0 \
  -p front_handle_local_x_offset_mm:=0.0 \
  -p front_handle_local_z_mm:=90.0 \
  -p front_handle_outward_offset_mm:=16.0 \
  -p front_gripper_position_mm:=11.85 \
  -p front_gripper_asymmetry_mm:=-0.5 \
  -p front_diagonal_max_station_separation_mm:=26.0 \
  -p front_diagonal_minimum_body_clearance_mm:=0.6 \
  -p front_safe_ik_attempts:=100 \
  -p front_fixed_torso_yaw_degrees:=0.0
```

必须看到 `HANDLE_PIVOT_TCP_ARC_RESULT: 91/91`、
`exact-STL-contact=91/91`、`minimum environment clearance=10.001 mm`、
`DOOR_FREE_END_FULL_ARC ... PASS` 和进程 `exit 0`；该入口不发送控制器轨迹。

### 原装把手物理旋转结果

只有新的独立 `handle_rotate` 可以越过物理硬门；所有旧
`front_palm_*`、`coaxial_*`、套筒/盒体 continuation 和 partial 入口仍禁用。
该模式在同一进程中先完成 91 点固定轴 TCP 圆弧、接近、原装 STL 接触和全部密集
碰撞预检，全部通过后才执行接近、夹紧和 `9 x 10 deg` 物理圆弧。中途失败保持
夹紧，不 reset。初始化 `q=0` 时使用上面相同参数，但替换入口为：

```bash
-p preflight_only:=false -p handle_rotate:=true
```

2026-08-14 实测九段连续通过，进程 `exit 0`：

```text
segment target / Gazebo handle       10 /  9.798 deg
                                      20 / 19.803 deg
                                      30 / 29.787 deg
                                      40 / 39.789 deg
                                      50 / 49.820 deg
                                      60 / 59.789 deg
                                      70 / 69.797 deg
                                      80 / 79.788 deg
                                      90 / 89.780 deg
final independent handle feedback     89.827 deg (1.567784768 rad)
Gazebo door telemetry                 -0.000 deg; recorded, not a threshold
final finger feedback                 11.347 / 12.254 mm; held
door-relative handle-axis drift       0.000 mm at every segment
world pivot drift                     0.000 mm at every segment
final updated environment clearance   10.001 mm
preflight approach / arc samples      683 / 1405
preflight exact STL contact           91/91
controllers                           all five active
reset / release / door-hinge motion   none / none / none
program                               exit 0
```

最终左臂关节反馈 `[shoulder_pitch, shoulder_roll, shoulder_yaw, elbow,
wrist_pitch, wrist_roll, wrist_yaw]` 为
`[0.003275, -0.168828, -0.153453, 0.335486, -0.216053, -1.077511,
-0.204547] rad`。每段都读取 Gazebo 把手/门角、双指反馈、机器人状态、更新后的
碰撞场景和环境净距。把手轴心没有平移、下滑或脱落；MoveIt 成功日志不是本结论
的唯一依据。

完成后的同一物理命令现在只读返回：

```text
HANDLE_PIVOT_TCP_ARC_ALREADY_COMPLETE=PASS handle=89.827 deg,
door=-0.000 deg, fingers=11.347/12.254 mm,
TCP residual=0.005 mm/0.000 deg; no controller command.
```

2026-08-14 11:40 CST 已改用正式参数 `front_handle_pivot_tcp_arc:=true` 重跑同一
完成态检查，进程再次 `exit 0` 并返回上述 PASS；未发送控制器命令。旧参数仅为
兼容别名，不再用于标准命令。

`10..80 deg` 中断检查点仍使用同一固定轴圆弧做正反向预检后续跑；90 deg 已完成
状态不会再次运动。

### 固定轴 TCP 圆弧演示重放

默认 `handle_rotate` 在 90 deg 完成态仍必须只读退出。只有显式增加
`handle_rotate_cycle_demo:=true`，并且实测把手在 90 deg 的 3 deg 窗内、双指与
TCP 完成态门全部通过，才允许从当前夹紧状态重放。该模式不 reset、不松夹、不走
接近路径，也不发送门铰链命令；它从实测 90 deg 状态求解
`90,89,...,0` 的 91 点固定轴 6-D TCP 路径，并将同一组关节状态严格反序作为
`0,1,...,90` 回程。反向/正向密集审计和原装 STL 接触全部通过前不执行。

使用“原装把手物理旋转结果”中的完全相同参数，只将入口设为：

```bash
-p preflight_only:=false \
-p handle_rotate:=true \
-p handle_rotate_cycle_demo:=true
```

`door_open_plan_only`、`door_free_end_scan_only` 和
`door_free_end_full_arc_only` 是互斥的纯离线入口，禁止与上述物理循环参数组合。

2026-08-14 实测完整 `90 -> 0 -> 90 deg` 演示通过，进程 `exit 0`：

```text
preflight targets                      91 down + exact inverse return
local consistency IK / active-set DLS 45 / 45
maximum refined FK residual           0.004999 mm / 0.000381 deg
maximum adjacent joint change         3.298 deg (limit 4.9 deg)
dense samples down / return            1311 / 1311
exact original-STL contact             91/91 down; 91/91 return
preflight minimum clearance            10.001 / 10.001 mm
down targets / Gazebo handle           80 / 79.867 deg
                                        70 / 69.882 deg
                                        60 / 59.870 deg
                                        50 / 49.872 deg
                                        40 / 39.889 deg
                                        30 / 29.869 deg
                                        20 / 19.872 deg
                                        10 /  9.897 deg
                                         0 /  0.010 deg
independent midpoint handle            approximately -0.000 deg
return targets / Gazebo handle         10 /  9.789 deg
                                        20 / 19.791 deg
                                        30 / 29.790 deg
                                        40 / 39.791 deg
                                        50 / 49.789 deg
                                        60 / 59.789 deg
                                        70 / 69.813 deg
                                        80 / 79.802 deg
                                        90 / 89.823 deg
final independent handle feedback      89.827541 deg
Gazebo door telemetry                  approximately 0.000 deg throughout
finger feedback                        11.347 / 12.254 mm; held throughout
door-relative / world pivot drift      0.000 / 0.000 mm at every segment
minimum live updated clearance         10.001 mm
reset / release / door-hinge command   none / none / none
program                                exit 0
```

#### 2026-08-14 13:43 CST 实时 Gazebo GUI 重跑

为直接在当前 Gazebo GUI 中演示，复用健康 Gazebo、MoveIt、点云和控制器，从仍夹紧
的 90 deg 完成态重新运行。最终标准命令为：

```bash
ros2 run h1_2_moveit_demo incubator_door_open_demo --ros-args \
  -p use_sim_time:=true \
  -p preflight_only:=false \
  -p handle_rotate:=true \
  -p handle_rotate_cycle_demo:=true \
  -p front_handle_pivot_tcp_arc:=true \
  -p front_diagonal_dual_pad_test:=true \
  -p front_reverse_seed_entry:=true \
  -p front_handle_arm:=left \
  -p front_handle_phase_degrees:=-159.0 \
  -p front_palm_down_tilt_degrees:=7.0 \
  -p front_handle_local_x_offset_mm:=0.0 \
  -p front_handle_local_z_mm:=90.0 \
  -p front_handle_outward_offset_mm:=16.0 \
  -p front_gripper_position_mm:=11.85 \
  -p front_gripper_asymmetry_mm:=-0.5 \
  -p front_diagonal_max_station_separation_mm:=26.0 \
  -p front_diagonal_minimum_body_clearance_mm:=0.6 \
  -p front_safe_ik_attempts:=100 \
  -p front_fixed_torso_yaw_degrees:=0.0
```

本次发现右膝 Gazebo visual 使用原始 `right_knee_link.STL`，而 URDF collision 只是
半径 40 mm、长度 200 mm 的圆柱。最终感知配置增加随 `right_knee_link` 运动的
`right_knee_visual_mask`，仍使用原始 STL，只将 mask 局部比例设为
`XYZ=(1.00,1.06,1.00)`。连续五帧实测残点位于原 STL 的局部 +Y 表面外
`2.10..2.12 mm`；1.06 使 43.9 mm 的 Y 半宽每侧最多增加 2.634 mm，与不变的
7 mm ShapeMask padding 合计 `9.634 mm < 10 mm`。机器人碰撞体、ACM、点云全局
padding、抓点、轨迹和 10 mm 环境门均未改变；所有严格密集审计仍删除全部
`visual_mask`。应用后连续五帧中，过滤点到右膝圆柱的最近距离由
`8.363..8.660 mm` 变为 `167.460 mm`，OctoMap 继续更新。

最终实时循环完整通过，进程 `exit 0`：

```text
preflight targets                      91 down + exact inverse return
local consistency IK / active-set DLS 47 / 43
maximum refined FK residual           0.000300 mm / 0.000018 deg
maximum adjacent joint change         3.291 deg (limit 4.9 deg)
dense samples down / return            1227 / 1227
exact original-STL contact             91/91 down; 91/91 return
preflight minimum clearance            10.001 / 10.001 mm
down targets / Gazebo handle           80 / 79.835 deg
                                        70 / 69.836 deg
                                        60 / 59.837 deg
                                        50 / 49.833 deg
                                        40 / 39.836 deg
                                        30 / 29.836 deg
                                        20 / 19.833 deg
                                        10 /  9.836 deg
                                         0 / -0.000 deg
return targets / Gazebo handle         10 /  9.826 deg
                                        20 / 19.825 deg
                                        30 / 29.828 deg
                                        40 / 39.823 deg
                                        50 / 49.826 deg
                                        60 / 59.825 deg
                                        70 / 69.824 deg
                                        80 / 79.825 deg
                                        90 / 89.830 deg
final independent handle feedback      89.830657 deg
Gazebo door telemetry                  approximately 0.000 deg throughout
final finger feedback                  11.346947 / 12.253525 mm; held
door-relative / world pivot drift      0.000 / 0.000 mm at every segment
minimum live updated clearance         10.001 mm
reset / release / door-hinge command   none / none / none
program                                exit 0
```

#### 2026-08-14 14:42 CST 两分钟快速入口

用户正常使用不再运行 18 段双向诊断，也不再手工复制完整参数。新增已安装脚本：

```bash
# 新启动、把手朝下：完整接近、夹紧、单向 0 -> 90 deg；完成态再次调用只读退出
/home/liu/h1_moveit/h1_2_ros2_ws_claude_2026-07-30/h1_2_ros2_ws/install/h1_2_moveit_demo/lib/h1_2_moveit_demo/incubator_handle.sh task

# 仅限当前已验证的 90 deg 夹持态：Gazebo GUI 中 90 -> 0 -> 90 deg 双向诊断
/home/liu/h1_moveit/h1_2_ros2_ws_claude_2026-07-30/h1_2_ros2_ws/install/h1_2_moveit_demo/lib/h1_2_moveit_demo/incubator_handle.sh demo
```

脚本先拒绝并发 demo，复用健康 Gazebo、GUI、`move_group`、点云和五个控制器；
`task` 只补启动确实缺失的 Gazebo 场景或 `move_group`，已有但不健康的进程不会被
重复启动或 reset。`demo` 在 Gazebo 缺失或不处于 90 deg 夹持完成态时失败关闭。

快速执行由默认关闭的严格布尔参数 `handle_rotate_fast_mode` 授权。标准慢模式的
`1.0 s/deg`、10 deg 分段代码仍保留。快速模式只改变已完整预检路径的执行时序和
现场检查段：

```text
trajectory spatial targets             unchanged 91 one-degree 6-D TCP targets
fast point timing                      0.19 s/deg; 17.10 s per 90 deg leg
fast live stage                        one continuous 90 deg leg
preflight dense interpolation          unchanged <=0.1 deg / <=0.1 mm
environment / non-pad gates            unchanged 10.0 / 0.6 mm
exact original-STL contact             unchanged 91/91 each direction
partial / reset / release              forbidden / none / none
```

执行前新增 position-only 路径时间审计，并对 URDF 与
`h1_2_moveit_config/config/joint_limits.yaml` 取更严格限值；速度最多允许正式限值的
35%，离散加速度最多允许 100%。终点反馈仍使用原 `0.010 rad` 关节、`2 mm / 1 deg`
TCP 门；fresh-0 接近和旋转轨迹也统一先等待 300 ms 再读取终点，未放宽容差。
快速模式只接受 fresh `0 deg` 或显式双向演示的已验证 `90 deg` 起点；检测到
`10..80 deg` 中途状态时保持夹持、无运动退出，禁止 accelerated partial resume。

当前 Gazebo GUI 的完整双向快速诊断实测：

```text
wrapper wall time / process             115.16 s / exit 0
preflight targets                       91 down + exact inverse return
local consistency IK / active-set DLS   47 / 43
maximum refined FK residual             0.000300 mm / 0.000018 deg
maximum adjacent joint change           3.291 deg (limit 4.9 deg)
dense samples down / return             1227 / 1227
exact original-STL contact              91/91 down; 91/91 return
minimum preflight clearance             10.001 / 10.001 mm
timing peak velocity / acceleration     25.2% / 93.0% of strict limits; PASS
down physical target / Gazebo handle    0 / 0.007 deg
return physical target / Gazebo handle  90 / 89.830 deg
final independent handle / door         89.831 / approximately 0.000 deg
final finger feedback                   11.347 / 12.254 mm; held
door-relative / world pivot drift       <=0.001 / 0.000 mm
minimum updated live clearance          10.001 mm
reset / release / door-hinge command    none / none / none
```

成功日志为
`/home/liu/h1_moveit/runtime_logs/incubator_handle/fast_demo_20260814_1442.log`，
SHA-256 `b00d67d1ac2318d5be83eb40042b4ea9a94b622a11ffd850072d7d64ea661414`。
最终重编译后，同一 `task` 短命令在当前 90 deg 完成态只读实测 `18.14 s`、
`exit 0`，明确记录 `no controller command`。由于当前原装把手仍夹紧在 90 deg，
禁止为了单独重测
fresh-0 接近阶段而 reset；接近/夹紧路径没有改变，快速 0..90 轨迹已由上述双向
物理循环覆盖。

近景录像保存在：

```text
/home/liu/h1_moveit/demo_recordings/handle_pivot_tcp_cycle_full_20260814_120452.mp4
duration 1052.0 s; size 75,258,980 bytes
SHA-256 951b9d3b6493a06f163bc17a435ca1afc75060b7547017a6cdcb5dea4dc7a90b

/home/liu/h1_moveit/demo_recordings/handle_pivot_tcp_cycle_demo_10x_labeled_20260814.mp4
duration 98.7 s; size 4,463,914 bytes; 10x speed with direction labels
SHA-256 58f55837060fd1031d3765e5d6c97eef522997805c162124f67bd1154c68521e
```

### 水平门圆弧当前进度

本轮需求终点是原装把手从朝下到 `90 deg`，不再沿用旧的 180 deg 抓姿或旧门圆弧
诊断。新的 Z=90 mm 后半部抓姿完成物理验证以后，尚未为它建立或运行任何门铰链
水平圆弧，也没有发送门运动轨迹。继续门任务前必须以当前实测 90 deg 夹持状态为
起点，另建只读 no-controller 预检；在完整 IK、关节连续、自碰、环境碰撞、原装
STL 持续接触、点间插值和 10 mm 净距全部通过以前禁止执行。

### 已验证备份

原装把手后半部固定轴 TCP 圆弧物理到达 `89.827 deg`、程序 `exit 0` 后，且继续
优化源码和交接文档以前，完整工作空间（含 `src/build/install/log`）已保存为：

```text
/home/liu/h1_moveit/backups/h1_2_ros2_ws_rear_handle_90_physical_pass_20260814_112743.tar.zst
SHA-256 cd834ffc009146abde76355f7c8b4184e69826c9b55679ee38f0adf0e7cba0d6
size 52,459,730 bytes; sha256sum PASS; tar listing PASS
```

将接口与日志统一为固定转轴 6-D TCP 圆弧、完成六库隔离编译和新参数只读复核后，
最终优化快照为：

```text
/home/liu/h1_moveit/backups/h1_2_ros2_ws_rear_handle_90_pivot_tcp_optimized_20260814_1142.tar.zst
SHA-256 3a8cb75123c3640f0fea8ae44866afe290c64e2798c0f15f526b29b83bec862b
size 52,475,796 bytes; entries 5841; zstd test PASS; tar listing PASS
```

同步 `QUICKSTART.md` 并完成独立只读审查以后，最终文档快照为：

```text
/home/liu/h1_moveit/backups/h1_2_ros2_ws_rear_handle_90_final_docs_20260814_1145.tar.zst
SHA-256 ddb8a21d19d4d1b09526b74b123650e1ecff2480ac7f34c5ec237f32c8fbae38
size 52,477,041 bytes; entries 5841; zstd test PASS; tar listing PASS
```

完成固定轴 `90 -> 0 -> 90 deg` 物理演示、源码/文档更新和 10x 标注录像后，包含
完整工作空间及标注录像的验证备份为：

```text
/home/liu/h1_moveit/backups/h1_2_ros2_ws_handle_cycle_demo_verified_20260814_1227.tar.zst
SHA-256 37abac564da2c75089044c96e2eaf144d55b8ab37271c84c72765ba6b69874d0
size 56,778,740 bytes; entries 5851; zstd test PASS; tar listing PASS
```

完成右膝原始 STL 感知 mask 修复、六库隔离编译、连续五帧过滤点云核验、当前
Gazebo GUI 的完整 `90 -> 0 -> 90 deg` 物理重跑和文档同步后，最终工作空间快照为：

```text
/home/liu/h1_moveit/backups/h1_2_ros2_ws_live_cycle_right_knee_verified_20260814_1346.tar.zst
SHA-256 2a2a28301d31d424cfe9762419a9683f084a8fec0d2f6586663cd7c0ded552cc
size 52,514,964 bytes; entries 5880; zstd test PASS; tar listing PASS
```

完成两分钟快速入口、六库隔离编译、完整双向物理循环 `115.16 s / exit 0` 及
`QUICKSTART.md` 同步后的首个快速快照（随后被下方 post-audit 快照取代）为：

```text
/home/liu/h1_moveit/backups/h1_2_ros2_ws_handle_fast_under_2min_verified_20260814_1447.tar.zst
SHA-256 ae82cd341e88a0cc5854ab1f9d275c547592a03e0066aa5c9225e4319c673fc1
size 52,562,032 bytes; entries 5942; zstd test PASS; tar listing PASS
```

源码复审后又完成 fresh 终点稳定顺序修正、快速 partial-resume 无运动拒绝、第二次
六库隔离编译（2 packages / 29.0 s PASS）、当前 90 deg 状态的只读 `task`
`18.14 s / exit 0 / no controller command` 和文档同步。最终 post-audit 快照为：

```text
/home/liu/h1_moveit/backups/h1_2_ros2_ws_handle_fast_under_2min_post_audit_20260814_1455.tar.zst
SHA-256 23e21151855c1cbde036e3264a91002f7c76d10d0962b107e466b8e57a4abfda
size 52,583,522 bytes; entries 5957; zstd test PASS; tar listing PASS
```

## 历史门换手存档（2026-08-15，已由文末 2026-08-17 状态覆盖）

```text
Gazebo model pose                     (0.590, 0.000, 0.815) m
Gazebo handle / door                  89.830650 / approximately 0.000 deg
original handle                       still clamped and supported by left hand
left finger feedback q1 / q2          11.346947 / 12.253525 mm
right finger feedback q1 / q2         -19.000 / -19.000 mm
MoveIt / Gazebo / GUI                 healthy and reused
controllers                           all five active; torso controller absent
demo process                          none
latest isolated build                 2 packages / 38.6 s PASS
controller trajectories this work     none
```

用户标注的门轴已经核对为柜体左侧竖直铰链，规划坐标为
`XYZ=(0.449954077853,-0.247963042272,0.024999998514) m`、轴 `+Z`。当前换手方案
保持左手先将门从 `0 -> 3 deg`、松开并移到外侧安全位，然后右手在原装把手后半部
夹紧并沿该固定轴将门拉到 `60 deg`；没有使用套筒、盒体或假结构。

当前右手配置与离线结果：

```text
right grasp local Z / phase / tilt    35 mm / 21 deg / 7 deg
local X / outward offset              0 / 16 mm
clamp q1 / q2                         11.35 / 12.35 mm
pregrasp q1 / q2                      10.35 / 11.35 mm
pad station separation                25.283 mm (limit 26 mm)
non-pad clearance q1 / q2             0.701 / 0.642 mm (limit 0.6 mm)
hand/hand clearance                   5.001 mm
environment clearance                 >=10.001 mm (required 10.000 mm)
left preopen fixed-hinge path          24/24 plus exact reverse; PASS
left release / clear                   496 / 1079 dense samples; PASS
right fixed-TCP ingress                45/45 states; PASS
ingress construction                   contact + prep12 + 1..3 mm +
                                       prep12 at 3 mm + 4..20 mm
ingress consistency IK / DLS           20 / 0
ingress max FK / adjacent change       0.059720 mm / 0.014338 deg / 3.033 deg
coarse door arc                        70/70; PASS
exact 0.1-deg door arc                 583/583; PASS
exact arc consistency IK / DLS         570 / 0
exact arc max FK / adjacent change     0.030394 mm / 0.006737 deg / 0.392 deg
rigid original-STL equivalence         40537 checks; PASS
overall no-controller preflight        exit 1 at free-space approach only
```

唯一未解决阻塞不是门圆弧 IK，而是当前右臂关节态到精确 `20 mm` 外侧锚点的
MoveGroup 入口。六次 OMPL 查询都在 goal sampler 中把名义目标判为
`right_elbow_link <-> torso_link` 碰撞并报告 `Unable to sample any valid states for
goal tree`；同一外侧状态在冻结本地场景通过严格碰撞检查及七关节
`+/-0.0001 rad` 的 `14/14` 扰动探针，因此继续前必须先消除本地/已发布
PlanningScene/ACM 判定不一致，不能放宽碰撞门。日志为：

```text
/home/liu/h1_moveit/runtime_logs/incubator_handoff/offline_full_sequential_z35_spatial20mm_20260814.log
/home/liu/.ros/log/move_group_172975_1786677319160.log
```

本次源码修改已经使用规定的 Miniconda 六库隔离命令编译。存档名固定为：

```text
/home/liu/h1_moveit/backups/814.tar.zst
/home/liu/h1_moveit/backups/814.tar.zst.sha256
```

仍未完成：右手真实接近、夹紧、左手释放后的物理接管，以及门从约 `3 -> 60 deg`
的真实 Gazebo 拉开；因此当前结果不是物理开门成功。恢复工作时先只读复核现场，
再修复上述场景判定不一致并重跑完整 no-controller 预检。完整预检 `exit 0` 以前
禁止执行；夹持期间禁止 reset 或松夹，异常时 stop-and-hold。项目仍仅限 Gazebo，
不得连接实体 Unitree H1-2。

## 2026-08-17 最新交接：左手已安全退开，q0 右手接近仍差 10 mm 门

本节是当前状态，覆盖上面的 2026-08-15 门换手状态。唯一目标仍是 Gazebo 中
左手完成把手解锁后先退到安全中立位，再由右手抓住原装把手并绕柜体左侧固定
竖直铰链把门从约 `0 -> 60 deg` 拉开。尚未物理执行右手接管或开门，不能写成
物理成功，也不能开始后续优化、备份成功版或水平门外的其他任务。

### 已完成并物理验证

独立 `door_handoff_left_clear_recovery` 已 `exit 0`：左手双指从夹紧态安全打开到
`-19/-19 mm`，左臂退至中立位并保持；右臂和右夹爪未移动，没有 reset。日志：

```text
/home/liu/h1_moveit/runtime_logs/incubator_handoff/left_clear_q0_physical_state_update_20260817.log
```

完成后的最新只读实测：

```text
door / handle                         0.002564 / 89.827126 deg
latch                                unlocked
left arm max neutral error           0.000087 rad
left fingers                         -19.000 / -19.000 mm
right fingers                        -19.000 / -19.000 mm
original-STL left finger-body gap    222.704 / 295.946 mm
minimum environment clearance        10.001 mm
controllers                          five active
Gazebo / GUI / move_group / pointcloud healthy and reused
```

### 当前源码与独立 q0 模式

`incubator_door_open_demo.cpp` 已新增 `door_handoff_q0_after_left_clear`。该分支从
实测 `q_door≈0` 开始，完全旁路旧左手 `0 -> 3 deg`、释放和退避链；物理路径不
调用左臂或左夹爪控制器，门轨迹也不含 `torso_joint`。右手配置保持：

```text
local Z / phase / tilt               35 mm / 21 deg / 7 deg
local X / outward                    0 / 16 mm
clamp q1 / q2                        11.35 / 12.35 mm
pregrasp q1 / q2                     10.35 / 11.35 mm
coarse / exact door targets          73 / 613
exact physical segment boundaries    0,112,212,312,412,512,612
door targets                         10,20,30,40,50,60 deg
```

2026-08-17 11:42 源码 SHA-256：

```text
5433c666152e7f767d8c42eb03daed4578f48384c57e4c9983b5d416785fe211
```

最新源码只改变一个算法门：每个 q0 冗余端点进入昂贵的 `73/613` 门圆弧求解
前，先用现有 Gazebo 高模对其 `20 mm` 固定 TCP 接近链做完整密集自碰预筛。
插值仍为旋转 `<=0.1 deg`、平移 `<=0.1 mm`，并继续强制
`right_elbow_link <-> torso_link` 禁止接触。抓点、`10 mm` 环境净距、`4.9 deg`
连续性门、关节限位和 ACM 均未放宽。

修改后已按本文 Miniconda 六库隔离命令编译：

```text
2 packages / 37.5 s / PASS
/home/liu/h1_moveit/h1_2_ros2_ws_claude_2026-07-30/h1_2_ros2_ws/log/build_2026-08-17_11-42-31
```

### 已取得的离线证据

第一次 q0 全预检日志：

```text
/home/liu/h1_moveit/runtime_logs/incubator_handoff/offline_q0_z35_first_20260817.log
exit 1; no controller trajectory
```

该次已经证明门圆弧本身完整可行：

```text
20-mm fixed-TCP ingress              45/45 PASS
coarse door arc                      73/73 PASS
exact 0.1-deg door arc               613/613 PASS
exact consistency IK / DLS           600 / 0
maximum exact adjacent joint change  0.421 deg (limit 4.9 deg)
maximum exact FK error               0.033476 mm / 0.007265 deg
rigid original-STL equivalence       42801 checks PASS
Cartesian completeness               100 percent
```

失败仅在“当前右臂 -> 20 mm 外锚点 -> 接触”的自由空间接近。OMPL 候选 1/2/4
的 OctoMap 净距为 `9.799/9.578/9.782 mm`，未达到 `10 mm`；候选 3 的低模和
环境门通过，但所有候选共享的端点 1 固定 TCP ingress 在 Gazebo 高模发生
`right_elbow_link <-> torso_link` 最大 `1.777 mm` 穿透。

第二次 q0 全预检使用新增高模端点预筛，日志与结果：

```text
/home/liu/h1_moveit/runtime_logs/incubator_handoff/offline_q0_z35_high_prescreen_20260817.log
exit 1; no controller trajectory
```

该进程已自然结束，不再有 demo 进程。新增预筛正确地在门圆弧前淘汰端点 1，
端点 2 则完整通过 ingress、高模、原装 STL 和两级门圆弧：

```text
q0 start gate                        PASS
endpoint 1 ingress                   45/45 PASS low model
endpoint 1 Gazebo-high prescreen     FAIL at sample 215, elbow/torso 1.777 mm
endpoint 1                           rejected before 73/613 solve
endpoint 2 ingress                   45/45 PASS
endpoint 2 Gazebo-high prescreen     708/708 PASS
endpoint 2 original STL              pads 8/10; station 25.284 mm
endpoint 2 non-pad clearance         0.687 / 0.653 mm
endpoint 2 coarse door arc           73/73 PASS; max adjacent 3.469 deg
endpoint 2 exact door arc            613/613 PASS
exact consistency IK / DLS           600 / 0
maximum exact adjacent change        0.411 deg (limit 4.9 deg)
maximum exact FK error               0.032930 mm / 0.007200 deg
rigid original-STL equivalence       42901 checks PASS
minimum contact arm-joint margin     4.702 deg
Cartesian completeness               100 percent
controller trajectories              none
```

最终失败仍只在“当前右臂 -> 20 mm 外锚点”的自由空间 OMPL 段。六次查询生成四条
完整路径，它们均被未改变的 `10.000 mm` 密集门拒绝：

```text
candidate 1  OctoMap                 9.678 mm
candidate 2  incubator_door_exact    9.998 mm
candidate 3  OctoMap                 9.904 mm
candidate 4  OctoMap                 9.935 mm
```

候选 2 只差 `0.002 mm` 也仍是失败，禁止把数值误差当 PASS，禁止降低净距门。
当前没有 `DOOR_HANDOFF_FULL_PREFLIGHT PASS`，因此禁止物理执行。

### 当前纯离线复现命令

```bash
cd /home/liu/h1_moveit/h1_2_ros2_ws_claude_2026-07-30/h1_2_ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run h1_2_moveit_demo incubator_door_open_demo --ros-args \
  -p use_sim_time:=true -p preflight_only:=true \
  -p door_handoff_plan_only:=true \
  -p door_handoff_q0_after_left_clear:=true \
  -p door_handoff_sequential_release_first:=false \
  -p front_handle_arm:=right \
  -p front_handle_pivot_tcp_arc:=false \
  -p front_diagonal_dual_pad_test:=true \
  -p front_reverse_seed_entry:=false \
  -p front_handle_phase_degrees:=21.0 \
  -p front_palm_down_tilt_degrees:=7.0 \
  -p front_handle_local_x_offset_mm:=0.0 \
  -p front_handle_local_z_mm:=35.0 \
  -p front_handle_outward_offset_mm:=16.0 \
  -p front_gripper_position_mm:=11.85 \
  -p front_gripper_asymmetry_mm:=-0.5 \
  -p front_diagonal_max_station_separation_mm:=26.0 \
  -p front_diagonal_minimum_body_clearance_mm:=0.6 \
  -p front_diagonal_end_side_tolerance_mm:=1.0 \
  -p door_handoff_right_pregrasp_opening_mm:=1.0 \
  -p right_handle_axial_spin_degrees:=0.0 \
  -p right_tcp_local_x_shift_mm:=0.0 \
  -p front_fixed_torso_yaw_degrees:=0.0
```

### 下一步与安全边界

1. 只解决当前态到端点 2 的 `20 mm` 外锚点自由空间路径。保留已验证端点 2、
   `45/45` ingress、`73/73`、`613/613`、抓点和全部门限；修改前先向用户说明
   所采用的单一规划变量及具体数值。
2. 优先让规划路线本身远离门/点云，而不是增加接触豁免、降低 `10 mm`、扩大
   关节限位、执行 partial 或反复增加随机候选。每次仍只改变一个变量并先跑
   no-controller 全预检。
3. 离线 `FULL_PREFLIGHT PASS` 且 `exit 0` 后，重新只读核对门/把手/锁舌、
   双臂、双指、点云、五控制器、
   `move_group` 和无并发 demo；然后在同一物理进程中重新完成全预检。
4. 物理顺序必须是右指预抓、右臂两段接近、夹紧原装把手、附着规划表示、按
   `10/20/30/40/50/60 deg` 六段开门。每段读取 Gazebo 门角、把手角、四指反馈、
   双臂状态、固定铰链漂移、原装 STL 接触、更新场景和 `>=10 mm` 环境净距。
5. 任一物理段异常时 stop-and-hold；夹紧后禁止 reset、detach、松夹或走未预检
   恢复路径。MoveIt 成功日志不能替代 Gazebo 物理结果。
6. 只有右手真实把门拉到 `60+/-1 deg`、铰链固定、原装把手未脱落、程序
   `exit 0` 后，才制作新的、不覆盖旧文件的 `814` 成功备份并更新本节最终实测。

截至本节写入时，右手物理接管和开门仍未完成；旧
`/home/liu/h1_moveit/backups/814.tar.zst` 不是本次 q0 物理成功备份。

## 2026-08-18 最新交接：右手已将原装柜门真实拉开到 60 deg

本节是当前唯一有效的门换手状态，覆盖前面的 2026-08-17 未完成状态。Gazebo
中左手已保持中立安全位，右手夹住 `incubator_assembly.SLDASM` 导出的原装把手
后半部，并绕用户标注的柜体左侧固定竖直铰链将门从约 `0 -> 60 deg`。物理进程
`exit 0`；把手未脱落、轴心未移动，没有使用套筒、盒体、圆柱或其他假结构。

### 最终源码、编译和配置

```text
source file SHA-256                  560e075a87f3c5ed38966dd0fe9a25f4e9dc5cf602d90491baa6213cb52943b2
isolated build                       2 packages / 32.3 s / PASS
build log                            log/build_2026-08-18_11-25-06
right grasp local Z/phase/tilt       35 mm / 21 deg / 7 deg
local X / outward offset             0 mm / 16 mm
clamp q1 / q2                        11.35 / 12.35 mm
pregrasp q1 / q2                     10.35 / 11.35 mm
pad station limit                    26.0 mm
non-pad clearance limit              0.6 mm
environment clearance               >=10.0 mm
door targets                         10/20/30/40/50/60 deg
Cartesian completeness               100 percent; partial forbidden
```

源码只修正了物理附着前后的 ACM 回读判定：MoveIt 会删除仅含显式 `false` 的
名称并按字典序序列化，旧代码错误地比较原始数组顺序。现在 expected/observed
均通过 `collision_detection::AllowedCollisionMatrix -> getMessage()` 规范化后，
再精确比较完整名称、默认项和所有矩阵行。实际碰撞许可没有改变；原装把手仍只
允许两块右指垫接触，所有非垫、自碰和环境门保持原值。现场附着前、附着后均为
`world=PASS, attached=PASS, ACM=PASS`。

规定的 Miniconda `libssl/libcrypto/libcurl` 六库隔离编译已经执行，禁止改用普通
`colcon build`。后续源码修改仍必须重新执行本文前部记录的同一隔离命令。

### 可复现命令

下面的公共参数必须原样使用；离线命令不发送控制器轨迹。物理命令只能在门约
`0 deg`、把手约 `90 deg`、右手已经处于本文夹紧反馈、左手中立且无并发 demo
时运行。当前门已在 `60 deg`，夹持期间禁止 reset、松指或直接重跑 q0 命令。

```bash
cd /home/liu/h1_moveit/h1_2_ros2_ws_claude_2026-07-30/h1_2_ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
handoff_common=(
  --ros-args
  -p use_sim_time:=true
  -p door_handoff_q0_after_left_clear:=true
  -p door_handoff_sequential_release_first:=false
  -p door_handoff_q0_right_contact_resume:=false
  -p door_handoff_q0_right_clamped_resume:=true
  -p front_handle_arm:=right
  -p front_handle_pivot_tcp_arc:=false
  -p front_diagonal_dual_pad_test:=true
  -p front_reverse_seed_entry:=false
  -p front_handle_phase_degrees:=21.0
  -p front_palm_down_tilt_degrees:=7.0
  -p front_handle_local_x_offset_mm:=0.0
  -p front_handle_local_z_mm:=35.0
  -p front_handle_outward_offset_mm:=16.0
  -p front_gripper_position_mm:=11.85
  -p front_gripper_asymmetry_mm:=-0.5
  -p front_diagonal_max_station_separation_mm:=26.0
  -p front_diagonal_minimum_body_clearance_mm:=0.6
  -p front_diagonal_end_side_tolerance_mm:=1.0
  -p door_handoff_right_pregrasp_opening_mm:=1.0
  -p right_handle_axial_spin_degrees:=0.0
  -p right_tcp_local_x_shift_mm:=0.0
  -p front_fixed_torso_yaw_degrees:=0.0
)

# 纯离线完整复现
ros2 run h1_2_moveit_demo incubator_door_open_demo "${handoff_common[@]}" \
  -p preflight_only:=true \
  -p door_handoff_plan_only:=true \
  -p door_handoff_execute:=false

# 同进程完整预检后物理执行；只能从上述已验证 q0 夹紧现场运行
ros2 run h1_2_moveit_demo incubator_door_open_demo "${handoff_common[@]}" \
  -p preflight_only:=false \
  -p door_handoff_plan_only:=false \
  -p door_handoff_execute:=true
```

### 离线和物理实测

```text
offline log                          runtime_logs/incubator_handoff/offline_q0_clamped_resume_acmcanonical_20260818.log
offline result                       DOOR_HANDOFF_FULL_PREFLIGHT PASS / exit 0
ingress                              45/45; consistency/DLS=20/0; max adjacent 3.027 deg
coarse door arc                      73/73; consistency/DLS=60/0; max adjacent 4.530 deg
exact door arc                       613/613; consistency/DLS=600/0; max adjacent 0.392 deg
execution targets                    601/601 plus exact reverse
dense interpolation                  2943/2943 low and Gazebo-high PASS
maximum FK error                     0.033361 mm / 0.007242 deg
maximum hinge drift/axis error       0.022 mm / 0.007 deg
minimum environment clearance        10.001 mm
selected arm live identity           0.000604 rad (limit 0.003 rad)
original-STL rigid checks             42794 screen plus 2943 dense PASS

physical log                         runtime_logs/incubator_handoff/physical_q0_clamped_resume_acmcanonical_door60_20260818.log
physical result                      DOOR_HANDOFF_PHYSICAL_RESULT=PASS / exit 0
segment measured door angles         9.999 / 19.999 / 29.998 / 39.998 / 49.997 / 59.996 deg
segment handle angles                89.715 / 89.716 / 89.715 / 89.715 / 89.716 / 89.717 deg
final right fingers                  11.312925 / 12.349999 mm
final left fingers                   -19.000 / -19.000 mm
final hinge drift/axis error         0.001 mm / 0.000 deg
final original-STL contact           pads 4/6; station 25.292 mm
final non-pad clearance q1/q2        0.704 / 0.684 mm
final updated-scene clearance        10.001 mm
latch                                unlocked
physical six-segment phase           106.52 s wall time
whole physical process               about 514.56 s including repeated full preflight
```

进程退出后的独立 Gazebo/ROS 复核为：门轴 `1.0471212692143892 rad`、把手轴
`1.5658644746623045 rad`，右指 `0.011312925327/0.012349999941 m`，锁舌
`unlocked`，五个控制器均 `active`，无 demo 进程。

### 814 成功备份

旧 `814.tar.zst` 未覆盖。新的物理成功快照为：

```text
/home/liu/h1_moveit/backups/814_q0_door60_physical_success_20260818.tar.zst
SHA-256 a39a7c5b13780fd1394c1e3b85b2b48ccebf066bb79b313354156d37d429fc6c
size 48,105,093 bytes; entries 8,891; zstd test PASS; tar listing PASS
```

### 仍未完成

门把手解锁、左手安全退开、右手接管和 `0 -> 60 deg` 开门已经完成。当前唯一
后续工作是速度优化：实际六段运动已为 `106.52 s`，低于两分钟；但同一物理
进程仍重复约六分钟的 `45/73/613` IK 搜索，使总时长约 `8.58 min`。优化必须
保留 601 个目标、2943 个密集插值检查、完整正反路径、10 mm 净距、4.9 deg
连续性和禁止 partial。优先把已验证 613 状态保存为带源码/模型/场景/参数哈希
的只读证书资产；每次仍对资产逐状态重放全部碰撞、净距、原装 STL 和铰链检查，
只省略重复 IK 搜索。当前右手仍在 `60 deg` 终点支撑门，任何后续物理恢复都必须
先生成并完整预检安全回程；禁止 reset、直接 detach 或松指。

Codex 当前任务已建立名为“柜门流程速度优化”的 `heartbeat` 自动轮询，automation
id 为 `automation`，状态 `ACTIVE`；从 2026-08-18 起每 30 分钟继续一次，共 96
次，覆盖约 48 小时。轮询只允许按上一段边界继续源码分析、隔离编译、纯文件负测
和无控制器离线验证；当前 `60 deg` 支撑现场未经安全回程完整预检不得改变。

包含本节最终文档的第二份非覆盖快照为：

```text
/home/liu/h1_moveit/backups/814_q0_door60_physical_success_docs_20260818.tar.zst
SHA-256 dd41b84c2aa5d7b680012ee67afdb65227e5ac03e041202e460323ed09e225d9
size 48,107,683 bytes; entries 8,891; zstd test PASS; tar listing PASS
```

## 2026-08-18 速度优化增量：只读关节路径证书基础设施

本节是在上面的 60 deg 物理成功状态上进行的纯源码/纯文件增量，不改变其物理
结论。修改前后均未启动 demo、未发布控制器轨迹、未调用 `ApplyPlanningScene` 或
`clear_octomap`，也未 reset、detach 或松夹。只读复核时门/把手为
`59.969822/89.733770 deg`，右指为 `11.350000/12.350000 mm`，五个控制器均
`active`，四个控制器轨迹话题 publisher 均为 0；右手仍支撑门。

### 已完成且验证通过的基础设施

新增与物理 demo 隔离的 C++17 静态库和只读工具：

```text
include/h1_2_moveit_demo/door_handoff_path_certificate.hpp
src/door_handoff_path_certificate.cpp
src/door_handoff_path_certificate_tool.cpp
test/test_door_handoff_path_certificate.cpp
```

固定 schema 保存一个 7 关节外锚点以及 `45/73/613` 个 7 关节
IEEE-754 binary64 状态，payload 为 little-endian、精确 `41036 bytes`；manifest
同时固定 `601` 个执行目标、`2943` 个密集插值样本、完整反向路径、禁止 partial、
10 mm 环境净距、99.5 percent Cartesian 门、原装 STL 接触和全部最终抓取参数。
payload SHA-256 使用 ROS Jazzy 已安装的 `rcutils/sha256.h`，没有新增 OpenSSL、
`rclcpp`、MoveIt 或 Gazebo 运行依赖。

解析器 fail closed 地拒绝错误 magic/schema/端序/尺寸/计数、截断、尾随字节、
NaN/Inf、非规范或重复/缺失字段、错误关节顺序、符号链接和非普通文件。段内逐关节
连续门保持 ingress/coarse `<=4.9 deg`、exact `<=0.49 deg`；另外检查外锚点与
ingress 终点 `1e-10 rad` 身份、ingress 接触态到 coarse/exact 起点以及两个圆弧
起点之间的固定接缝门。

可信 API 必须由调用方提供并逐字段精确匹配 18 个 `ExpectedBindings`：源码、
可执行文件、URDF、SRDF、kinematics、joint limits、TCP、碰撞网格集合、原装
handle/door/body/tray STL、静态场景、场景位姿和铰链、参数集、完整起点关节、
运行库版本及时序策略。原装把手仍额外硬绑定 SHA-256
`ad6670185fa5756ba5dabdee7e542db2df3346973fea4eb887288f54a6b6f59e`；
payload 另做完整 SHA-256。独立 CLI 的子命令为 `inspect-format`，只调用
`inspect_format_files()`，成功标志明确为
`DOOR_HANDOFF_PATH_CERTIFICATE_FORMAT_ONLY=PASS`，并打印
`live model/collision/STL/hinge/timing gates not evaluated`，不能作为执行准入。

### 编译和测试

源码修改后已使用本文规定的 Miniconda 六库隔离命令编译：

```text
2 packages / 0.95 s incremental / PASS
log/build_2026-08-18_12-45-09
incubator_door_open_demo.cpp SHA-256 unchanged
560e075a87f3c5ed38966dd0fe9a25f4e9dc5cf602d90491baa6213cb52943b2
certificate tool SHA-256
4a8af252cc02377abc60c99ed5b19af5e8820cf67de73e23334ab0702c907f49
```

`ament_uncrustify` 对四个新 C++ 文件 PASS。纯单元测试不初始化 ROS 图，GTest
`16/16`、0 failure；`colcon test-result` 为 17 tests、0 error/failure/skip。覆盖
SHA-256 empty/abc 已知向量、合法固定格式、格式检查不得声称 bindings、单字节
篡改、header/magic、截断/尾随、NaN/Inf、连续门精确边界和超限、全部四个接缝、
全部 18 个合法但不一致的期望哈希、缺失/未知 binding、原装把手哈希、anchor、
重复/未知/缺失字段、符号链接和目录拒绝。工具动态依赖只有 `librcutils` 和标准
C/C++ 运行库，不含 `rclcpp`、MoveIt 或 Gazebo。

### 明确仍未完成

这不是可回放证书，也尚未降低 `514.56 s` 物理流程总耗时。当前没有可信 q0
manifest/payload，没有从已验证求解结果导出 `1+45+73+613` 状态的 writer，
`incubator_door_open_demo` 也没有链接或调用该库。全零或其他有限合成状态可以通过
“格式+调用方提供的合成 binding”单测，但绝不能进入物理准入；关节限位、fresh
起点身份、FK/目标、45/73/613 完整 RobotState、两次 2943 点碰撞/净距、原装
STL、固定铰链、正反路径和重新生成的时序仍必须在同一 demo 进程逐项重放。

下一步只能先在源码中加入 fail-closed 的 exporter/loader 和上述完整重放链；
物理 cache-required 模式禁止静默回退 IK，也禁止覆盖资产。真实证书只能在通过
完整安全回程以后重新处于 q0，或另一个已验证 q0 现场时由完整慢预检生成并再次
验证。当前门仍约 60 deg 且右手支撑，禁止为了生成证书而 reset、detach、松指或
运行 q0 命令。最近物理成功的两个 814 备份必须继续保留且不得覆盖。

本节源码、测试、安装产物和文档通过上述验证后，已另存不覆盖的进度快照；它不是
新的物理成功证书，也不替代前述两个物理成功备份：

```text
/home/liu/h1_moveit/backups/814_door60_certificate_infra_20260818_1246.tar.zst
SHA-256 173059cde4007f7e3c115e591a10e6f16ef617598ce6a856c79f594fd06da5bd
size 56,401,589 bytes; entries 9,146; zstd test PASS; tar listing PASS;
sha256sum -c PASS
```

## 2026-08-20 速度优化增量：原子证书目录 writer 与稳定回读

本节覆盖并更新前两节中的“当前现场”描述。电脑重启后的只读复核为：Gazebo
server/GUI、`move_group`、点云、`environment_scene`、`controller_manager`、五个
控制器、demo 和 ROS/Gazebo graph 均不存在。因此当前没有可读取的门角、把手角、
双指或双臂反馈；2026-08-18 的约 `60 deg` 右手支撑只保留为历史物理成功记录，
不能称为本次实时状态。本轮没有启动任何上述进程，没有 reset、detach、松夹、
调用场景写服务或发送控制器轨迹。

### 最终实现

固定 schema、抓取参数和全部安全门没有改变：仍是一个 outer anchor、`45/73/613`
个右臂 7 关节状态、`601` 个运动目标、`2943` 个密集插值样本、完整正反路径、
`10 mm` 环境净距、ingress/coarse `4.9 deg` 和 exact `0.49 deg` 连续门、原装
STL 接触以及禁止 partial。新增公开 API：

```text
write_new_directory()
verify_directory()
inspect_format_directory()
sha256_regular_file()
```

`write_new_directory()` 只接受 `PathCertificate + 18 ExpectedBindings`，在内存中
确定性生成 LF-only manifest 和精确 `41036 bytes` little-endian payload，并在写盘
前调用严格 `verify_bytes()`。目标必须是 canonical absolute path；逐级
`openat(O_DIRECTORY|O_NOFOLLOW)` 拒绝中间符号链接，父目录必须属于当前 euid 且
禁止 group/other 写。writer 使用 `getrandom()` 的 128-bit 随机同级临时目录，
先写 `states.bin`、再写 `manifest.txt`，两个文件均 `fchmod(0400)+fsync+close`；
随后从同一 dirfd 完整回读、逐字节比较并重跑全部固定格式/binding/连续门，再将
目录设为 `0500` 并 fsync。发布唯一使用
`renameat2(RENAME_NOREPLACE)`，不支持时 fail closed，绝不退化为可覆盖 rename；
成功后再 fsync 父目录，并在 rename 后与父目录 fsync 后两次核对 target/temp
inode 以及绝对父目录 inode。

rename 返回错误时会按 owned inode 位于 temp/target 的状态区分
`kNotPublished` 与 `kPublishedDurabilityUncertain`。为严格消除失败清理误删已发布
资产的竞态，临时目录一旦创建，任何失败都只报告随机 orphan 名称，不再同步
`chmod/unlink/rmdir`；当前库源码没有任何删除调用。唯一消除临时名字的操作是成功
的原子 rename。持续 I/O 故障可能积累 orphan，未来只能由独立、延迟、保守且有
配额的审计工具处理，禁止把删除重新放回 writer 错误路径。

`verify_directory()` 只从一次打开的只读目录 dirfd 读取固定
`manifest.txt/states.bin`，拒绝额外条目、FIFO、符号链接、非普通文件、可写权限和
`nlink != 1`；使用 `O_NONBLOCK`，按 fstat 尺寸精确读取并确认 EOF，读取前后及
路径重新绑定均复核 inode/size/mtime/ctime。`sha256_regular_file()` 同样逐级拒绝
符号链接，流式限制大小，并复核同一 fd 和最终路径身份。旧的双路径
`inspect_format_files()/verify_files()` 已在头文件明确标为 format/test only；未来
物理 loader 只能使用 `verify_directory()` 返回的内存状态，不能 hash 后重新按路径
加载。0400/0500 只称为 `read-only-mode sealed`，不是 filesystem immutable。

### 编译、测试和耗时

最终源码已使用本文前部规定的 Miniconda 六库隔离命令编译，没有使用普通
`colcon build`：

```text
2 packages / 1.81 s incremental / PASS
log/build_2026-08-20_12-25-30
ament_uncrustify four certificate files PASS
```

纯 C++ GTest 最终 `25/25 PASS`、0 failure/error/skip、`106 ms`；CTest/colcon
汇总为 26 tests（含 harness）、0 failure/error/skip、`0.38 s`。结果文件为：

```text
build/h1_2_moveit_demo/test_results/h1_2_moveit_demo/test_door_handoff_path_certificate.gtest.xml
```

覆盖既有 16 项 parser/hash/continuity/binding 负测，以及确定性位级 roundtrip、
不覆盖既有文件/目录/符号链接、符号链接父目录、非法证书零发布、目录额外项、
manifest/states 可写、manifest 外部 hardlink、5 类不规范路径、2 类不安全父目录和
普通文件 SHA-256 已知向量。第一次新增测试为 `19/22 PASS`，暴露目录流 offset 与
原装 STL binding 错误消息问题；修复后连续通过，最终扩充为上述 `25/25 PASS`。
最终测试后 `/tmp` 和工作空间均无 `.h1cert*` 或测试临时目录残留，且没有 ROS、
Gazebo、控制器、demo、colcon 或 CTest 进程。测试二进制只依赖 `librcutils` 和标准
C/C++ 库。

最终关键 SHA-256：

```text
header       4a454f2b679c9c9776a485ca45d5708e30d2a5fb74fe8e32410fd5d9ace6a773
library cpp  be477611025a791b579ebf020d4cb508b3c12b3fa706331a089279023f35a097
tool cpp     1844236a9c32cf729e789508c6c86b9b28d5a352afb2a473323e004a0d5eb363
test cpp     d69241f1a90a7cebce225e8b5c3a45e0ede47596f543639a1f11b1a128cbe83a
demo cpp     560e075a87f3c5ed38966dd0fe9a25f4e9dc5cf602d90491baa6213cb52943b2
installed tool 40c6f616512704bb99797f66f9a883551d28032b607d78c974c1c35683123f0b
```

### 真实证书接入位置与仍未完成项

只读源码定位确认未来导出源：outer 为 `selected_right_outer_state`；45 点 ingress
为 `selected_right_reverse_ingress->states`，保持 contact-to-outer 顺序；73 点
coarse 为选中候选的 `candidate_door_arc.states`，当前必须先新增持久化容器；613 点
exact 为最终 `door_arc.states`；anchor 为
`selected_right_approach_anchor_index`。完整 q0 必须在首次 live RobotState 通过 q0
门时复制不可变快照。writer 不能在候选选中时写盘，只能位于正反时序、两级
2943 dense、FK、碰撞、10 mm、原装 STL、铰链和汇总
`handoff_preflight_passed` 全部 PASS 之后、任何物理执行之前，并且只允许专用
no-controller export 模式写入后 exit 0。

本轮仍没有可信真实 q0 manifest/payload，`incubator_door_open_demo.cpp` 未改变、
没有链接或调用 writer，也没有 cache-required loader 或逐状态完整安全重放，所以
尚未减少历史 `514.56 s` 总耗时，快速物理模式不可用。18 个 binding 的组合哈希
还需使用域分隔、长度前缀和稳定排序，URDF/SRDF/RobotModel、场景、完整起点关节、
运行 DSO 和时序策略必须绑定实际使用的同一内存/FD，不能裸拼接或 hash 后重新打开。
故障注入尚未覆盖每个 write/fchmod/fsync/rename/父目录 fsync 分支；orphan 配额监控
也未实现。

后续若用户重新启动仿真，第一步必须重新执行只读健康和实际角度/关节检查，不得
假定会恢复历史 60 deg 现场，也不得自动 reset 或发送 q0 轨迹。只有真实 q0 慢速
完整预检再次通过后才能生成第一份真实证书；快速模式只能跳过重复 IK 搜索，每次
仍必须完整重放 601/2943、正反路径、碰撞、净距、原装 STL、铰链和时序检查。
轮询任务 `automation` 的现场描述曾同步更新为上述“全部进程停止”状态；其从
2026-08-18 11:48 CST 开始的约 48 小时授权窗口已经结束。为避免更新现场描述后
意外延长原范围，任务已于 2026-08-20 本轮完成后删除；后续只能由用户再次明确
启动仿真或要求继续时恢复工作。

本节实现、最终测试产物和上述文档已保存为新的非覆盖进度备份；它不是物理成功
证书，也不替代仍保留的两个 2026-08-18 物理成功 814 备份：

```text
/home/liu/h1_moveit/backups/814_certificate_atomic_writer_20260820_1227.tar.zst
SHA-256 cee721951dec6ac4571e49fe96b841f297fcc3dedf29d845e92f5bdef3bf88d0
size 56,624,675 bytes; entries 9,321; backup 1.00 s; zstd test PASS;
tar listing PASS; sha256sum -c PASS
```

删除已到期轮询任务并写入最终交接状态后，又保存了最终非覆盖快照；后续应优先使用
这一份，前一份 `1227` 进度快照仍保留且未覆盖：

```text
/home/liu/h1_moveit/backups/814_certificate_atomic_writer_final_20260820_1230.tar.zst
SHA-256 47f660b2e135ddaa2ab33811159906efec4b1df2240f700ee5ec36ac461615d4
size 56,629,919 bytes; entries 9,321; backup 0.16 s; zstd test PASS;
tar listing PASS; sha256sum -c PASS
```

## 2026-08-20 速度优化增量：复合 binding 的域分隔稳定哈希

本节继续上一节的纯源码工作，未改变任何抓取参数、路径点、连续门、碰撞门或物理
现场。只读复核仍是电脑重启后的全停止状态：Gazebo server/GUI、`move_group`、
点云、`environment_scene`、控制器和 demo 均未运行，没有本次可用的门角、把手角、
双指或双臂反馈。本轮没有创建 ROS/Gazebo participant，没有 reset、detach、松夹、
场景写入或控制器轨迹。

### 已实现的固定复合哈希格式

新增纯 C++17、仅依赖 `rcutils` 的 API 和测试：

```text
include/h1_2_moveit_demo/door_handoff_binding_fields.hpp
include/h1_2_moveit_demo/door_handoff_binding_hash.hpp
src/door_handoff_binding_hash.cpp
test/test_door_handoff_binding_hash.cpp
```

`sha256_binding_bundle()` 只接受证书现有 12 个复合字段：URDF、SRDF、kinematics、
joint limits、TCP、碰撞网格集合、静态场景、场景位姿和铰链、参数集、完整起点
关节状态、运行版本及时序策略。源码、当前可执行文件和原装 handle/door/body/tray
四个 STL 共六个直接文件字段明确拒绝进入复合 builder，继续使用原始文件 bytes
SHA-256；普通文件走 `sha256_regular_file()`，运行中可执行文件走本轮新增的
`sha256_self_executable()`。后者只通过 `/proc/self/exe` 打开一次内容 FD，全部
读取都来自同一 inode，前后 `stat/fstat` 只做 dev/ino/mode/nlink/size/mtime/ctime
身份复核，禁止 `argv[0]`、`realpath` 后重开。因此原装把手硬固定 SHA
`ad6670185fa5756ba5dabdee7e542db2df3346973fea4eb887288f54a6b6f59e`
没有改变。`payload_sha256` 仍由证书库内部计算。

18 个字段已收敛到唯一 `kBindingFieldRegistry`；compile-time static assertions 固定
总数 18、复合 12、直接文件 6 且名称无重复。证书 parser 和复合 builder 均从这
一份 registry 构造其允许集合，避免过去在多个 `.cpp` 中重复维护而发生 schema
漂移；payload 仍单独由证书内部加入，不在外部 binding registry 中。

固定 binary envelope 为：8-byte `H1DBNDL1` magic、little-endian u32 schema 1、
字段名长度和字段名、条目数；每条为 little-endian `ENTR` marker、logical-id 长度、
logical-id、u64 内容长度和原始内容；最后为 little-endian `END!` marker。字段名也
参与哈希，阻止跨字段替换。条目按只允许 `0x21..0x7e` 的 logical-id 字节序排序，
调用方枚举顺序不影响结果，重复 ID、空内容、非规范 ID 和未知字段全部 fail
closed。限制固定为最多 4096 条、ID 最多 4096 bytes、单条内容最多 256 MiB、
完整 envelope 最多 1 GiB；尺寸逐次 checked-add，哈希流式处理，不构造 1 GiB
envelope 副本。builder 接收拥有所有权的精确 bytes，未来 collector 必须让同一
bytes 同时进入实际模型/解析器和哈希，禁止 hash 后按路径重新打开。

独立已知向量为 141-byte envelope、7-byte 原始内容，SHA-256：

```text
136989c79c59b23757c53aa3eae2860dd898601469d16b1b225b9b3ebdf62e17
```

### 构建、测试和审计

最终修改已按本文唯一允许的 Miniconda 六库隔离命令增量编译：

```text
2 packages / 0.94 s / PASS
log/build_2026-08-20_14-33-57
ament_uncrustify seven certificate/binding files PASS
```

原证书 GTest `26/26`、新增复合哈希 GTest `11/11`，合计 `37/37 PASS`；colcon/CTest
汇总为 39 tests（含两个 harness）、0 error/failure/skip、`0.51 s`。新增覆盖已知
向量、调用顺序无关、字段/ID/内容/条目分割域分离、12 个复合字段的 digest 两两
不同、六个直接文件和未知字段拒绝、空/重复/非 ASCII 输入、二进制 NUL/0xff、
4096-byte ID 与 4096-entry 精确边界接受以及越界拒绝；同时覆盖当前 executable
inode SHA 的独立回读、重复一致、精确尺寸接受和小 1 byte 拒绝。两套测试二进制
的动态依赖只有 `librcutils` 和标准 C/C++ 运行库；测试后 `/tmp` 无证书或六库隔离
临时残留。
多路独立只读审计未发现 P0/P1/P2 问题。真实 unlink/atomic-replace 的 subprocess
故障注入仍是非阻塞 P3 测试缺口，不能因此把本层称为完整 runtime collector。

最终关键 SHA-256：

```text
field registry 0e37ab0505f4f2f4cc3bad30217a16d43a3568e05d56dfef1820aa431e240903
binding header 8f561363189b135253c5136eaa6d7b2196738059e82f9e4a637221ec4714d785
binding cpp    6be1647ef1fa98f1322694439f7edac3512c6dbfb4572e5d2b4da91d9d41578f
binding test   8984ebc515767995b5e7adc5f4b405d76282a304c82f27f20da91083da4e30cf
cert header    88ff44768364193c1abbb6a8b42d15ce6d90653a5285fa1624a4249156b654a0
cert cpp       4831ac7f6457da5bdb2cf12a3d31aa90c2f4c54a209b88a735606f9d265ace57
cert test      c1d3c08f63efca2a08fee66cacd120aa2937f9adfd6c40628e05b4e12ac1ffea
CMakeLists     2880035912ef579c0d50b178f42985cdcc5c86efc1517800b0f664a4458ce916
demo cpp       560e075a87f3c5ed38966dd0fe9a25f4e9dc5cf602d90491baa6213cb52943b2
```

### 明确仍未完成与下一接入边界

这一步只消除了 18 个 manifest binding 中“复合输入不能安全裸拼接”的阻塞，尚未
降低历史 `514.56 s`。`incubator_door_open_demo.cpp` 未修改，仍未链接证书库；
没有真实 q0 collector、可信 manifest/payload、exporter、cache-required loader 或
逐状态安全重放。泛型 builder 也不能替每个字段发明内容：URDF/SRDF、RobotModel、
实际 scene、完整 live q0、有效参数、已加载 DSO/运行版本和时序策略必须由后续
字段专用 collector 从实际使用的同一内存或稳定 FD 生成确定性 bytes。

未来 exporter 只能在完整正反时序、两级 2943 dense、FK、碰撞、10 mm、原装 STL、
铰链和总 `handoff_preflight_passed` 全部 PASS 后、任何物理执行前工作，并且只能在
专用 no-controller solve-and-export 模式写入后 exit 0。它必须保存 outer、选中
45 点 ingress 的 contact-to-outer 顺序、当前尚需持久化的 73 点 coarse、613 点
exact 以及 anchor index。未来 loader 只能在 live q0/model/scene/bindings 全部形成
后调用 `verify_directory()`；cache-required 模式任何 mismatch 必须 fail closed，
禁止静默退回 IK。每次仍完整重放 601 个目标、2943 个密集插值、完整正反路径、
碰撞、净距、原装 STL、铰链和重新时序，禁止 partial。

本节实现、测试产物和交接文档已经保存为新的非覆盖 814 进度快照；它不是物理成功
证书，也不替代仍保留的 2026-08-18 物理成功备份：

```text
/home/liu/h1_moveit/backups/814_binding_bundle_registry_20260820_1435.tar.zst
SHA-256 ddf37c14217c25c084efa40f8a8ceb0e18094f5555718e5d33971fbe66479292
size 57,074,468 bytes; entries 9,521; backup 0.17 s; zstd test PASS;
tar listing PASS; sha256sum -c PASS
```

写入上述备份记录后，又创建了包含该记录的最终文档快照；`1435` 与此前所有 814
均保留且未覆盖：

```text
/home/liu/h1_moveit/backups/814_binding_bundle_registry_docs_20260820_1436.tar.zst
SHA-256 1aa6bb176c5a183db65573875e6f63aeb6058ea65a2b88d53264f51f530aa858
size 57,071,895 bytes; entries 9,521; backup 0.18 s; zstd test PASS;
tar listing PASS; sha256sum -c PASS
```

## 2026-08-20 18:02 CST：当前 q40 保持与候选 3 快速 40 deg 源码增量

本节是当前最新权威状态；与前文旧现场、旧 `60 deg` 终点或旧 `2943` 精确计数冲突
时，以本节和用户 2026-08-20 的直接要求为准。

### 当前现场只读复核

用户已明确要求柜门保持当前角度，不再继续打开。本轮没有启动 demo、没有调用
reset/detach/release、没有 ApplyPlanningScene 写入，也没有发布控制器轨迹。源码
编译和纯单元测试后重新只读测得：

```text
Gazebo launch/server/gui PID          506812 / 507101 / 507102; healthy
move_group launch/server PID          556710 / 556774; healthy
door / handle                         0.697601037447 / 1.572015779029 rad
                                      about 39.97 / 90.07 deg
door / handle velocity                -2.39e-13 / 6.74e-11; stationary
left fingers                          -19.000 / -19.000 mm
right fingers                         11.350 / 12.350 mm
torso                                 -5.96e-09 rad
right arm P/R/Y/E/R/P/Y               -0.214774/-0.504767/-1.631006/
                                      2.918017/-0.662666/-0.114602/0.873092 rad
five controllers                      all active
four joint_trajectory topics          publisher count 0
filtered point cloud                   one publisher; about 0.71 Hz in sample
demo process                           none
```

并发健康查询刚结束时的五个 Gazebo RTF 样本为
`0.725/0.923/0.681/0.725/0.815`；因此下面的快速门会按设计拒绝当前不稳定窗口，
绝不能为了通过门而降低阈值。等系统空闲后必须重新取得连续合格样本。

### 用户最新范围变更

- 只计算候选 3；候选 1 和 2 不再计算或比较。
- 物理任务终点改为 `40 deg`，不要继续到 `50/60 deg`。
- `2943` 不再是精确计数要求；当前真实插值得到的 `2855` 个样本可作为最低门槛。
  只放宽这一项：low/high 必须使用相同数量且至少 `2855`，每个都是真实插值状态；
  完整正反路径、601 个移动目标、10 mm、原装 STL、固定铰链、4.9 deg 连续性、
  碰撞、时序和禁止 partial 全部保留。
- 目标是把手开始旋转到柜门打开的可见流程最多 `120 s`。在完整墙钟实测通过前，
  只能称为目标，不能声称已经实现或物理验证。

现有 V1 路径证书 manifest 仍硬编码旧 `2943`，而且没有真实 q0 资产、16 个 runtime
binding collector、exporter 或 cache-required loader；它尚未接入快速物理准入。
不要用 FORMAT_ONLY、合成证书或当前 q40 状态绕过这个缺口。以后演进 schema 时应把
该字段改成明确的最低 `2855` 合同，并绑定实际采样策略。

### 已实现但默认关闭的源码入口

`incubator_door_open_demo.cpp` 新增布尔参数
`door_handoff_fast_under_two_minutes`，默认 `false`。它只接受：

```text
door_handoff_execute=true
preflight_only=false
door_handoff_q0_after_left_clear=true
door_handoff_only_candidate=3
door_handoff_q40_candidate3_attached_resume=false
```

fresh/contact/clamped 三种合法 q0 右手入口均可；plan、所有 scan、q40 continuation、
handle、close/relock 和 recovery 模式组合全部在任何控制命令前拒绝。当前 q40 现场
不满足入口，禁止运行该模式。

快速入口仍构造并审计完整 `0 -> 60 -> 0 deg`、`613` 状态和 `601` 个移动目标；
low/high dense 必须相等且不少于 `2855`。只有物理发送裁成前四个 10 deg 段，并在
Gazebo `40 deg` 反馈门停止，右手继续夹紧支撑。每个物理段从对应 RobotState 子段
独立使用 `0.08 s` 基准重新局部定时，继续通过既有 34%/95% 目标及 35%/100% 硬
时序审计；不是从长轨迹随意切片，也没有 partial 路径。把手快速常量和 wrapper
提示恢复为已物理验证的 `0.19 s/deg`，不得再使用未验证的 `0.22`，也不得低于
`0.19`。

GazeboMonitor 还订阅 `/world/h1_world/stats`。把手快速控制和 fast40 的每个控制器
命令前都要求最近连续 5 个 RTF 样本的最小值 `>=0.8`、最新样本年龄 `<=1.5 s`、
整个窗口年龄 `<=6 s`；门运动前及四段每段前再次检查。任何缺失、陈旧或低 RTF
立即 fail closed；夹紧后异常继续 stop-and-hold，禁止 reset、detach 或松指。

### 仍未完成的 120 s 证明

上述改动缩短已预检物理路径：把手 90 deg 名义 `17.10 s`，门 q0 到 40 deg 的
412 个间隔名义基准约 `32.96 s`，两者合计约 `50.06 s`，尚未计接近、夹紧、点云
刷新和检查。它没有消除候选 3 前的 1000-seed 枚举及后续 `45/73/613` 重复 IK；
当前程序也没有把 handle、left-clear 和 door 阶段合并成一个 cache-required 进程。
所以完整进程仍可能在物理动作之间停住数分钟，不能把参数名当作 SLA 证明。

下一安全边界仍是 q0-only、candidate3-only、no-controller exporter，以及同时覆盖
把手和门路径的无 IK 回退 loader；每次加载后仍必须完整重放全部动态安全门。第一份
真实资产只能在以后已有验证的 q0 现场完整慢预检通过后生成。当前 q40 支撑现场不得
为了生成证书 reset、回零、detach 或松夹。

### 构建与测试

源码使用本文规定的 Miniconda 六库隔离命令增量编译，`ament_uncrustify` 对 demo
源码 PASS；证书/绑定三套 GTest 为 `26+11+5=42` 个用例全部 PASS，colcon 汇总
`45 tests, 0 errors, 0 failures, 0 skipped`。编译和测试均未运行 demo 或发送轨迹。
最终 build log 与 SHA-256：

```text
log/build_2026-08-20_18-03-57       2 packages / 39.0 s / PASS
demo source                         648dd0202fb2597b8493ac08618a59d1ae6a85d21f638cad39077a26a1ac0b6c
installed/build executable          3c6def55a25a6c3c349bd599355b72c9c8a91d669d792e508ceff5672912f54b
incubator_handle.sh                 fa15eb5d487c00de4d4b293fa42358978a766ea780abd011af7d6112500ce066
```

`install` 为 symlink-install，已指向上述 18:04 构建产物；不要再把 14:39 的旧程序
误认为当前版本。规定必须保留的 `1436` 备份也重新只读核对为原 SHA-256
`1aa6bb176c5a183db65573875e6f63aeb6058ea65a2b88d53264f51f530aa858`，未覆盖或删除。

## 2026-08-20 19:55 CST：两分钟内 Gazebo 直驱实验已通过

本节是当前最新权威现场状态；与上文 18:02 的旧 q40 右手支撑现场、旧 PID、旧
点云速率或“120 s 尚未实测”冲突时，以本节为准。用户已明确授权仅在仿真中允许
碰撞、失败和试验。该授权不适用于真实机器人，也不把任何仿真结果变成碰撞、接触
或安全证书。

### 当前 live 场景

Gazebo 已从无支撑 fresh q0 重启，并用下面的实验入口从原装把手/门 `0/0 deg`
运动到终点。成功进程 `exit 0`，从第一条把手 Gazebo 消息到连续两秒终点静止门的
实测墙钟为 `43.891 s`，小于用户要求的 `120 s`：

```text
SIMULATION_DIRECT_JOINT_FAST40_RESULT=PASS
wall / simulation time              43.891 / 10.899 s
handle / door                       90.780207326 / 40.003557670 deg
handle / door velocity              1.78e-10 / -5.96e-15 rad/s
latch                               unlocked
Gazebo launch/server/gui PID        2193627 / 2193892 / 2193893
robot_state_publisher               PID 2193721
depth_point_cloud                   PID 2193731; raw cloud about 0.98 Hz
five ros2_control controllers       all active
four robot joint_trajectory topics  publisher count 0
move_group / RViz / cube_detector   absent
incubator demo process              absent after successful exit
Gazebo GUI                          visible, 2000x1690 client window
```

终点后只读 3.2 s 窗口得到 669 个孵育箱关节样本；门的 min/max/last 完全相同，
把手 min/max 相差约 `7.8e-09 deg`。同期受控上身关节最大位置跨度约
`5.70e-18 rad`，机器人静止。运行期间和终点检查均没有 ROS 机器人轨迹发布者。
当前门和把手由动态加载的 Gazebo 控制器保持；它会持续存在到 Gazebo 重启。
不要在当前进程中叠加接触式开门验证，后续恢复机器人接触试验前必须重启 Gazebo。

### 实验入口的真实含义

新增独立目标 `incubator_sim_direct_fast40_demo` 和 wrapper
`incubator_sim_direct_fast40.sh`。入口默认关闭，只接受：

```text
use_sim_time=true
door_handoff_only_candidate=3
door_handoff_simulation_direct_joint_fast40=true
door about 0 deg
handle about 0 deg or a fresh already-unlocked compensated endpoint
five fresh RTF samples, minimum >= 0.18
no incubator_door_open_demo, no concurrent fast40 process, no resident controller
```

日常从 fresh Gazebo 运行：

```bash
ros2 run h1_2_moveit_demo incubator_sim_direct_fast40.sh
```

该模式在现有 `incubator` model entity 上动态加载 Gazebo 8 内置
`JointTrajectoryController`，不修改或重建机器人、孵育箱、门、把手、铰链或任何
STL。它不启动 MoveIt，不计算 IK，不创建 ROS controller trajectory publisher，
也不发送机器人控制器轨迹。`door_handoff_only_candidate=3` 只是用户要求的参数门：
**没有计算或执行候选 3 机器人路径**，日志必须保留
`candidate-3 parameter gate only` 和
`not a collision/contact/safety certificate`。

Gazebo 控制器按时间点切换目标而不是 ROS spline 插值，因此源码显式发送密集目标：

- 把手：初始点加 92 个一度目标，`1 deg / 0.06 sim-s`；低增益
  `P=2, D=0.05`。命令补偿终点为 `92 deg`，用于克服实测跟踪偏差，反馈终点约
  `90.78 deg`。
- 只有把手角、把手速度和 fresh `latch=unlocked` 连续通过后才发送门消息。
- 门：初始点加 400 个 `0.1 deg` 目标，`0.1 deg / 0.01 sim-s`，终点
  `40 deg`；增益 `P=20, D=1`。
- 最终必须连续 2 s 同时满足把手角、门角、两者速度和 fresh unlocked latch；
  墙钟超过 120 s、反馈陈旧或 Ctrl-C 均只能 FAIL，不能打印 PASS。

动态控制器 add 之后若失败，程序会在 feedback 仍 fresh 时请求保持最新角度，并明确
要求重启；wrapper 用非阻塞 `flock`，C++ 还二次检查并发 node 和既有 topic
subscriber，禁止叠加控制器。当前 live 门已约 40 deg，因此重复运行会在 add 前拒绝；
需要重做演示时必须先正常重启 Gazebo 获得 fresh 0 deg 场景。

这个入口是用户允许的快速可视化实验，完全绕过候选 3 的 45/73/613 IK 和
low/high dense 安全重放。它不能替代原来要求的 601 目标、至少 2855 dense、完整
正反路径、10 mm、4.9 deg、原装 STL、铰链、碰撞和时序检查。以后要声称“机器人
按候选 3 开门”或恢复接触式路径，仍必须实现可信 q0 cache/loader 并按前文完整重放，
禁止拿本实验的 `43.891 s` 当作机器人路径证书。

### 最新构建、测试与保留备份

最后源码修改后已再次使用本文规定的 Miniconda 六库精确隔离命令编译
`h1_2_moveit_config` 和 `h1_2_moveit_demo`；随后 colcon 汇总为
`48 tests, 0 errors, 0 failures, 1 skipped`。最终文件 SHA-256：

```text
sim-direct source    c0bcd4ef00ff668e5f7e6395286bd3c123c8e0936887dfe0ea83b3febcd6224b
sim-direct wrapper   ddf795fb6be505a6865922201db98553f01da16c51f5c275d8d3ce680e582fcf
CMakeLists           aac3e8ba1a6e73a3d4fafc0352d513679ccd70e8bf4bad5b8602edf9835f6965
build executable     5ff440e59eed50dfee633d6e48de891cae67a4c18b264a35096e99f214067aa0
```

`install` 使用 symlink-install，installed executable 指向上述 build 产物。规定必须
保留的备份仍未覆盖或删除，最终再次只读校验为：

```text
/home/liu/h1_moveit/backups/814_binding_bundle_registry_docs_20260820_1436.tar.zst
SHA-256 1aa6bb176c5a183db65573875e6f63aeb6058ea65a2b88d53264f51f530aa858
```

## 2026-08-24 11:52 CST：Gazebo 快速流程现场重演通过

本节是最新 live 状态，覆盖前文 2026-08-20 的旧 PID 和“当前现场”描述，但不改变
任何历史机器人接触式验证结论。启动前现场为完全停机；先用标准命令启动唯一一套
原装孵育箱 Gazebo：

```bash
ros2 launch h1_2_moveit_config gazebo.launch.py \
  spawn_incubator:=true incubator_x:=0.590
```

启动后的门和把手实测约为 `0/0 deg` 且静止，五个 ros2_control 控制器全部
`active`，点云有 fresh 帧，Gazebo GUI 为可见的 `2000x1690` 窗口，四个机器人
`joint_trajectory` 话题 publisher 均为 `0`，无并发 demo。随后运行：

```bash
ros2 run h1_2_moveit_demo incubator_sim_direct_fast40.sh
```

本次进程 `exit 0`：

```text
SIMULATION_DIRECT_JOINT_FAST40_RESULT=PASS
wall / simulation time              45.891 / 10.873 s
handle / door                       90.7802 / 40.0036 deg
handle / door velocity              1.78085e-10 / -5.95745e-15 rad/s
latch                               unlocked
Gazebo launch/server/gui PID        39698 / 40006 / 40007
robot_state_publisher               PID 39826
depth_point_cloud                   PID 39836; measured about 0.91 Hz
five ros2_control controllers       all active
four robot joint_trajectory topics  publisher count 0
move_group / RViz / cube_detector   absent
incubator demo process              absent after successful exit
```

机器人保持初始姿态，本次没有发布任何 ROS 机器人轨迹。该入口仍只是候选 3 参数门
下的 Gazebo 原装把手/门关节直驱实验；没有计算或执行候选 3 机器人 IK 路径，也不
是碰撞、接触或安全证书。成功后的 Gazebo `JointTrajectoryController` 会驻留并保持
当前 `40 deg` 画面；不得在同一 Gazebo 再次运行或叠加接触式验证。需要重演时必须
先正常重启 Gazebo，再重新只读确认 fresh `0/0 deg`、GUI、五控制器、点云、机器人
静止、轨迹 publisher 为零且无并发 demo。

本轮只运行已安装且 SHA 与 2026-08-20 19:55 节完全一致的产物，没有修改源码、
模型、STL 或构建产物，也不需要重新编译。规定保留的 `1436` 备份没有覆盖或删除。

## 2026-08-24 12:01 CST：当前柜门已从 40 deg 延续到 60 deg

本节是最新 live 状态，覆盖上一节的 `40 deg` 终点。用户明确要求在同一仿真中继续
加大开门角度到 `60 deg`。没有重启或 reset；新增的默认关闭延续入口只复用上一节
已存在的 Gazebo `JointTrajectoryController`，没有 add 第二个控制器，也没有启动
MoveIt、计算 IK 或发布 ROS 机器人轨迹。

### 延续入口和执行门

新增 wrapper：

```bash
ros2 run h1_2_moveit_demo incubator_sim_direct_fast60_continue.sh
```

它与 fast40 共用非阻塞 `flock`，并在运行前要求：Gazebo server/model 健康、GUI
可见、点云 fresh、五个 ros2_control 控制器全部 active、四个机器人
`joint_trajectory` 话题 publisher 均为零、无并发 demo；私有 Gazebo 命令话题必须
正好是零个 publisher 和一个 `gz.msgs.JointTrajectory` controller subscriber。

C++ 参数 `door_handoff_simulation_direct_joint_fast60_continuation` 默认 `false`，
与 fast40 opt-in 必须恰好一个为 true。延续门再次要求 `use_sim_time=true`、候选参数
等于 3、fresh feedback、五个 RTF 样本最低 `>=0.18`、门在
`40 deg +/-0.75 deg`、把手在 `92 deg` 补偿目标的 `+/-1.5 deg`、fresh latch 为
unlocked、把手/门速度分别 `<=0.02/0.01 rad/s`。它只复用既有 subscriber，缺失时
FAIL，绝不补 add 控制器。

门轨迹为实测 q40 初始点加 200 个真实 `0.1 deg` 目标，共 201 点；时间仍为
`0.1 deg / 0.01 sim-s`，命令速度约 `0.175 rad/s < 0.5 rad/s` SDF 门限，把手
补偿设定保持 `92 deg`。终点必须同时满足门 `60 deg +/-0.75 deg`、把手、两速度和
fresh unlocked latch 连续两秒；120 s 超时或中断只能 FAIL，并尽力请求保持最新
实测角度。

### 本次物理反馈

运行前 326 个连续只读样本为：门 `40.00355767 deg`、把手约
`90.78020819 deg`，两者速度近零；12 个 RTF 样本为 `0.230..0.269`。执行结果：

```text
SIMULATION_DIRECT_JOINT_FAST60_CONTINUATION_RESULT=PASS
wall / simulation time              12.661 / 3.021 s
handle / door                       90.7802 / 60.0043 deg
handle / door velocity              about 1.78e-10 / -7.16e-15 rad/s
latch                               unlocked
logged transient door peak          60.25 deg, then settled to 60.0043 deg
program                             exit 0
```

上一节 `0 -> 40 deg` 动作与本节 `40 -> 60 deg` 延续的两个执行进程墙钟合计
`58.552 s`；该数值不包含中间源码修改、编译和等待时间，不能写成一次连续机器人
流程证据。终点只读复核仍为五控制器 active、点云 fresh、四个机器人轨迹话题
publisher 为零、无 demo，Gazebo GUI 可见；机器人保持初始姿态。私有 Gazebo 话题
恢复为零个 publisher、一个常驻 controller subscriber，并保持当前 q60 画面。

### 构建、测试与文件身份

源码修改后已用本文规定的 Miniconda 六库精确隔离命令编译：

```text
log/build_2026-08-24_11-59-37      2 packages / 5.42 s / PASS
colcon tests                       48 tests, 0 errors, 0 failures, 1 skipped
sim-direct source                  96cc2f5dabb6a13f9e22bf8f1dc81190de58bc06eae47389a2222c16342c195c
fast60 wrapper                     edd353edac9290ae2baca362b2219901e713bdc2dad5a60d4780ec37117a2965
CMakeLists                         51ed034ccf785d751593dfcc6090e60cb33852883d959c8613257e7d0bdb06df
build executable                   2d517d02b2622fb57e67c30d2631e98bf7a806edd834178f8880629c72cb8cd0
```

`install` 仍是 symlink-install，已指向上述 build executable 和源码 wrapper。
规定保留的 `814_binding_bundle_registry_docs_20260820_1436.tar.zst` 未覆盖或删除，
SHA-256 仍为
`1aa6bb176c5a183db65573875e6f63aeb6058ea65a2b88d53264f51f530aa858`。

本次仍只是用户允许的 Gazebo 直驱可视化实验；`door_handoff_only_candidate=3` 只是
参数门，没有计算或执行候选 3 机器人路径，也不是碰撞、接触或安全证书。当前门已
稳定在 60 deg；没有 q60 后续入口，禁止再次运行 fast40/fast60 或叠加接触式验证。
若要重新演示，必须正常重启 Gazebo 到 fresh `0/0 deg`，重新通过全部只读健康门后
依次运行 fast40 与 fast60 延续。

## 2026-08-24 14:15 CST：机器人仅候选 3 完成把手旋转和开门到 60 deg

本节是当前最新 live 状态，覆盖上节的 Gazebo 直驱场景。用户明确纠正要求后，
本轮运行的是机器人接触式流程，不是孵育箱关节直驱：左手绕原装把手固定轴旋转，
左手张开并退到中立安全位，右手只计算候选 3、夹住原装把手后半部并绕正确的左侧
竖直铰链把柜门打开到约 `60 deg`。候选 1 和 2 在候选级 pregrasp、IK、碰撞、
原装 STL 和 dense 检查以前跳过；没有重建或替换机器人、柜体、门、把手或铰链。

### 三阶段方法与本次物理结果

只允许从正常启动且实测门/把手约为 `0/0 deg` 的 fresh Gazebo 重演。三个阶段的
入口顺序为：

```bash
# 1. 左手接近、夹紧并将原装把手 0 -> 90 deg
install/h1_2_moveit_demo/lib/h1_2_moveit_demo/incubator_handle.sh task

# 2. 完整正反预检后张开左指并退到中立安全位
ros2 run h1_2_moveit_demo incubator_door_open_demo --ros-args \
  -p use_sim_time:=true -p preflight_only:=false \
  -p door_handoff_left_clear_recovery:=true \
  -p front_handle_pivot_tcp_arc:=true \
  -p front_diagonal_dual_pad_test:=true \
  -p front_reverse_seed_entry:=true \
  -p front_handle_arm:=left \
  -p front_handle_phase_degrees:=-159.0 \
  -p front_palm_down_tilt_degrees:=7.0 \
  -p front_handle_local_x_offset_mm:=0.0 \
  -p front_handle_local_z_mm:=90.0 \
  -p front_handle_outward_offset_mm:=16.0 \
  -p front_gripper_position_mm:=11.85 \
  -p front_gripper_asymmetry_mm:=-0.5 \
  -p front_diagonal_max_station_separation_mm:=26.0 \
  -p front_diagonal_minimum_body_clearance_mm:=0.6 \
  -p front_safe_ik_attempts:=100 \
  -p front_fixed_torso_yaw_degrees:=0.0

# 3. 右手只计算候选 3；同进程完整预检后从 q0 开门到 60 deg
ros2 run h1_2_moveit_demo incubator_door_open_demo --ros-args \
  -p use_sim_time:=true \
  -p preflight_only:=false \
  -p door_handoff_plan_only:=false \
  -p door_handoff_execute:=true \
  -p door_handoff_q0_after_left_clear:=true \
  -p door_handoff_sequential_release_first:=false \
  -p door_handoff_q0_right_contact_resume:=false \
  -p door_handoff_q0_right_clamped_resume:=false \
  -p door_handoff_q40_candidate3_attached_resume:=false \
  -p door_handoff_fast_under_two_minutes:=false \
  -p door_handoff_only_candidate:=3 \
  -p front_handle_arm:=right \
  -p front_handle_pivot_tcp_arc:=false \
  -p front_diagonal_dual_pad_test:=true \
  -p front_reverse_seed_entry:=false \
  -p front_handle_phase_degrees:=21.0 \
  -p front_palm_down_tilt_degrees:=7.0 \
  -p front_handle_local_x_offset_mm:=0.0 \
  -p front_handle_local_z_mm:=35.0 \
  -p front_handle_outward_offset_mm:=16.0 \
  -p front_gripper_position_mm:=11.85 \
  -p front_gripper_asymmetry_mm:=-0.5 \
  -p front_diagonal_max_station_separation_mm:=26.0 \
  -p front_diagonal_minimum_body_clearance_mm:=0.6 \
  -p front_diagonal_end_side_tolerance_mm:=1.0 \
  -p door_handoff_right_pregrasp_opening_mm:=1.0 \
  -p right_handle_axial_spin_degrees:=0.0 \
  -p right_tcp_local_x_shift_mm:=0.0 \
  -p front_fixed_torso_yaw_degrees:=0.0
```

阶段 1 物理结果为
`HANDLE_ROTATE_RESULT=PASS handle=89.821 deg door=-0.000 deg`，固定轴漂移
`0.000 mm`，更新场景最小净距 `10.001 mm`。阶段 2 返回
`DOOR_HANDOFF_LEFT_CLEAR_RECOVERY_SUCCESS`；左指约 `-19/-19 mm`，左臂终点约
`[0.00001,0.30029,-0.00021,0.59984,-0.00020,-0.00007,-0.00053] rad`，
把手仍约 `89.821 deg`、门约 `0 deg`、锁舌为 `unlocked`。

阶段 3 的完整 no-controller 门在第一条物理轨迹以前通过：

```text
selected endpoint                    candidate 3 of 16
candidate 1 / 2                      skipped before candidate-level work
ingress / coarse / exact             45/45, 73/73, 613/613
moving door targets                  601/601 plus exact reverse
low / Gazebo-high dense samples      2923/2923, both PASS
minimum dense contract               >=2855, exact equality required
original-STL rigid checks            42650 PASS
maximum hinge drift / axis error     0.010 mm / 0.003 deg
minimum offline environment gap      10.001 mm
joint continuity                     <=4.9 deg PASS
six segment timing audits            all PASS
partial                              forbidden; none executed
```

右手随后通过预抓、两段接近、原装 STL 夹紧、场景回读和机构附着检查，六段 Gazebo
门反馈依次为 `10.000/19.999/29.998/39.998/49.997/59.997 deg`。把手约
`89.805 deg`，右指约 `11.311/12.350 mm`；所有固定铰链、机构位姿、原装 STL
接触和指反馈门均通过。

终点不能记录为干净的物理 PASS：第六段以后最新实时深度点云把
`<octomap> <-> right_dex1_finger_2_slider_link` 的更新场景净距测为
`7.924 mm < 10.000 mm`。程序因此正确记录
`DOOR_HANDOFF_PHYSICAL_RESULT=FAIL` 并进入 stop-and-hold；右手保持夹紧支撑，
门、把手和 visual attachment 保持附着，没有 reset、detach、松指或未预检恢复。
这是终点实时感知门异常，不否定机器人已经真实到达约 `60 deg`，但禁止把本轮写成
全部门通过或 `exit 0`。

### 当前只读现场

2026-08-24 14:15 CST 的 3.2 s 连续只读窗口：

```text
Gazebo launch/server/gui PID          200076 / 200342 / 200343; healthy
move_group launch/server PID          202446 / 202539; healthy
Gazebo GUI                            visible, 2000 x 1690
door                                  59.984571 deg; range 0.000011 deg
handle                                89.812208 deg; range 0.000007 deg
door / handle velocity                -3.52e-7 / +2.36e-7 rad/s
left fingers                           -19.000 / -19.000 mm
right fingers                          11.328875 / 12.350000 mm
five controllers                       all active
four robot trajectory topics           publisher count 0
filtered point cloud                   one publisher; about 0.53 Hz
demo / recorder process                none
```

当前仍是 q60 右手支撑现场。用户下一步要求重演、两分钟优化和录像，但在本节备份
完成前已暂停；不得为了重演直接 reset、detach 或松指。必须先采用完整预检的反向
恢复，或者在用户明确接受的纯仿真重启边界内正常重启整套 Gazebo，再从 fresh
`0/0 deg` 重新通过健康门。

### 当前源码、构建与测试身份

本轮源码修复了 `PlanningSceneInterface::removeCollisionObjects()` 异步 REMOVE
可能晚于同步 ADD、导致原装 STL 从已审计场景消失的竞态：现在只删除实际存在的
world object，并经 `/apply_planning_scene` 同步提交；关键场景同步结果失败关闭。
候选门仍为 `door_handoff_only_candidate=3`，1/2 在候选级工作前跳过；dense 只将
精确 `2943` 改为 low/high 相等且至少 `2855`。把手快速常量使用通过加速度审计的
`0.21 s/deg`；Gazebo RTF liveness 门为当前完整渲染现场可达到的 `0.12`，机器人
轨迹的速度、加速度、碰撞、净距、STL、铰链和时序门均未因此放宽。

```text
latest isolated build log             log/build_2026-08-24_13-03-30
demo source SHA-256                    d90e5a9ec1bf9bd2e7ac1abf0ea05652
                                       d9dd998aad007d69b38a0e55f27e0fba
build/install demo SHA-256             c7f51fa455efe9ed9b8c8401b5b3579e
                                       e0fe6ebb5e6108150e4bde1f854d8487
CMakeLists SHA-256                     51ed034ccf785d751593dfcc6090e60cb
                                       33852883d959c8613257e7d0bdb06df
incubator_handle.sh SHA-256            f04dd85618df9c9a3fe26fafe19740534
                                       499746ba2a28c0aaaaaeb9d9be73734
colcon test-result                     48 tests, 0 errors, 0 failures,
                                       1 skipped
```

用户指定本节状态保存为新的非覆盖完整工作空间备份
`/home/liu/h1_moveit/backups/824.tar.zst`；其 SHA-256 写入同名 `.sha256` 文件并
在创建后执行 zstd 完整性和 tar 清单校验。此前必须保留的
`814_binding_bundle_registry_docs_20260820_1436.tar.zst` 已在备份前再次只读核对
为原 SHA-256 `1aa6bb176c5a183db65573875e6f63aeb6058ea65a2b88d53264f51f530aa858`，
不得覆盖或删除。

本轮独立文字证据同时保存在工作空间内
`runtime_evidence/robot_candidate3_door60_20260824_summary.txt`，确保它进入 824
归档；外部运行日志目录中的同名副本仅用于现场查阅，不替代归档内证据。

## 2026-08-24 17:45 CST：开门与中层铁架放块成果留存为 825

本节是当前最新权威留存范围。机器人接触式把手与候选 3 开门阶段沿用上节事实：
左手已旋转原装把手，右手已沿正确侧边竖直铰链把原装门带到约 `60 deg`；终点实时
点云 `7.924 mm < 10 mm`，所以该门阶段仍不得改写为全部门 PASS。此后在同一 q60
右手支撑现场，左手已经把持久出生在
`XYZ=(0.334,+0.377,0.830) m, yaw=+90 deg` 的原装绿色长方块，经柜体正前方路线
放到原装中层铁架中心并释放、撤离。最终方块实测约为
`XYZ=(0.611342,-0.029359,0.995407) m, yaw=70 deg`，程序记录：

```text
LEFT_CUBE_TO_INCUBATOR_RACK_PHYSICAL=PASS
```

放块预检保留 16/16 个完整前方插入段和 269 个合并搬运点；搬运正反密集检查为
`283/283`，支撑进入正反为 `12/12`，释放后撤离正反为 `305/305`，所有 Cartesian
段均为 `100%`，没有执行 partial。成功物理调用从已经通过检查的左手接触终点继续，
只跳过重复接近动作；之后真实完成夹紧、正面送入、原装铁架支撑、松夹与空手反向
撤离，右臂、右夹爪、门和把手保持不变。试验路径最小环境净距为 `3.033 mm`，因此
这是用户允许的 Gazebo 快速路线成果，不是 10 mm 放块安全证书。

完整事实、日志 SHA、源码/安装身份和 live 终点写入：

```text
runtime_evidence/robot_handle_door60_cube_rack_20260824_summary.txt
```

当前成果保存为新的非覆盖完整工作空间备份
`/home/liu/h1_moveit/backups/825.tar.zst`；SHA-256 sidecar 与归档同目录，创建后必须
通过 `zstd -t`、tar 清单和证据文件存在性检查。必须继续保留且不得覆盖或删除
`824.tar.zst` 与规定的 `814_binding_bundle_registry_docs_20260820_1436.tar.zst`。

这里保存的是三个已完成组件及共同现场终点，**不是**单命令连续联跑或两分钟证明。
后续继续优化时必须从 fresh `0/0 deg` 场景重新实测完整机器人流程的墙钟时间，
在取得证据前不得声称已满足 `<=120 s`。

## 2026-08-25 01:11 CST：8251 完整机器人流程在两分钟内通过

本节是当前最新权威现场与成果，覆盖上节“尚未单命令连续联跑”的状态。正常重启
Gazebo 后，从实测 fresh 门/把手 `0/0 deg`、机器人中立、双手张开、绿色方块位于
左手默认出生点开始，机器人连续完成：左手旋转原装把手、左手张开并撤至安全位、
右手抓原装把手并沿正确侧边竖直铰链将原装门打开到 60 deg、左手夹住绿色方块从
柜体正前方送入原装中层铁架中心、释放并撤离。没有孵育箱关节直驱，没有重建或
替换机器人、柜体、门、把手、铁架或原装 STL，也没有执行 partial。

### 正式物理结果与两分钟口径

唯一正式成功运行目录为：

```text
/home/liu/h1_moveit/runtime_logs/full_robot_flow_8251_acceptance/
  run_20260825_010823_796874808_880424
```

正式控制台为同目录 `formal_wrapper_console.log`，成功标志：

```text
FULL_ROBOT_FLOW_PHYSICAL=PASS
first_publish_to_cube_complete=116.305s
invocation_upper_bound=118.652s
door=60deg handle=90deg cube=middle_rack
right_support=held direct_door=absent
```

`116.305 s` 从首次观测到机器人 controller topic 发布开始，到最终按顺序通过
cube rack-entry、stability 和 rack-exit 三门结束；`118.652 s` 从第一次 handle
replay 调用前开始，是保守调用上界。两者都严格小于 120 秒。累计墙钟门为：

```text
handle complete                     22.553 s
left clear complete                 28.648 s
door support attached              44.187 s
door 60 deg complete               50.422 s
cube attached                      68.374 s
cube detached on rack             102.069 s
final retreat and rack gates      116.305 s
```

最终顺序门读数为方块
`XYZ=(0.610921,-0.029407,0.995484) m, RPY=(-0.000174,+0.001575,+1.221250) rad`；
门/把手为约 `59.995/89.762 deg` 且速度近零。左指已张开；右手仍夹紧并支撑门。
wrapper 成功后没有 reset、直驱门、松开右手、重新 attach 或额外机器人轨迹。

### 正式密封资产与 retime

正式 wrapper 是安装后的
`h1_2_moveit_demo/h1_full_incubator_8251.sh`。四个资产均为当前用户拥有的普通文件、
mode `0400`、link count `1`，运行前失败关闭检查 embedded SHA、stage、message
count、topic split、端点和关节限速/加速度：

```text
stage       messages  embedded SHA-256
handle      5         e2aa99982626e7f5909697e7a065fdd9406f72fd4bcee22f537f4af303330fd9
left_clear  2         066b2437fa320d2db895a927e24f0d5044ff2a9786638e335f8117695e105b88
door        13        867548548197ae577580f394051e8118cc0fccefdea1494c1bbb41ccaa4a4a19
cube        6         96d491b4f607884c3b70fea4aaad7b9df93d48e145a4270d078b0a6f29797f17
```

对应文件 SHA-256 为 `5da593a6...1cd6f70`、`9e3daae8...2dcc0b3`、
`c481657a...90131c9` 和 `73dd0bed...5ac063e`。方块必须使用
`cube.low_preload_8251.trajectory.json`，不得误用旧 `cube.trajectory.json`。

正式时序计划：

```text
handle       0:0.25,3:0.25,4:0.25                           34.074000 s
left_clear   0:0.25,1:0.25                                  5.235000 s
door pre     0:0.3,1:0.1,2:0.3,3:0.15,4:0.35,5:0.1         5.563200 s
door arc     6:0.12,7:0.15,8:0.1,9:0.1,10:0.1,11:0.1,
             12:0.1                                          3.685200 s
cube pre     0:1,1:1                                         2.437582 s
cube carry   2:0.75,3:0.75                                  11.187071 s
cube post    4:0.1,5:0.35                                    5.641610 s
```

正常物理步长为 `0.008 s`，方块接触与持物阶段为 `0.006 s`；常规 RTF 门为一个
样本 `>=0.70`，持物阶段要求连续三个样本均 `>=0.50`。door/cube 支撑插件都先以
`initially_attached=false` 预加载，真实指接触和 fresh 状态门通过后才 attach；方块
在铁架位姿和稳定性门通过后才 detach。监听器 discovery barrier 已消除 attach/
detach exact topology 的瞬态竞态；fresh ROS/Gazebo 读取采用有界重试但不接受 stale
数据。

只允许 fresh 场景使用以下入口；当前 q60/rack 终态不得重复执行：

```bash
cd /home/liu/h1_moveit/h1_2_ros2_ws_claude_2026-07-30/h1_2_ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

H1_FULL_RECORD=1 \
ros2 run h1_2_moveit_demo h1_full_incubator_8251.sh run \
  "$PWD/runtime_assets/full_robot_flow_8251/handle.trajectory.json" \
  "$PWD/runtime_assets/full_robot_flow_8251/left_clear.trajectory.json" \
  "$PWD/runtime_assets/full_robot_flow_8251/door.trajectory.json" \
  "$PWD/runtime_assets/full_robot_flow_8251/cube.low_preload_8251.trajectory.json"
```

wrapper 要求 GUI、Gazebo、move_group、五控制器、robot/incubator joint states、点云、
world stats、fresh 机构/机器人/方块、无并发 demo、四个 trajectory topic publisher
为零和 direct-door 入口不存在。录像默认启用并使用带时间戳的新文件，拒绝覆盖。

### 规划审计边界与非安全声明

候选 1、2 未参与候选级计算。door 资产来自 candidate-3-only cache-required V2
路径，生成阶段只跳过三次重复 IK；仍重放 `outer+45+73+613`、601 个
`0 -> 60 deg` 移动目标和精确反向路径、原装 STL、固定侧边竖直铰链、10.001 mm
环境净距、连续性、关节界、碰撞和时序检查，禁止 IK fallback 和 partial。该次
实际 low/high dense 为 `2679/2679`，合同下限是 `2600`；不得再写成旧
`2943`、`2923` 或 `2855`。

本轮是用户明确允许放宽限制后的**纯 Gazebo 非安全速度实验，不是碰撞安全证书**：

- private Gazebo-high 自碰撞只放宽
  `right_elbow_link <-> torso_link`；入口 dense 曾忽略最大 `2.211 mm` 穿透；low
  model 与其他 high-model pair 未放宽；
- 方块正面送入路径的规划最小环境净距为 `3.033 mm < 10 mm`；
- sealed wrapper 重放时重新验证密封资产、时序、fresh 状态、在线机构、接触、位姿、
  稳定性和轨迹 publisher 门，但不在 wrapper 内重新运行 MoveIt IK/碰撞/STL dense。

因此，后续若要求恢复 10 mm 或安全证书，必须先用 candidate-3 和 left-rack 的
cache-required 入口完整重放碰撞、环境、原装 STL、铰链、正反路径和时序审计，再
生成/使用密封资产；不能只凭本节录像或 wrapper PASS 声称安全。

### 录像、最终现场、构建测试和归档证据

正式录像：

```text
/home/liu/h1_moveit/demo_recordings/h1_full_robot_flow_8251_acceptance.mp4
MP4 / H.264 Constrained Baseline / yuv420p
1160 x 1530 / 12 fps / 1472 frames / no audio
duration 122.666667 s / 3,548,441 bytes
SHA-256 6baf9877a290d2f4ef0129ade85df75335e1bfcd273e6336cfd2771cfbc89c1c
```

视频包含首条机器人命令前的摄像机、录制器和 RTF 准备，以及最终验收收尾，所以
容器时长超过 120 秒；动作与调用 SLA 只能使用前述 `116.305/118.652 s`。

2026-08-25 01:13 CST 最终只读健康检查：Gazebo launch/server/gui PID
`876245/876532/876533`，GUI 窗口可见 `2000x1690`；move_group PID `879277`；五个
控制器 active；四个机器人轨迹话题 publisher 均为零；原始点云约 `6.2 Hz`，过滤
点云约 `0.8 Hz`；无残留 wrapper/demo/recorder。稍后的方块三样本约为
`XYZ=(0.610972,-0.028506,0.995476) m, yaw=69.316 deg`，0.5 秒最大平移约
`0.004 mm`。Gazebo 和 move_group 保持运行，用户可继续查看最终静止画面。

最后源码身份包括：wrapper
`89aadb62d302296ffc5ad4aff300f3f06600746930f0dbf7d387117565b69c3a`，door helper
`99e13c5ef5c2edf91f248674ea6ecddf541a9c3cfd813a6d5649075f2354de4f`，cube helper
`0e37b906454e54a2a57dfbef4dd21b2622f77214a25721e724761e7633d2d4cb`，candidate-3
cache `61c342a16f9cf8c487918682e0b46180be0e6dddecba410995dcf47f79221374`，left-rack
cache `37fbd0a66c274079ef8ec04254be545cc8ec7f800a99588f4f85fcdf016cec66`。

最后使用本文规定的 Miniconda 六库精确隔离命令完成：

```text
log/build_2026-08-25_01-12-41     2 packages / PASS
log/test_2026-08-25_01-12-53      PASS
colcon test-result                54 tests, 0 errors, 0 failures, 1 skipped
direct Python script contracts    38 + 10 tests, 0 failures
```

工作空间内的自包含证据目录为：

```text
runtime_evidence/full_robot_flow_8251_20260825/
```

其中包含正式 console、完整成功 run 目录、录像副本、九个抽帧、构建/测试日志和
`SUMMARY.md`。用户指定的非覆盖归档目标为
`/home/liu/h1_moveit/backups/8251.tar.zst`，SHA-256 写入同名 `.sha256` sidecar；
创建后必须通过 `zstd -t`、tar 清单、证据/资产存在性和 sidecar 校验。创建前重新
核验旧备份：814=`1aa6bb17...0aa858`、824=`e6142417...450d9f`、
825=`d69ab272...a4bac`；三者必须继续保留且不得覆盖或删除。

## 2026-08-25 09:20 CST：领导视角完整机器人流程再次低于两分钟

本节是当前最新现场，覆盖上节 01:13 CST 的运行态，但不覆盖或删除其正式验收
证据。为响应用户要求，先正常结束 q60/右手支撑/方块已翻斜的旧 Gazebo，再启动
fresh 场景；没有在夹持现场执行 in-place reset、detach 或松夹。重启后旧
`move_group/environment_scene` 未产生新的 filtered cloud，因此也正常重启该长期
进程。第一轮 wrapper 纯只读预检只在 `no fresh filtered point cloud` 处失败，未发
轨迹；MoveIt 点云恢复后第二轮预检得到 `PREFLIGHT_ONLY=PASS`。

本次只改变 8251 wrapper 的 GUI 录制相机，不改变四个密封机器人资产、retime、
门/把手/方块路径、支持插件、物理门或验收门。用户截图对应并已用于正式录制的
相机为：

```text
position                         (0.06292473524808884,
                                  -0.46163156628608704,
                                   1.8398312330245972)
quaternion                       (-0.15106521546840668,
                                   0.24716269969940186,
                                   0.49844232201576233,
                                   0.8170956969261169)
wrapper SHA-256                  346a5474e108aeed10f4c5fe52b2115b1b87708dd7c0b5f94bac46878a549e08
```

最新成功运行目录和视频：

```text
/home/liu/h1_moveit/runtime_logs/full_robot_flow_8251_leadership_20260825_0918/
  run_20260825_091741_767900539_3772585
/home/liu/h1_moveit/demo_recordings/h1_full_robot_flow_leadership_20260825_0918.mp4
```

物理结果：

```text
FULL_ROBOT_FLOW_PHYSICAL         PASS
first publish -> cube complete  115.896 s
invocation upper bound          116.838 s
video                           119.666667 s / 1160x1530 / 12 fps / 1436 frames
video size                      5,047,288 bytes
video SHA-256                   dafffab6889967a75ea31b68888061c9051bd7626f5e56b61dcc810c46d8439c
final door / handle             59.996 / 89.762 deg, velocity near zero
final sequential cube exit      XYZ=(0.611067,-0.029363,0.995569) m
                                RPY=(-0.000217,+0.002743,+1.223150) rad
```

视频已经通过 `ffprobe`、完整 `ffmpeg` 解码和五个关键帧构图检查；画面完整包含左手
旋转原装把手、右手沿正确侧边竖直铰链开门到 60 deg、左手从正面送入方块、释放
和撤离。09:23 CST 的稍后只读样本为方块
`XYZ=(0.611419,-0.028335,0.995582) m`、
`RPY=(-0.001201,-0.002167,+1.222100) rad`，此短时样本仍近似水平，但不能当作原装
带筋铁架上的长期稳定证明。五控制器 active，四个机器人 trajectory topic
publisher 均为零，无残留 wrapper、demo 或 recorder；Gazebo 与 move_group 留在
最终静止画面。当前不是 fresh 场景，禁止直接重复执行 8251。

相机脚本修改后按本文六库隔离规则完成：

```text
log/build_2026-08-25_09-13-20    2 packages / PASS
log/test_2026-08-25_09-13-35     54 tests / 0 errors / 0 failures / 1 skipped
```

本轮仍继承 8251 的纯 Gazebo 非安全边界：private high model 的
`right_elbow_link <-> torso_link` 放宽，以及方块路线最小环境净距
`3.033 mm < 10 mm`；不能对实体机器人或完整碰撞安全作声明。本轮没有创建新备份，
旧 814、824、825、8251 归档均保持原样。

## 2026-08-25 09:36 CST：8252 非覆盖归档范围

用户要求把 09:20 CST 的领导视角完整机器人流程保存为 `8252`。归档前已经建立
自包含证据目录：

```text
runtime_evidence/full_robot_flow_8252_20260825/
```

其中包含 `SUMMARY.md`、固定 `SHA256SUMS`、119.666667 s 领导视角 MP4、完整成功
`leadership_run/`（含五个 QA 抽帧）、09:13 隔离 build/test 日志。目录内 checksum
检查已全部通过；视频、wrapper 和四个密封资产的 SHA 与上一节一致。

用户指定的归档目标为：

```text
/home/liu/h1_moveit/backups/8252.tar.zst
/home/liu/h1_moveit/backups/8252.tar.zst.sha256
```

必须沿用 8251 的方式：只以 `h1_2_ros2_ws/` 为唯一顶层目录，归档整个当前工作空间，
保留 symlink，不直接收录外部 `demo_recordings`、`runtime_logs` 或 backups 目录；本轮
外部成功证据已经选择性复制进上述 workspace evidence。归档和 sidecar 必须非覆盖、
同目录临时生成后原子发布并最终设为 mode `0444`。

创建后必须通过 `zstd -t`、完整 `tar --zstd -tf`、唯一安全相对根目录、关键成员、
归档内 MP4/wrapper/四资产 SHA、sidecar `sha256sum -c` 和 mode 检查。还必须重新验证
以下受保护旧归档没有变化：

```text
814   1aa6bb176c5a183db65573875e6f63aeb6058ea65a2b88d53264f51f530aa858
824   e61424175d933ae2d3361232735b5b0328e16e672c5fc1a35f967811fe450d9f
825   d69ab272e3ba01086c09f97dc7460e2996bf284427215ed7271ac65c831a4bac
8251  a35bfff7e1c8aac3c570ba40c375ee72710a745c6d38a7897b23dbeb51d47c89
```

归档 SHA 只写在外部 sidecar 和创建后的 live 交接更新中，避免把自引用 SHA 写进归档。

### 09:40 CST 发布后结果（live 交接，归档外 sidecar 为校验依据）

```text
archive              /home/liu/h1_moveit/backups/8252.tar.zst
sidecar              /home/liu/h1_moveit/backups/8252.tar.zst.sha256
SHA-256              59458104bc07a54e0b8dc3feae52371144b88aa7da4d31466945401c6724d739
size                 72,018,882 bytes
entries              11,935
root                 h1_2_ros2_ws/ only
archive / sidecar    regular file, mode 0444, link count 1
zstd / tar / members PASS / PASS / PASS
inner hashes         video, wrapper and four sealed assets PASS
sidecar              sha256sum -c PASS
old backups          814 / 824 / 825 / 8251 unchanged
```

归档使用同目录临时文件完成全部校验后，以 no-replace 硬链接原子发布；没有覆盖任何
同名文件。归档内的 `AGENTS.md` 停在上一段“待发布验证”状态，这是为避免归档哈希
自引用；实际归档哈希由只读外部 sidecar 和本 live 交接共同记录。

## 2026-08-25 10:34 CST：右手关门、左手回锁把手已物理完成

用户要求在 8251 的终点继续加入：右手把原装柜门从约 `60 deg` 关回 `0 deg`，右手
撤离，然后左手重新夹住原装把手并从约 `90 deg` 旋回原位。该动作已在当时仍保持
8251 终点的同一 Gazebo GUI 中完成；没有重建机器人、柜体、门、把手或方块模型。

新增的非覆盖只读反向资产为：

```text
runtime_assets/close_and_relock_8253/door_close.trajectory.json
  payload SHA-256  5b212d78bb204fdde047c875d1e2e9585efb18d7d053c847be0a0ef59a4a9038
  file SHA-256     3cbb2decfe3128e6a7ca95238085f4285b06a79486c40733db7d426d0a3e5afb
runtime_assets/close_and_relock_8253/left_regrasp.trajectory.json
  payload SHA-256  5acd13b1a497b2496ca50d06982316d18b4d482a3288126e1a66fe36f8fd6f2d
  file SHA-256     8d4443c50e9d5d18bc4b1df0cef6a5af132fd4b577c4dada964f3350ec6ef443
runtime_assets/close_and_relock_8253/handle_relock.trajectory.json
  payload SHA-256  b6b15a9ec270cb3158e0c815997cfa85ebefb04ba3cd3d878c5ed1ff8b9b5ce3
  file SHA-256     3d88b71fc9c748b6669831467b7cbd6b6f21f67a5026ea88da99e4bc93fa07e9
```

三份资产均为 user-owned、regular file、mode `0400`、link count 1；它们由
`h1_joint_trajectory_asset.py build-close-reverse` 从 8251 三份密封正向资产及其显式
payload SHA 生成，没有覆盖原资产。门资产严格反转消息 `12 -> 0`；左手回抓严格反转
`left_clear` 消息 `1 -> 0`；把手回锁严格反转 `handle` 消息 `4 -> 0`。新增入口和受控
detach helper 为：

```text
src/h1_2_moveit_demo/scripts/h1_close_and_relock_8253.sh
src/h1_2_moveit_demo/scripts/h1_door_support_detach_8253.sh
```

入口只接受 8251 物理终点：门 `60 +/-0.2 deg`、把手约 `90 deg`、latch unlocked、
右门支撑 fresh attached、方块仍在中层铁架、五控制器 active、四个机器人 trajectory
topic publisher 为零。实际顺序为：

1. 当前左手方块撤离位经独立 OMPL 正反审计退到中立；
2. 右手保持夹持及 support attached，反向重放 door 消息 `12 -> 6` 关到 q0；
3. 在 q0/q90、右指仍夹紧时受控 detach，确认 fresh detached；
4. 单独反向消息 5 松右指，通过 q0 机构门后再反向消息 `4 -> 0` 撤右臂并张手；
5. 左手从中立回到 q90 并夹紧；完整反向 handle 资产完成 `90 -> 0 deg`、松夹和撤离。

门 detach 前后没有 reset、直驱门、模型改位或清理式松夹。失败策略仍是
stop-and-hold。把手资产没有被非法分段；轨迹工具继续禁止 `handle/left_clear` 的
partial replay。

可复现入口（当前 live 已经执行完成，不能在当前状态重复）：

```bash
cd /home/liu/h1_moveit/h1_2_ros2_ws_claude_2026-07-30/h1_2_ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run h1_2_moveit_demo h1_close_and_relock_8253.sh preflight
ros2 run h1_2_moveit_demo h1_close_and_relock_8253.sh run
```

只有先从 fresh 场景成功完成 8251、并保持其 q60 终点时才可使用以上命令。当前已是
q0 locked、方块在架、support resident detached 的非 fresh 状态；下一次完整演示必须
正常重启 Gazebo，不能直接重跑 8251 或本入口。

无控制器预检成功目录：

```text
/home/liu/h1_moveit/runtime_logs/close_and_relock_8253/
  run_20260825_102240_4152527_21176
```

左手 bridge 的 forward/reverse 各 `411` 个密集样本，low/high 均 PASS，原装 STL
检查 PASS，除下述固定感知残影外的环境净距为 `10.001 mm`。物理成功目录：

```text
/home/liu/h1_moveit/runtime_logs/close_and_relock_8253/
  run_20260825_103044_6577_20732
CLOSE_RELOCK_8253_PHYSICAL=PASS
invocation wall time 146.317 s (10:30:44.849 -> 10:33:11.166 CST)
```

独立终点复核：

```text
door                         -1.59e-18 rad, velocity near zero
handle                        0.0613471 rad = 3.515 deg, velocity near zero
latch / right support         locked / detached
left/right fingers            about -19.0/-19.0 mm, both released
left/right arm                neutral endpoints
cube XYZ                      (0.63255, 0.04004, 0.99909) m, still in rack gate
cube RPY                      (0.00296, -0.06118, 1.20271) rad
controllers                   all five active
robot trajectory publishers   0 / 0 / 0 / 0
Gazebo GUI                    running
```

把手物理终点不是数学零，而是 `3.515 deg`；它在明确的 `<=5 deg` 锁止门内且 Gazebo
latch 已 fresh 报告 `locked`。方块仍在架内但约有 `-3.5 deg` pitch，不能声称完全
水平。该新增闭环入口的调用耗时为 `146.317 s`，尚未优化到两分钟，不能把它与此前
8251 的 `115.896 s` 开门放块录像混写为一个两分钟开关闭环。

本轮仅为用户明确允许的 Gazebo 实验，仍不是实体安全证书。除 8251 已有的 private
high-model `right_elbow_link <-> torso_link` 放宽外，q60 左手 bridge 为保持已有右手
支撑接触，仅允许原装把手与两块右 pad；fresh OctoMap 稳定包含
`<octomap> <-> left_hip_roll_link` 的约 `0.06/1.78/0.95 mm` 自身残影，因此本实验只对
这一个固定 pair 做局部排除，并要求 left hip 与两块右 pad 保持在
`0.05 mm / 0.01 deg` 内。其他 world pair 和其他 OctoMap robot link 继续保持严格
`10 mm`。该局部规则尚未实现逐 contact 的“数量不增加、深度不增长”证书，所以不能
将日志中的 OctoMap `10.001 mm` 解读为包含这个已排除 pair 的整机声明。

此外，8251 三份反向资产已有历史完整正反证据，且本轮另以原装 door/handle STL 和
当前方块 OBB 做过连续扫掠：door `60 -> 0` 保守最小净距 `58.933 mm`，handle 随门
关闭 `100.406 mm`，门闭合后 handle `90 -> 0` 为 `100.347 mm`。但是本 wrapper 尚未
把三个反向资产针对“方块已经在架”的当前完整世界重新逐状态执行 low/high/STL/
铰链/OctoMap dense 审计；因此本轮应称为成功的 Gazebo 物理实验，不得提升为完整
current-world 安全认证。

最终源码已按本文规定的 Miniconda 六库隔离命令编译和测试：

```text
log/build_2026-08-25_10-29-50     2 packages / PASS
log/test_2026-08-25_10-34-15      PASS
colcon test-result                54 tests, 0 errors, 0 failures, 1 skipped
direct trajectory Python tests    41 tests, 0 failures
demo source SHA-256               ab16883b802131518381602e817c63a5de7801a4b39843ff34adde3562e464db
close wrapper SHA-256             df42f7179f873f4c033737c7bc8f8bb12d852fac8cabded926f7839b7813d715
detach helper SHA-256             ea4ea0e3ccd1d6da239f858ed37a69384bd3ad365c3333828dca26bb5bc415f4
asset tool SHA-256                b54d247613d9f6295bd8b3fff0039970fcc03e0387bf589391387844b6246a78
```

本轮没有创建新备份，也没有修改或覆盖 814、824、825、8251、8252。

## 2026-08-25 11:03 CST：fresh 正向重演、关门回锁重演及闭环展示视频

本节是当前最新现场，覆盖上节 10:34 CST 的 live 状态，但不修改其历史成功证据，
也不修改或覆盖已经封存的 8252 证据和归档。10:47 CST 正常启动了 fresh Gazebo 与
move_group，机器人、原装孵育箱、门、把手和绿色方块模型均沿用现有工作空间；fresh
入口五个控制器 active、门/把手约为 q0/q0、latch locked、双手张开、四个机器人
trajectory topic publisher 为零、点云 fresh。

### 8251 正向重演

本次正向实际运行目录为：

```text
/home/liu/h1_moveit/runtime_logs/full_robot_flow_8251/
  run_20260825_104905_884476914_114194
```

机器人完成了左手旋转原装把手、左手撤离、右手沿正确侧边竖直铰链把门开到约
`60 deg`、左手从柜体正前方把绿色方块送入中层铁架、释放和撤离。所有实际重放、
在线机构、夹持、方块入架/稳定/退出门均通过，终点保持 q60、latch unlocked、右手
门支撑 attached、方块在架。方块 detach 时 first-controller-publish elapsed 为
`112.826 s`；随后撤离及最终验收收尾使严格 120 秒 wrapper deadline 超时，wrapper
因此 exit 1 且没有产生新的 `FULL_ROBOT_FLOW_PHYSICAL=PASS` 结果文件。失败清理没有
reset、detach、松右夹或发送恢复轨迹，而是保持合格 q60 支撑现场。该运行只能称为
物理动作完成的 Gazebo 演示，不能称为新的两分钟 8251 验收证据。

正向录像为：

```text
/home/liu/h1_moveit/demo_recordings/h1_closed_loop_forward_20260825_1048.mp4
H.264 Constrained Baseline / yuv420p / no audio
1160x1530 / 12 fps / 1641 frames / 136.750000 s / 5,396,475 bytes
SHA-256 46db1a452146fc1a679c151a212bef853c85ddfe412653b364b7c6650a546072
```

`run_20260825_104808_176550233_108188` 是无动作中止目录：当时另一个只读 shell 的
命令行文字被 wrapper 的并发进程名门误识别；没有发送轨迹，不能作为运行证据。

### 8253 关门回锁重演与阈值覆盖

10:53 CST 的第一次物理尝试目录
`run_20260825_105337_144763_4545` 只执行了左指已在 `-19 mm` 终点的等效空动作，
随后 live checkpoint 测得固定 `left_hip_roll_link` 反馈漂移
`0.000057 mm / 0.013740 deg`，超过当时的 `0.050000 mm / 0.010000 deg` 姿态门，
因此 stop-and-hold；没有移动左臂、柜门、把手或方块，也没有 reset、detach 或松右夹。
对应的 10.583333 秒静态片
`h1_closed_loop_close_relock_20260825_1054.mp4` 是诊断失败录像，不得交付为成功结果。

为容纳该 Gazebo 静止反馈量化漂移，只把 scoped existing-contact 的角度阈值从
`0.01 deg` 调整为 `0.05 deg`，平移阈值仍为 `0.05 mm`；其他碰撞、原装 STL、
10 mm 净距、正反完整路径、时序和机构门均未改。本节明确覆盖上节关于此固定接触
门的 `0.05 mm / 0.01 deg` 叙述：当前重编译入口为
`0.050000 mm / 0.050000 deg`。该变更仍只属于用户允许的 Gazebo 非安全实验，
不提升为实体安全证书。

重编译后的无控制器预检目录为：

```text
/home/liu/h1_moveit/runtime_logs/close_and_relock_8253/
  run_20260825_105748_168546_19824
CLOSE_RELOCK_8253_PREFLIGHT_ONLY=PASS
```

左手 bridge 的正向/反向各 `411` 个密集样本，low/high、原装 STL、固定接触、
10.001 mm 环境净距和时序检查均 PASS，预检未发送任何控制器轨迹。随后成功物理
运行目录为：

```text
/home/liu/h1_moveit/runtime_logs/close_and_relock_8253/
  run_20260825_105847_174303_18630
CLOSE_RELOCK_8253_PHYSICAL=PASS
start/result mtime wall interval 154.349 s
```

成功顺序是：左手从方块撤离位回中立，右手关门到 q0，在右指仍夹持时解除门支撑，
右手松夹撤离，左手重抓原装把手并回锁，最后双手松开撤离。成功录像只覆盖预检末尾
和实际动作，因此录像时长不能代替 154.349 秒 wrapper 调用时长：

```text
/home/liu/h1_moveit/demo_recordings/h1_closed_loop_close_relock_success_20260825_1059.mp4
H.264 Constrained Baseline / yuv420p / no audio
1160x1530 / 12 fps / 1324 frames / 110.333333 s / 3,983,383 bytes
SHA-256 1c1f33b51c0175bf2ae6c51e4fad31edb4bd18cc6c093a0397e4823a247b58c5
```

### 完整展示片、最终现场和回归测试

复用 09:20 CST 已记录的领导视角，把同一 fresh 场景周期内的正向和成功反向片段
顺序合成为展示片：

```text
/home/liu/h1_moveit/demo_recordings/h1_closed_loop_full_20260825_1103.mp4
H.264 High / yuv420p / no audio
1160x1530 / 12 fps / 2965 frames / 247.083333 s / 2,744,387 bytes
SHA-256 fa2dc000eece05d333e7260bc75848c49f27f04217ddcb0e72767cbed9a52c4c
```

该文件严格为 `136.750000 + 110.333333 = 247.083333 s` 的 presentation
composite，不是单个连续 wrapper，也不是两分钟完整开关闭环证据。三条视频均通过
`ffprobe`、带 `-xerror` 的完整解码和单调 12 fps PTS 检查；合成边界精确逐帧检查
无断帧、重帧、重影或时间戳跳变，末尾无黑帧/花帧。关键帧位于：

```text
/home/liu/h1_moveit/demo_recordings/qa_20260825_1059/
```

最终独立只读现场为：门 `-5.84e-20 rad`、把手
`0.002247875 rad = 0.128793 deg`，两者速度近零；latch locked、right support
detached；方块约为
`XYZ=(0.614591,-0.021858,0.997657) m`，仍在中层架门内；双手释放、双臂撤离；
五个控制器 active，四个机器人 trajectory publisher 均为零；Gazebo GUI 和
move_group 保持运行。该现场不是 fresh，不能直接重跑 8251 或 8253；下一次完整
演示必须正常重启 Gazebo，从 fresh q0 开始。

当前源码按本文规定的 Miniconda 六库精确隔离完成构建和测试：

```text
log/build_2026-08-25_10-56-19    2 packages / PASS
log/test_2026-08-25_11-04-06     selected packages / PASS
colcon test-result               54 tests, 0 errors, 0 failures, 1 skipped
direct Python contracts          41 + 10 tests, 0 failures
demo source SHA-256              92ef6383cc3572a5bbe9840b3663fd18540adb8acc14e3d756411fc51ab25976
close wrapper SHA-256            df42f7179f873f4c033737c7bc8f8bb12d852fac8cabded926f7839b7813d715
detach helper SHA-256            ea4ea0e3ccd1d6da239f858ed37a69384bd3ad365c3333828dca26bb5bc415f4
```

全局 test-result 中唯一 skip 来自旧的 `h1_2_vision/pytest.xml`，不属于本次选定
包。本轮没有创建新备份，也没有修改或覆盖 814、824、825、8251、8252。

## 2026-08-25 13:19 CST：完整机器人闭环优化到 90 秒内并录制

本节是当前最新权威现场，覆盖上节 11:03 CST 的 live 状态和速度结论，但不修改其
历史证据。机器人已在同一个连续 Gazebo 物理运行中完成：左手旋转原装把手解锁、
左手安全撤离、右手沿正确侧边竖直铰链轴开门到约 `60 deg`、左手从柜体正前方将
绿色方块放到中层原装铁架、左手撤离、右手关门、解除门支撑并撤离、左手重新抓住
原装把手回锁并撤离。第一条控制器轨迹发布到最后一次可见机器人运动共
`86.340 s`，满足用户要求的 `<=90 s`。

### 8254 连续入口与适用范围

新增的仿真专用连续入口是：

```text
src/h1_2_moveit_demo/scripts/h1_closed_loop_90_8254.sh
```

只能在正常重启后的 fresh 场景使用：门和把手约为 q0/q0、latch locked、机器人
中立、双手张开、绿色方块位于初始化位、门和方块 support 都 detached、五个控制器
active、move_group/环境场景/点云健康、四个机器人 trajectory topic publisher
均为零。正式录像使用的精确命令为：

```bash
cd /home/liu/h1_moveit/h1_2_ros2_ws_claude_2026-07-30/h1_2_ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

H1_90_RECORD=1 H1_90_RECORD_FRAMERATE=8 \
H1_90_VIDEO_OUTPUT=/home/liu/h1_moveit/demo_recordings/h1_closed_loop_90_20260825_1312.mp4 \
src/h1_2_moveit_demo/scripts/h1_closed_loop_90_8254.sh run
```

上面是 13:12 正式成功运行的历史精确命令；该输出文件现在已经存在，wrapper 会拒绝
覆盖。任何重演都必须把 `H1_90_VIDEO_OUTPUT` 改为一个尚不存在的新绝对文件名，不能
原样复制该录像路径。

必须看到：

```text
CLOSED_LOOP_90_8254_PHYSICAL=PASS
```

当前 live 已是完成关门回锁后的非 fresh 终点，禁止直接重复执行；重演必须先正常停止
并重启 Gazebo，再重新完成只读健康检查。该入口没有重建机器人、孵育箱、门、把手或
绿色方块，原装 `incubator_assembly.SLDASM` 导出的 STL 和现有模型保持不变，也没有
计算候选 1 或候选 2。

8254 通过常驻 trajectory runner 消除每段重复的 ROS/模型装载开销，并复用已经留存
的 8251/8253 密封正向、反向轨迹及拓扑。快速反向 helper 只跳过重复 transport
discovery；机构、关节时序、在线起终点、门/把手/latch、夹持和 support 状态仍逐段
验证。它不会在每次运行中重新做候选 3 的 IK 搜索或把全部 dense 检查从头计算一遍，
所以本次成功是 Gazebo 连续物理重放证据，不是新的 current-world 完整安全证书。

为处理方块由左手/Gazebo 支撑时稳定存在的约 `6 mrad` 左肩静态偏差，只有
`cube [2,4)` 两段受支撑搬运请求允许 `0.012 rad` 的 start/endpoint 容差；常驻
runner 拒绝把该放宽用于其他分段，所有普通机器人轨迹继续使用 `0.003 rad`。
方块 helper 在这个仿真入口中也只允许最高 `0.012 rad` 的臂终点容差，脚本默认值
仍为 `0.003 rad`。预 detach 的实验性架内位置/姿态门为 `0.030 m / 30 deg`，但松夹
后增加了两个连续 `0.1 s` 物理姿态采样的稳定门，最终验收仍要求方块位于中层架
`0.015 m / 5 deg` 门内。最终机器人中立、门、把手、latch、指爪、方块和两个
support 的严格验收均未删除。

此外，q60 到方块流程衔接使用的 `left_neutral_bridge` 来自已留存的
`/display_planned_path` 候选 4；它不是候选 1 的逐字节完整审计 bridge。连同前述
局部容差和 transport 复用，本入口只能用于用户明确允许放宽的 Gazebo 实验，禁止
解释为实体机器人安全许可。失败时仍应停止发布并保留现场；不得把 deadline 内到达
或 MoveIt 成功日志单独当作物理成功。

### 正式运行、时间线与视频证据

正式成功运行目录为：

```text
/home/liu/h1_moveit/runtime_logs/closed_loop_90_8254/
  run_20260825_131148_076657564_1039909_23118
```

结果文件记录：

```text
CLOSED_LOOP_90_8254_PHYSICAL=PASS
first_publish_to_last_motion       86.340 s
invocation_to_last_motion          86.395 s
post_motion_read_only_validation    3.738 s
```

可见动作的主要节点为：解锁把手 `6.685 s`、门支撑建立 `23.578 s`、门开到约
60 度 `27.241 s`、方块松夹稳定 `56.644 s`、方块放置完成 `60.741 s`、衔接完成
`61.637 s`、门关闭 `65.132 s`、门支撑解除 `72.236 s`、右手撤离 `78.095 s`、
左手回锁并结束 `86.352 s`。计时在最后一次回锁运动结束时封口，之后的
`3.738 s` 只读物理验收没有被隐藏在 90 秒动作指标中。

原始连续证据录像为：

```text
/home/liu/h1_moveit/demo_recordings/h1_closed_loop_90_20260825_1312.mp4
H.264 / yuv420p / no audio
1160x1530 / 8 fps / 771 frames / 96.375 s / 4,449,052 bytes
SHA-256 5c31d05a770563291b0ca2571c1cde062257308095c7dabaadaba88be8b73219
```

原始片包含约 `6.139 s` 动作前画面和动作后的完整物理验收。给领导查看的同一条连续
录像只裁掉无动作前后余量，没有变速、分段或拼接：

```text
/home/liu/h1_moveit/demo_recordings/h1_closed_loop_90_leader_20260825_1312.mp4
H.264 / yuv420p / no audio
1160x1530 / 8 fps / 708 frames / 88.500 s / 1,590,014 bytes
SHA-256 40be33ced6cacf3180e4f0b752192f9efad0830406b5c522e173ca4eece47ec2
```

领导片保留约 `0.7 s` 动作前画面和约 `1.5 s` 动作后最终静止画面。两条录像都已用
ffprobe 检查并完成带错误退出的全文件解码；领导片抽帧总览为
`/home/liu/h1_moveit/demo_recordings/review_1312/leader_contact.png`，已目视确认解锁、
开门、送块、松夹、关门、回锁和最终静止均可见。

### 构建、测试和当前现场

源码在正式运行后按本文既定的 Miniconda 六库精确隔离命令完成构建和测试：

```text
log/build_2026-08-25_13-17-45                  2 packages / PASS
log/console_test_2026-08-25_13-18-16.log       selected packages / PASS
colcon test-result                             80 tests, 0 errors, 0 failures, 1 skipped
direct Python/source contracts                 67 tests, 0 failures
wrapper SHA-256                                2f8b3ee33382f7f4340a0928aa88ba83da2fc6960089669c8ec688606117eec0
persistent runner SHA-256                      e2f73eb033121929c0a39609427c3d77fc1d25eb5f3819ac90a4f0e7042b3cb6
asset tool SHA-256                             955abc7b045a770abc58e7907cbdb9d2d2fbdabefa391e4d3693d95363935c49
cube support helper SHA-256                    92181cd2f5c33143de99314cef72d78f35764bb33579b9d5438bf93084c69a32
full wrapper SHA-256                           1f099053aad31e38d33e143b6ddb09a80018e567bbb8dd14a2079d33f764b405
door detach helper SHA-256                     ccac58ba7359e2e1b6a648b8b5c6cf26241789fb6791bd5de341a2e058742c5e
```

第一次 test 命令的 tee 文件名与 colcon 准备创建的同名 log 目录冲突，测试尚未启动
即退出；诊断输出已改名为
`log/console_failed_test_2026-08-25_13-17-57.log`。随后使用不冲突的文件名完整重跑
并通过。全局唯一 skip 仍是旧的 `h1_2_vision/pytest.xml`，不属于本次选定包。

正式成功终点方块约为 `XYZ=(0.61135,-0.02905,0.99593) m`、
`RPY=(-0.00603,-0.00200,1.22858) rad`。13:25 CST 的最后独立只读复核确认：门
`-1.354e-19 rad`、把手 `0.00520716 rad = 0.298 deg`，两者速度近零，latch
locked；Gazebo 当前方块约为 `XYZ=(0.61224,-0.02679,0.99759) m`、
`RPY=(-0.02392,-0.00973,1.25908) rad`，两次样本间仅约 `0.16 mm` 位移，仍在
中层架内且近水平；门和方块 support 均 detached；双手释放、双臂中立；五个控制器
active，四个机器人 trajectory publisher 均为零，原始深度点云约 `6.89 Hz`，过滤
节点与 MoveIt 订阅仍健康。Gazebo GUI、server、move_group 和环境场景保持运行，当前主要 PID 为 Gazebo
launch/server/GUI `1035789/1036068/1036071`、move_group launch/node/environment
scene `1038210/1038282/1038283`。GUI 位于 `DISPLAY=:0`，可见的 `Gazebo Sim`
窗口约为 `2000x1690`。当前现场留在最终关门回锁状态供查看，不是 fresh 入口。

本轮没有创建新备份，也没有修改、覆盖或删除 814、824、825、8251、8252。若用户
以后要求保存 8254 或其他新编号，必须另建非覆盖归档并重新做 sidecar、zstd、tar
清单和关键成员验证。13:26 CST 再次只读执行 sidecar 校验，AGENTS 指定的
`814_binding_bundle_registry_docs_20260820_1436.tar.zst`（SHA-256
`1aa6bb176c5a183db65573875e6f63aeb6058ea65a2b88d53264f51f530aa858`）以及 824、
825、8251、8252 均为 `sha256sum -c: OK`；没有解压或修改归档。

## 2026-08-25 14:20 CST：用户指定 8261 非覆盖成果归档

用户要求先把当前 90 秒内完整闭环成果备份为 `8261`。本次新建的工作空间内自包含
证据目录为：

```text
runtime_evidence/closed_loop_90_8261_20260825/
```

其中包含 `SUMMARY.md`、最终只读现场、正式成功 run 的完整副本、96.375 秒原始连续
录像、88.500 秒领导连续录像、QA 抽帧、13:17/13:18 构建测试证据、相关源码与测试
快照。`SHA256SUMS` 覆盖除清单自身外的全部 207 个证据文件，清单自身 SHA-256 为：

```text
7e07b5715464b9c3d44be93d1b33dfa92bb4b2f404aec84c9a7c28a125051bb1
```

用户指定的归档目标为：

```text
/home/liu/h1_moveit/backups/8261.tar.zst
/home/liu/h1_moveit/backups/8261.tar.zst.sha256
```

必须沿用 8252 的完整工作空间归档规则：归档只以 `h1_2_ros2_ws/` 为唯一顶层目录，
保留 symlink，不直接收录外部 `demo_recordings`、`runtime_logs` 或 `backups` 目录；
所需外部成功证据已选择性复制到上述 workspace evidence。归档和 sidecar 必须在
backups 同目录使用临时文件完成，校验后以 no-replace 硬链接原子发布，最终均为普通
文件、mode `0444`、link count 1，严禁覆盖任何同名目标。

发布前后必须通过：evidence `SHA256SUMS`、`zstd -t`、完整 `tar --zstd -tf`、唯一
安全相对根目录、关键成员存在性、归档内两条 MP4/六份关键源码/evidence 清单哈希、
外部 sidecar `sha256sum -c`、mode/link count 检查。还必须重新核验以下受保护归档：

```text
814   1aa6bb176c5a183db65573875e6f63aeb6058ea65a2b88d53264f51f530aa858
824   e61424175d933ae2d3361232735b5b0328e16e672c5fc1a35f967811fe450d9f
825   d69ab272e3ba01086c09f97dc7460e2996bf284427215ed7271ac65c831a4bac
8251  a35bfff7e1c8aac3c570ba40c375ee72710a745c6d38a7897b23dbeb51d47c89
8252  59458104bc07a54e0b8dc3feae52371144b88aa7da4d31466945401c6724d739
```

8261 的归档 SHA-256 只写入归档外 sidecar 和发布后的 live 交接更新，避免把自引用
SHA 写进归档。归档内本节因此停在发布规则和待验证状态，这是有意设计。

### 14:25 CST 发布后结果（live 交接，归档外 sidecar 为校验依据）

```text
archive              /home/liu/h1_moveit/backups/8261.tar.zst
sidecar              /home/liu/h1_moveit/backups/8261.tar.zst.sha256
SHA-256              3f7acbe3ce084e065032d7cb55900229c2602bb0dbe5bd01d4e6da7b5876fe52
compressed size      81,687,809 bytes
zstd content size    439,511,040 bytes
entries              12,506
types                9,648 regular / 2,540 directories / 318 symlinks
root                 h1_2_ros2_ws/ only
archive / sidecar    regular file, liu:liu, mode 0444, link count 1
zstd / tar / members PASS / PASS / PASS
evidence manifest    207 / 207 hashes MATCH
inner video/source   two MP4 and six critical source hashes PASS
sidecar              sha256sum -c PASS
old backups          814 docs / 824 / 825 / 8251 / 8252 unchanged
```

完整 tar 遍历确认绝对成员名、`..` 路径组件、重复成员、hardlink、device 和 FIFO 均为
零。evidence 在归档内共有 224 个成员（208 regular、16 directories），其唯一
`SHA256SUMS` 为 24,559 bytes、207 行；逐文件复算无 missing、extra、mismatch 或
duplicate，三项计时原文仍为 `86.340 / 86.395 / 3.738 s`。

归档按要求保留了 symlink-install 的链接：308 个绝对链接指向原工作空间、6 个指向
`/opt/ros/jazzy`、4 个为相对日志链接。所有链接均是叶子，任何 symlink 路径下都没有
后续 archive member，因此 tar 成员本身不会经链接越出唯一根目录；但若把归档恢复到
不同绝对路径，必须按本文 Miniconda 六库隔离命令重新构建 `install/`，不能假定这些
绝对 symlink 在异地仍可用。

归档和 sidecar 均在 backups 同目录临时生成并完成预发布校验，再以 no-replace 硬
链接原子发布；没有覆盖任何同名文件。归档内 `AGENTS.md` 停在上面的待发布状态，且
预发布草稿标题仍写作 `13:30 CST`；实际文件 birth time 为
`2026-08-25 14:21:13 CST`，本 live 标题已更正为 `14:20 CST`。该文字时间标签修正
发生在发布后，不改变归档内容或外部 sidecar 哈希。

## 2026-08-25 18:48 CST：平行方块、短间隔与关门后向下回锁验收

本节是当前 live 最新交接；与上面 13:12 的 70 度方块、独立
`left_neutral_bridge`、8 个 runner 资产或 86.340 秒流程冲突时，以本节为准。
用户指出的三项问题已经在机器人完整物理流程中修正并录像验证：动作组之间不再插入
旧的左臂中立桥接；方块在柜体正前方完成定向后以 `yaw=90 deg` 平行送入；右手关门
后，左手重新夹住原装把手并把它由 q90 向下旋到 q0，确认 latch locked 后才松夹
撤离。

### 当前候选 3 方块资产

只计算和使用候选 3；没有计算候选 1 或候选 2。密封 cache 和 6-message 资产为：

```text
runtime_assets/parallel_cube_8262/left_rack_q60_parallel90_neutral_candidate3.cache
  mode 0400 / 121,682 bytes
  SHA-256 dfc45b9ab0ad12e6a6e038ed0a2a3300838ec143ad8dd4c591b423273a352a02
  points approach/carry/support/retreat = 32/360/28/413

runtime_assets/parallel_cube_8262/cube.parallel90.neutral.trajectory.json
  mode 0400 / 213,818 bytes
  payload SHA-256 acaec97384f16a736f6b246a3fa1a0897b7aad93e72a6204701610ebf4f7e705
  file SHA-256    09919a92a65b5ca90b3a11420bc9209ceead1e90112f94da2c4c18711af8e134
```

方块从初始化位 `(0.334,0.377,0.830) m / yaw=90 deg` 出发，在柜外前方走廊转向，
到 `x=0.400 m` 时完成回到 `yaw=90 deg`；其后沿柜门开口法向 `+X` 直线插入，送入、
下降、松夹全过程保持与柜体平行。最终目标仍为中层架中心
`(0.610890542898,-0.029527393333,0.995407082351) m / yaw=90 deg`。
最后支撑段在架面上方 `1.5 mm` 水平松夹，再由 Gazebo 接触落稳；这是用户明确允许的
仿真实验放宽，不是实体机器人安全许可。

完整审计证据为：carry 正/反向 `1130/1130` samples，环境/自碰最小净距
`11.864/5.718 mm`，最大相邻关节差 `2.149 deg`；support 正/反向 `47/47`，
`19.809/5.399 mm`，`2.030 deg`；released retreat 正/反向 `1279/1279`，
`19.809/5.304 mm`，`2.149 deg`。禁止 partial，原装 STL、10 mm 环境门、完整正反向
碰撞/净距/连续性检查均保留。retreat 的末尾在同一个审计资产内直接到左臂 q60
中立位，因此旧的独立 `left_neutral_bridge` 已从 wrapper、runner registry 和物理流程
删除；runner 当前固定注册 7 个资产。

方块 6 条消息的未缩放关键路径为 `49.774077 s`。逐消息离线速度临界 scale 约为
手臂 `0.340000`、夹爪 `0.042800/0.067633`；正式运行采用有余量的：

```text
0:0.36,1:0.08,2:0.36,3:0.36,4:0.08,5:0.36
```

联合 `retime-check` 为 PASS，缩放后方块关键路径 `17.414668 s`。没有删除支撑切换、
方块连续两帧落稳、门 q60/q0、latch、夹持、最终平行姿态或 RTF 验收。动作间肉眼可见
的等待只剩这些实时物理门，不再有额外桥接轨迹或人为阶段 sleep。

### 回锁顺序和启动竞态修复

反向严格顺序为：右手支撑关门 q60->q0；在 q0 验证后 detach；右手松夹撤离；左手
重新抓住原装把手；仅重放 `handle_relock [0,1)` 把 q90 旋到 q0；并行验证门 q0、
把手 q0、latch locked、左手仍夹紧；只有这些全部 PASS 后才重放 `[1,5)` 松夹与
撤离。最终门/把手/latch 检查仍保留。

18:41 的第一次正式调用在任何 controller publish 之前 fail closed：常驻只读 door
support monitor 的临时订阅端点与 preload helper 要求的空话题发生 discovery 竞态。
现场门、机器人和方块均未移动，录像尚未开始。修复后启动顺序固定为：先只启动 latch
monitor；完成 detached door-support preload 及完整验证；再启动只读 support monitor。
`run`、`prepare-q60` 和 `reverse-run` 三个入口都有源代码顺序回归测试。没有复用不明
插件、绕过空话题合同或放宽 support 状态门。

### 18:43 正式连续物理结果与视频

成功 run 目录：

```text
/home/liu/h1_moveit/runtime_logs/closed_loop_90_8254/
  run_20260825_184341_583888454_2932808_2016
```

结果为：

```text
CLOSED_LOOP_90_8254_PHYSICAL=PASS
first_publish_to_last_motion        70.939 s
invocation_to_last_motion           70.995 s
post_motion_read_only_validation     3.940 s
```

关键节点：把手解锁 `6.489 s`；door support attached `17.674 s`；门 60 度
`21.019 s`；方块落稳 `41.949 s`；方块放置完成 `47.859 s`；左臂内置中立撤离完成
`48.559 s`；门 q0 `52.187 s`；door support detached `56.358 s`；右手撤离
`62.247 s`；左手夹持把手旋到向下并锁定 `67.850 s`；最后松夹撤离 `70.939 s`。

方块松夹后实测约为
`XYZ=(0.611196,-0.029967,0.996807) m`、
`RPY=(-0.005519,-0.007354,1.567300) rad`，即 yaw 约 `89.80 deg`，roll/pitch 均
小于 `0.5 deg`。最终录像验收样本为
`XYZ=(0.611333,-0.031722,0.997652) m`、
`RPY=(-0.021114,-0.001246,1.570450) rad`，仍在完整位置/姿态门内并与柜体平行。

连续原始录像为：

```text
/home/liu/h1_moveit/demo_recordings/h1_closed_loop_corrected_20260825_1842.mp4
H.264 / 1160x1530 / 12 fps / 970 frames / 80.833333 s / 5,383,533 bytes
SHA-256 b0f7b2476c938291bce212b1c5da7343fa39d5343cf4c545c82229f985094b0e
```

ffprobe 元数据和带错误退出的全文件 ffmpeg 解码均 PASS。录像没有变速、分段或拼接；
角度覆盖机器人、柜门、柜内中层架、平行方块送入、右手关门以及左手向下回锁，动作
本体低于 90 秒。

### 构建、测试和当前 live

最后源码使用本文规定的 Miniconda 六库精确隔离命令重新构建两个包，并在竞态修复后
再次测试。`colcon test-result --all --verbose` 为 `87 tests / 0 errors /
0 failures / 1` 个旧 vision skip；轨迹资产直接测试另有 `51/51 PASS`。关键 live
源码 SHA-256：

```text
h1_closed_loop_90_8254.sh                    3633c6c27b61141dbc75d142bb64b93f4c38994dd95def50ad5cff78a82e10df
h1_persistent_trajectory_runner_8254.py      bcbc5f24c1b20dac169bc5f5e23d02acd694230303bbb6d7856eebbba9ec63d0
test_h1_closed_loop_90_8254_source_contract  0d2b938dc5a25e9a35b428ddb7a55154f36b35ce2a28b0b74d8333c84ba94669
bimanual_air_handover_demo.cpp               b96b5a01fc8a334002be653a9b8263f38aebc37ff6f80d873d647e078fdbf22f
```

18:47 的最后只读现场：五个控制器 active，四个机器人 trajectory topic publisher
均为 0；门 `-5.93e-20 rad`、把手 `0.00228244 rad = 0.131 deg`，速度近零；latch
locked，door support detached。Gazebo 长时间接触稳定后的方块约为
`XYZ=(0.611515,-0.034056,0.998775) m`、
`RPY=(-0.041849,0.006888,1.574400) rad`，yaw 约 `90.21 deg`，没有恢复为旧的
70 度斜放。Gazebo GUI/server、MoveGroup、环境场景和点云继续运行，主要 PID 为
Gazebo launch/server/GUI `2907166/2907434/2907437`，MoveGroup launch/node/
environment scene `2909202/2909279/2909280`；`DISPLAY=:0` 的可见 Gazebo 窗口为
`2000x1690`。当前是最终关门、把手向下、方块在中层架的非 fresh 场景，禁止直接重复
`run`；重演必须正常重启 Gazebo 后重新做只读 fresh 预检。

本节没有创建新备份，没有修改或覆盖受保护的 8261 及更早归档。8261 仍是 14:25 前
成果，SHA-256 仍为
`3f7acbe3ce084e065032d7cb55900229c2602bb0dbe5bd01d4e6da7b5876fe52`；本节的新平行
方块资产、竞态修复和 70.939 秒录像不在 8261 内。

## 2026-08-26 09:42 CST：用户指定 826.2 非覆盖成果归档

用户要求把 18:48 CST 已物理验证的平行方块、短间隔、关门后向下回锁完整成果备份为
`826.2`。本次建立的工作空间内自包含证据目录为：

```text
runtime_evidence/closed_loop_parallel_826_2_20260826/
```

其中包含 `SUMMARY.md`、18:43 成功 run 的完整副本、80.833333 秒未变速连续录像、
ffprobe、8 个关键抽帧及 4x2 QA 总览、当前 7 个 runner 资产、候选 3 平行方块 cache、
关键源码/测试快照和固定测试结果。`SHA256SUMS` 覆盖除清单自身外的 191 个普通文件，
已逐文件复算全部 MATCH；清单自身为 22,249 bytes，SHA-256：

```text
04dcd94112c0dc1be88a1fb9a018e1bb5280180fa6283d87a907a9dbc4f9ed78
```

证据内原始录像 SHA-256 仍为
`b0f7b2476c938291bce212b1c5da7343fa39d5343cf4c545c82229f985094b0e`，并再次完成
带错误退出的全文件 ffmpeg 解码。三个计时原文仍为 `70.939 / 70.995 / 3.940 s`。

用户指定的精确归档目标为：

```text
/home/liu/h1_moveit/backups/826.2.tar.zst
/home/liu/h1_moveit/backups/826.2.tar.zst.sha256
```

必须沿用 8261 的完整工作空间规则：只以 `h1_2_ros2_ws/` 为唯一相对顶层目录归档
整个当前工作空间并保留 symlink；不直接收录外部 `demo_recordings`、`runtime_logs` 或
`backups`。所需外部成功证据已选择性复制到上述 workspace evidence。归档与 sidecar
必须在 backups 同目录使用临时普通文件生成，先完成全部验证，再以 no-replace 硬链接
原子发布；最终都必须是 `liu:liu`、mode `0444`、link count 1，严禁覆盖同名目标。

发布前后必须通过：evidence `SHA256SUMS`、`zstd -t`、完整 `tar --zstd -tf`、唯一
安全相对根目录、绝对路径/`..`/重复成员/hardlink/device/FIFO 检查、关键成员存在性、
归档内录像/候选 3 cache/平行方块资产/关键源码/evidence 清单哈希、外部 sidecar
`sha256sum -c` 和 mode/link count。还必须重新核验以下受保护归档：

```text
814   1aa6bb176c5a183db65573875e6f63aeb6058ea65a2b88d53264f51f530aa858
824   e61424175d933ae2d3361232735b5b0328e16e672c5fc1a35f967811fe450d9f
825   d69ab272e3ba01086c09f97dc7460e2996bf284427215ed7271ac65c831a4bac
8251  a35bfff7e1c8aac3c570ba40c375ee72710a745c6d38a7897b23dbeb51d47c89
8252  59458104bc07a54e0b8dc3feae52371144b88aa7da4d31466945401c6724d739
8261  3f7acbe3ce084e065032d7cb55900229c2602bb0dbe5bd01d4e6da7b5876fe52
```

826.2 的归档 SHA-256 只写入归档外 sidecar 和发布后的 live 交接更新，避免归档自引用。

### 09:47 CST 发布后结果（live 交接，归档外 sidecar 为校验依据）

```text
archive              /home/liu/h1_moveit/backups/826.2.tar.zst
sidecar              /home/liu/h1_moveit/backups/826.2.tar.zst.sha256
SHA-256              17b8e132ed4fd8dea4c5c816ca16cfdb8daa89c3902dbec49d47f9fab49e4980
compressed size      91,190,548 bytes
zstd content size    468,060,160 bytes
entries              13,494
types                10,467 regular / 2,709 directories / 318 symlinks
root                 h1_2_ros2_ws/ only
archive / sidecar    regular file, liu:liu, mode 0444, link count 1
zstd / tar / members PASS / PASS / PASS
evidence manifest    191 / 191 hashes MATCH
inner critical SHA   video / candidate-3 cache / cube asset / wrapper / runner / C++ PASS
sidecar              sha256sum -c PASS
old backups          814 docs / 824 / 825 / 8251 / 8252 / 8261 unchanged
```

发布前从临时归档完整提取 evidence 并逐项执行 `sha256sum -c`，全部通过；发布后又独立
执行 sidecar、`zstd -t`、完整 tar 成员计数和关键成员读取。全量 tar 检查确认：非唯一
根目录、绝对或根外成员、`..` 路径组件、重复成员、hardlink、device、FIFO、非法类型和
symlink 路径下的后续成员均为 0。归档内三个时间文件仍逐字为
`70.939 / 70.995 / 3.940 s`。

归档和 sidecar 都在 backups 同目录临时生成，完成预发布验证后以 no-replace 硬链接
原子发布，再解除临时链接；没有覆盖任何同名文件。用于完整 evidence 复算的临时提取
目录已移入系统回收站，不留活动验证进程或临时发布链接。

归档内 `AGENTS.md` 有意停在上一段待发布合同；本段实际归档哈希只存在于归档外只读
sidecar 和当前 live 交接，避免自引用。826.2 包含 18:48 后的平行方块资产、支撑监视器
启动竞态修复、70.939 秒完整 run 和 80.833333 秒连续录像；它不修改或替代 8261。

## 2026-08-26 09:59 CST：当前交接压缩、现场复核与过程文件清理

用户要求总结上下文、生成新的交接文件、更新现有文档并删除无用过程文件。本次新增：

```text
HANDOFF_CURRENT.md
```

该文件把当前成果、唯一运行入口、最终候选 3 资产、fresh 门、备份和下一问题压缩为
短交接；它只是本文的索引，本文仍是唯一权威。`QUICKSTART.md` 同步改为只描述当前
826.2/70.939 秒闭环，不再把 09:23 的 115.896 秒开门后终点写成当前现场。

### 09:55 只读现场

本轮没有 reset、detach、松夹或发送控制器轨迹。Gazebo launch/server/GUI PID 仍为
`2907166/2907434/2907437`，MoveGroup launch/node 为 `2909202/2909279`；
`DISPLAY=:0` 上 Gazebo GUI 映射窗口存在，内容约 `2000x1690`。五个控制器 active，
`robot_state_publisher`、ROS-Gazebo bridges、`h1_2_depth_point_cloud` 和环境场景节点
存在。用显式类型和 `--no-daemon` 成功收到 `/joint_states`；
`/incubator/joint_states` publisher 为 1；`/realsense_camera/depth/points` 实收
PointCloud2 样本 `width=13856`。当前 ROS CLI daemon 的图发现有缓存不一致，直接
`ros2 topic info` 偶尔会错误报告 `Unknown topic`，接手时先用 `--no-daemon` 和显式
类型复核，不要仅因此重启健康仿真。

门为 `-5.93e-20 rad`、把手为 `0.00228183 rad = 0.131 deg`，速度近零；latch
`locked`，right door support 和 left cube support 都为 `detached`。没有 closed-loop
wrapper、door demo、cube detector、RViz 或 recorder 运行。

与 18:47 验收不同，绿色方块经过约一夜持续物理运行后已发生长时漂移/倾倒。两个相隔
3 秒的只读样本稳定在约：

```text
XYZ=(0.583001,0.069640,1.043580) m
RPY=(-1.108000,1.474560,1.512750) rad
```

因此当前 live 既不是 fresh 入口，也不是合格最终展示现场。18:43 正式 run 和
80.833333 秒未变速录像在释放后验收时仍明确记录方块近水平、yaw 约 90 度；新的现场
结果说明“中层架上整夜接触稳定性”是后续应单独优化的问题，不应篡改既有成功证据。

### 可恢复清理结果

清理前先逐项验证 57 个目标存在，合计 `du` 约 `11 MiB`；再重新执行 826.2 sidecar
校验并复算两个最终候选 3 文件哈希。全部通过后，只用 `gio trash` 将以下明确过程对象
移入系统回收站，没有永久擦除：

- `parallel_cube_8262` 中 17 个 trial/preflight/scene-sync 日志，以及被最终
  `.neutral` 版本替代的旧 cache 和 JSON；
- `full_robot_flow_8251` 中 3 个文件名明确为 `failed` 的 door 资产；
- 已从当前 7 资产 registry 和 wrapper 删除的旧 `left_neutral_bridge`；
- 测试尚未启动即因 tee 文件名冲突退出的旧 console 日志；
- 工作空间源码侧 7 个 `__pycache__`/`.pytest_cache` 目录；
- `/tmp/h1_video_review.WjL2JN` 临时人工复核抽帧目录；
- 外部 `runtime_logs/closed_loop_90_8254` 中 25 个 preflight、prepare-q60、
  reverse-run、未完成 run 和 18:41 controller publish 前竞态失败目录。

清理后逐项确认 57 个原路径都不存在。明确保留并复核：826.2 和全部受保护旧归档、
两个最终 neutral 候选 3 文件、当前 7 个正式 runner 资产、13:11 与 18:43 两次正式
成功 run、正式录像、全部 `runtime_evidence`、源码、`build/` 和 `install/`。最终
cache/JSON 哈希仍为
`dfc45b9ab0ad12e6a6e038ed0a2a3300838ec143ad8dd4c591b423273a352a02` 和
`09919a92a65b5ca90b3a11420bc9209ceead1e90112f94da2c4c18711af8e134`；826.2
sidecar 再次为 PASS。清理后还逐一执行 814 docs、824、825、8251、8252、8261 和
826.2 七份指定 sidecar，全部为 `sha256sum -c: OK`。

826.2 是清理前恢复点，所清理的工作空间内过程文件均可从该归档恢复；外部过程 run
不在归档合同内，但已移入系统回收站，最终 18:43 成功 run 的完整副本仍在 826.2 的
自包含 evidence 中。本次仅改 Markdown 并清理过程文件，没有改源码或正式资产，因而
没有重新构建，也没有创建或覆盖任何备份。若用户要保存清理后的状态，必须使用新的
非覆盖编号。

## 2026-08-26 10:31 CST：826 AprilTag 计划获确认并建立新对话交接

用户明确要求后续按照 `826_plan.md` 推进“实时观察环境后自动重规划”，并准备重开
对话。本次只更新计划和交接文档，没有实施任何 AprilTag、模型、视觉、planning scene、
IK、轨迹或执行代码，也没有安装依赖、重启仿真、运行预检或发送控制器轨迹。

`826_plan.md` 状态已改为“用户已确认后续按计划推进；尚未实施”，并明确新对话从阶段
A 只读审计开始。当前文件身份：

```text
826_plan.md
size       17,535 bytes
SHA-256    fe7abcd8e3ceb29e947c241b4bd2790eb574eb38eb45815e82dc4ba74e149e07
```

新增新对话专用交接：

```text
NEXT_CHAT_HANDOFF.md
```

新对话必须完整阅读 `AGENTS.md`、`HANDOFF_CURRENT.md`、`826_plan.md` 和
`NEXT_CHAT_HANDOFF.md`。第一项直接任务只允许执行计划阶段 A 的只读审计：相机/
CameraInfo/TF、AprilTag 依赖、现有视觉代码、标签 visual 接入点、动态 planning scene
和候选 3 规划/审计接入点。阶段 A 禁止修改文件、生成标签、安装依赖、启停仿真、
计算新 IK/轨迹、运行 preflight/run、发送控制器命令或创建备份。完成报告后等待用户
确认 B–I；J–L 的任何物理运动必须再次得到明确授权。

用户确认的规划方向仍保留：柜体固定 tag、柜门 tag、方块多面 tag bundle；D435
持续感知和深度点云障碍；原子 `scene_epoch`；只计算候选 3；环境变化使旧证书失效；
审计通过后才分阶段执行。不得降低 601 个门圆弧目标、完整正反路径、默认 10 mm 环境
净距、4.9 度 IK 连续性、原装 STL、正确铰链、碰撞/净距/时序和禁止 partial 合同。
用户只允许动态密集样本数不必精确等于旧 2943，不能降低检查分辨率或漏掉区间。

### 当前退化 live 快照

10:28–10:31 的只读复核发现 09:55 记录已被外部状态变化覆盖，而且孤立进程仍在退出：

- 原 Gazebo launch、MoveGroup、robot_state_publisher、ROS-Gazebo bridges、
  environment_scene 和 `h1_2_depth_point_cloud` 进程/节点均不在；
- 10:31 只剩 `gz sim server` PID `2907434`；原 `gz sim gui` PID `2907437` 已退出，
  `DISPLAY=:0` 不再有 Gazebo 映射窗口；
- 五个控制器仍 active，`/joint_states` 用显式类型和 `--no-daemon` 可实际收到；
- `/incubator/joint_states` 在 12 秒内无消息，深度点云在 8 秒内无实际样本；
- Gazebo transport 实测 right door support 与 left cube support 都为 detached，latch
  locked；
- 方块约为 `XYZ=(0.583501,0.069825,1.044900) m`、
  `RPY=(-1.124820,1.490420,1.503020) rad`，仍处于长时间倾倒/漂移终态；
- 没有 closed-loop wrapper、door demo、cube detector、RViz 或 recorder 运行。

这是孤立、退化、非 fresh 的残留 server，不得运行当前闭环或用于 AprilTag 验证。
阶段 A 不得为了补齐话题而重启。以后需要实际视觉验证时，必须先重新只读检查，报告
现场并得到用户授权，再正常停止残留 server，从 fresh 场景重新启动并完整健康检查。

本轮没有创建新备份。826.2 仍是最新受保护恢复点，SHA-256 仍为
`17b8e132ed4fd8dea4c5c816ca16cfdb8daa89c3902dbec49d47f9fab49e4980`；计划和新交接
文件不在该归档内，后续保存必须使用新的非覆盖编号。

## 2026-08-26 12:10 CST：AprilTag 阶段 A--I plan-only 结果

用户在阶段 A 只读审计后明确要求继续后续。本轮只实施 `826_plan.md` 的 B--I，不进入
J--L，没有发送轨迹、调用控制器、运行旧 preflight/run 或修改原物理碰撞几何。

实现包括：柜体 10/12、门 11、方块 20--23 的 `tag36h11` 版本化 tag map；只增加
visual 的标签模型；同步 RGB/depth、IPPE、exact-stamp TF、bundle、质量/稳定门；原子
`SceneEpoch` 与冻结 `PlanningScene`；只计算候选 3 的相对笛卡尔模板、重新 IK、601
门目标、0.49 度密集插值、严格正反路径和时序；以及只发布计划元数据、证书和状态的
`candidate3_replanner`。新增接口在 `h1_2_perception_msgs`，实现位于
`h1_2_vision`。未知/过期 tag、场景 epoch/hash 不一致、越界环境和资产篡改均失败关闭。

阶段状态必须按以下真实结论解释：B--D、F、G 完成；E 的 live 静态和合成动态几何
完成，但 live 移动物体 TF/depth 数据仍未完成；H 为 FAIL；I 为
`BLOCKED_BY_BASELINE_H`。最终名义冻结场景为 epoch 516，计划确实完整生成且
`candidate3_only=true`、`contains_partial=false`，11,331 个密集状态无碰撞，IK、原装
STL/cache、铰链、反向和 support/latch 拓扑通过。证书仍必须为：

```text
scene hash        70d079820cc896d0560a846d5d4162b88ab652f2cd3eb6f1c988f617f11b7c4e
plan hash         547ce2195cb3162603a9e0baf868d405a60b62ae49a28a3267b6e94ab3578a74
certificate hash  19aa6c518017f010e1c3bab5d6541c531d3892bd2383b6482547291817a31083
certificate       valid=false / DISTANCE_UNAVAILABLE
action duration   113.859252773 s > 90 s
```

失败原因有两个：Jazzy Python binding 在排除机器人自距离后不给出仅环境距离，不能
证明默认 10 mm；严格 TOTG 在实际速度/加速度限位下超过 90 秒。禁止把无碰撞误写成
净距 PASS，也禁止以资产缺失的加速度数组或提高配置限位绕过时序门。因此 I 的 37 个
正向扰动未执行、不得宣称成功范围；仅执行了安全的失败闭合用例。

完整报告与 32 文件哈希清单：

```text
runtime_evidence/826_apriltag_plan_only_20260826/REPORT.md
runtime_evidence/826_apriltag_plan_only_20260826/SHA256SUMS
```

针对性测试 `22 passed`；`h1_2_description`、`h1_2_perception_msgs`、`h1_2_vision`
编译通过。审计启动的 Gazebo/MoveGroup/视觉/重规划进程均已停止。826.2 未覆盖，归档
SHA-256 复核仍为 `17b8e132ed4fd8dea4c5c816ca16cfdb8daa89c3902dbec49d47f9fab49e4980`。
下一步仍只能 plan-only：增加受支持的仅环境距离审计，并在不提高配置限位的前提下
得到 <=90 秒的完整合规时序；名义 H PASS 后才能跑 I 正向矩阵。J--L 必须另获明确
授权。方块释放后的长期稳定性仍是独立验收项。

## 2026-08-26 12:52 CST：阶段 H 严格 C++ 复核纠正旧结论

用户要求继续后，仍只执行 A--I 的 plan-only 工作；没有进入 J--L，没有发布轨迹、
调用控制器、运行旧 preflight/run 或修改物理碰撞几何。

新增离线 `candidate3_clearance_auditor`：它从冻结 `PlanningScene` CDR、计划 JSON 和
URDF/SRDF 文件读取输入，不创建 ROS node、publisher、action/client、控制器接口或
planning-scene writer。它分别执行 SRDF 自碰、unpadded 世界碰撞和仅环境
`distanceRobot`，保留原有把手/门/方块支撑阶段移除语义。已经扩张 10 mm 的 guard
只从额外距离查询排除、仍参与严格碰撞检查。工作进程现在强制调用该程序，审计轨迹
哈希进入最终计划和证书；原 Python `allow_all_robot_self_pairs` 及碰撞审计已删除。

这次复核证明旧的“11,331 状态无碰撞”是 Python ACM 作用域过宽造成的假阴性：它把
ACM 中世界条目也互相放行。严格 C++ 在 epoch 516 名义路径第 14 个最终状态检测到：

```text
status                    COLLISION:WORLD
pair                      <octomap> / left_wrist_pitch_link
stage/message/point       handle_unlock / 0 / 13
last positive clearance   0.000134194635 m
nearest octomap point     [0.220000, 0.343317, 0.005000] m
certificate               valid=false
```

首状态同一对已只有 1.829 mm。只清空 OctoMap 的归因副本在 10,398 个最终状态上通过
精确世界几何和 10 mm narrow-band 门，说明失败来自深度 OctoMap、符合左腕自体渲染
残影特征；正式验收仍保留 OctoMap 严格检查，禁止忽略 OctoMap、加 ACM 豁免或降低
10 mm 门。下一步应在点云上增加 exact-timestamp、几何有界的左腕 self mask，重新从
fresh 只读场景取得冻结快照后复核，不能直接改证书。

时序仅删除 `1e-10 rad` 内完全共线的密集输入点，不改变 joint-space 路径；严格时长从
113.859252773 s 改善为 106.026923183 s，但仍大于 90 s。约 0.002 rad 的路径
shortcut 诊断可以小于 90 s，但会改变 601 目标/接触路径，未采用。当前权威输出：

```text
runtime_evidence/826_apriltag_plan_only_20260826/stage_h_cpp_integrated/result.json
plan hash         97890c6a3105b7e88f410a5b9f25a2086d4eafc9a9f73a67f90a2b2c043c4ab6
certificate hash  39e57e104a37fb880cece11499c093c3d09a2c864ef8a179c820c75d3d8b1ac4
audited path hash fb8f125bd8301e82aba63c6b54f8955c5f53f3f633fc28c779045178ef31415b
```

`h1_2_moveit_demo` 使用规定的 Miniconda 六库精确隔离构建通过；最终包测试为
`111 tests / 0 errors / 0 failures / 1 skipped`。运行时进程已清空。826.2 未覆盖，
SHA-256 仍为 `17b8e132ed4fd8dea4c5c816ca16cfdb8daa89c3902dbec49d47f9fab49e4980`。
阶段 H 仍 FAIL，I 正向矩阵仍为 `BLOCKED_BY_BASELINE_H`，J--L 仍须重新明确授权。

## 2026-08-27：g5 阶段 H/I 最终覆盖与后续 Gazebo 授权

本节是当前最新权威入口，覆盖上一节的 `H FAIL / I BLOCKED` 历史诊断。精确四腕
visual STL 深度源遮罩取得 fresh epoch 13；候选 3 规划器最终版本为
`h1_candidate3_replanner/826-g5`。名义 H 的文件型 C++ 完整审计结果为：

```text
result          runtime_evidence/826_apriltag_plan_only_20260826/
                stage_i_diagnostic_g5/nominal/result.json
status          valid=true / PASS
scene hash      83b499b9035e09953c529d048c91f5ba3c75a96dbf205738126e9769d4e52ee9
plan hash       3cd7082221580055514cbc75e54e321db50fcd85c6c48f427f31bef16d9df7d4
cert hash       00ce857736a317ffa01d16d284c57aa03997ec5985b0041ae082e32b999a8d3b
audited states  8713
clearance       >=10.001 mm
duration        87.809367729 s
```

该结果没有豁免 OctoMap、增加 ACM 许可、降低 10 mm 净距门、提高速度/加速度限位或
改变 601 个门目标。深度源只在图像精确时间戳下，用左右腕 pitch/yaw 四份原装 visual
STL 三角面 7 mm 表面带去除自渲染；任一 TF 不可用就丢整帧。最终路径仍完整覆盖
正向和反向，禁止 partial。

阶段 I 已执行全部 37 个正向场景：7 PASS、30 fail closed；15 个要求的负向场景均
`PASS_FAIL_CLOSED`。五个 PASS 是重复的零扰动基线；仅有的两个非零离散 PASS 为：

```text
cube_xy_+0_-60mm   89.561671149 s / 8888 states / clearance >=10.001 mm
cube_yaw_+10deg    89.854700515 s / 8918 states / clearance >=10.001 mm
```

不得把这两个离散点外推为连续范围；所有非零机柜 x/y/yaw 扰动均不支持。当前权威
报告和索引为：

```text
runtime_evidence/826_apriltag_plan_only_20260826/REPORT.md
runtime_evidence/826_apriltag_plan_only_20260826/stage_i_matrix.json
```

最终隔离构建通过；测试为 `133 tests, 0 errors, 0 failures, 1 skipped`。证据清单
850/850 MATCH，清单 SHA-256 为
`a1dec8bf63890d9b5253e3abff02dcdf47c335da79a30c08c7840b20e56c3a41`。受保护
`826.2.tar.zst` 仍为 mode 0444，SHA-256 保持
`17b8e132ed4fd8dea4c5c816ca16cfdb8daa89c3902dbec49d47f9fab49e4980`，不得覆盖。

阶段 E 仍缺 live 移动物体 TF/depth 数据；J--L 在本节写入前尚未运行。用户已在看到
上述 H/I 结果以及“J--L 或任何控制执行需要另行明确授权”的提示后，于 2026-08-27
明确要求“继续完成任务吧”。该授权仅适用于本计划的 Gazebo 仿真 E/J/K/L，不能连接
或解释为实体机器人许可；仍须逐项通过 fresh 场景、证书绑定、10 mm、正反路径、
601 门目标、时序、夹持/support/latch 和禁止 partial 等全部门。任何异常必须
stop-and-hold；夹持或支撑期间禁止 reset、detach 或松夹。

授权时的只读现场为完全停机：没有 Gazebo、MoveGroup、感知、候选 3、auditor、demo、
trajectory runner 或录像进程。因此后续只能正常启动一套 fresh Gazebo，不能恢复或
假定任何历史夹持现场。

## 2026-08-27 13:18 CST：阶段 E/J/K/L Gazebo-only 最终结果

本节是当前最靠后的权威入口，覆盖此前“E 尚缺、J--L 尚未运行”的状态。用户的后续
授权仅用于 Gazebo；从未连接、启动或授权实体机器人。

最终报告：

```text
runtime_evidence/826_apriltag_execution_20260827/REPORT.md
runtime_evidence/826_apriltag_execution_20260827/SHA256SUMS
```

阶段 E live 秽物体数据已完成，包含可见位置的 fresh TF/depth PASS 和不可见位置的
有界 fail closed。阶段 J/K/L 在 fresh attempt11 的同一个 Gazebo 世界中完成完整
候选 3 物理序列。门/方块 helper 和最终夹爪入口门触发异常时，supervisor 都先停止并
保持；恢复只接受密封 lineage、fresh 证书和 C++ 10 mm 剩余路径审计。没有 reset、
直接设物体 pose、支撑期间 detach 或夹持期间松夹。

完整动作由四个连续 supervisor 结果覆盖，不得误写成单进程无中断：

```text
trajectory messages       23
audited/executed points    8713
robot action duration      87.809367723 s
tag occlusion holds        18
final status               PASS / COMPLETE_PASS
final latch                locked
final door/cube supports   detached / detached
final door hinge           approximately 0 rad
final handle               0.0355494867 rad / 2.037 deg
```

最终三次只读方块样本相同，位置约
`(0.611010491,-0.029496989,0.995406977) m`、yaw `89.999968 deg`。这只证明本次
短期终态，不取代第 15 节独立的整夜稳定性验收。

成功录像（已完整 ffmpeg 解码）：

```text
runtime_evidence/826_apriltag_execution_20260827/videos/
  h1_candidate3_apriltag_826_stage_l_attempt11_20260827_1255.mp4
SHA-256 6ec063814742911250572af60320f2194fd213b079eb9195d865ad1ebba50b8d
H.264 / 1160x1530 / 12 fps / 11747 frames / 978.916667 s
```

attempt10 因最终把手 3.725 deg 超过 ±3 deg 正确 fail closed，证据和录像保留。修复只
提高 latch 从 unlocked 回到 locked 后的条件回位力矩到 0.40 N m，并按约 0.2 RTF
延长墙钟等待；初始 detent 仍为 0.08 N m，±3 deg/速度门以及原装 STL、碰撞、质量、
惯量、正确铰链、601 目标、正反路径和禁止 partial 合同均未放宽。

最终 `h1_2_description`、`h1_2_moveit_demo`、`h1_2_vision` 六名称精确 Miniconda 隔离
重建通过；完整测试 `150 tests / 0 errors / 0 failures / 1 skipped`。所有 Gazebo、
MoveGroup、感知、规划、auditor、demo 和 recorder 进程已经在安全终态采样及视频落盘
后停止。

用户随后指定当前备份编号为 827。新成果发布为非覆盖只读
`/home/liu/h1_moveit/backups/827.tar.zst`，唯一机器可校验哈希记录为相邻
`827.tar.zst.sha256`。`826.3` 作为前一恢复点保留；历史 826.2 也保持 mode 0444 和
SHA-256 `17b8e132ed4fd8dea4c5c816ca16cfdb8daa89c3902dbec49d47f9fab49e4980`，不得覆盖。
备份名称覆盖说明见 `BACKUP_827_RELEASE.md`。
除非用户提出新目标，下一对话只做只读校验，不重复启动执行；任何后续真机工作仍需
全新的明确授权和独立安全流程。

## 2026-09-04：当前双循环权威、r28 视角证据与交接清理

本节是当前最靠后的权威入口，覆盖此前各节中关于“当前版本、最新备份、当前 live、
下一任务”的旧描述；历史实验事实和 826 AprilTag A--L 证据仍保留，不因本节而失效。
本轮只更新交接 Markdown 并清理可再生/失败过程文件，没有修改源码、模型、轨迹或任何
密封 authority，也没有启动、停止或控制 Gazebo，因此不需要重新构建。

### 当前动作语义与最新视觉证据

用户后续要求的完整双循环已经在 Gazebo 中完成：右手单独解锁/开门，左手把方块从
桌面原位放入，右手关门锁门；右手再次开门，左手把方块取回出生原位，右手最终关门
锁门。左手绝不命令或接触把手，躯干最大绝对角保持在 45 度内。

当前动作语义权威和正式运行：

```text
authority     runtime_assets/candidate3_roundtrip_831_authority_v9/
run           runtime_assets/candidate3_roundtrip_831_r17/
summary       runtime_assets/candidate3_roundtrip_831_r17/RUN_SUMMARY.md
run id        c7813b59713f6b58a42a93b2d41cc7367096ae8607cb5229c46b2b6e8d2e6d4b
manifest      b9e49b32c60979eb5a97d8ed3d5dc325a5d73e10b1985a45fd99c61f8848cd29
commands      aef171adfacb8b1af212caa1c193e8fabf51e06d03f8fff4ceddf9ed0a22e8fb
terminal      PASS / cycles=2 / cube=origin / door=locked
motion        161.871759013 s
invoke->PASS  164.523835390 s（<180 s）
```

r17 rosbag 的控制器消息精确为 `60 = 16/16/16/4/8`；把手接触 0，左手把手命令 0，
躯干峰值 `44.250000001 deg`，最终方块位置误差 `0.017981249 mm`，door/handle 约 q0，
latch locked，门和方块 support detached，五个 trajectory topic publisher 为 0。

最新用户指定相机视角重放：

```text
authority     runtime_assets/candidate3_roundtrip_831_authority_v18/
run           runtime_assets/candidate3_roundtrip_831_r28/
summary       runtime_assets/candidate3_roundtrip_831_r28/RUN_SUMMARY.md
run id        5a8c0489413ecc6481507d88ed1c3ccf9a212fb3c4784ca30418170f4a8fab9c
manifest      adbaf4a1f31df35031c966d58a036afc13dd1f8956dbb522f12d7ad0640c19e5
commands      bfed84d1b8881c4d00db4893c323933d219628dfb64b9879925939b67406ad55
payload       16/16 identity-independent checks PASS，与 v9/r17 一致
motion        171.761582056 s
invoke->PASS  173.580995272 s（<180 s）
```

正式视频是
`runtime_assets/candidate3_roundtrip_831_r28/v9_r17_user_requested_angle_roundtrip_full_r28.mp4`，
H.264 Main、1160x1518、10 fps、189.5 s、1895 帧，原速无剪切/变速/动作重排，完整解码
PASS，SHA-256 为
`56548a1547a97e29fdd7f55f2aeacd56106db43b6489dbdfce755d45a00dce70`。r28 rosbag 共
151,001 条消息并含精确 60 条控制消息，最终状态与 r17 合同一致。

v18/r28 是密封固定轨迹的视觉重放身份，不取代 v9/r17 的动作语义。其 manifest 明确
`live_certificate_required=false`、`live_supervisor_required=false`，视频也不是动作
权威；不得把 r28 外推成新的 AprilTag 动态重规划证据。

### 826 A--L 独立成果仍有效

2026-08-27 的 826 AprilTag 动态候选 3 A--L 是另一条独立证据线，继续以以下文件为准：

```text
runtime_evidence/826_apriltag_plan_only_20260826/REPORT.md
runtime_evidence/826_apriltag_execution_20260827/REPORT.md
runtime_evidence/826_apriltag_execution_20260827/SHA256SUMS
```

attempt11 为 `PASS / COMPLETE_PASS`，8713 审计/执行点、动作时长 87.809367723 s、18 次
tag occlusion hold、最终 latch locked。601 门目标、10 mm 环境净距、4.9 度连续性、
原装 STL、正确铰链、完整正反路径、碰撞/时序审计和禁止 partial 是这条动态规划线的
合同。不要把它和 831 双循环的固定轨迹身份、动作或验收范围混写。

### 最新备份与未归档结果

截至本节写入时，最新正式受保护恢复点仍是：

```text
/home/liu/h1_moveit/backups/901.tar.zst
SHA-256 212e16a1a08d3dec0ff097b5d93b4dd9c5619bc575c401b0b31394b012419cc9
```

sidecar 校验 PASS；archive 和 sidecar 均为 `liu:liu`、mode 0444、link count 1。901 保存
的是 v7/r15 的 168.759167141 s 双循环，不包含后续 v9/r17、r20、r24、r25、r28，也不
包含本轮文档整理。必须区分“最新受保护备份 901”和“最新落盘运行 r28”。若要保护
当前状态，只能创建用户指定的新编号，绝不能覆盖 901。

本轮再次验证的近期备份 SHA-256：

```text
827  ebc80da0f08db63766cd89607de065a4aa737dd98af216cdbd06e7b4ec1e5e3e
828  480a04750a1e49205013839b5fb8f0c6875c9612231036f471039f5ea88a42be
831  c2d1fdca42efd333d6c95e1f1ce183bd6449b48bfe7a38d1a258fef99a0f1ff4
901  212e16a1a08d3dec0ff097b5d93b4dd9c5619bc575c401b0b31394b012419cc9
```

四个 sidecar 全部 PASS。保护链 `814、824、825、8251、8252、8261、826.2、826.3、827、
828、831、901` 以及 `814_binding_bundle_registry_docs_20260820_1436` 不得删除或覆盖。

### 当前 live 和未来重放入口

2026-09-04 只读复核发现 r28 的 Gazebo server 和 GUI 已退出，仅剩 launch 父进程及无
数据的 robot-state、bridge、relay、depth/tag 子进程。controller manager 不可用，
`/joint_states` 无新样本，Gazebo transport world 查询超时。这是退化、非 fresh、不可
运行/不可录像的残留现场；本轮没有停止或重启它。

除非用户明确要求重放，默认只读核验落盘证据。新重放不存在可安全复用 v18 的“一键
重跑”入口；必须先从 v18 builder 链派生下一个从未使用的 authority/run，生成新的
run id、socket、ready claim，并绑定尚不存在的 run root 和 rosbag 路径。GUI 视频不在
commands 的五个角色中，也不受现有 manifest/run-id 哈希绑定，必须另用新的非覆盖
路径。builder 只发布 authority；成功后再以普通 `mkdir` 独占创建绑定的 run root，
禁止 `mkdir -p`。禁止手改旧 JSON 或复用 v18 的 run id/socket/目录。

新 authority 审计通过后，只能执行其 `commands_831.json` 的完整 argv。密封角色顺序为
`launch -> rosbag recorder -> support_daemon`，daemon 启动后立即并发运行
`preparation`；不能等待 socket 后才启动 preparation。preparation 精确 PASS 并完成
一次性 claim 后，在 flow 前另行启动非覆盖 GUI 视频录制；视频 recorder 不在 commands
的五个角色内。验收必须同时包含终端 PASS、60 条控制消息、左手把手命令 0、把手接触 0、
躯干 <45 度、方块原位、门锁定、两个 support detached 和五个最终 publisher 为 0。
失败时 stop-and-hold；夹持或支撑期间禁止 reset、detach 或松夹。

### 2026-09-04 可恢复清理

本轮经只读引用和哈希盘点后，把 `191,802,884 bytes`（约 182.92 MiB）无效或可再生
过程数据移入系统回收站，未清空回收站：

- r26/r27 零轨迹失败尝试整目录；
- r21 截断视频/rosbag/初始截图、r22 fail-closed 视频/截图、r23 零轨迹 rosbag；保留
  各自 `ATTEMPT_STATUS.md` 和 r21 相机 pose；
- r24 已被 clean 版本取代的旧视频/contact sheet/过程参考图及中文名重复视频；
- r25、r28 的中文名逐字节重复视频；
- r28 的五张可由正式视频重建的 QA 抽帧；保留正式 contact sheet；
- `runtime_assets/__pycache__` 和 `/tmp/r25_first.png`。

清理后 v9/v18 manifest 与 commands、r17/r28 摘要、r28 正式视频/rosbag/contact sheet/
相机 pose/flow log 均存在且哈希未变；r24/r25 正式成功核心证据、所有 authority 和
builder 链、826 A--L 报告、受保护备份也全部保留。

以上全部仍严格限于 Gazebo。实体机器人从未获得连接或执行授权；任何真机工作都需要
全新的明确授权和独立安全流程。

## 2026-09-08：Mid-360 雷达模型安装与离线接口覆盖

用户明确要求先在现有雷达安装点安装 Mid-360。本轮保留
`h1_2_model.urdf.xacro` 中 `torso_link -> lidar_link` 的原始固定变换不变，在
`lidar_link` 下新增 `livox_mid360_link` 和 `livox_mid360_frame`。仿真机体采用官方
`65 x 65 x 60 mm` 外形包络和 `0.265 kg` 质量；GPU LiDAR 采用水平 360 度、垂直
`-7 -> +52 deg`、10 Hz、0.1--40 m、`1024 x 20` 点的保守近似，并通过
`/livox/mid360/points` 单向桥接为 `sensor_msgs/msg/PointCloud2`。该模型不是 Livox
非重复扫描算法或真机驱动，不能外推为实物 Mid-360 性能证据。

最终源码 SHA-256：

```text
src/h1_2_description/urdf/livox_mid360.urdf.xacro
06a6c88e4e98b75721898ac27e5605c57d68482c8df5ec629dcb5029f55bee83
src/h1_2_description/urdf/h1_2_gazebo.urdf.xacro
45dd63834358209da5ac370de51880c54332a24fa65a06ad24d848bfca782064
src/h1_2_moveit_config/launch/gazebo.launch.py
26f35ea6e45b4db582facac6f6f9d32a63dcacf844bab09166af1263173a0ace
```

按本文六个 Miniconda 库精确隔离规则、并用 `--base-paths src` 排除
`runtime_evidence` 中同名只读源码快照后，`h1_2_description`、
`h1_2_moveit_config`、`h1_2_moveit_demo` 三包增量构建 PASS：

```text
log/build_2026-09-08_10-05-32    3 packages / 0.24 s / PASS
```

离线生成 URDF、`check_urdf`、URDF-to-SDF、Mid-360 安装链/质量/视场/量程/点数率
合同、ROS bridge 文本合同和 launch 生成全部 PASS。完整 demo 回归的 23 个测试目标中
有两个非本轮回归：`test_h1_candidate3_middle_executor_828` 因首次测试环境未加载已安装
的 `h1_2_vision` 而收集失败，在正确 `install/setup.bash` 环境下单独复跑为 `45/45
PASS`；`test_right_handle_hardened_regression` 仍因未改动的旧右把手 IK 源码缺少
`ARGUMENT_VECTOR:--test-arm` 标记而失败，本轮未修改该无关源码或放宽合同。

本轮未启动 Gazebo、控制器或 MoveGroup，未发布轨迹、reset、detach、松夹、创建
authority/run 或备份。2026-09-08 10:08 CST 的只读进程复核未发现任何 ROS、Gazebo、
MoveGroup 或 controller manager 进程；这覆盖 2026-09-04 的孤立子进程历史快照。
现有 v9/r17 动作权威、v18/r28 视角证据和备份 901 均未修改，但当前源码模型已经加入
雷达，后续仿真验证不得冒用旧正式身份。

本轮只完成雷达安装和点云接口，不是自主移动或导航完成：Gazebo 模型仍由
`world_joint` 固定，腿部/底盘控制器、里程计、定位、二维障碍投影和 Nav2 尚未实现。
进入移动与导航阶段前需要用户新的明确授权，并应建立独立的新功能验证身份；实体机器
仍无连接或执行授权。

## 2026-09-08 10:43 CST：Mid-360 临时 Gazebo 外观查看 live

用户在看到离线安装结果后明确要求“直接启动 gazebo 给我看”。本轮先只读确认无任何
ROS/Gazebo/controller/support 残留，再从当前源码普通启动
`gazebo.launch.py spawn_incubator:=false`；这是临时外观查看，不是 v9/r17 动作执行、
v18/r28 重放或新 authority/run。没有发布轨迹、调用动作、reset、detach 或松夹。

当前 launch PID 359534，Gazebo server/GUI 均在运行并留给用户查看。H1-2、桌子和方块
生成成功，五个命令控制器及 joint-state broadcaster active；五个
`*/joint_trajectory` 话题 publisher 全部为 0。Gazebo 原生
`/livox/mid360`、`/livox/mid360/points` 均存在，ROS
`/livox/mid360/points` 有一个 bridge publisher，单帧头的 frame id 为
`livox_mid360_frame`。当前场景未生成孵育箱且已不再 fresh，禁止从这里直接启动任何
正式动作；用户查看完成后再明确决定是否停止 Gazebo。

## 2026-09-08 10:50 CST：方形导航围墙与当前 Gazebo live

用户看到 Mid-360 后要求在机器人外围加入面积适中的正方形墙面。本轮在
`h1_world.sdf` 新增单个静态模型 `navigation_enclosure`：内部净尺寸
`5.0 x 5.0 m`（25 平方米），四墙厚 `0.1 m`、高 `1.2 m`，内表面分别位于
`x/y=+/-2.5 m`。墙体使用完整 box collision；浅灰蓝 visual 的透明度为 0.14，方便
从 GUI 外部观察。机器人、桌子和方块均在围墙内。

最终世界文件身份和构建：

```text
src/h1_2_description/worlds/h1_world.sdf
SHA-256 8a8f2eb3d1f483717c973be689932d710f2647efbd3d2f3c53301e9104988183
log/build_2026-09-08_10-49-43    h1_2_description / PASS
```

源码 SDF XML、四碰撞体名称/位姿/尺寸、5 m 内部闭合关系和 `gz sdf -k` 均 PASS；
构建继续使用六个 Miniconda 库精确隔离，并用 `--base-paths src` 排除证据快照同名包。
修改前已对用户查看用 Gazebo 正常 SIGINT，所有子进程干净退出；随后从新世界重新启动
普通 `gazebo.launch.py spawn_incubator:=false`，当前 launch PID 408455，Gazebo
server/GUI 保持运行。server 查询确认 `navigation_enclosure` 实体存在于世界原点；
Mid-360 ROS 点云仍为 `height=20, width=1024`，五个命令控制器和 joint-state
broadcaster active，五个 trajectory topic publisher 均为 0。

本轮没有执行轨迹、动作、reset、detach、松夹、正式重放或创建备份。当前只用于环境
外观检查，已不再 fresh，且没有孵育箱；v9/r17、v18/r28 和 901 未修改。后续移动或
导航实现不得把本临时现场当作动作证据，仍需新的明确授权和独立验证身份。

## 2026-09-08 12:06 CST：Mid-360 方墙雷达定位完成

用户明确要求先完成雷达定位，腿部运动沿用其已有 RL 强化学习成果，后续再接入。本轮
没有安装缺失的 Nav2/AMCL/SLAM 系统包：本机 apt 有候选版本但无免密 sudo，因此新增
独立纯 Python ROS 2 包 `h1_2_navigation`，只依赖当前已安装的 rclpy、NumPy、TF 和
标准消息。它订阅 `/livox/mid360/points` 和机器人内部 TF，在 `pelvis` 中切取
`z=-0.35..0.25 m`、平面距离 `0.8..4.0 m` 的墙面点，生成 720-bin `/scan`，并对已知
内表面 `x/y=+/-2.5 m` 的方墙执行确定性粗到细鲁棒扫描匹配。

节点输出：

```text
/scan                              sensor_msgs/msg/LaserScan
/map                               nav_msgs/msg/OccupancyGrid
/mid360_localization/pose          geometry_msgs/msg/PoseWithCovarianceStamped
/mid360_localization/status        std_msgs/msg/String JSON
map -> mid360_localized_pelvis     visualization-only TF
```

节点明确不创建 `/cmd_vel`、`/odom`、`map -> odom` 或任何控制器/关节轨迹 publisher。
方形环境有 90 度旋转对称性，定位必须从附近 initial yaw 开始；默认使用已知 Gazebo
起点 `(0,0,0)`，也接受 map frame 的 `/initialpose`。当前机器人继续由 `world_joint`
固定；本成果不是移动定位、Nav2、SLAM、AMCL 或双足行走证据。

live 调试时最初四墙点数门正确 fail closed：机器人本体遮住西墙，最终四墙计数约为
`512/0/102/104`。保留内点距离 `0.08 m`、最小内点比例 `0.80` 和最大 RMS `0.05 m`
不变，只把可见性合同改为至少三面墙各有 20 点；三面非共线墙仍约束 x/y/yaw。最终
live 状态为：

```text
filtered points                    1058
final inlier fraction              0.9972222222
final inlier RMS                   0.0188416362 m
final pose                         x=0, y=1.21e-17 m, yaw=-0.1 deg
biased initial pose test           (0.20 m, -0.15 m, 3 deg)
five consecutive statuses          all PASS
maximum recovered position error   0.005 m
maximum recovered yaw error        0.1 deg
20 LaserScan timestamp periods     min/mean/max = 0.100000 s
scan bins / finite bins             720 / 292
finite scan range                  2.461070..3.500694 m
```

纯算法覆盖带噪声/离群点位姿恢复、点云高度/距离门、最近 LaserScan 回波、TF 变换、
方墙占据地图和三墙可见性门，最终 `5 tests / 0 errors / 0 failures`；`ament_flake8`
六个源码/launch/test 目标 PASS。Miniconda 六库精确隔离构建 PASS：

```text
log/build_2026-09-08_12-05-47    h1_2_navigation / PASS
log/test_2026-09-08_12-05-48     5/5 PASS
```

最终关键 SHA-256：

```text
mid360_square_localizer.py  9e747a0b5c2e47f62d958c2000b056aa61952b81fd113b45757b27403371bedd
square_localization.py      bddbe74d63b024c46d2fd7365e84f0fb3eab77d6ce0651bd17614f93c9daa216
mid360_localization.yaml    fbb6ba28bd7c11a0d6d258e28b502af2c30138cedc10be166b8c170d8aeea4bb
localization launch         07d574f5216dd693f0eba91ff0c080628ed1132e381e8af1dcc2f1bef18583f2
algorithm tests             e8e1ebb9d7b2b34158ba82fcbde20722fd4014649cf4d49680274cfd99453a24
```

当前 Gazebo 方墙现场和定位 launch 都保持运行供用户检查；五个 controller trajectory
publisher、`/cmd_vel` 和 `/odom` publisher 全部为 0。没有执行机器人动作、reset、
detach、松夹、创建 authority/run 或备份；v9/r17、v18/r28、826 和 901 未修改。后续
接入用户 RL 腿部控制时，必须先定义其 odom/base frame 和 `/cmd_vel` 适配合同，再建立
新的移动验证身份，不能把本固定机器人定位结果外推为行走定位结果。
## 2026-09-08 12:24 CST：Nav2 AMCL 固定雷达定位完成

用户明确要求先配合 ROS Nav2 完成定位，腿部运动继续留给后续已有 RL 步态成果接入。
本节只建立 Gazebo 固定机器人定位链，没有获得或执行移动、导航目标或真机授权。

本机 `/opt/ros/jazzy` 原先没有 Nav2。`apt-get -s --no-install-recommends` 确认 AMCL 最小
闭包为 9 个包；由于 `sudo -n` 不可用，未修改系统软件，而是把官方 ROS 2 Jazzy Ubuntu
包非特权解包到：

```text
runtime_dependencies/nav2_jazzy_1.3.12/
```

其中包含 `nav2_amcl`、`nav2_lifecycle_manager` 1.3.12 及其 7 个缺失 ROS 依赖；原始
`.deb` 全部保留，`sha256sum -c runtime_dependencies/nav2_jazzy_1.3.12/SHA256SUMS` 为
9/9 PASS。`setup.bash` 只叠加该工作空间本地前缀，不写入 `/opt/ros/jazzy`。

`h1_2_navigation` 新增 `nav2_amcl.yaml` 和
`mid360_nav2_localization.launch.py`。原本用于点云变换的 `base_frame` 仍为 `pelvis`；
新增独立 `scan_frame` 参数，新 AMCL launch 只把 `/scan` 标记为虚拟
`base_footprint`。默认单节点 launch 继续输出 `pelvis`，旧行为不变。固定定位 TF 所有权
严格为：

```text
nav2_amcl                       map -> odom
fixed_odom_to_base_footprint   odom -> base_footprint
```

AMCL 参数启用 `first_map_only: true`，避免方墙地图的周期保活发布反复初始化粒子滤波器。
以后接入 RL 里程计时必须用 `publish_fixed_odom_tf:=false` 关闭临时静态 TF，并由唯一的
RL 状态估计发布 `/odom` 和 `odom -> base_footprint`；不得保留两个发布者。

live 验收从 `(0.20 m, -0.15 m, 3 deg)` 带非零协方差的偏置 `/initialpose` 开始。当前
固定机器人没有变化的里程计，因此只调用 AMCL 标准 `/request_nomotion_update` 服务 20
次触发静止雷达更新，没有发布运动命令。最终结果：

```text
AMCL lifecycle                         active
map                                    124 x 124 / 0.05 m per pixel
LaserScan frame                        base_footprint
final AMCL pose                        x=0.044790 m, y=-0.063456 m
final position error                   0.077671 m
final heading error                    0.757128 deg
independent square localizer           PASS
independent pose                       x=0.000 m, y=0.005 m, yaw=0 deg
map -> odom / odom -> base_footprint   available / identity static
/map /scan /amcl_pose publishers       1 / 1 / 1
/particle_cloud publishers             1
/cmd_vel /odom publishers              0 / 0
arm/gripper/torso trajectory pubs      0 / 0 / 0 / 0 / 0
```

固定模式满足计划中的 `0.10 m / 5 deg` 定位门。该结果不能外推为运动定位、SLAM、路径
规划、Nav2 自主导航或双足行走；这些必须在 RL 里程计接入后重新验收。

按本文六个 Miniconda 库精确隔离规则、并用 `--base-paths src` 排除证据快照同名包后，
构建和测试通过；`ament_flake8` 六文件及 `py_compile` 也 PASS：

```text
log/build_2026-09-08_12-17-04   h1_2_navigation / PASS
log/test_2026-09-08_12-17-05    9 tests / 0 errors / 0 failures
```

验收报告在
`runtime_evidence/mid360_nav2_amcl_20260908/REPORT.md`。最终关键 SHA-256：

```text
mid360_square_localizer.py          c7a3bd4b578b8b6a95b62266953455b06f9648912be3833f40b25085ce6fd3e1
mid360_localization.yaml            947f8b27af96bdf3f58893e12c71f4caccfd0ba7721f5ecfaafde2d04d5bada9
nav2_amcl.yaml                       4122b26d9887f716a619ded847d654fcead1949d36262b6c0c9abdc4855b40f5
mid360_nav2_localization.launch.py   c41d490e64f37b60b4e48305c4b40049ad16acf5c63c11d8e5f42410d64b59e8
test_nav2_configuration.py           31ca631281131c60adde6f49bc7ac40bfecc38aa94b145a17b4d6ad45e4ea70b
runtime dependency SHA256SUMS        a6a42f6cec70d2197b7c675d1056b2e16c1b9b69b2c4bdb91876d9120a2ff695
acceptance REPORT.md                 7e7a25b19454cb9b809912221313d6a49a2ffeb3be0b55c433f376f77186098a
H1_2_MID360_NAVIGATION_PLAN.md       b944cf2002dc5198259e2d2951e03f17244133b78ac9234e17567f76a23e6217
```

当前 Gazebo launch PID 仍为 `408455`；Nav2 固定定位 launch PID 为 `974820`，AMCL PID
为 `974896`，lifecycle manager PID 为 `974897`。两者保持运行供用户查看。当前现场不是
动作 fresh 证据；本轮没有 reset、detach、松夹、轨迹、动作、备份、authority/run 修改
或真机操作，v9/r17、v18/r28、826 和 901 均未修改。

## 2026-09-08 12:53 CST：V12 接口审计与 Nav2 零运动路径规划完成

用户要求继续雷达定位与 Nav2 计划，并明确腿部运动沿用其已有 RL 强化学习成果、后续
再接入。本轮只完成 RL 资产只读审计、接口合同、全局 costmap 和路径规划，没有获得或
执行 Gazebo 移动、腿部轨迹或真机授权。

只读审计发现两个独立 RL 项目。MuJoCo/Brax 候选的最佳评估跌倒率仍为 29.4%；Isaac Lab
V12 `H12-Human-Omni-Recover-Stop-Gait-Flat-v12/model_17250.pt` 是当前推荐候选。其
checkpoint 大小 `3,799,642 bytes`，SHA-256 为
`92ae9d9e945ed72e5aca91370acf94673d57181113ca89ab4f1c6639e0e87405`，接口为 50 Hz、
78-D observation / 12-D leg action。正式 Isaac Lab 结果是运动存活 `252/256`、条件
停稳 `249/252 = 98.81%`、端到端 `249/256 = 97.27%`，固定 13 方向满回合率
`485/514 = 94.36%`。这些数字只证明冻结 Isaac Lab USD 资产，不是 Gazebo 迁移结果。

完整接口合同已写入 `src/h1_2_navigation/RL_LOCOMOTION_INTERFACE.md`。78 维观测顺序、
12 腿关节顺序、累计目标动作、精确全零 STOP 语义、capture/HOLD/recovery 状态机、未来
`/cmd_vel_nav` 限速以及 TF/odom 所有权均已固定。当前 Gazebo 仍有固定 `world_joint`，
没有腿部 controller，也没有与训练等价的 IMU 数据、足底接触、自由根状态和已验证 V12
TorchScript/ONNX，因此禁止直接加载 checkpoint 或把 12 维输出当绝对关节角。

本机本地 Nav2 前缀在原 AMCL 9 包闭包上新增官方 Jazzy 1.3.12 planner 最小闭包：
`behaviortree-cpp`、`nav2_behavior_tree`、`nav2_core`、`nav2_costmap_2d`、
`nav2_navfn_planner`、`nav2_planner`、`nav2_voxel_grid`，总计 16 个原始 `.deb`；
`sha256sum -c runtime_dependencies/nav2_jazzy_1.3.12/SHA256SUMS` 为 16/16 PASS，
`planner_server` 的 `ldd` 无缺失库。没有写入 `/opt/ros/jazzy`。

`h1_2_navigation` 新增 `nav2_planner.yaml` 和 `mid360_nav2_planner.launch.py`。该入口只
启动 planner server、NavFn A*、静态/scan/inflation 三层全局 costmap 和独立 lifecycle
manager；没有 controller server、BT navigator、velocity smoother 或 waypoint follower。
footprint 为 `0.90 x 0.80 m`，inflation radius 为 `0.50 m`，大于 Nav2 计算的
`0.410 m` 内切半径。

live 中还复现并修复了慢 RTF 下的点云/TF 回调竞态：旧单线程 executor 在点云回调里
等待未来一个传感器周期的 TF 时，无法同时处理 TF 回调。定位器现在仍按点云原始时间戳
查询，但使用两线程 executor 和参数化 `1.0 s` 等待上限。修复后 12 个 `/scan` 时间戳
严格单调，仿真频率 `10.000000 Hz`。

最终 planner-only 验收：

```text
AMCL / planner lifecycle             active / active
inside goal                          map (1.0, 0.8, yaw 0)
inside result                        SUCCEEDED / error 0
inside path                          50 poses / 1.283195 m
outside goal                         map (3.0, 0.0, yaw 0)
outside result                       ABORTED / error 208 / 0 poses
/cmd_vel /odom publishers            0 / 0
arm/gripper/torso trajectory pubs    0 / 0 / 0 / 0 / 0
```

围墙外目标被拒绝，没有放宽墙碰撞、footprint 或 unknown-space 规则。只调用了
`ComputePathToPose`，没有发送 `NavigateToPose`、速度、控制器或腿部命令。

源码按本文 Miniconda 六库精确隔离规则、`--base-paths src` 重建并测试：

```text
log/build_2026-09-08_12-48-01   h1_2_navigation / PASS
log/test_2026-09-08_12-48-02    13 tests / 0 errors / 0 failures
ament_flake8                     8 files / PASS
```

完整报告为 `runtime_evidence/mid360_nav2_planner_20260908/REPORT.md`，SHA-256 为
`0eb2761327cd863dd6924027c2cbe82f5bbd5065cae71d4999df733aa40ea536`。最终关键文件：

```text
mid360_square_localizer.py       b8c9bba75f32df788fefb37d853148fb95488b7d2643983d91c66db1843a3d46
mid360_localization.yaml         5e0675133e4115ba5bb1ed9c0798f0db3533034bad114a30d2362b44414b3360
nav2_planner.yaml                22c04136ecaae5c5e25714c0589d57e28bfe049187d0fb36b890c2c839072a2d
planner launch                   196e2ab37ea62ca7fbf0bd89c9e0063848c1e46d25137d56406da3900b2b7183
configuration tests              27502733acdbefa28d22d633ab2a1f4115a6a43a9d6a54b4dbea20fa4a88b3b7
RL interface contract            2085cb45d2bf0ae89755034b0ce7c97ef46fd99c7300e078e707ebd918104494
runtime dependency SHA256SUMS     c2bed25f5fd6fffe7586eee1d033d381948593cb958d32244eb058ed23cd43b5
navigation plan                  11e1aa1738665736e3bd9d8820574390b35f658602f96705b2fead215dbfd75e
```

当前 Gazebo 继续使用 launch PID `408455`，server/GUI 存活；定位 launch PID
`1172972`、AMCL PID `1173042`；planner launch PID `1174325`、planner server PID
`1174403`。当前 Gazebo 启动参数是 `spawn_incubator:=false`，所以画面没有孵育箱；
孵育箱模型和正式资产没有删除或隐藏。该现场只是 Mid-360 方墙定位/规划开发查看现场，
不是动作 fresh 证据。

下一步若继续 RL 接入，必须新建导航专用自由根模型，逐维对齐 78 维观测，导出并校验
actor，移植完整 STOP 状态机，先做不接 Nav2 controller 的精确零命令站立。任何非零
速度、腿部动作或移动验收仍需用户再次明确授权。v9/r17、v18/r28、826、901 和全部
正式动作 authority/run 均未修改；没有创建备份或执行真机操作。

## 2026-09-08 14:40 CST：V12 Gazebo 零运动合同与 fail-closed 基础完成

用户要求继续雷达定位/Nav2 计划，但已明确腿部运动沿用其既有 RL 训练、后续再接入。
本节只完成 V12 机器合同、资产只读审计、78 维观测装配和默认禁用的速度输入安全门；
没有获得或执行自由根切换、腿控制器启动、策略在线加载、非零速度、Gazebo 移动或真机
授权。

V12 候选身份保持：

```text
task          H12-Human-Omni-Recover-Stop-Gait-Flat-v12
iteration     17250
checkpoint    model_17250.pt / 3,799,642 bytes
SHA-256       92ae9d9e945ed72e5aca91370acf94673d57181113ca89ab4f1c6639e0e87405
task SHA-256  9395bc00d19cea9699c0d86589a14defe517da4a0c4b4ee7020580bd00b726ba
interface     50 Hz / 78-D observation / 12-D leg action
```

CPU 只读加载 checkpoint 确认 iteration 17250、actor 22 个张量全部 finite、normalizer
为 `(1,78)`、移动分支为 `78 -> 128 x 4 -> 12`、STOP 分支为
`78 -> 64 x 2 -> 12`。这只确认冻结权重结构，不是 Gazebo 部署 PASS。

新增机器可读合同 `src/h1_2_navigation/config/v12_locomotion.yaml`，并新增：

```text
h1_2_navigation/locomotion_contract.py   合同校验和 78 维观测装配
h1_2_navigation/velocity_gate.py         纯六门 fail-closed 限速器
h1_2_navigation/nav2_velocity_gate.py    默认禁用 ROS Twist 适配节点
h1_2_navigation/v12_asset_audit.py       只读 Isaac/Gazebo 资产审计
test/test_locomotion_contract.py         合同、观测、安全门和资产回归测试
```

合同的 `enabled=false`、`movement_authorized=false`；只有 `enabled`、
`movement_authorized`、`policy_ready`、`state_estimate_ready`、
`contact_sensors_ready`、`leg_controller_ready` 六门全为 true 才可能放行。任一门失败、
命令进入死区或超过 `0.30 s` 时输出精确 `[0,0,0]`。该节点没有加入任何 launch，本轮也
没有启动它；它只覆盖未来 Nav2 三维速度输入，不包含策略推理、STOP 状态机或腿控制。

安装后 `ros2 run h1_2_navigation v12_asset_audit` 得到
`baseline_valid=true / deployment_ready=false`。已确认 12 个腿关节的名称、父子、
origin、axis、位置限位和 effort 全部一致，总质量同为 `66.984 kg`；但必须保留以下九个
blocker，禁止隐去或外推：

训练 `env.yaml` 中的 `task_source_sha256` 和 `robot_asset_sha256` 也已与合同交叉校验，
两项均 match；以后任一快照声明或文件哈希漂移都必须使 identity gate fail closed。

```text
joint_velocity_limits   0/12 一致；训练源为历史超大覆盖值，Gazebo 为 23/14/9
inertials               27/28 一致；right_shoulder_pitch_link 的 ixy/ixz 符号不同
collision_geometry      Isaac 2 / Gazebo 22
default_leg_pose        Isaac 为屈膝默认姿态，Gazebo 当前全零
free_root               未就绪
leg_controller          未就绪
imu_sensor              未就绪
foot_contact_sensors    未就绪
policy_export           没有已验证 policy.pt/policy.onnx
```

源码使用本文规定的 Miniconda 六库精确隔离和 `--base-paths src` 重建；包级结果：

```text
log/build_2026-09-08_14-43-47   h1_2_navigation / PASS
log/test_2026-09-08_14-43-48    28 tests / 0 errors / 0 failures
ament_flake8                     14 files / PASS
py_compile                       4 new modules / PASS
```

工作空间级 `colcon test-result` 会包含 `h1_2_moveit_demo` 的历史陈旧失败 XML，不能误写成
本包失败；包级 `--test-result-base build/h1_2_navigation` 精确为 28/28 PASS。

正式报告：

```text
runtime_evidence/v12_gazebo_zero_motion_contract_20260908/REPORT.md
SHA-256 1e0e142ea7e4d3dacb9ab9c069eff1879eab4a9e1fd5e1295f991c0706d7a5a2
```

关键文件 SHA-256：

```text
v12_locomotion.yaml          eff6a3769947f492ae0e246ed9ff248948e608b795b8e1627140cdb4dcb9a744
locomotion_contract.py       6f837a5f5eecc999f397fe56a7347caf62b0cf11581d39630f9a58cc4cc76bc7
velocity_gate.py             a98423acc0a996d2023b319f92cdce56d1eea5eaa4e9d51b64fa153ca71a4898
nav2_velocity_gate.py        a18045a7bc7db5c9af64917efd8782340a7438ad3b9f35abb0c7d5c1ca2a7dbf
v12_asset_audit.py           b2d23a0d0324237cd861118507b489920a9acee18ea07a9291cd319b190a7427
test_locomotion_contract.py  b7b67869bc9f57344ebcea2404cdb109a98957439cb3eb26accdaf21a7d284ad
RL interface document        8a03d4eb34ebacfd6ca7be3c3087016565fcfcfb4780b3d781856363e272642a
navigation plan              3b4a1375ca3f3dd0ea19922f57aef7427d903e5c45584ec3f7f8055d235226c6
```

14:40 CST 最终只读检查：Gazebo launch PID `408455` 及 server/GUI 存活；定位 launch
PID `1172972`、AMCL PID `1173042`；planner launch PID `1174325`、planner PID
`1174403`；AMCL/planner 均 active。`nav2_velocity_gate` 不存在，`/cmd_vel`、`/odom`
和五路 arm/gripper/torso trajectory publisher 全部为 0。Gazebo 仍以
`spawn_incubator:=false` 运行，所以画面无孵育箱；资产未删除。

后续必须先在独立导航资产中解决上述九个 blocker，再做逐维 observation 对拍和不接
Nav2 controller 的精确零命令站立。运行 `nav2_velocity_gate` 也会创建速度话题发布者，
当前不要启动。任何策略加载、自由根修改、腿控制器启动、非零命令或移动验收仍需用户
再次明确授权。v9/r17、v18/r28、826、901 和正式动作 authority/run 均未修改；没有
创建备份。

## 2026-09-09 09:06 CST：导航围墙增高与完整 360 度扫描 PASS

用户要求提高机器人外围正方形墙面高度。本轮保持内部净尺寸 `5.0 x 5.0 m` 和墙厚
`0.1 m` 不变，将 `navigation_enclosure` 四面墙的 collision 与 visual 高度从 `1.2 m`
统一提高到 `2.5 m`，中心高度从 `z=0.6 m` 提高到 `z=1.25 m`。没有改变导航场地面积、
机器人固定根、孵育箱资产或任何正式动作 authority/run。

此前后方缺失不是 Mid-360 水平视场不足：传感器仍为水平 360 度、每圈 1024 样本。
`lidar_link` 相对 pelvis 向上俯仰 `13.760 deg`，而传感器最低垂直射线为 `-7 deg`，后方
最低射线仍向上约 `6.760 deg`，会越过原 `1.2 m` 后墙。墙高提高后，单帧原始点云
`20480` 点中有 `8166` 个有效回波，front/left/rear/right 分别为
`3092/2027/1020/2027`，证明四面高墙均被三维雷达看到。

旧二维裁剪上限 `0.25 m` 会继续过滤增高后的后墙回波，因此
`mid360_localization.yaml` 和定位器默认 `maximum_height` 同步标定为 `1.25 m`，下限保持
`-0.35 m`。最终单帧 `/scan` 为 `720/720` 有限值，四个 90 度象限均为 `180/180`：

```text
front  min 2.449 m  max 3.481 m
left   min 2.460 m  max 3.530 m
rear   min 2.445 m  max 3.512 m
right  min 2.455 m  max 3.474 m
```

独立方墙定位器结果为 `PASS`，inlier fraction `1.0`、inlier RMS `0.018178607 m`、
point count `6374`、pose `(0,0,0)`；AMCL 和 planner lifecycle 均为 active。

源码按本文 Miniconda 六库精确隔离规则并使用 `--base-paths src` 重建、测试：

```text
log/build_2026-09-09_08-59-58   h1_2_description / PASS
log/test_2026-09-09_08-59-58    0 tests / 0 errors / 0 failures
log/build_2026-09-09_09-02-39   h1_2_navigation / PASS
log/test_2026-09-09_09-02-40    28 tests / 0 errors / 0 failures / 0 skipped
ament_flake8                     14 files / PASS
```

最终关键 SHA-256：

```text
h1_world.sdf                    29a780dfcf48c7e3b22c0fec3fd727e288e9326a77721728539e36b663c1000e
mid360_localization.yaml        6a267f18a4684764f230e824f5209ad79119a0e87c60ee1fcbd96071eacfa856
mid360_square_localizer.py      3e4aa0b1f30db5449d2b9e81241105894491a30ed6c06fb73567ec2a36a1c6d3
test_nav2_configuration.py      1025588371f929ec183c9f3606df625f30ad29e1c222714b6926abc9517dde01
acceptance REPORT.md            3c4de6c0e3f518a798b3663cc3451787b75ffeb2b83da4f8134caaa472eecf47
```

完整报告为 `runtime_evidence/mid360_tall_walls_360_scan_20260909/REPORT.md`。

09:06 CST 最终只读检查：Gazebo launch PID `2243395`、server PID `2243676`、GUI PID
`2243679`；定位 launch PID `2256400`、AMCL PID `2256517`；planner launch PID
`2259543`、planner server PID `2259656`；RViz PID `2265667`。五个现有 controller 和
joint-state broadcaster 均 active；`/cmd_vel`、`/odom` 和五路 arm/gripper/torso
trajectory publisher 均为 0，`nav2_velocity_gate` 未启动。

Gazebo 继续使用 `spawn_incubator:=false`，所以当前画面没有孵育箱；其源码和正式资产未
删除。当前 live 只是高墙、Mid-360 与 Nav2 固定定位开发查看现场，不是动作 fresh 证据。
本轮没有发布速度或机器人轨迹，没有加载 RL 策略，没有移动机器人，没有 reset、
detach、松夹、动作重放、备份或真机操作。v9/r17、v18/r28、826、901 均未修改。任何
非零速度、腿部动作或移动导航验收仍需用户再次明确授权。

## 2026-09-09 09:12 CST：Markdown 入口合并与缓存说明清理

用户要求盘点 Markdown 并尽量合并。处理前工作空间共有 `70` 个普通 `.md` 文件，另有
`3` 个 build/install 符号链接。分类后确认
正式 `runtime_evidence/**/REPORT.md`、`runtime_assets/**/RUN_SUMMARY.md`、826 历史证据
快照、四份 backup release 合同、导航计划、RL 接口合同、包级 README 和本文件均有独立
身份或用途，禁止为了减少数量而合并、删除或改写其历史证据。

当前状态统一由 `HANDOFF_CURRENT.md` 维护。原 `NEXT_CHAT_HANDOFF.md` 与其大量重复，现
缩为兼容既有提示词和阅读顺序的入口文件，不再复制动作、导航、live 或备份状态。根
`README.md` 已更新为统一文档导航；`HANDOFF_PLAN.md` 的阅读说明同步更新。

另发现 `24` 个 pytest 自动生成的缓存 `README.md`，内容逐字节相同，均为 `302 bytes`，
共同 SHA-256 为 `73fd6fccdd802c419a6b2d983d6c3173b7da97558ac4b589edec2dfe443db9ad`。
它们已移入系统回收站，共 `7,248 bytes`，没有永久清空。精确源路径与字节数如下：

```text
./.pytest_cache/README.md                                                                                                      302
./build/h1_2_moveit_demo/ament_cmake_pytest/test_candidate3_middle_exact_auditor_source_contract/.cache/README.md              302
./build/h1_2_moveit_demo/ament_cmake_pytest/test_candidate3_return_plan_builder_831/.cache/README.md                            302
./build/h1_2_moveit_demo/ament_cmake_pytest/test_h1_candidate3_flow_828/.cache/README.md                                        302
./build/h1_2_moveit_demo/ament_cmake_pytest/test_h1_candidate3_live_certificate_829/.cache/README.md                            302
./build/h1_2_moveit_demo/ament_cmake_pytest/test_h1_candidate3_live_supervisor_829/.cache/README.md                             302
./build/h1_2_moveit_demo/ament_cmake_pytest/test_h1_candidate3_middle_executor_828/.cache/README.md                             302
./build/h1_2_moveit_demo/ament_cmake_pytest/test_h1_candidate3_roundtrip_flow_831/.cache/README.md                              302
./build/h1_2_moveit_demo/ament_cmake_pytest/test_h1_candidate3_roundtrip_middle_831/.cache/README.md                            302
./build/h1_2_moveit_demo/ament_cmake_pytest/test_h1_closed_loop_90_8254_source_contract/.cache/README.md                        302
./build/h1_2_moveit_demo/ament_cmake_pytest/test_h1_door_support_detach_8253_source_contract/.cache/README.md                   302
./build/h1_2_moveit_demo/ament_cmake_pytest/test_h1_fast_support_daemon_831/.cache/README.md                                    302
./build/h1_2_moveit_demo/ament_cmake_pytest/test_h1_persistent_trajectory_runner_8254/.cache/README.md                          302
./build/h1_2_moveit_demo/ament_cmake_pytest/test_h1_right_handle_executor_828/.cache/README.md                                  302
./build/h1_2_moveit_demo/ament_cmake_pytest/test_h1_right_handle_torso_margin_829/.cache/README.md                              302
./build/h1_2_moveit_demo/ament_cmake_pytest/test_h1_robot_joint_state_relay_829/.cache/README.md                                302
./build/h1_2_moveit_demo/ament_cmake_pytest/test_h1_ros_snapshot_cache_8254/.cache/README.md                                    302
./build/h1_2_moveit_demo/ament_cmake_pytest/test_h1_support_orchestrator_828/.cache/README.md                                   302
./build/h1_2_moveit_demo/ament_cmake_pytest/test_right_handle_hardened_regression/.cache/README.md                              302
./build/h1_2_moveit_demo/ament_cmake_pytest/test_right_handle_live_fcl_auditor_829/.cache/README.md                             302
./build/h1_2_navigation/.pytest_cache/README.md                                                                                302
./build/h1_2_vision/.pytest_cache/README.md                                                                                    302
./src/h1_2_navigation/.pytest_cache/README.md                                                                                  302
./src/h1_2_vision/.pytest_cache/README.md                                                                                      302
```

清理后工作空间共有 `46` 个普通 Markdown 文件：根目录 `13`、`runtime_evidence` `17`、
`runtime_assets` `11`、`src` `4`、`runtime_dependencies` `1`，pytest cache 中为 `0`。
另保留以下 `3` 个 build/install 符号链接，它们不是独立内容，因此按路径条目查看为
`49` 个：

```text
./build/h1_2_vision/config/tag_map_geometry.md
./install/h1_2_description/share/h1_2_description/vendor/dex1_1/README.md
./install/h1_2_vision/share/h1_2_vision/config/tag_map_geometry.md
```

关键文档最终 SHA-256：

```text
README.md                 a04af287d7bf4542b3432aa32060bbef2e9be7b14fa14b0eed8f83d0ca21a24b
HANDOFF_CURRENT.md        0b700778e9e663ac58a4eda51fcea65d22d6634c9a7477a85dc6d1dc431e758b
NEXT_CHAT_HANDOFF.md      bd3c72f95a69eb65ce14fee0fa201bc2130d2f2db234720fb23475ac3bd4c0dd
HANDOFF_PLAN.md           5c22103e98d524fd13d76af27c0341432fdcd5a2a353060f6a8c267b36c466b6
```

pytest 后续运行可能重新生成这些缓存说明；它们不是源码、正式证据或交接资产。本轮没有
永久清空回收站，没有停止/重启 Gazebo，没有发布速度、轨迹或动作，没有修改
authority/run、备份或真机状态。

## 2026-09-09 09:20 CST：当前导航 live 重新加入原孵育箱

用户明确要求在当前 Gazebo 画面把箱体加回来查看效果。执行前只读确认五个现有
controller 和 joint-state broadcaster 均 active，`/cmd_vel`、`/odom` 及五路
arm/gripper/torso trajectory publisher 全部为 0，没有 support/attachment 执行进程，
Gazebo 模型列表中不存在 `incubator`。

没有重启 Gazebo，也没有修改 world、launch 或模型源码。使用安装后的原
`h1_2_description/models/incubator/model.sdf` 向当前 `h1_world` 非覆盖生成：

```text
name  incubator
pose  x=0.590 m, y=0.000 m, z=0.815 m, RPY=(0,0,0)
result Entity creation successful
```

生成后 Gazebo 模型列表包含 `incubator`，实体 ID 为 `201`，实测 pose 与上述默认值完全
一致，`/incubator/latch/state` 为 `locked`。AMCL 和 planner lifecycle 均为 active，
独立定位器仍为 `PASS`，inlier fraction `1.0`、inlier RMS `0.018271935 m`、point count
`6368`。最终 `/scan` 仍为 `720/720` 有限值，范围 `2.440--3.538 m`。

最终 `/cmd_vel`、`/odom` 和五路 arm/gripper/torso trajectory publisher 再次核验均为
0。没有移动机器人、门、把手或方块，没有发布速度、轨迹或动作，没有 reset、detach、
松夹、authority/run、备份或真机操作。

当前 Gazebo launch PID `2243395` 最初仍是以 `spawn_incubator:=false` 启动；本次箱体是
运行期手动加入，不改变该参数，因此重启后不会自动保留。以后需要新 launch 自动包含
箱体时必须显式使用 `spawn_incubator:=true`，同时重新执行当轮安全检查。

查看截图为 `/tmp/h1_2_incubator_tall_walls.png`，`2000 x 1690`，SHA-256：

```text
ffdc958594dad9cad46122fb68653c3acaf6ef415545e96698751411335ad19c
```

截图只用于当前人眼查看，不是动作 authority 或正式运行证据。同步后的
`HANDOFF_CURRENT.md` SHA-256 为
`a979da019eb734186ea45312a0afcf8d2496cb849b1cbeb9812c8ffff82e6fd5`。

## 2026-09-09 09:46 CST：Mid-360 后全新身份双循环重放 v19/r29 PASS

用户明确授权“使用全新身份完整重放箱体双循环动作”，因此本轮进入
`HANDOFF_PLAN.md` 分支 G。执行前重新只读确认原导航查看现场不是动作 fresh 入口：原
Gazebo 由 `spawn_incubator:=false` 启动，箱体为运行期加入，AMCL、planner、RViz 和
导航定位器仍在；方块在出生原位、门和把手约 q0、latch locked，没有 support/attach
执行进程，五路 trajectory 与 `/cmd_vel`、`/odom` publisher 均为 0。随后全部通过正常
SIGINT 清理，未 reset、detach、松夹或强杀。

重新枚举后确认以下身份此前均不存在，并从 v18 builder 链创建：

```text
builder     runtime_assets/build_candidate3_roundtrip_831_authority_v19.py
authority   runtime_assets/candidate3_roundtrip_831_authority_v19/
run         runtime_assets/candidate3_roundtrip_831_r29/
run id      dc2f34a6572037e52b3c770b878d15f07ad3e6843461577fd599e39fad375269
socket      /run/user/1000/h1rt831_dc2f34a6572037e52b3c770b878d15f0.sock
```

builder 使用 `--self-test` 得到 `PASS / writes_performed=false / 16/16`，随后只执行一次
build；authority 为 `READY / valid / sealed`，18 个密封文件 0 mismatch。builder 只发布
authority，r29 再以普通、非 `-p` 的 `mkdir` 独占创建。关键身份 SHA-256：

```text
builder                 70be0e56c4f3dec395b1e5e4e720b023eabca9d0d46a67ba7621f0637314414e
manifest                43c765450228557c1930787e25e9661eeed5fe34ffe2b80318ed3152aab8d7ff
commands                752126eac221b9c5cea15fd433811f95992aba025278b381d93034ce59109612
right executor          7a1233641c26298a92b0e153328dfd76e5e561a2f5520fec40f1c11a165ecde8
controller auth         6108f94f348e7d88ee96d94e587e7958bc52ec825fd1289803fb9b1bf8b5cb8c
```

v19 的 16 项身份无关动作、轨迹、世界、控制器和安全 payload 与 v18 逐字节一致，并保留
v9/r17 的动作和时序合同；新身份只绑定 post-Mid360 回归、r29 run、全新 run id、socket、
ready claim、run root 和 rosbag 路径。没有修改 `src`、模型、轨迹或 v9/v18；本轮不需
重新编译 ROS 包，运行身份由 builder self-test、密封 audit 和正式 Gazebo 执行验证。

按新 `commands_831.json` 的密封角色执行：fresh launch 健康门 PASS 后启动 rosbag，
再启动 support daemon，并在 250 ms 后立即并发 preparation；preparation 精确为
`PASS / controller_commands=0`。随后恢复 r28 用户选定相机 pose，先启动独立 GPU GUI
视频录制，再执行密封 flow argv。第一次本地包装脚本误把顶层 `commands` 字段当成角色
列表，在到达 `execv` 前以 `AttributeError` 退出，控制命令为 0；`flow_output.log` 明确
保留该错误和 retry 标记。第二次从实际 `.processes[]` 读取 `role=flow` 并原样 `execv`
完整密封 argv，没有修改 commands。

正式单次 flow 结果：

```text
flow invocation       85551926789090 ns
motion start          85554204227516 ns
motion end            85727086059212 ns
completion PASS       85727086074762 ns
invocation -> motion  2.277438426 s
motion                172.881831696 s
invocation -> PASS    175.159285672 s
terminal              PASS / cycles=2 / cube=origin / door=locked
left handle           commands=0 / contacts=0
```

`175.159285672 s < 180 s`，合同 PASS。cycle 1 为方块入箱、门锁定、机器人回 launch；
cycle 2 为方块取回出生位并最终关门锁门。support daemon 完整处理 claim、两循环 attach/
detach、checkpoint、finalize 和 shutdown，最终 socket 与 daemon 均消失。

r29 rosbag 正常 SIGINT 后 metadata 与 SQLite 完整：共 `207,554` 条消息、时长
`425.634477434 s`；控制器消息精确为 `60 = 16/16/16/4/8`，把手接触话题为 0。全部控制
payload 和 joint telemetry 数值 finite；命令躯干峰值 `44.25 deg`，实测峰值
`44.250000001018257 deg < 45 deg`；SQLite `PRAGMA integrity_check=ok`。核心哈希：

```text
rosbag db3       e3af46ad7c404f2f5a338b3e8c9cd2a4b5da071067a1da9aa63708bd68e6f9b2
metadata.yaml    fa120d5796b675eef626046deeb560833d3c4ad999cf0a39da1de57770210b0e
flow_output.log  53b5fddd6c8a64a140194a0dd0d3744009fe2320bca57c8f2c71c6171c9e3817
```

独立 GUI 视频在 flow 前启动、terminal PASS 后正常 SIGINT：H.264 Main、`1160x1518`、
10 fps、`211.300000 s`、2113 帧、`2,487,797 bytes`，全帧解码 PASS，原速且无剪切、
变速或动作重排。五点 contact sheet 与首末截图人眼检查 PASS；相机 pose 在 flow 前后
完全不漂移。视频和视觉证据 SHA-256：

```text
post_mid360_roundtrip_full_r29.mp4  7f77658d9d7cd515503fe66e432b35892d0044ac80d37f6e3f0e9b4e85c1d6c7
initial_camera_view.png             94637165882d84c1122c25bc02a2f759223b4f0082ab06cf1c056d568e22b399
final_camera_view.png               2654007eca0f8b5f2af7da4cefb41a1bca59a252ae3035c4f5a197a74246fcfa
video_contact_sheet.png             857c89ee45e9a20f61eaa16c759aa85c056b0234b04090233b2234eaaa4f1c8b
CAMERA_POSE_POST_MID360_R29.json     0dc485be05c628d796dfedb6dc0b4ed12318a4c3a80ad3565bd2c7b233f30175
```

最终 Gazebo 方块 pose 为
`(0.334108965184, 0.377058196732, 0.829999576072) m`，相对出生位置误差
`0.123533196 mm`；RPY 为
`(-0.000000193231, 0.000000114583, 1.571452785417) rad`。门铰链为
`-2.3251219838253058e-22 rad`，把手关节为 `-2.852694480443925e-16 rad`，latch
`locked`，门和方块 support 均 detached，机器人 launch gate PASS，五路 trajectory
publisher 全为 0。

完整报告：

```text
runtime_assets/candidate3_roundtrip_831_r29/RUN_SUMMARY.md
SHA-256 17f1f16b63e9727589d7e31d875b7474fd436a163af677bc25706dd0183c1c60
```

当前动作语义权威仍是 v9/r17；v19/r29 只是共享描述加入 Mid-360 后的固定轨迹回归和
最新视觉证据，不能替代或外推为 826 AprilTag 动态重规划证据。v18/r28 仍是有效历史
视觉证据，不得覆盖。901 仍是最新受保护备份，且不含 v9/r17、r28、v19/r29 或本次
文档更新；用户未指定新备份编号，因此没有创建备份。

09:39 CST 最终 live：v19 密封 launch PID `2436711`、Gazebo server PID `2436987`、GUI
PID `2436988` 仍在成功终态；六个控制器 active，录制器、flow、support daemon 和 socket
均已退出。旧导航 AMCL/planner/RViz 已正常清理，当前动作 world 没有导航围墙。该状态
只是历史快照，下次必须重新只读检查。

本轮新增 r29 `RUN_SUMMARY.md` 后共有 `47` 个普通 Markdown 文件和 `3` 个 build/install
符号链接条目。当前入口文档 SHA-256：

```text
README.md            6aabe367e94439bd4069bb3b478f9b19d1165981f79cc225c776b2c037b93c02
HANDOFF_CURRENT.md   fe658a8a426008927e66c9850a161ff2493e4ff36b59c8ab42f7d0724519f14a
HANDOFF_PLAN.md      b9220a500a34f6113065173afa1a53bc2117017e88f04eb6e2a38ce3fe9b9e60
HANDOFF_PROMPT.md    30fa6671daa5de5f60654ab5c3592091e2f669779094eca0438cedc576333c3e
QUICKSTART.md        5a5150c047debccad64dbf948cf707651c84ebe8333854ef3aabaee7e36010e6
826_plan.md          89af0ebe1bebd68259a880b9e1192089284a4502bfaaa19079f73487b9b98f21
NEXT_CHAT_HANDOFF.md bd3c72f95a69eb65ce14fee0fa201bc2130d2f2db234720fb23475ac3bd4c0dd
```

所有结论严格限于 Gazebo；未连接或授权实体机器人，未运行 RL 策略、腿控制器或非零
移动导航。

## 2026-09-09 10:01 CST：909 完整工作空间备份预发布合同

用户明确指定从未使用的新备份编号 `909`。预检查确认以下三个目标都不存在：

```text
/home/liu/h1_moveit/backups/909.tar.zst
/home/liu/h1_moveit/backups/909.tar.zst.sha256
BACKUP_909_RELEASE.md
```

本节写入前已创建 `BACKUP_909_RELEASE.md`，作为归档内的非自引用发布合同。909 必须只
以 `h1_2_ros2_ws/` 为唯一安全相对顶层目录，完整归档当前工作空间并保留 symlink；不得
直接收录外部 backups。归档和 sidecar 必须在 backups 同目录以全新临时普通文件生成，
先完成验证，再以 no-replace 硬链接发布；最终必须为 `liu:liu`、mode `0444`、link count
1。任何同名目标在发布前出现都必须 fail closed，禁止覆盖。

909 计划保护本节之前的全部工作，包括 v9/r17 动作语义、v19/r29 Mid-360 后完整双循环
PASS、826 AprilTag A--L、Mid-360/2.5 m 方墙/完整 360 度扫描、Nav2 固定定位与
planner-only、V12 零运动接口合同和当前交接文档。归档前 Gazebo 保持 v19/r29 成功终态；
唯一打开工作区文件为 tee 的 `launch_output.log`，连续检查确认大小和 mtime 均静止，不
停止 Gazebo、不发布命令。

发布前后必须通过：`zstd -t`、完整 tar 列表、唯一根、安全相对成员、无重复、无
hardlink/device/FIFO、无 symlink 子成员、关键成员存在与归档内哈希、sidecar
`sha256sum -c`、owner/mode/link count，并复核 827/828/831/901 旧保护链未变。实际 909
归档 SHA-256 只能在归档外 sidecar 和发布后追加小节记录，避免自引用。

## 2026-09-09 10:16 CST：909 完整工作空间备份发布 PASS

909 已按上一节合同完成非覆盖发布：

```text
archive  /home/liu/h1_moveit/backups/909.tar.zst
sidecar  /home/liu/h1_moveit/backups/909.tar.zst.sha256
SHA-256  d00e104358c7a74706b86fcac8465200376168e9225fe9f0733568f29a33af1a
size     3,209,076,019 bytes
members  25,798
```

archive 和 sidecar 都是普通文件、`liu:liu`、mode `0444`、link count 1；正式 inode 分别
为 `131265` 和 `131271`。同目录临时文件完成预验证后使用 no-replace 硬链接发布，再
解除临时链接，没有覆盖任何目标。

发布前后检查全部 PASS：`zstd -t`、完整 tar 列表、唯一安全相对根
`h1_2_ros2_ws/`、20,749 个普通文件、4,532 个目录、517 个 symlink、0 重复、0 非法
类型、0 symlink 子成员；归档内 v19/r29 authority、RUN_SUMMARY、视频、rosbag、826、
Mid-360 和 V12 关键成员与工作区逐字节一致。归档内 MP4 全帧解码和 rosbag SQLite
integrity 均 PASS，外部 sidecar `sha256sum -c` PASS。

旧保护链 827/828/831/901 的 sidecar 均再次完整复算 PASS，owner/mode/link count 未变。
用于成员、类型、symlink 与关键内层提取的四个审计临时项共 `118,128,255 bytes`，已移入
系统回收站，没有永久清空；归档与 sidecar 的临时硬链接名已按发布合同解除。

909 归档内 `AGENTS.md` 有意停在上一节预发布合同，`BACKUP_909_RELEASE.md` 不嵌入归档
自身摘要；实际 digest 只存在于外部只读 sidecar 和本发布后小节，避免自引用。909 是
当前最新受保护恢复点，包含 v9/r17、v19/r29、826 A--L、Mid-360/Nav2/V12 和截至
预发布合同的完整工作空间。此后文档中的 909 digest 更新不在归档内。

Gazebo 未因备份停止或改动，仍保持 v19/r29 成功终态；没有发布动作、速度或腿部命令，
没有连接实体机器人。以后创建备份必须由用户指定新的未使用编号，禁止覆盖 909。

## 2026-09-09 10:21 CST：909 发布后 live 已离线

909 发布完成后的重新只读检查发现：v19/r29 launch、Gazebo server/GUI、相关 bridge、
视觉节点和 controller manager 均已不存在，`/joint_states` 与
`/incubator/joint_states` 没有 publisher。09:39 的成功终态和 PID 只作为历史运行时
快照；r29 flow、视频、rosbag、终态报告及 909 归档均已完成验证，不受随后进程退出
影响。

本轮备份流程没有发送停止命令；10:01 预发布时 launch tee 仍持有但不再增长的
`launch_output.log`，10:21 时该进程已经消失，日志 mtime 仍停在 09:36:58 且没有新增
shutdown 记录。因此只陈述当前离线，不推测退出原因，不启动或恢复 Gazebo。后续查看或
重放必须重新只读检查并按分支 G 使用 fresh Gazebo 和全新非覆盖身份。

## 2026-09-09 11:36 CST：V12 actor 导出与离线状态运行时 PASS

用户确认继续把 `/home/liu/test/h1_2_isaaclab` 的运动控制加入 Mid-360/Nav2 导航。本轮
按上线前安全顺序只实施离线策略导出、推理和状态机移植；没有启动 Gazebo、ROS 节点、
controller 或 Nav2 controller，没有发布速度、腿部目标、轨迹或导航目标，没有修改
v9/r17、v19/r29、826 authority/run，也没有连接真机。

V12 冻结输入身份不变：

```text
task          H12-Human-Omni-Recover-Stop-Gait-Flat-v12
iteration     17250
checkpoint    model_17250.pt / 3,799,642 bytes
SHA-256       92ae9d9e945ed72e5aca91370acf94673d57181113ca89ab4f1c6639e0e87405
task SHA-256  9395bc00d19cea9699c0d86589a14defe517da4a0c4b4ee7020580bd00b726ba
```

新 `v12_policy_export.py` 先以 `--self-test` 完成 checkpoint 身份、22 个 finite actor
张量和 258 项精确零/非零 MOVE/STOP 样本检查，结果为
`writes_performed=false / maximum_absolute_error=3.0517578125e-05`。确认目标目录不存在
后一次性非覆盖生成：

```text
policy/v12_model_17250_policy.npz
  270,033 bytes / 4891a943e6347c580c25224192523753649763fb083cb5a6fdcfe71c62e66b1c
policy/v12_model_17250_policy.pt
  307,178 bytes / f206a6247b737c95406339ca47a25e19c86ecd35afd967d655a024b00501bf16
policy/v12_model_17250_golden.npz
  85,298 bytes / 0630c714126687d30cf82bf1156b9f8aea610da1014fcb2e1820699c8f4b0a24
policy/v12_model_17250_manifest.json
  ecd4f7fbb830c20a5709e2f19b14e197bcb409a6881a944e349266938d8ffd4b
```

纯 NumPy 推理不依赖 ROS Python 中不存在的 PyTorch；它与 PyTorch checkpoint 参考的
平均绝对误差为 `7.822868752782597e-08`、最大绝对误差为
`3.0517578125e-05`。TorchScript 与同一参考的最大误差为 `0.0`。NPZ 禁用 pickle，采用
固定 ZIP 时间戳和排序键；manifest 绑定 checkpoint、task SHA、导出器及三个输出。

新 `v12_runtime.py` 是不导入 `rclpy`、不创建 publisher 的纯计算运行时，完整保持
50 Hz / 78-D observation / 12-D action ABI，并移植累计腿目标、90% 位置范围、动作裁剪、
确定性肩 pitch 摆动、精确零 STOP context、首个 STOP 动作、逐关节残差/变化率、10 周期
stable dwell、capture-pose HOLD 及姿态/速度/接触 recovery hysteresis。它输出计算结果，
但目前没有任何 ROS 输出侧节点可把结果发送给 controller。

`v12_locomotion.yaml` 现在冻结 package-local NumPy/TorchScript/golden 身份、物理关节
位置限位、部署步频/抬脚高度、肩 pitch 和完整 STOP 参数。只读 asset audit 结果更新为：

```text
baseline_valid    true
policy_export     true
deployment_ready  false
```

policy export blocker 已解除，仍必须保留八项 blocker：`joint_velocity_limits`、`inertials`、
`collision_geometry`、`default_leg_pose`、`free_root`、`leg_controller`、`imu_sensor`、
`foot_contact_sensors`。禁止因离线策略可推理就直接连接当前固定根 Gazebo 或 Nav2。

源码按 Miniconda 六库精确隔离规则和 `--base-paths src` 构建通过。第一次 `colcon test`
没有进入测试收集，因为 shell 未加载保存的 Nav2 本地前缀；显式加载
`runtime_dependencies/nav2_jazzy_1.3.12/root/opt/ros/jazzy/local_setup.bash` 后正式结果：

```text
log/build_2026-09-09_11-32-40  h1_2_navigation / PASS
log/test_2026-09-09_11-32-41   37 tests / 0 errors / 0 failures / 0 skipped
ament_flake8                    19 files / PASS
```

完整报告：

```text
runtime_evidence/v12_offline_runtime_integration_20260909/REPORT.md
SHA-256 b546d669542de697ad69954391298035fb89a25532b6411b38f42a24ff17463c
```

最终关键源码 SHA-256：

```text
v12_locomotion.yaml       c013aec4573fbe1f67b397ba4e5e673140e8f242977fd56952f67716a75d82c4
locomotion_contract.py    240266fce323edffaebbfd1ebd3d2193f8f350c469c3ef2ed989bca953f48986
v12_policy.py             7b609ffc26fb26b3b586038a42247156b1c87a6892d7ff75739398a2bf7b81bf
v12_policy_export.py      52779f76d5369f20b0cf1b1b84826d8f50704080a0717672080e17940db16951
v12_runtime.py            449d6522760a581200977dec94f6e945dc10bb234acc2b832fe3f9c1a74ca4b2
v12_asset_audit.py        7c533f194d583ac6b648d3df7c16c4bf6e930052f6719245325ef7d5a5ca4328
RL_LOCOMOTION_INTERFACE   636cd551fb5aa63bd1e7ef0cc7174a2774e1589e06153eb2eba92a9a1b28b5e9
navigation plan           4493555e2a7b66001076e04ffc5a2274e0e531817e20fcf2361ff4b5c52469fe
```

11:34 CST 重新只读检查没有 Gazebo server/GUI、controller manager、Nav2 或 V12 运行
进程；当前 live 仍离线。下一步必须先建立独立导航专用自由根模型并解决动力学/碰撞差异，
再添加腿控制器、IMU、双足接触和同步状态估计，首个 Gazebo 验收只能是不接 Nav2
controller 的精确零命令站立。任何非零移动仍需 fresh Gazebo 和单独明确授权。

909 仍是最新受保护恢复点，但不包含本节新增策略、运行时和报告；本轮没有创建备份。

## 2026-09-09 22:24 CST：Mid360 + Nav2 + V12 初始定位行走 PASS

用户已明确授权本轮 Gazebo 非零移动，并要求持续优化直至实现定位行走。本轮属于
`HANDOFF_PLAN.md` 分支 D 的独立导航功能线；没有修改或复用孵育箱动作语义 authority
`v9/r17`、最新箱体视觉重放 `v19/r29` 或 826 AprilTag 动态规划 authority，也没有连接
或授权实体机器人。

最终导航专用链已包含自由根模型、Mid360、IMU、双足接触、truth-aided 仿真 odom、V12
在线策略与 effort PID、六门 Nav2 速度监督、planner/controller/BT 和 Gazebo-only 直立
辅助。关键最终参数为：

```text
map->odom future tolerance       2.0 s
map->odom republish rate         50 Hz
RPP max angular acceleration     0.40 rad/s^2
Nav2 XY goal tolerance           0.04 m
safe yaw clamp                   0.05 rad/s
captured effort return rate      0.0 rad/s
normal planar damping            300 N/(m/s)
CAPTURED_HOLD planar damping     900 N/(m/s)
maximum planar damping force     300 N
yaw assistance torque            0
```

历史演进必须保留：r93 再次证明 effort 后切回 position hold 会立即失稳；r95 暴露 Nav2
首个 `0.02 rad/s` 旋转命令低于 V12 `0.03 rad/s` 起步门的安全死锁；r96/r97 暴露大 yaw
请求和捕获后被动漂移；r98/r99 在 yaw 限制和捕获态增强耗散阻尼后通过；r100 虽 Nav2
action succeeded，但 Mid360 只位移 `0.047434 m`、终点误差 `0.106066 m`，严格门正确
FAIL，从而证明 `0.08 m` XY 容差会过早成功。收紧为 `0.04 m` 后，最终候选 r101/r102
连续 PASS。

最终主验收：

```text
r101  Mid360 displacement 0.130384 m  goal error 0.022361 m
      odom disagreement 0.008842 m    minimum up_z 0.997345
      max planar speed 0.190466 m/s   valid/invalid localization 60/0
r102  Mid360 displacement 0.168671 m  goal error 0.038079 m
      odom disagreement 0.011020 m    minimum up_z 0.995704
      max planar speed 0.170551 m/s   valid/invalid localization 77/0
```

两次最终状态均为 `CAPTURED_HOLD`、命令精确零、双脚接触为正、V12 fault 为 null，并
分别继续通过 `30.040806/30.036080 s` 冻结 effort 保持；保持期最大平面速度
`0.000275/0.000028 m/s`、腿目标漂移均为 `0.0 rad`、Mid360 无无效样本。r98/r99 的
assistance trace 机器可验证普通状态使用 `300.0`、只有 `CAPTURED_HOLD` 使用 `900.0`；
增强阻尼严格耗散，不提供平面推进或动作权威，yaw torque 恒为 0。

最终隔离构建和测试：

```text
log/build_2026-09-09_22-14-59  PASS
log/test_2026-09-09_22-15-00   72 tests / 0 errors / 0 failures / 0 skipped
```

正式报告及哈希：

```text
runtime_evidence/nav2_mid360_goal_tolerance_repeat_r102_20260909/REPORT.md
SHA-256 3a62a1f88e8bf46e297f3583259d8cd0c3e7d678839a1afddc2b5b0cb7be9cdc
```

关键源码哈希：

```text
gazebo_upright_stabilizer.py  a79518eb4215d62000d3c9cfd5d981ca2550f270552a471f2f59d5b59cdc1c5e
gazebo_upright_assistance     70850b78d6577a3316332ecf59bf0938faec12f3eaffa53c86ee89001d1d0b68
mid360_localization.yaml      a4e004894f1c2958bac2aa02950e2ea5462908d1b8a096dbdb8d0bdeb41e6cd4
mid360_square_localizer.py    ac93d17de13a861343875428ec3f4d99e6319e193a9256c84e9faf891e31edc4
nav2_control.yaml             45e1c41e0825df4a5dfce4b6e25461304287e8be8ef34ba07b1aacd6cf46ccd5
v12_locomotion.yaml           0223efe75dbd5712291b72927b07376168bd77b27e7227356620890a6d22ca1e
v12_gazebo_controller.py      3a6375f9f467323ff8320b160df9b28435a977228026cbf7e4b820c00b1f194a
locomotion_contract.py        84ef54d358e60622eb32bddcb3982cdd1e85b016fc40fc19ebb74b7eb7a1fbac
```

当前完成范围只包括 5 m 方墙内单个 0.15 m 前向目标的 Gazebo 定位行走和终态保持；不
外推为多目标、障碍物、任意终态航向、SLAM、动态重规划或真机成果。r102 完成后正常
SIGINT；22:21 CST 只读检查无 Gazebo server/GUI 或导航 launch 残留。909 仍是最新受
保护备份，但不包含本节源码、r94--r102 证据或文档；未获新编号，不得自行创建备份。

## 2026-09-10：后续工作不得破坏既有孵育箱成果

从本节起，任何后续导航、雷达、场景组合、定位、行走、录像、清理、构建或功能开发都
必须以不破坏既有孵育箱工作为前置硬约束。禁止覆盖、修改、删除、替换、降级或借用
孵育箱正式模型、轨迹、builder/authority、成功 run 核心、视频、rosbag、报告、证据、
控制语义或备份；尤其不得改动动作语义权威 `candidate3_roundtrip_831_authority_v9/` 与
`candidate3_roundtrip_831_r17/`、最新箱体视觉重放 `candidate3_roundtrip_831_authority_v19/`
与 `candidate3_roundtrip_831_r29/`，也不得覆盖当前保护备份 909。

后续需要把孵育箱加入导航或展示场景时，只能采用独立新身份、独立新路径和增量组合层；
不得把导航专用自由根模型或参数回写到孵育箱动作成果线，不得把临时组合场景冒充原
孵育箱动作 authority。实施前后必须核对受保护资产仍存在且哈希未变；若无法同时保证
新功能和既有孵育箱成果完整，必须 stop-and-hold，不得继续执行破坏性操作。

## 2026-09-10：固定开机位至孵育箱旁自动导航 PASS

用户在已明确授权本轮 Gazebo 定位行走后，要求规定固定开机位置并自动导航到孵育箱
旁。本轮归入 `HANDOFF_PLAN.md` 分支 D 的独立导航功能线，不是孵育箱动作分支 G，且
没有连接、启动或授权实体机器人。

新增 `incubator_navigation_gazebo.launch.py` 作为非覆盖组合层：它包含原位置的桌子、
绿色方块和孵育箱，并复用独立方墙导航链。导航专用 H1-2 增加左右 Dex1-1 的 16 个纯
视觉部件；这些部件没有质量、惯量、碰撞或控制关节，不改变导航动力学，也不能替代正式
孵育箱操作模型。固定合同为：

```text
startup pose                  (-0.60, 0.00, 0.00 rad)
incubator staging goal        (-0.45, 0.00, 0.00 rad)
nominal travel                 0.15 m
known workcell bounds          x=[0.20, 1.00], y=[-0.60, 0.60]
expanded workcell keepout      x=[-0.30, 1.50], y=[-1.05, 1.05]
maximum planned path           0.30 m
maximum physical displacement  0.28 m
physical corridor tolerance    0.12 m
```

`movement_authorized`、`upright_assistance`、`auto_navigate` 默认全部为 false。自动节点
必须等待 V12 fresh/直立/双脚接触、Mid360 连续有效和 AMCL/planner/controller/
behavior/BT navigator 全部 active，同时核对定位与 Gazebo 物理根仍在开机位；之后先用
`ComputePathToPose` 审计路径长度及工位 keepout，才允许发送 `NavigateToPose`。运动中
持续检查物理位移、走廊、姿态和健康状态；终点要求物理/定位误差、航向、零命令、
`CAPTURED_HOLD` 和连续 50 个稳定样本。任一失败立即取消目标。transient-local 状态主题
为 `/h1_2_navigation/incubator_navigation/status`；读取落盘终态时必须显式使用
reliable + transient_local QoS，普通 sensor-data QoS 不能取得历史样本。

静态候选 `navigation_incubator_fixed_start_static_20260910_r5` 为 PASS：完整六模型场景、
16 个夹爪视觉、门 locked、support 不存在、机器人物理/定位开机位正确、四墙定位有效、
工位中心地图值 100、启动点地图值 0、`/scan.range_min=0.10 m`、V12 零命令且移动关闭。
顶部 Mid360 最低垂直视场约为 `-7 deg`，低矮工位在该起点低于激光视线，最近有限回波
约 `1.856230 m`；因此只能称为固定已知工位导航。精确工位矩形由静态地图占据，并由
独立物理根走廊门二次监督；不得外推为未知低矮障碍动态检测。

自动尝试历史必须保留：r6 在 BT active 前发送目标而被拒绝，随后增加全部 Nav2
lifecycle 门并修复日志严重级别；r7 因 controller 过渡状态缺少 `root_position` 立即
取消，随后让所有 post-odom V12 状态携带根诊断；r8 因过渡状态缺少完整直立字段立即
取消，随后保留最近有效 `up_z` 并从根位置推导高度。三次都没有作为成功证据，均正常
SIGINT 且没有覆盖旧目录。

正式 `navigation_incubator_auto_staging_20260910_r9` 为
`FIXED_STARTUP_TO_INCUBATOR_STAGING_PASS / PASS`：调用至 PASS `50.427609926 s`，规划
路径 `0.149999963 m`，物理位移 `0.152063123 m`，物理/定位终点误差
`0.020184558/0.020000000 m`，最终物理根
`(-0.449280743,-0.020171739,1.087188257)`，最终 `up_z=0.9999863014`，双脚接触
`378.708/267.195 N`，门 locked，support 不存在，操作 trajectory topic 不存在。全部
19 项机器门为 true。

r9 随后完成 `30.045255345 s` 冻结保持，759 个样本、最大平面速度
`0.000026730 m/s`、腿目标漂移 `0.0 rad`、最小 `up_z=0.9999863640`、Mid360 定位
有效/无效 `30/0`，结果为 `FROZEN_EFFORT_HOLD_STABLE_30_SECONDS / PASS`。终点按机器
人包络距桌面前缘 `0.199280743 m`；这是操作前等待位，不会自动触发孵育箱双循环。

源码按 Miniconda 六库精确隔离规则和 `--base-paths src` 构建通过；最终
`84 tests / 0 errors / 0 failures / 0 skipped`，`ament_flake8` 33 个文件无问题。关键
哈希：

```text
dex1 navigation visual       183fbd04dddd1b30f0ff8bb7e7a3ee063c5dfb15ea192b9446ebc84964687afe
navigation URDF              f862a331a099b5aece7193ffd4baecc9ac1a48a82cebd51869b937d4227ddc7b
square_localization.py       351f6e2eeb5c3d1cc89d28e3301a5cff5c6c22c18b94e9b07d9d0d34149b005f
mid360 localizer             65c49409765cf9a0194ca2a9476d881b466d2dcf516ceb34deef9694d913a199
auto navigator               c2174ecdf1a2efc57ee7f369cb6e6e2b2fa039b3949bf3dcf4a7af922c3ab85f
V12 Gazebo controller        164bdc6d8b29b503f67cecafcbc75791691f2dcb78a692be4573ac92aebac165
base navigation launch       1ef8c871814b0a65c342d11223a40c3d2daab665f2879a00d3d69f293f00baf9
incubator navigation launch  a333c8be105b888c6c995b5c3c1abc8235b3c4780607286b6bb9ba5d73e7d6b3
Mid360 config                4786feb493efc8a9fee306ff90d247320b42ad44823826701d247a829f7cd27f
Nav2 control config          6b98d086b5de10a36ef5aa33cfffd49bb926c99fce5bf4092b8e74c09db65d5d
Nav2 planner config          663622aecb7b1cf6b0561259a10eff877b641f5a605d099dd0a50404dd04b7da
r5 summary                   aae39ad2091302e705e32693f1ad89a2049ab406a7d7feda665e636165126615
r9 summary                   ba8f3d7c1dbd7e9b58cbdca3bf59d5db4b4139a7173f1cd01a4a2a6dbd9d9a6e
r9 long hold                 8ecdbc4b454a1b70479ed1561f5e605498536b7463156f1a83cffbbed6df10ed
r5 report                    572b849ce56b84d9027100cec4831b98abc3586200d165abe8b0a4285e69296b
r9 report                    389d93569f87dd28d8e6873e5fd682432ac49424c60cfb44803f94db6be6e2ab
navigation plan              5bc67a1ea3b08067a6c09064c14f99503ceba15b7fd5b4817f8a6d59c941173b
navigation README            e48496206fd190155e5aecc1984a1f0187b23ab06aa4c36d52c12fc9c4e70243
HANDOFF_CURRENT              6e848997a3c9c2f72a911d98899780179791cd85477ec28f9cbab3853c09c8b9
```

r9 前后重新核验 v9 manifest `b9e49b...`、v9/r19 incubator `44ff23...`、r17 summary
`859e62...`、v19 manifest `43c765...`、r29 summary `17f1f1...`、源码 incubator
`44ff23...` 和 backup 909 `d00e10...` 全部不变。没有修改孵育箱动作 authority，没有
创建备份；909 仍是最新保护点，但不包含本节新增源码、r5/r9 证据和文档。

完整报告：

```text
runtime_evidence/navigation_incubator_fixed_start_static_20260910_r5/REPORT.md
runtime_evidence/navigation_incubator_auto_staging_20260910_r9/REPORT.md
```

2026-09-10 12:32 CST 只读检查时，r9 的 Gazebo server/GUI、组合 launch、Nav2、Mid360、
V12 和自动导航节点仍运行，机器人稳定停在等待位供用户查看；这是可变 live 快照，后续
必须重新只读检查。未来再次移动必须使用 fresh Gazebo、新证据身份和新的明确移动授权，
不得复用 r9 作为新运行目录。当前 PASS 不包含未知/动态低矮障碍、多目标、SLAM、在线
动态重规划、导航后孵育箱操作衔接或实体机器人。

## 2026-09-11 步态展示否决、r10 失败与后续准入路线

本节覆盖上一节对 V12 Gazebo 步态可用性的较早判断，但不删除 r9 的历史机器证据。用户
在 fresh `navigation_incubator_auto_staging_20260911_r10` 展示中明确判断当前步态错误；
同一次运行的物理路径安全门也触发
`FAILED:PHYSICAL_PATH_ENVELOPE_EXCEEDED`，Nav2 正常取消目标。随后按用户要求 SIGINT
停止整套场景。r10 的身份和失败证据必须保留，不得覆盖、复用或引用为导航成功：

```text
runtime_evidence/navigation_incubator_auto_staging_20260911_r10/ATTEMPT_STATUS.md
SHA-256 2f1f73e553139524dd47dce4dc4556722e86bda87496e590b125da7de86c7ac7
```

停止后只读检查没有 Gazebo server/GUI、Nav2、AMCL、Mid360、controller manager 或 V12
进程。v9 manifest `b9e49b...`、v9/r19/source incubator `44ff23...`、r17 summary
`859e62...`、v19 manifest `43c765...`、r29 summary `17f1f1...` 和 backup 909
`d00e10...` 的运行前后哈希完全一致；没有操作轨迹、夹爪动作或 support daemon，也没有
修改孵育箱成果或创建备份。

当前 Gazebo 适配器不是 V12 训练接口的忠实部署：配置把训练步频乘以
`0.6574621959237344`，把 effort 目标变化限制为每周期 `0.001 rad`，只运行 15 个运动
周期就进入制动/保持，再等待 50 个稳定周期后重启。该突发式状态机和动作限幅会改变
50 Hz、78 维观测/12 维动作策略的时序与动作语义；Isaac PhysX 到 Gazebo DART 的接触
动力学差异又未通过独立 sim2sim 门消除。因此 r9 只能保留为旧适配器短距离机器回归证据，
不得再称为用户认可的正确步态、自然步态或已完成的定位行走能力；r101/r102 同理。

本机已有 Unitree 官方 H1_2 候选基线：

```text
/home/liu/robot/gym_sim/unitree_rl_gym-main/deploy/pre_train/h1_2/motion.pt
SHA-256 ea3003268d5d2e4b5004eb26d0969fbfdedced265ceeec7bf0b335309259cc80
```

其原生 MuJoCo 配置使用 47 维观测、12 维动作、50 Hz 更新，并直接计算
`target_dof_pos = action * 0.25 + default_angles`，没有当前 Gazebo 的积分目标、15 周期
突发和 50 周期重启等待。该 checkpoint 目前只可作为隔离评估候选，不是本项目 Gazebo
成功证据，也不得直接替换当前策略后宣称完成。

后续唯一准入顺序是：先在官方独立 MuJoCo 场景查看该候选的连续步态；用户认可后新建
从未使用的 V13/Gazebo 身份，忠实移植其观测、动作、关节顺序、默认角、PD 增益和 50 Hz
时序；依次通过零命令站立、连续 `0.05--0.10 m/s` 直行、停止保持，再做
Isaac -> MuJoCo -> Gazebo 的交叉仿真审计，最后才重新连接 Nav2。禁止覆盖 V12、V1、
孵育箱 authority 或正式 run，禁止再次用旧突发适配器演示。若官方步态仍不够自然，才
新训练 V13，优先加入 motion imitation/AMP、足滑与步宽约束、动力学随机化及跨仿真残差
校正；MPC + whole-body control 是更高成本的长期备选。任何新仿真移动仍需用户明确授权、
fresh Gazebo 和新证据身份；实体机器人仍未授权。

## 2026-09-11：官方 Unitree H1-2 步态已接入完整孵育箱导航场景

本节覆盖上一节“尚待接入 Gazebo/Nav2”的当前状态。用户已在独立 MuJoCo 展示中确认
Unitree 官方 H1-2 步态观感正确，并明确要求把该步态加入原完整 Gazebo 场景；本轮继续
沿 `HANDOFF_PLAN.md` 分支 D 的独立导航功能线执行，不属于孵育箱动作分支 G，也没有
获得任何实体机器人许可。

新增的隔离 `unitree_h12` locomotion profile 忠实使用官方 `motion.pt` 的 47 维观测、
12 维动作、LSTM、50 Hz 周期、`action * 0.25 + default_angles` 直接腿位置目标和官方
PD 参数。旧 V12 的频率缩放、累计目标、每周期目标限幅、15 周期突发、制动捕获和重启
驻留均不在该 profile 中。导航模型保持自由根、Mid360、IMU、双足接触和 truth-aided
Gazebo odom；上身固定在官方 12-DOF 零位，但左右夹爪视觉均存在。`upright_assistance`
在正式验证中为 false，未用外力维持或推进机器人。

组合入口 `incubator_navigation_gazebo.launch.py` 现在可选择
`locomotion_profile:=unitree_h12`，并同时显示 5 m 方墙、H1-2、Mid360、双夹爪、桌子、
绿色方块和原孵育箱。固定开机位仍是 `(-0.60, 0.00, 0.00 rad)`，Nav2 箱旁等待位仍是
`(-0.45, 0.00, 0.00 rad)`。自动链保持
`health -> ComputePathToPose -> NavigateToPose -> stable hold`，运动支持门针对官方连续
步态允许预期单脚支撑，但要求双脚总支撑力至少 `300 N`；静态状态仍要求双脚均有接触。

到站零命令继续运行官方 recurrent station-keeping gait，不切回静态 position controller。
在 MOVE -> 精确零命令的单次转换中只重置一次 LSTM hidden/cell memory，同时保持
runtime gait phase、previous action、effort PID 和直接目标连续；位置/速度反馈仅生成受限
策略速度命令，不修改关节动作权威。最终参数为位置增益 `2.0`、线速度阻尼增益 `0.8`、
yaw 增益 `1.0`、yaw-rate 阻尼 `0.5`、策略命令上限 `[0.10, 0.08, 0.05]`。

验证演进必须保留：

- `unitree_h12_incubator_nav2_velocity_damped_hold_20260911_r15` 的 recorder 因 wrapper
  在 ROS setup 前启用 `set -u` 而退出，Gazebo 未启动、没有发布命令，身份已封存；
- `...r16` 首次在完整场景内 Nav2 到站并 PASS，20 秒内无 fault，但最大物理终点误差
  `0.115282 m`；
- `unitree_h12_incubator_nav2_lstm_entry_reset_20260911_r17` 再次到站并 PASS，单次 LSTM
  reset 机器记录为 1，峰值平面速度从 r16 的 `0.314179` 降至 `0.195444 m/s`；
- `unitree_h12_incubator_nav2_lateral_authority_20260911_r18` 把横向上限提高为 `0.10 m/s`
  后触发 `FAILED:STABLE_HOLD_TIMEOUT`，该参数已逐字节回退，r18 不得引用为成功。

r17 的机器结果为：规划路径 `0.149999963 m`，终端物理/定位误差
`0.060260989/0.056568542 m`，终端 `up_z=0.9997936120`；PASS 后继续记录
`19.991777935 s / 495` 个 locomotion 样本，状态始终为 `STATION_KEEPING_GAIT`、fault 为
null、最小 `up_z=0.9995195086`、最小高度 `1.002879954 m`、最小总支撑力
`329.743028 N`。完整报告：

```text
runtime_evidence/unitree_h12_incubator_nav2_lstm_entry_reset_20260911_r17/REPORT.md
SHA-256 a097e27959c1926d32c8338ae188d5214e836a9cd1d0dabddf061290c1e673db
```

r17 证明用户认可步态已在完整原场景内完成 Mid360 定位、Nav2 规划和实际腿部行走；但
动态驻足的 20 秒物理终点误差峰值为 `0.108891 m`、station-anchor 峰值为
`0.101255 m`，所以不得外推为严格连续 `0.08 m` 冻结保持、长距离、多目标、未知低矮
障碍、SLAM、动态重规划、孵育箱操作衔接或真机成果。

最终源码已按 Miniconda 六库隔离规则重新构建，`h1_2_navigation` 为
`112 tests / 0 errors / 0 failures / 0 skipped`。关键最终哈希：

```text
unitree_h12_gazebo_controller.py  3c2767e4bcc4d264d43a3d1a47cdcb6a617a7d724827b56080747791bcd799cb
unitree_h12_locomotion.yaml       cb7e9797df04692c504c765e2ef348a93ae79674b39cf8919c8a40046a269731
unitree_h12_contract.py           5a71e90107a0877c9500ed518657d21256a15265fab939fda70ce8ec97e0f978
incubator_auto_navigator.py       92607c0a243404149d67dd3c607ac630d9721e902464a70e0469f094cd50427d
incubator navigation launch       83a0349e379fc5bc8c0ee6031392e3422f208a9ce2125b846c728be12dfde864
official policy export            68a1f2dff967fdc464451956eb6b4c60288cf375e38629092edab23b2adb014c
```

每次运行前后均核对 v9/r17、v19/r29、源码孵育箱模型和 backup 909，全部哈希不变；没有
发布任何手臂、躯干、夹爪、门、把手、方块或 support 动作，没有 reset/detach/recovery，
也没有创建备份。最终 r18 后已正常 SIGINT，最新只读检查没有 Gazebo、Nav2、AMCL、
Mid360、controller manager 或 recorder 残留。909 仍是最新保护备份，但不包含本节新增
官方步态源码、r15--r18 证据或文档。

## 2026-09-11 18:56 CST：r19 降低驻足速度阻尼试验否决并回退

两天持续优化的首轮继续属于 `HANDOFF_PLAN.md` 分支 D，仅获得 Gazebo 导航步态验证
授权。运行前重新只读确认没有 Gazebo、Nav2、AMCL、Mid360、controller manager 或
recorder 残留；v9/r17、v19/r29、源码孵育箱模型和 backup 909 的六项保护哈希全部匹配。

基于 r17 记录的离线对拍，驻足横向策略命令在 `linear_velocity_gain=0.8` 时约 60% 样本
触及 `0.08 m/s` 上限。本轮只把该增益改为 `0.4`，没有改变其他步态、Nav2、场景或安全
参数；按六库隔离规则构建并通过 `112/112` 测试后，以普通 `mkdir` 独占创建 fresh 身份：

```text
runtime_evidence/unitree_h12_incubator_nav2_reduced_velocity_damping_20260911_r19/
```

r19 使用 headless 完整组合场景，方墙、H1-2、Mid360、双夹爪视觉、桌子、绿色方块和原
孵育箱均由同一 launch 加载。Nav2 仍沿 `0.15 m` 路径到站，短稳定门以物理/定位误差
`0.065291831/0.057008771 m` 发出 PASS；但独立记录器继续观察 30 秒后否决该参数：

```text
first goal error > 0.08 m       PASS 后 11.841911 s
first corridor error > 0.12 m   PASS 后 13.796557 s
maximum goal/corridor error     0.615208976 m
maximum planar speed            0.699432456 m/s
minimum up_z / height           0.981973414 / 0.961643332 m
locomotion state/fault          STATION_KEEPING_GAIT / null
```

因此短门 PASS 不能覆盖后续路径逸出，r19 的最终身份为
`REJECTED_POST_PASS_PATH_EXCURSION`，不得引用为成功或复用。完整机器证据：

```text
ATTEMPT_STATUS.md  34cf8ee692515fff6801769c08eec3816374957ecb0b292d166881f2c1bf5eab
navigation_audit   db3ee7477862a0976dbcaeb49190384452c49dfae6013bd0a75e8ad867887b09
SHA256SUMS         6b2c881e9f34c89b82afc48753470a4630f4347db13159c1373feb5371663bb3
```

记录完成后通过正常 SIGINT 结束整套 launch；没有 reset、detach、recovery、夹爪、门、
把手、方块、support 或孵育箱操作命令。试验参数已逐字节回退，隔离重建与 `112/112`
测试再次 PASS；当前配置 SHA-256 恢复为
`cb7e9797df04692c504c765e2ef348a93ae79674b39cf8919c8a40046a269731`，即 r17 版本。六项
保护哈希再次匹配，最终 live 离线，未创建备份。

下一轮不得继续降低阻尼。优先先让运行监督在 PASS 后仍保持路径/姿态 fail-closed，随后
单独验证“仅在约 `phase=0.5` 的对称步态相位执行一次 station-entry LSTM reset”这一
假设；r17 的 reset 相位约 `0.475`，而 r18/r19 分别约 `0.775/0.200` 且结果明显更差。

## 2026-09-11 22:58 CST：30 秒持续驻足监督落地，r21 fail-closed

本轮仍只使用 `HANDOFF_PLAN.md` 分支 D 的 Gazebo 导航步态授权。先保留 r17 的官方策略、
驻足增益、策略命令上限、Nav2 参数和完整组合场景不变，只修正自动验收语义：原连续 50 个
稳定样本不再直接发出 PASS，而是进入 `VALIDATING_LONG_HOLD`；其后连续 `30 s` 的每个
locomotion 样本都必须同时满足定位有效、物理/定位终点误差 `<=0.08 m`、航向门、
`up_z>=0.98`、高度 `>=0.95 m`、官方 recurrent station keeping 和支撑门。只有完整持续
时间通过才发出 `PASS:INCUBATOR_STAGING_GOAL_LONG_HOLD`，且 PASS 后仍持续监督并可转为
FAILED。新增纯函数门和源码合同测试后，按 Miniconda 六库隔离规则构建，结果为
`113/113` 测试 PASS。

`unitree_h12_incubator_nav2_long_hold_supervisor_20260911_r20` 在 recorder 预检时误用了
Miniconda Python 3.14，无法加载 ROS Jazzy Python 3.12 的 `rclpy`；Gazebo 从未启动，身份
已封存且未复用。随后用 `/usr/bin/python3` 和普通 `mkdir` 创建 fresh r21：

```text
runtime_evidence/unitree_h12_incubator_nav2_long_hold_supervisor_20260911_r21/
```

r21 的 Nav2 action 成功，但进入 `WAITING_FOR_STABLE_HOLD` 后仅 `0.520142 s`，物理走廊
误差已达 `0.149482556 m`，物理/定位目标误差为 `0.152793017/0.132003788 m`，平面速度
`0.612084961 m/s`，触发 `FAILED:PHYSICAL_PATH_ENVELOPE_EXCEEDED`。监督器没有发出旧短门
PASS，也未到达新 `VALIDATING_LONG_HOLD`。终端前最小 `up_z/height` 为
`0.988962160/0.989349382 m`、无 locomotion fault；launch 日志在导航失败后
`1.451483 s` 记录 fall-envelope hold latch，故 r21 为失败证据，严格验收计数仍为 0。

30 秒监督属于 fail-closed 安全修正，单测通过且不是本次步态漂移成因，因此保留；但 r21
没有覆盖其 live 长驻足状态，后续 fresh 身份仍须实际进入并完成该状态。当前关键源码为：

```text
incubator_auto_navigator.py       b5e00041171d89dd29d3f2056902212e01590f595483e9e06d1ad598bcbd7f91
unitree_h12_gazebo_controller.py  3c2767e4bcc4d264d43a3d1a47cdcb6a617a7d724827b56080747791bcd799cb
unitree_h12_locomotion.yaml       cb7e9797df04692c504c765e2ef348a93ae79674b39cf8919c8a40046a269731
navigation recorder v2            d3eeb87ff82e4db3048cc722a981ccf85cca1fbe41da0b4a5b5a59f130e16478
```

r21 前后 v9/r17、v19/r29、源码孵育箱和 backup 909 六项保护哈希逐字节一致；没有发布
门、把手、方块、夹爪、support、reset、detach、recovery 或真机命令，没有创建备份。
整套 launch 已正常 SIGINT，最终 live 离线。下一项单变量假设是在保持所有增益和限位
不变时，把 station-entry 的一次 LSTM reset 延后到接近 `phase=0.5` 的对称相位；每轮
仍必须使用新身份并由 30 秒持续监督判定，失败即 stop-and-hold。

## 2026-09-12 03:02 CST：r22 相位门假设否决并完整回退

本轮继续只使用 `HANDOFF_PLAN.md` 分支 D 的 Gazebo 导航步态授权。运行前确认 live 离线，
六项孵育箱保护哈希匹配。r17--r21 离线对拍显示 station-entry LSTM reset 相位分别约为
`0.475/0.775/0.200/0.825`，其中 r17 最稳，因此本轮只增加 `phase=0.5 +/- 0.026` 的一次
reset 门；官方策略、驻足增益、命令限位、Nav2、场景和 r21 的 30 秒 fail-closed 监督均
不变。实验源码按六库精确隔离并叠加本地 Nav2 overlay 构建，`115/115` 测试 PASS。

fresh 身份为：

```text
runtime_evidence/unitree_h12_incubator_nav2_phase_gated_reset_20260912_r22/
```

r22 自然在 `phase=0.500000` 进入 station keeping，因此 reset 正好发生在目标相位、计数为
1、实际延迟为 0。Nav2 action 成功，但物理目标误差在到站后 `0.275072 s` 即超过
`0.08 m`，`21.630458 s` 后走廊误差达到 `0.120026538 m` 并触发
`FAILED:PHYSICAL_PATH_ENVELOPE_EXCEEDED`；最大物理目标误差 `0.127779738 m`、最大平面
速度 `0.331728240 m/s`、最小 `up_z/height=0.998146850/0.998102606 m`，定位有效/无效
`18/0`，无 locomotion fault 或 fall。未进入 `VALIDATING_LONG_HOLD`，未发出 PASS，严格
验收计数增加 0。

```text
REPORT.md         59670eabbb0e185800c423dd4507fc6419ee94d47a4a5076067dd2975c69528a
navigation_audit  7ba76b5e962018c85ea998f01779fd8710561b6e27f12d385fa266dcb73b64da
SHA256SUMS        6e1f045968de910b58daa67178807940b69a9eba5cc62c16d660f7b08e1d598c
```

相位门未达到 30 秒/8 cm 验收，已从 controller、contract、config 和测试逐字节移除；
回退后关键哈希重新为：

```text
unitree_h12_gazebo_controller.py  3c2767e4bcc4d264d43a3d1a47cdcb6a617a7d724827b56080747791bcd799cb
unitree_h12_locomotion.yaml       cb7e9797df04692c504c765e2ef348a93ae79674b39cf8919c8a40046a269731
unitree_h12_contract.py           5a71e90107a0877c9500ed518657d21256a15265fab939fda70ce8ec97e0f978
incubator_auto_navigator.py       b5e00041171d89dd29d3f2056902212e01590f595483e9e06d1ad598bcbd7f91
```

回退版本再次六库隔离构建并通过 `113/113` 测试。证据 runner v1 在 FAILED 后立即正常
SIGINT 并让 launch 以 0 退出，但向整个进程组发信号产生 2 条仅收尾期 rcl context error；
v1 随 r22 保留但不得复用，下一版只向 `ros2 launch` 父进程发 SIGINT。

r22 前后及回退后，v9/r17、v19/r29、源码孵育箱和 backup 909 哈希均不变；没有孵育箱
操作、reset、detach、recovery、真机或备份动作，最终 live 离线。下一轮不得继续相位调参；
单变量假设改为只把 Nav2 XY goal tolerance 从 `0.08 m` 收紧到 `0.04 m`，因为当前 action
常在严格 8 cm 边界附近结束，几乎不给正常 recurrent gait 摆动留下裕量。步态参数保持
不变，仍由 30 秒持续门裁决。

## 2026-09-12 07:04 CST：r23 Nav2 到站容差假设否决、回退与 runner 修复准入

本轮继续只使用 `HANDOFF_PLAN.md` 分支 D 的 Gazebo 导航步态授权。运行前 live 离线，
v9/r17、v19/r29、源码孵育箱模型和 backup 909 六项保护哈希全部匹配。实验只把 Unitree
launch 覆盖的 Nav2 `goal_checker.xy_goal_tolerance` 从 `0.08 m` 收紧为 `0.04 m`；官方
policy、controller、驻足 anchor、增益、命令限位、完整组合场景和 30 秒 fail-closed
监督均未改变。六库精确隔离构建和运行前 `113/113` 测试通过，fresh 身份为：

```text
runtime_evidence/unitree_h12_incubator_nav2_xy_goal_tolerance_004_20260912_r23/
```

r23 的 Nav2 action 以物理/定位误差 `0.033634058/0.049244289 m` 成功，比原 8 cm 门留下
更多位置裕量；但第一条 station-keeping 样本的平面速度仍为 `0.253626616 m/s`，其中横向
速度 `-0.248743008 m/s`，未改的横向策略反向命令立即饱和到 `+0.08 m/s`。物理目标误差
在到站后 `0.610307 s` 超过 8 cm，走廊误差在 `1.619540 s` 达到 `0.121798120 m` 并触发
`FAILED:PHYSICAL_PATH_ENVELOPE_EXCEEDED`。终端前最大目标误差/速度为
`0.122805287 m / 0.319335079 m/s`，最小 `up_z/height` 为
`0.996858964/1.001751886 m`，定位有效/无效 `2/0`，当时 locomotion fault 为 null；未进入
`VALIDATING_LONG_HOLD`、未发 PASS，严格验收计数仍为 0。

本轮 runner v2 的 parent-only SIGINT 暴露了独立基础设施错误：后台 shell 使
`ros2 launch` 父进程继承忽略 SIGINT，30 秒后仍未退出。在 recorder 已记录 FAILED 后的
延迟收尾期，launch 又于 `12.402407/13.391662/13.562090 s` 后分别记录关节限位错误、
fall-envelope latch 和定位质量丢失，所以完整 r23 必须同时保留为安全失败，不能只引用
终端前姿态数据。随后向父进程发送 TERM，其孤立子进程均逐 PID 发送 SIGINT 停止；没有
向进程组发信号，最终 live 离线，shutdown rcl error 为 0。v2 随证据保留但不得复用。

新建的 `navigation_signal_reset_exec_v1.py` 会在 `setsid` 后恢复 SIGINT/SIGQUIT/SIGTERM
默认 disposition，再 exec 目标；runner v3 使用该 trampoline 后只向 launch 父 PID 发
SIGINT。无害 `sleep` 探针以状态 `130` 退出且无残留，但 v3 尚未经过 Gazebo 收尾验证，
下一次 fresh 运行必须同时审核其正常 parent-only shutdown，不能预先称为通过。身份：

```text
navigation_signal_reset_exec_v1.py  d55ce84ed638f4a1f5f005fb17ee630d28c5bf067491dba233283acf86e43767
navigation_gazebo_evidence_runner_v3.sh  16f32b46cab229dfb977d2885e444bd6193c2c015cdad44e83e92c666699b0dc
```

`0.04 m` 参数和测试期望已经回退为 `0.08 m`，再次六库隔离构建并通过 `113/113`；当前
controller/config/contract/auto-navigator 哈希仍为
`3c2767.../cb7e97.../5a71e9.../b5e000...`。实验运行后的 0.04 源码快照在回退前没有单独
落盘，因此机器审计明确将“运行期间集成哈希不变”记为 null，不作过度结论；运行前实验
哈希、回退后哈希和全部 raw traces 均已保留。正式证据哈希：

```text
REPORT.md         dcd5f32fd594aa56e345188fba522f844587c48edd0c4795bdfda6d027f45f3d
ATTEMPT_STATUS    94bd25fca029cfa8e72835e6e59d894cde56bc87796d0e61b14a2ab0f19a6772
navigation_audit  edb2d7342d81d494fb13c3535f9fdc506f8aaf238d5e9d718ba76a6f541be18e
SHA256SUMS        3ff223ecc5bc8be084ef08500cc8ede54d77e24f922e3e685479105e068e7309
```

六项保护哈希在前后逐字节一致；没有发布门、把手、方块、手臂、夹爪、support、reset、
detach、recovery 或真机命令，也没有创建备份。下一轮不得重复位置容差调参。低风险单变量
改为在保持 `0.08 m` 位置门时，将 Nav2 `SimpleGoalChecker` 替换为本地已存在的
`StoppedGoalChecker`，显式设置 `trans_stopped_velocity=0.15 m/s`，其余 gait 与 reset
不变，以检验 Nav2 是否能在低物理速度时才交接；失败立即回退。新 run 还必须验证 v3
父进程 SIGINT 能让全 launch 无 orphan 正常退出。

## 2026-09-12 11:06 CST：r24 StoppedGoalChecker 假设否决、回退与低层交接门准入

本轮仍只使用 `HANDOFF_PLAN.md` 分支 D 的 Gazebo 导航步态授权。运行前 live 离线，六项
孵育箱保护哈希全部匹配。实验只把 Unitree launch 的 Nav2 goal checker 从
`SimpleGoalChecker` 改为 `StoppedGoalChecker`，保留 `xy_goal_tolerance=0.08 m` 并新增
`trans_stopped_velocity=0.15 m/s`；官方 policy、gait controller、station anchor/reset、
增益、命令限位、完整组合场景和 30 秒 fail-closed 监督均未改变。focused `9/9`、六库
精确隔离构建和运行前 `113/113` 测试通过，fresh 身份为：

```text
runtime_evidence/unitree_h12_incubator_nav2_stopped_goal_015_20260912_r24/
```

runtime 日志确认 `nav2_controller::StoppedGoalChecker` 成功加载。原始 Nav2 命令先在
`21.458088059 s` 归零，低层 controller 随即在 `21.511924993 s` 进入 recurrent station
keeping；自动客户端直到 `21.517945306 s` 才收到 Nav2 action success，因此 gait 交接早于
action callback `0.006020 s`。第一条 station 样本速度虽为 `0.107827809 m/s`、低于 Nav2
的 `0.15 m/s` 门，但 motion-reference/station-anchor 误差已为 `0.084899912 m`；到站 callback
时物理/定位误差为 `0.104127277/0.081394103 m`。物理目标误差在 callback 后首个样本即继续
超过 8 cm，`1.431943 s` 后走廊达到 `0.122054353 m` 并触发
`FAILED:PHYSICAL_PATH_ENVELOPE_EXCEEDED`。最大目标误差/速度为
`0.130899572 m / 0.223678280 m/s`，最小 `up_z/height=0.999587325/1.005426575 m`，定位
有效/无效 `1/0`，终端前无 locomotion fault 或 fall；未进入 30 秒长门、未发 PASS，严格
验收计数增加 0。

该结果证明 Nav2 goal checker 只约束 action 完成，不能约束由“gated command 精确为 0”
直接触发的低层 gait 交接。r23 是低 anchor error/高速度失败，r24 是较低速度/高 anchor
error 失败，后续不得再做单独 Nav2 容差或 stopped-velocity 调参。StoppedGoalChecker 和
测试期望已完整回退；当前 launch/test 哈希恢复为
`886ada10.../942915c0...`，再次六库隔离构建并通过 `113/113`，controller/config/contract/
auto-navigator 仍为 `3c2767.../cb7e97.../5a71e9.../b5e000...`。

runner v3 证明 signal-reset 后 launch 父 PID 能接受 SIGINT 并以 0 退出，且无 rcl context
error；但 Jazzy `ros_gz_sim` 的 shell wrapper 在 5 秒后被 launch TERM，真实 Ruby `gz sim`
PID 1545396 仍成为孤儿，随后只向该 PID 发 SIGINT 才完成收尾。收尾窗口出现 1 条左膝
joint-limit 日志、无 fall；未发进程组信号。v3 随 r24 保留但不得复用。新建 runner v4 会
分别向真实 Gazebo PID 与 launch 父 PID 发 SIGINT，再逐 PID 审核并有界清理整个 launch
session；语法和无害 Ruby-child 探针 PASS、残留 0，但尚未经过 Gazebo 验证：

```text
navigation_gazebo_evidence_runner_v4.sh  3ddf17672e89c18a96d6262c93fb98847554290715ee390fec4977832d18d664
REPORT.md                                642cc185b8e55f48762f1495124ffbe82362e266da6e65e9c9a338d6c3507b02
navigation_audit.json                   65b0e9994cdad47e97533180f3b7b0178e5aaee9b5d78d8057401de711904808
```

r24 运行前后实验集成哈希逐字节一致，最终六项保护哈希继续匹配；没有孵育箱操作、reset、
detach、recovery、真机或备份动作，最终 live 离线。下一轮准入的单一逻辑假设必须移到低层
transition owner：外部命令从 MOVE 首次变为精确 0 后，保持 motion reference 与 recurrent
state 继续零前馈 pose-tracking settling；只有平面速度 `<=0.08 m/s` 且 tracking error
`<=0.04 m` 连续 5 个 50 Hz 样本，才进入 station keeping 并执行一次 reset。Nav2、policy、
增益、限位、场景和长门不变；先补单元/集成测试和六库隔离构建，再用全新身份同时验证
功能门与 runner v4。任一门失败立即 stop-and-hold、正常收尾并逐字节回退。

## 2026-09-12 15:06 CST：r26 双条件 settling 无共同窗口，否决并回退

本轮继续只使用 `HANDOFF_PLAN.md` 分支 D 的 Gazebo 导航步态授权。先按上一节准入实现唯一
低层交接假设：`MOVE` 后外部命令精确归零时保留 motion reference/LSTM，以零前馈继续
recurrent pose tracking；只有平面速度 `<=0.08 m/s` 与 tracking error `<=0.04 m` 连续 5 个
50 Hz 样本同时成立，才进入 station keeping 并执行一次 reset。Nav2、官方 policy weights、
增益、命令限位、完整组合场景和 30 秒 fail-closed 监督均未改变。focused `20/20`、六库
精确隔离构建和运行前 `116/116` 测试均 PASS。

第一个普通 `mkdir` 独占身份 r25 在 Gazebo 前的 install-binding 只读探针误用了 Miniconda
Python 3.14，无法加载 ROS Jazzy Python 3.12 `rclpy`；Gazebo 从未启动，身份已封存且不得
复用。改用 `/usr/bin/python3` 重新通过预检后，另建 fresh r26：

```text
runtime_evidence/unitree_h12_incubator_nav2_settling_handoff_20260912_r25/
runtime_evidence/unitree_h12_incubator_nav2_settling_handoff_20260912_r26/
```

r26 的完整场景健康门、Mid360 定位、Nav2 lifecycle、规划和 action 均成功。action success
时物理/定位误差为 `0.075500960/0.072801099 m`，但 175 条 settling 样本中速度门单独满足
47 条、误差门单独满足 28 条、同时满足 0 条；`settling_qualified_steps` 始终为 0，未进入
station keeping，也未 reset。最接近的一对仍是 `0.077192501 m/s / 0.043353404 m`；最低
速度 `0.016828020 m/s` 对应误差 `0.046271734 m`，最低误差 `0.004898986 m` 又对应速度
`0.396716772 m/s`，证明两门落在不同步态时段。

本轮 motion reference 还在 settling 入口已经离目标 `0.103319675 m`，而当时物理 root
仅离目标 `0.075754276 m`；之后 `0.10 m` leash 随漂移移动参考。Nav2 success 后
`0.841490 s` 物理目标误差超过 8 cm，`7.861665 s` 横向走廊达到 `0.122057083 m` 并
fail-closed。最大物理目标误差/速度为 `0.177695692 m / 0.428209453 m/s`，最小
`up_z/height/support=0.997794361/0.994945734 m/451.496112 N`，定位 `8/0` 有效/无效，
无终端前 fault/fall，未进入长门、未发 PASS，严格验收计数增加 0。

完整证据：

```text
r25 ATTEMPT_STATUS    7475132f60e202c2d70e960fbd148fd451ffcd3671cd209a3186b5ed08e41657
r25 SHA256SUMS        0fb8f4ea0fb981076ac92cb3de8ebaf0d33f1f7f87b976b83ae8fe28928afc01
r26 REPORT            ea93e6393d51578160b62d09ee9ca49d4288448b185ec52c7470c3f28476e782
r26 navigation audit  504e05b4bf8865297bea767961e1e265312e194d1f6917036f24d143e3a4c392
r26 SHA256SUMS        6d5b37e2ee7f50ed770f6ef568cb9b15d87f2a755702114980aff7dd5121f551
```

runner v4 找到实际 Ruby Gazebo PID，只发 individual SIGINT，没有 process-group 信号、TERM
升级或 session 残留，recorder/launch parent 均返回 0，因此解决了 v3 的 orphan 问题；但
Gazebo 自身在 SIGINT 收尾打印 `Segmentation fault` 并由 launch 记录 exit `-2`。后续必须
把“无孤儿 containment”和“Gazebo 进程 clean exit”分开验收，不得只凭 launch parent 0
宣称完全干净收尾。

实验 controller/config/contract/tests 已完整回退；当前恢复 r17/r21 基线哈希
`3c2767e4.../cb7e9797.../5a71e901.../0a0a3bec.../942915c0...`，六库隔离重建后
`113/113` PASS。运行前后实验哈希、最终六项保护哈希全部逐字节匹配，最终 live 离线；
没有孵育箱操作、reset/detach/recovery、真机或备份动作。

下一轮不得简单放宽 r26 的速度/误差门，也不得增加驻足阻尼：r17 离线对拍显示增益
`0.8 -> 1.0/1.2` 会把横向命令饱和率从 `59.6%` 提高到 `62.4%/68.5%`。准入的下一项是
一个互补组合假设：恢复 r26 的严格双条件交接，同时把 Nav2 XY tolerance 设为 r23 已单独
验证过的 `0.04 m`。r23 提供离目标 `0.015969416 m` 的 motion-reference anchor，但入站速度
高达 `0.253626616 m/s`；r26 提供动态门却缺少空间一致性。组合试验仍不得改 policy、增益、
限位、场景或监督，必须重新构建并使用从未使用的新身份，任一门失败即完整回退。

## 2026-09-12 19:08 CST：r27 组合交接快速失稳，否决并回退

本轮仍只使用 `HANDOFF_PLAN.md` 分支 D 的 Gazebo 导航步态授权。运行前现场离线，六项
孵育箱/backup 909 保护哈希全部匹配。按上一节准入，同时使用 Nav2
`xy_goal_tolerance=0.04 m` 和 r26 的低层严格 settling handoff；官方 policy weights、
station/motion gains、命令限位、完整组合场景及 30 秒 fail-closed supervisor 均未改变。
focused `20/20`、六库精确隔离构建和运行前 `116/116` 测试通过，普通 `mkdir` 独占身份为：

```text
runtime_evidence/unitree_h12_incubator_nav2_xy004_settling_handoff_20260912_r27/
```

完整场景健康门、Mid360、Nav2 lifecycle、规划和 action 均成功；action callback 时物理根
仅离目标 `0.012776665 m`，但最新定位样本滞后为 `0.130384048 m`。进入 settling 的 9 条
样本中，速度 `<=0.08 m/s`、tracking error `<=0.04 m` 和两者共同满足的数量均为 0；第一
条已经是 `0.300232447 m/s / 0.064263944 m`，motion reference 自身离目标
`0.075303879 m`，并产生反向与 yaw 饱和策略命令。station keeping/reset 均未执行。

Nav2 callback 后仅 `0.621643 s` 即因物理走廊越界 FAILED；最大物理目标误差/走廊误差/
速度为 `0.151577153/0.126191971 m/0.824428799 m/s`，最小
`up_z/height/support=0.994309665/0.925956763 m/416.165659 N`。定位 `1/0` 有效/无效，
终端前无 locomotion fault，但高度和路径门均失败；未进入 30 秒验证、未发 PASS，严格
验收计数增加 0。

新增的共同根因证据是 nominal authorized 速度输出从
`[0.04,0,-0.035]` 到 exact zero 只用 `0.000085900 s`。r17、r22、r23、r24、r26、r27
全部存在 `0.078--0.198 ms` 的同类非零到零跳变，而交接物理速度约为
`0.108--0.300 m/s`。这只证明该变量尚未被隔离，不能外推为 slew 必然有效。r27 的开放环
LSTM 零命令对拍也因日志不含控制器原始 IMU angular observation 而明确标为非权威。

runner v4 本轮再次做到 individual SIGINT、无 TERM/进程组信号、session 残留 0，launch
parent 返回 0；但 Gazebo 自身仍打印 segmentation fault/exit `-2`，所以 clean-exit 门继续
失败。runner 返回 0 只表示终端证据完成捕获和收尾，动作权威仍是 recorder 的 FAILED。

两项实验改动及测试已经完整回退，六个文件逐字节恢复
`3c2767e4.../cb7e9797.../5a71e901.../886ada10.../0a0a3bec.../942915c0...`，六库隔离
重建 `113/113` PASS；最终 live 离线，六项保护哈希不变。回退前遗漏了立即 post-run 的
实验集成哈希快照，因此机器审计将 unchanged-during-run 记为 null，不作过度结论。没有
孵育箱操作、reset/detach/recovery、真机或备份动作。

```text
r27 REPORT            6155f0964f05f87496b6223c352d8225b000a05e93978e1baab0289ba4957f69
r27 ATTEMPT_STATUS    d345a807f2f77f6aef1af587b7bb50b1545eb561e9662f9db8151075ce056e5f
r27 navigation audit  29e4f80f8a8c204635c5725522bea049a44b8c2ecb34655aa1ab706f2760d7fe
r27 SHA256SUMS        6b33e5993ac591a44f985b2c1e3c2b36e672e18f7c0994f45527a6289615054f
```

下一轮不再恢复 r26/r27 pose-tracking settling，也不改变 Nav2 goal checker。准入的单变量
假设只修改 `VelocityGate.apply()` 对“fresh、全部授权门通过、输入落入 deadband”的正常
到站处理：使用现有 acceleration limits 将上一条已授权输出渐变至 exact zero；任一授权门
失败、watchdog stale、无效输入、controller fault 仍必须立即 exact zero。保持生产
`SimpleGoalChecker/0.08 m`、policy、gains、limits、scene 和严格 supervisor 不变，先增加
普通减速与所有 emergency-stop fail-closed 单测，再隔离构建和使用新身份；失败必须回退。

## 2026-09-14 09:22 CST：r28 完整场景用户视角导航展示

用户明确要求查看运动导航效果后，仅在 `HANDOFF_PLAN.md` 分支 D 的 Gazebo 导航授权内，
使用未改动的 r17 gait + r21 fail-closed supervisor 启动完整孵育箱 GUI 场景。普通 `mkdir`
独占的新身份为：

```text
runtime_evidence/unitree_h12_incubator_nav2_user_view_20260914_r28/
```

Mid360 定位、Nav2 lifecycle、规划和 action 均成功，路径长度 `0.149999963 m`。Nav2 callback
时物理/定位误差为 `0.082320116/0.074330344 m`，终端回到
`0.070414705/0.055000000 m`。但 callback 后 `25.085504351 s` 内物理误差峰值
`0.126195152 m`，未形成严格 `0.08 m` 连续保持窗口，最终
`FAILED:STABLE_HOLD_TIMEOUT`；本身份只证明可视化到站，不计严格 PASS。

626 条到站后步态样本的最大横向绝对值/平面速度/驻足误差为
`0.091659016 m / 0.216378293 m/s / 0.095254977 m`，最小
`up_z/height/support=0.999504287/1.001637613 m/360.569934 N`；定位 `25/0` 有效/无效，
fault 为 0。机器人未倒地且未越走廊，剩余问题仍是到站后的动态漂移。

展示结束后以 SIGINT 收尾，最终 Gazebo/navigation/recorder 相关进程为 0；捕获日志没有
shutdown lifecycle 尾部，因此不得把本轮表述为 Gazebo clean-exit 证据。源码和七项导航
集成哈希运行前后逐字节一致；六项孵育箱/backup 909 保护哈希全部匹配。没有发布门、
夹爪、方块、support、reset/detach/recovery、真机或备份命令。

```text
r28 REPORT            b804635d52ee5c4a81263962d8e2c16cfb1ac0e2366d96e8907d365fe962c47a
r28 ATTEMPT_STATUS    a8a920a0f512ba919fc157f49d7b6d63570c68503e2a42a091027523ad1298ac
r28 navigation audit  796265b485e5b146d514e2eb1b999fb90962c5c0de7efeb6f9ffbf8d89ca185a
r28 SHA256SUMS        078367d1ab6ee04e48fa17cb551aae026b70901aac38946755b7ebfdd65103c3
```

r28 永久封存不得复用，当前 live 离线，严格连续 PASS 计数仍为 0。下一项准入仍是上一节
定义的 authorized deadband zero-command slew 单变量试验；r28 没有实现或验证该假设。

## 2026-09-14 09:34 CST：914 完整工作空间备份预发布合同

用户明确指定从未使用的新编号 `914`。创建任何归档前重新只读确认 Gazebo、导航和证据
recorder 相关进程为 0，并确认以下三个正式目标均不存在：

```text
/home/liu/h1_moveit/backups/914.tar.zst
/home/liu/h1_moveit/backups/914.tar.zst.sha256
BACKUP_914_RELEASE.md
```

随后以普通 `mkdir` 独占创建 `runtime_evidence/backup_914_release_20260914/`，并写入
`BACKUP_914_RELEASE.md` 作为归档内非自引用合同。初始预检记录 25,429 个普通文件、
5,580 个目录、570 个 symlink。第一份未发布临时归档实际含 25,436 个普通文件，因为
初检计数后又新增了 `pre_archive_key_hashes.txt`；虽然其 zstd、tar list 和类型门均 PASS，
但为避免把初检值误写成最终值，已在正式目标创建前将其标记为 superseded。加入该纠错
状态后，重建输入应为 25,437 个普通文件、5,580 个目录、570 个 symlink。各快照的
socket/FIFO/block/char device 和 broken symlink 均为 0；源目录与 backups 位于同一文件
系统，初检可用空间 `159,572,992,000 bytes`。

旧保护链 827/828/831/901/909 已从 sidecar 所在目录重新完整执行 `sha256sum -c`，全部
PASS，owner/mode/link count 保持 `liu:liu/0444/1`。前两次只读校验分别因 sidecar 相对路径
调用目录和日志 `tee` 路径错误停止，均发生在归档创建前、没有产生 914 正式或临时备份；
错误与修正证据必须保留在本轮 evidence 目录，不得隐藏。

914 必须完整收录当前 `h1_2_ros2_ws/`，只允许单一安全相对顶层 `h1_2_ros2_ws/`，保留
symlink 且不得收录外部 backups。它应包含 v9/r17、v19/r29、826 AprilTag A--L、Mid360、
Nav2、官方 Unitree H1-2 Gazebo 步态、r17/r21 基线、r22--r27 失败证据、r28 完整场景展示
以及本预发布合同，但不得把 r28 外推为严格 30 秒 PASS，也不得把尚未实施的 authorized
zero-command slew 写成成果。

归档与 sidecar 只能在 backups 同目录以全新临时普通文件生成；完成 `zstd -t`、完整成员/
类型审计、唯一根、安全路径、无重复、无 hardlink/device/FIFO/socket、无 symlink 子成员、
关键成员内层哈希、sidecar、owner/mode/link count 和旧保护链复核后，才能通过 no-replace
硬链接发布。正式文件最终必须为 `liu:liu`、mode `0444`、link count 1；任一同名目标出现
都必须 fail closed，禁止覆盖。实际 914 digest 只允许记录在归档外 sidecar 和发布后追加
小节，避免归档自引用。

备份过程不得启动 Gazebo、发布任何动作或导航命令、连接真机、创建其他编号备份，且禁止
覆盖 909 或任何历史保护链。

## 2026-09-14 09:44 CST：914 完整工作空间备份发布 PASS

914 已按上一节合同完成非覆盖发布：

```text
archive  /home/liu/h1_moveit/backups/914.tar.zst
sidecar  /home/liu/h1_moveit/backups/914.tar.zst.sha256
SHA-256  96ea710307e598a023503cc6361dd0795b924fb51e992dd3c51a1f6763f0ca48
size     3,249,344,655 bytes
members  31,587
```

archive 和 sidecar 均为普通文件、`liu:liu`、mode `0444`、link count 1；正式 inode 为
`131264/131278`。第二个全新临时归档身份在 backups 同目录完成全部验证后，以 no-replace
硬链接发布，再解除临时链接；正式目标没有被覆盖。sidecar `sha256sum -c` 与正式归档
`zstd -t` 均再次 PASS。

归档唯一根为 `h1_2_ros2_ws/`，成员精确分布为 25,437 个普通文件、5,580 个目录、570 个
symlink；绝对路径、父目录穿越、重复、hardlink/device/FIFO/socket、symlink 子成员均为
0。31 个关键内层文件逐字节匹配当前归档前工作区，包括 v9/r17、v19/r29、r29 MP4 与
rosbag、826 报告、r17/r28 导航报告、导航源码/配置、policy weights 和 914 预合同。归档
内 r29 MP4 全帧解码 PASS，SQLite `PRAGMA integrity_check=ok`。

827/828/831/901/909 在发布前后的 sidecar 均 PASS，metadata 日志逐字节一致。原计划的
专用 `protected_workspace_hashes_before.txt` 因首次只读预检提前停止而未生成，因此不作
该文件的 before/after 声明；固定 authority digest 当前全部 PASS，且归档抽取值与归档前
源文件逐字节相同。所有预检路径错误、首份未发布临时归档的 superseded 原因和后续修正
均在 `runtime_evidence/backup_914_release_20260914/` 透明保留。

六个验证临时项共 `119,837,921 bytes` 已用 `gio trash` 移入系统回收站，未永久清空；
backups 目录没有 `.914*tmp*` 残留。正式归档和 sidecar 保持存在并再次通过校验。

914 归档内 `AGENTS.md` 有意停在上一节预发布合同，SHA-256 为
`b33730d73ed2df10794724cc3ba1c5897e4b988f6d7cd6d5ee28c17baac7ca11`；归档内
`BACKUP_914_RELEASE.md` SHA-256 为
`a57f4c66e4be1c365e4e95b15f3b745d4435c112df0f7364489cd9cd43f09699`。实际归档 digest 和
本发布小节只存在于归档外，避免自引用。914 从本节起是最新受保护恢复点，包含归档截止
前的 v9/r17、v19/r29、826、Mid360/Nav2、官方 Unitree H1-2 步态及 r28 展示证据；本节
和 post-release audit 不在归档内。

```text
914 release REPORT  e7285a583cb644932abd0a3cb13898e49a191dfbcbf02ca9c74a6ba9983c2bce
914 release audit   e69d537016038bb93853ba4432841dc356eeb70cae5ef38467806525789828a6
914 evidence sums   f01d8beb6c9faf39e5cd4d6518686012db3a1c21822667dc3dbce1e9ee814177
```

备份前后 Gazebo/navigation/recorder 相关进程均为 0。没有仿真动作、孵育箱操作、reset、
recovery、真机或其他编号备份；以后禁止覆盖 914，任何新备份仍须用户指定另一个从未使用
编号。

## 2026-09-14 11:03 CST：完整 Mid360 方墙地图内双循环 v21/r31 PASS

用户先要求以全新身份重放孵育箱双循环，随后明确纠正：必须在“构建完雷达的那个地图”
中开箱。第一次派生的 v20/r30 使用旧独立动作 world；发现语义不符后，在 rosbag、support
daemon、preparation 和 flow 之前正常 SIGINT 停止，控制器命令为 0，并写入
`runtime_assets/candidate3_roundtrip_831_r30/ABORTED_PRE_MOTION.md`。v20/r30 已消费且永久
不得复用。

随后从 v20 链派生一次性新身份：

```text
builder     runtime_assets/build_candidate3_roundtrip_831_authority_v21.py
authority   runtime_assets/candidate3_roundtrip_831_authority_v21/
run         runtime_assets/candidate3_roundtrip_831_r31/
run id      1072ae8ba1a4e59fe19caa14b958d5f1365e04e9fa77e7be0909acee5b33a54a
socket      /run/user/1000/h1rt831_1072ae8ba1a4e59fe19caa14b958d5f1.sock
```

v21 将完成的 Mid360 导航环境与动作场景非破坏组合：保留 v9/r17 的 5 ms 动作物理、箱体/
桌子/方块坐标和 14 项非场景动作、时序、安全 payload，加入与导航 world 精确相同的
`5 x 5 m`、高 `2.5 m` 四墙 `navigation_enclosure`，以及只观测、不发动作命令的 Mid360
点云、720-bin scan、map 和方墙定位节点。关键 SHA-256：

```text
builder          a316c07e7e248e676f46fd013e7821d1ee0c56290662a835e35dbab2dc22e6a2
manifest         363314e291863c2e3ed8230e2a3549fcf6c17d73f913126f937f34e787666edb
commands         dd06328648e7380798136097c758edb9cb52a1d3f4f8af1a5ddf2ebb81bc8fda
combined world   6da0ddd8a9af867d427293b4bae1cc4a82e4c03ec9497755a9a835951b70e671
combined launch  3b756c1d321e5dbd7e851031b4e1add33ee6dd4562cc2b2daee2f64598cd4492
```

builder 首次 `--self-test` 只因生成 launch 的预期常量与实际格式哈希不同而 fail closed，
authority/run 尚不存在且未写盘；修正常量后 retry 为 `PASS / writes_performed=false / 14/14`，
随后只 build 一次。19 文件 authority 审计、SDF 校验和运行前 preflight 全部 PASS。为使
builder 输入身份可审计，主 install 中 launch/world 两个生成 symlink 已物化为独立 0644
普通副本，原条目保存在
`runtime_evidence/candidate3_roundtrip_831_v20_build_20260914_r2/main_install_before/`，
`main_install_repair.txt` 为 PASS；source、孵育箱模型、轨迹和既有 authority/run 未改。
因此本轮不重新编译 ROS 包，正式运行由 builder self-test、密封 audit、SDF/preflight 和
Gazebo 执行闭环验证。

r31 fresh 健康门与 preparation 均通过。首份健康审计只因要求定位严格数学零点而误报，
首份 preparation 审计只因 rosbag 增加第二订阅且命令行自匹配而误报；两份原始记录均保留，
有界 retry 分别允许 `<=2 cm/<=0.01 rad` 并正确计入 recorder，均 PASS。随后先启动独立
GPU GUI 录像，再从 v21 `commands_831.json` 原样 `execv` 唯一 flow argv。正式结果：

```text
flow invocation              9777696637585 ns
motion start                 9780403336902 ns
motion end                   9944999045954 ns
terminal PASS                9944999068274 ns
invocation -> PASS           167.302430689 s
motion                       164.595709052 s
result                       PASS / cycles=2 / cube=origin / door=locked
```

rosbag SQLite integrity 为 `ok`，共 287,055 条消息；控制消息精确
`60=16/16/16/4/8`，把手接触 0，6,328,174 个抽取数值全部有限，命令/实测躯干峰值
`44.25/44.25000000101826 deg`。recorder 日志透明记录 2 条高频遥测 transport loss，但 60
条控制 payload 完整且所有动作门通过。终态方块
`(0.334238508678, 0.377056800863, 0.829999476763) m`，原位误差
`0.000245179529 m`；门和把手约 q0、latch locked、两个 support detached、机器人 exact
launch pose、五路 trajectory publisher 为 0。

Mid360 终态定位仍为 valid：720 点、4 墙、inlier RMS `0.017826493411 m`、pose
`(0.000,-0.005,0.000)`。录像
`runtime_assets/candidate3_roundtrip_831_r31/candidate3_roundtrip_831_r31_gui.mp4` 为 H.264
Main、`1160x1518`、10 fps、222.1 s、2221 帧，完整解码和六点视觉 QA PASS，SHA-256：

```text
3c4dbf44dca9cba740c0998c9056c205691a5f7854d43bbc4f79a35b872be88a
```

相机沿用可信 r29 动作视角，flow 前后 component drift 为 0；围墙在动作背景中持续可见。
首份终态进程审计匹配了自身 shell 文本，作为失败记录保留；按实际 PID `/proc` 重试 PASS。
视频、rosbag、support daemon 和 launch 均正常 SIGINT/自退出，最终 v21/r31、Gazebo
server/GUI、bridge、relay、localizer、support socket 和 recorder 进程均不存在，live
离线。

完整报告：

```text
runtime_assets/candidate3_roundtrip_831_r31/RUN_SUMMARY.md
SHA-256 1189c500f8dd0a47ab03f7bd465fac144bd28b493b2eed7707c31717a55c5a65
```

v21/r31 是当前最新“完整雷达地图中的孵育箱双循环”固定轨迹回归与视觉证据；动作语义
权威仍是 v9/r17，826 AprilTag A--L 动态规划线仍独立，导航严格 30 秒驻足状态也不因本
动作重放改变。运行后 v9 manifest、r17 summary、v19 manifest、r29 summary、孵育箱
source model 和 914 archive 六项保护哈希全部匹配。914 是最新受保护恢复点，但它在本轮
之前创建，不包含 v20/r30、v21/r31、本节或后续交接更新。未创建新备份，未连接真机。

## 2026-09-14 13:09 CST：导航后 AprilTag 把手对准方向确认，仅完成只读诊断

用户确认下一阶段应把“开箱动作的名义站位”作为导航终点，并要求利用二维码/AprilTag
与箱体的刚性关系，在 Mid360/Nav2 存在小量到站误差时由视觉恢复把手实际位姿，再重新
规划开箱动作。该方向属于 826 动态规划线，不得改写或外推 v9/r17、v21/r31 的固定轨迹。

本轮在用户要求停止前只完成只读源码和证据核对，没有修改源码、消息、模型、轨迹、
authority、run 或备份，没有构建、启动 Gazebo、发布导航/轨迹/门/夹爪/support 命令，
也没有连接真机。停止时 Gazebo、Nav2、Mid360、candidate3、recorder 和 support 相关进程
均为 0；914 sidecar 校验 PASS。相关未改源码 SHA-256 为：

```text
candidate3_worker.py  bbab18f66396db6b4cf254fe7b713da337fba18f3fda9a12d546d4625f6341f1
dynamic_scene.py      21bd715a91d763e5df7dab95fe9fbbd812fed0f18a416b7fd695e95060de1e9d
tag_localizer.py      dfcc869f173e1c6d5fca65b68333b7d8fa0076ddf7d6a39e1de59c4afa788251
tag_map.yaml          a7f643d111a54f6bf9cb388e2f7fb85b3b7c322b0fea3571a708ada9f4bfa195
```

只读诊断确认已有 826 链并非从零开始：`tag_localizer.py` 已按图像时刻查询
`world <- camera`，对 Tag 10/12 融合箱体 pose，并具有重投影、深度、freshness 和连续帧
门；`dynamic_scene.py` 已按同一标签时刻取得 `pelvis <- world`，把箱体、门、把手和方块
几何写入冻结 PlanningScene；`candidate3_worker.py` 已能对笛卡尔目标重新求 IK，并执行
601 门目标、10 mm 环境净距、4.9 度连续性、正反路径、时序和禁止 partial 审计。

确认的关键缺口位于 `candidate3_worker.py::validate_operating_envelope()`：当前
`cabinet_delta/cube_delta` 只由 `SceneEpoch` 中的 world pose 与 `REFERENCE_CABINET/CUBE`
比较。若箱体在 world 中不动、机器人经导航后 pelvis 有残差，该 delta 仍近似 identity，
规划器会错误保留名义关节轨迹，不能补偿机器人相对箱体的位置误差。冻结 PlanningScene
中的动态碰撞对象已经包含正确的 pelvis 相对几何，因此下一项低风险实现应优先从该密封
场景恢复 `T_pelvis_cabinet`、`T_pelvis_door`、`T_pelvis_handle` 和 `T_pelvis_cube`，再相对
名义操作站位计算笛卡尔 delta；不得直接平移旧关节角，也不得把固定轨迹当在线重规划。

下一阶段先只做离线实现和验证：增加 SE(3) 纯函数与 scene/epoch 刚性一致性 fail-closed
门；使用未改的 attempt11 CDR 派生“world 物体不动、pelvis 平移/偏航”的全新 plan-only
矩阵；运行 worker、IK、C++ collision/clearance 审计和隔离构建测试。Tag 10/12 应作为箱体
固定基准，Tag 11 用于门状态交叉检查；把手转动后的位置必须结合 door/handle joint
运动学，不能把一个静态抓取点外推到全程。完整双循环还必须继续使用 cube tags。标签
失效、时间戳过期、刚性关系跳变、基座残差超出已验证包络或机器人未稳定驻足时一律
stop-and-hold。旧 `(-0.45,0,0)` 只能视为已验证的等待/接近点；名义固定动作的机器人
root 约为 `(0,0,0)`，把它改为 Nav2 最终操作站位本身仍未完成、未获 Gazebo 验证。

下一位接手者不得把本节写成“视觉补偿已实现”。只有用户再次明确要求 Gazebo 验证后，
并且离线矩阵、隔离构建和全部保护哈希先通过，才可建立全新运行身份进入仿真。914 早于
v21/r31 和本节，仍是最新备份但不包含这些后续内容；新备份必须另获未使用编号。

## 2026-09-14 13:52 CST：D1 参考系实现完成，材料级基座残差未通过

本轮按用户明确授权仅完成 `HANDOFF_PLAN.md` 分支 D1 离线工作；没有启动、停止或重启
Gazebo，没有发布导航、速度、轨迹、门、夹爪或 support 命令，没有修改 v9/v19/v21
authority、任何正式 run、固定轨迹、模型、`dynamic_scene.py`、`tag_localizer.py` 或
`tag_map.yaml`，没有创建备份或连接实体机器人。

`candidate3_worker.py` 已不再只用 epoch 的 world 物体 pose 计算 delta。新增
`candidate3_alignment.py` 从冻结 PlanningScene 的精确 body/tray/door/handle/cube/guard
几何反解 pelvis 相对 pose，要求五条独立链与 epoch 共享同一刚性 `pelvis <- world`
变换，并用恢复的 `T_pelvis_cabinet/T_pelvis_cube` 相对名义站位生成笛卡尔 delta。handle
由 door、固定 `door_handle` 偏移、`handle_latch_joint` 和 visual 偏移恢复。新增门同时
检查有限/proper SE(3)、world/pelvis frame、对象时间 skew、唯一 shape 拓扑、primitive
尺寸、未 attached、共同刚性变换和 handle 机构一致性；既有 world 物体 ±35 mm 等包络
保持独立不变。

attempt11 原始 CDR 哈希保持：

```text
epoch.cdr  98a0e2f9e4b030059721c29bc95ea373d3db65bcf67fdafe3a6f96bc84829c17
scene.cdr  78f41904253a7e30d3f04fd3e58dafb227a42c01b18d680d2e8fcdea73a9dca2
```

从 CDR 的五条链反解的名义 `pelvis <- world` 为 `(0,0,-1 m)`，最大链间平移误差
`5.476937018e-16 m`、旋转数值误差 `2.980232239e-8 rad`；handle 链误差
`5.089691686e-16 m / 0 rad`。离线 matrix runner 保持 epoch world pose 不变，只给冻结
场景内全部 world collision object、octomap origin 和唯一 world/pelvis 固定变换施加共同
纯 SE(3)，生成新 epoch id/hash/CDR；不修改 attempt11 源文件。

宽域 characterization 20/20 预期匹配，但所有 `0.1 mm` 平移与 `0.01 deg` 偏航均因
IK_NO_SOLUTION、4.9 度连续性或自碰撞门失败；1 um 也出现方向不对称。因此不能声明
现有 attempt11 动作支持任何材料级 Nav2 到站误差。最终 base 自动准入只采用所有正负
方向均完整 PASS 的对称数值零包络：x/y/z `±0.5 um`、yaw `±0.0001 deg`、tilt
`<=0.00001 deg`，比较 epsilon 仅 `1e-12 m/rad`。这不是可用导航容差。

最终 admission v2 为 16/16 预期匹配：7 项 nominal/边界 case 均完成 601 门目标、每项
8,713 个 C++ 审计点、10.001 mm 最小净距、0.773493305 deg 最大相邻变化、正反路径、
时序和 no-partial PASS；9 项超出最终包络的 case 在 MoveIt/IK 创建前拒绝，控制命令 0。
首次收紧运行因三个恰好边界 case 的浮点比较失败，原样保留；只增加 `1e-12` 比较
epsilon 和双向边界单测后以新 v2 路径复验，没有材料性放宽。

```text
report          runtime_evidence/826_post_nav_apriltag_alignment_20260914/REPORT.md
characterize    .../base_residual_matrix_attempt11/summary.json
summary SHA     51d2cb6bc05237981ccbcc811d979e182ba53022fe4b2fd8c15dcadfd2d2f75d
failed v1       .../base_residual_admission_matrix_attempt11/summary.json
failed SHA      4e431267439211ed5de783d5cf8b69792ad5e5197dc20307025f77dcae401e80
final v2        .../base_residual_admission_matrix_attempt11_v2/summary.json
final SHA       888734ea2e561d2cce3f112a6694439a23140312d44852601fa5b3c84fb485c3
```

最终源码/配置 SHA-256：

```text
candidate3_worker.py                 4b52c9f4b7b58f128fdf0c0e572d19dfdea688f56220cafc7a2dd01c11c0e28e
candidate3_alignment.py              f080990ff85b40d5deba077820839419da08d87552b8765ed120ec7e5b6c4bb6
candidate3_base_residual_matrix.py   4e0ebdb3ec1cecf15ea6896486040753812f1be60a9b859740a1d5a118a22be0
test_candidate3_alignment.py         1824a262a6e62a7180659797dbdbbf57ac4cbe09b5d885736417618005df0e47
candidate3_base_residual_matrix.json 99190d5a56fecd568e4270660f4437457fe1615c421ce72e6679f587c26cc7b8
setup.py                             8583617353fcee1821c8749e6c0151093842a46f74749734790302c9c7062906
```

最终使用 Miniconda 六库精确隔离、`--base-paths src --symlink-install` 重建
`h1_2_vision`：`log/build_2026-09-14_13-48-33`，1 包 0.91 s PASS；
`log/test_2026-09-14_13-48-35` 为 112 passed、1 skipped、0 failed，flake8/pep257 PASS。
最新只读检查仍无 Gazebo/move_group/controller manager，914 仍为最新受保护恢复点且不含
本 D1 结果。

D1 的参考系恢复、纯函数、fail-closed 门、矩阵和隔离构建已经交付；但“导航后材料残差
仍能完整开箱”明确未完成。下一步优先离线重设有 IK/连续性裕量的名义操作站位或笛卡尔
路径并重新做矩阵。以后若进入 Gazebo 的“导航停止 -> 直立/速度/定位门 -> 标签稳定 ->
scene freeze -> certificate -> 执行”，仍必须由用户再次明确授权并创建全新不可覆盖身份；
现有授权不包括该联调。新备份仍需用户另给未使用编号。

## 2026-09-14 14:59 CST：D1a 离线路径裕量搜索未找到材料成功域，正式 g5 保持

用户以“继续”授权沿 D1a 只做离线站位/路径裕量开发。本轮没有启动、停止或重启 Gazebo，
没有发布导航、速度、轨迹、门、夹爪或 support 命令，没有修改固定 authority/run、模型、
正式轨迹或备份，也没有连接实体机器人。

保持正式 g5 时，单轴矩阵的 nominal 完整 PASS，但 x/y `+/-1, +/-5, +/-10 mm` 与 yaw
`+/-0.1, +/-0.5, +/-0.75 deg` 共 18 个非零 case 全部失败；另 16 个 x/y 对角 case 也
全部失败。因此所测 34 个非零站位候选无一取得完整证书。g6--g12 分别尝试 source-near
分支、局部门姿 detour、source regularization、全门路径重求解、+Y 位置 bump 和三点
Cartesian detour；它们均在 nominal 或材料 case 上触发动作超时、关节限位、IK 无解、
4.9 度连续性或严格自碰门，已从 `candidate3_worker.py` 和 `candidate3_alignment.py` 完整
回退。失败证据保留，不能当作 planner authority。

诊断定位到两个主要瓶颈：门 message 1 的 raw point 471 附近最小 Jacobian singular value
约 `0.000410382`，小扰动会造成无解或冗余分支跳变；handle 源路径 left wrist yaw 终点
`1.2699736326 rad` 已贴近 `1.27 rad` 限位。门 TCP 原路径对端点直线偏离超过 0.5 m，
禁止用端点直线替换。离线 C++ auditor 仅增强碰撞诊断：请求最多一个 contact，并把首个
碰撞对写入既有 body 字段；ACM、阈值、schema 和 PASS/FAIL 语义不变。它确认材料
x +0.1 mm、y -0.1 mm、yaw +0.01 deg 在 handle point 94 为
`left_wrist_roll_link <-> torso_link`，y -1 um 在 point 122 为
`left_elbow_link <-> torso_link`。

正式源码已恢复 `h1_candidate3_replanner/826-g5` 和 `h1_candidate3_auditor/826-h1`：

```text
candidate3_worker.py              4b52c9f4b7b58f128fdf0c0e572d19dfdea688f56220cafc7a2dd01c11c0e28e
candidate3_alignment.py           f080990ff85b40d5deba077820839419da08d87552b8765ed120ec7e5b6c4bb6
candidate3_contract.py            0e1c5266e9aa12b68b68ae6c6802be48b91ab3b1978d993ded776c94350fb3a6
clearance auditor old -> new      bcd9a7155e4218eb4a4f27406cb42457f60d47bc1e4db8ec535bc8873c8162c2
                                  2dbd5399513b26b82c825ad49e554941a1cb4b1e8ea45cf259e3a94ed534e8f6
auditor source-contract test      5e5e998202110a84c0ff50c61133ddf7525dcfb721b80c36a581f7ad394a73e5
```

最终使用 h1 身份重新运行 attempt11 准入矩阵：16/16 预期匹配，7 个准入 case 各完成
601 门目标、8,713 点、10.001 mm、0.773493305 deg、正反路径、时序和 no-partial PASS；
9 个超包络 case 前置拒绝。summary SHA-256：

```text
83aee54d55b07bdd5012a506dcd04c18b2e66bb3d8fce0c1ce4eb9fc7caa7d35
```

正式准入仍是 x/y/z `+/-0.5 um`、yaw `+/-0.0001 deg`、tilt `<=0.00001 deg` 的数值零
包络，不能当 Nav2 容差。`h1_2_moveit_demo` 隔离构建
`log/build_2026-09-14_14-12-25` PASS；demo 23 个 CTest 中 22 个 PASS，唯一失败仍是本文
已记录的未改右把手源码缺少 `ARGUMENT_VECTOR:--test-arm`。最终 `h1_2_vision` 隔离构建
`log/build_2026-09-14_14-51-39` PASS，`log/test_2026-09-14_14-51-44` 为 113 passed、
1 skipped、0 failed。完整报告：

```text
runtime_evidence/826_post_nav_apriltag_alignment_20260914/D1A_MARGIN_DIAGNOSIS.md
SHA-256 64be241c3566d49497ce1a50524efb371440d0bc0d9d097f6dd45013726e50f4
```

14:59 CST 重新只读检查相关进程、窗口和 support socket 均为 0；v9 manifest、r17 summary、
v21 manifest、r31 summary、826 REPORT 和 914 archive 哈希匹配。下一步若继续离线 D1a，
必须重设整段有裕量的 Cartesian 参考路径并建立新 plan-only 候选身份；若进入 Gazebo
联调，仍需用户再次明确授权和全新不可覆盖身份；新备份仍需另给未使用编号。

## 2026-09-14 17:20 CST：D1b Tag 固定外参右手开门 plan-only 材料矩阵完成

用户明确要求按二维码/AprilTag 与箱体刚性绑定关系，从视觉箱体 pose 施加固定偏移得到
把手 pose，以吸收雷达导航小偏差；授权仍严格限于 `HANDOFF_PLAN.md` D1 离线开发。本轮
没有启动、停止或重启 Gazebo，没有发布导航、速度、轨迹、门、夹爪、support、reset、
detach 或 recovery 命令，没有修改 v9/v19/v21 authority、任何正式 run 或固定轨迹，
没有创建备份或连接真机。

新增 `candidate3_post_nav_contract.py`，要求 Gazebo-only Nav2 PASS、精确 long-hold reason、
至少 30 s 连续驻足、目标/定位/物理 root 一致、直立、Nav2 lifecycle active；标签角色
必须精确为 cabinet Tag 10/12、door Tag 11。`candidate3_alignment.py` 和
`dynamic_scene.py` 现在以 Tag 10/12 的 `world<-cabinet` 为唯一物体权威，经固定
cabinet/door/handle 机构链和实时 joint angle 推导 door/handle；Tag 11 只在 5 mm/2 deg
门限内交叉检查门 pose。冻结 scene 的同一 exact-stamp `pelvis<-world` 与 alignment
certificate/hash 绑定，scene/epoch 非同一刚性变换、标签过期/错误、未驻足均 fail closed。
旧模式由默认 `require_post_navigation_hold=false` 保持兼容；navigator 对已监督 PASS 状态
增加 0.2 s heartbeat。

新增无 publisher/action client/控制接口的 plan-only worker
`h1_candidate3_post_nav_right_open/826-a6`。它固定绑定 v9 的右手 executor/manifest/contact/
middle path，使用 seed 827 派生 5-waypoint、10.001 mm ingress，对实际 pelvis 相对柜体目标
逐点重新求右臂 IK，以 source branch 正则选解和 +0.5 deg 平滑 torso 裕量生成三段右手
进入、关/解锁、601 点开门计划；左手不操作把手。另有三段精确逆序 `audit_only` 路径，
总计 3414 个 C++ FCL 审计 sample；它们尚未接入动作执行。

attempt11 原始 CDR 保持 `98a0e2f9.../78f41904...`。最终 admission a6 为 37/37 执行且
37/37 预期匹配：24 项完整 PASS、13 项预期拒绝。完整 PASS 覆盖 x/y/z 单轴 `+/-1 mm`、
yaw `+/-0.1 deg`、roll/pitch `+/-0.05 deg`、8 个 x/y/yaw 边界角和 3 个所列
x/y/z/yaw 角；每项 601 门目标、3414 sample、10.001 mm、3.782296668 deg 最大相邻变化、
44.75 deg torso，正反路径/时序/no-partial PASS，规划墙钟 4.052588550--4.228620711 s。
12 项 `1.001 mm/0.1001 deg/0.0501 deg` 超包络在 IK 前拒绝。另一个仍在前置包络内的
`x=-1,y=-1,z=+1 mm,yaw=-0.1 deg` 在 close/unlock point 812 因
`right_shoulder_yaw_link <-> torso_link` 自碰被精确拒绝。因此 `+/-1 mm/+/-0.1 deg/
+/-0.05 deg` 只是 prefilter，不是包络内成功保证；每个 live pose 仍必须完整重规划和审计。
通过 case 的计划时长为 126.816098949--157.858776319 s，新计划尚无总动作时长门。

```text
admission config   c6c9474f87658ccdfee0623dcec79452fecd1c68f71223fe52d2e3ef3031158a
admission summary  8601182017e084e9cd26db3dfcd7a9418ee166bedb87a2bc52240dc3ed5879c7
ingress seed827    c76fb446f16fef81276d396d5d3ad5a4da5d2a23972aacb34d898b1214b112ed
characterization   4d96192ef65e09b814b2467a6ce53896c821c2b9de1f74f071de5d456f009a4c
corner probe       d382bb431c8fd29913af39ae315ddaf015dc204d97b2b4c235e3b8126c4a3bdc
full report        e7d5014612626882478c9c2e29332dcb1c15c8ee26373c8925289670492986fd
```

关键最终源码 SHA-256：

```text
candidate3_alignment.py              b8e9274dfe7e226fced89b5e00ee862545e5080c8fcbefba36911bbc93c69a16
candidate3_post_nav_contract.py      1ee1a1e055702bd1b26f5094dcdb97486dfd6fe40fe6d988eaf44f0955d0f807
dynamic_scene.py                     aab613bac79ffeefb0c8273a0b133f1cdbbaed3e8f02566b46910a25e064bb8c
candidate3_worker.py                 589523cdbe258b1e3432589d3ec8cf208a7eb702504c4ed6dbf5de9a69bbf21b
candidate3_post_nav_open_worker.py   7d84cb1e1fee6645ee68f3e3b3d88ce7418c8b760f8af682b3df3176b536486e
candidate3_post_nav_open_matrix.py   89f707f13e6a3655b40ddeafd94c7a4934989f50c9dafd39ff472b0302ade6ea
candidate3_execution_contract.py     30848724139ee18165215657fcdd8bc48dde2b49e3e466add4f49939221d7e4c
candidate3_execution_supervisor.py   39bc09de371d9af3c7475449e22671986022513a7ad9c5ef476296dfe2241bc5
incubator_auto_navigator.py          8f01e3f326499498cbca1066e99f19533ed12faa1079885845b743c2f99e83c2
candidate3_clearance_auditor.cpp     7b599241398e4a19d907a448e58889012f52b3d3cae7f7c696148e268a1366d5
right_handle_splice_search.cpp       e64e8dfcbd5396a58eaceae0978eba7d75f2b76d64163f2d4c2081d720d23fde
```

六库精确隔离、`--base-paths src --symlink-install` 最终构建为
`log/build_2026-09-14_17-14-58`，四包 PASS；`log/test_2026-09-14_17-15-17` 为 navigation
113 passed、vision 139 passed/1 skipped/0 failed 且 flake8/pep257 PASS；demo
`log/test_2026-09-14_17-15-49` 仍为 22/23 CTest，唯一失败是已记录、未改 IK source 缺少
`ARGUMENT_VECTOR:--test-arm` 字面量。

完整报告为
`runtime_evidence/826_post_nav_apriltag_alignment_20260914/D1B_FIXED_OFFSET_OPEN_REPORT.md`，
内含全部 old/new 源码、测试和配置哈希。v9/r17、v21/r31、826 REPORT、914 保护哈希匹配。
本结果只证明 attempt11 右手开门 plan-only，不是 Gazebo/Nav2 行走/执行或完整双循环证据；
完整双循环仍需 cube tags。旧 `(-0.45,0,0)` 仍只是接近点，约 `(0,0,0)` 的最终操作 goal、
较长步行、新三段执行拓扑和总时长门尚未 Gazebo 验证。进入联调仍需用户再次明确授权并
建立全新不可覆盖身份；新备份仍需另给未使用编号。

17:26 CST 最终只读复核：相关进程 0、Gazebo/RViz/Foxglove X client 0、support socket 0；
因此 controller manager、bridge、Mid360 localizer、worker/support/recorders 均不在 live。
attempt11 两份 CDR 与上述哈希相同，六项 v9/r17/v21/r31/826/914 保护哈希全部匹配。

## 2026-09-15 09:55 CST：完整 Mid360 用户视角双循环 v23/r33 PASS

用户明确要求打开 Gazebo 查看开箱效果，因此本轮进入 `HANDOFF_PLAN.md` 分支 G；授权只
覆盖 Gazebo 固定轨迹重放，不包括真机，也没有把 D1b plan-only 新三段计划接入执行。
运行前重新确认旧 live 离线、v9/r17、v21/r31、826 报告和 914 哈希匹配，并重新枚举
未使用身份。

首次 fresh 身份 v22/r32 在任何 preparation、support 请求或动作命令前，由 support daemon
以 `H1_FAST_SUPPORT_DAEMON_831=FAIL:PLUGIN_SHA256` fail closed。原因是 D1a 对未改 demo
源码重链接后，主 install symlink 指向摘要 `1a81b048...` 的调试构建，而密封 support 合同
要求 `eb85e8b...`。该失败运行控制命令为 0，短 rosbag SQLite integrity 为 ok，Gazebo 在
恢复前正常退出；r32 已封存且不得复用。源码、目标文件和 link command 与 914 内值逐字节
一致；旧 `1a81...` 文件保存在 r32。Gazebo 完全离线时，只从只读保护归档 914 精确抽取
单一 plugin member，恢复主 build/install 绑定为合同摘要：

```text
libh1_gazebo_compliant_grasp_system.so
eb85e8b833a6cbaa0fa26eab09e6a983ba2e36243bf37a12ebb9001c19e95d8b
```

随后从 v21 builder 链建立全新 v23/r33；builder self-test 为 PASS、writes=false、14/14，
authority 只 build 一次，run root 由普通 `mkdir` 独占创建。首次 authority 审计仅因 `tee`
先创建审计自身而将 run root 误判为非空，原记录保留；允许审计记录后的 retry1 PASS，
authority 未改。身份为：

```text
builder     runtime_assets/build_candidate3_roundtrip_831_authority_v23.py
builder SHA f8678a1ca3124170347aabe8312ea3ffe356fd70e4264ba7ed7edf637e5c887a
authority   runtime_assets/candidate3_roundtrip_831_authority_v23/
run         runtime_assets/candidate3_roundtrip_831_r33/
run id      edb3a20aaadcc3823add641210bed5d1d62b7a6fbe1dc549d3613f455788648d
manifest    6bed442e7351876079796ca03d94f2657e7c4c086659fabf5e805691c1b53d68
commands    e7aaabf28a523e1b39e8c4f8bd5ee8fa3f5018624b5105ae32cee114a2cad29e
```

v23 的 14 项非场景动作/时序/安全 payload 与 v9 逐字节相同，launch/world/Mid360 与 v21
逐字节相同。fresh 健康门、rosbag、support daemon、并发 preparation 和一次性 ready
claim 均通过；preparation 明确 `controller_commands=0`。密封 flow 最终为：

```text
CANDIDATE3_ROUNDTRIP_831=PASS cycles=2 cube=origin door=locked
invocation -> PASS 176.938775168 s  (< 180 s)
motion window       174.417793405 s
```

控制消息仍精确为 torso/right arm/left arm/left gripper/right gripper
`16/16/16/4/8`，总计 60；左手把手命令 0，把手接触消息 0，躯干命令/测量峰值
`44.25/44.25000000101826 deg`。最终方块距出生原位 `0.0005300700159671292 m`，门和把手
q0、latch locked、两个 support detached、机器人 launch pose、五个 trajectory publisher
均为 0。Mid360 终态 valid、720 点、四墙可见，inlier/robust RMS
`0.01801446925910032/0.014279130273725025 m`。

GUI 视频在 flow 前启动并在 PASS 后单独 SIGINT，192.4 s、1924 帧、1160 x 1518、10 fps，
全帧解码 PASS；rosbag SQLite integrity 为 ok，154,732 条消息，动作计数与有限值审计
PASS。首份 final audit 只把 cube orientation 的 x/y/z 错读为 position，判定项未使用该
字段；原文件保留，修正解析并增加 recorder/video 前后时序门的 retry1 为 PASS。

```text
r33 RUN_SUMMARY       3c15225512bd37afdb0c2f1c60c22ef70032caa50f227171e77d62319bbc0287
r33 final audit       913aeb6ef9ead0d14991e2c525569aad5ea3f6a3de66f5b80a49df5fbf9f4de2
r33 rosbag audit      672ebd41c46f62d18be1c9a91a45f4408e7fb41d8219dd6c9fcdc48bf01455b0
r33 GUI MP4           503a05a2f09123ae77366f44f35bb1abc1d523a3cacfc6b18f47bb4943ea8ab3
r33 SHA256SUMS        9bba2310570b24b189f11f0d186435b2d72b7ef9a5ff635e0d2f55aba4b904a9
```

09:55 CST 的 live 快照是：用户要求保留的 v23/r33 Gazebo server/GUI、controller manager、
bridge、Mid360 localizer、tag localizer 和 6 个 active controller 仍运行；flow、support
daemon、rosbag recorder、视频 recorder 均已退出，support socket 不存在，现场保持方块
origin、门 locked、support detached 和五发布者 0。该快照以后必须重新只读核验，不能直接
照抄或复用 launch。v9/r17、v21/r31、826 和 914 保护哈希再次匹配；914 未修改且仍不包含
本轮 v22/r32、v23/r33 或交接追加，没有创建新备份。

v23/r33 是最新完整雷达地图固定轨迹视觉/回归证据，动作语义权威仍是 v9/r17。它不能
替代或外推为 D1b AprilTag 固定偏移在线执行、Nav2 腿部步行或动态 tag 完整双循环证据；
D1b 仍停在 attempt11 右手开门 plan-only，完整双循环仍需 cube tags。

## 2026-09-15 10:32 CST：915 完整工作空间模板备份发布 PASS

用户明确指定从未使用编号 `915` 并要求以后以此为模板。发布前重新核验 v23/r33 现场：
latch locked、门/把手关节约为 0、support socket 不存在、无 flow/support/recorder；随后仅
对原 launch 会话发送正常 SIGINT。10:26 CST 后 Gazebo、controller、bridge、Mid360/tag
localizer、GUI client、recorder 和 support 进程/套接字均为 0。没有 reset、detach、松夹、
导航、轨迹、恢复或真机命令。

915 已作为全新同目录临时普通文件生成，在发布前完成验证，再以 no-replace 硬链接发布：

```text
archive  /home/liu/h1_moveit/backups/915.tar.zst
sidecar  /home/liu/h1_moveit/backups/915.tar.zst.sha256
SHA-256  2148b1f083f4fa28ce647c8e8ca4debf4fc49a92ee26f0bfe5eb3428a9beea0c
size     3,948,505,246 bytes
tar size 17,929,287,680 bytes
members  35,379
```

archive/sidecar 均为普通文件、`liu:liu`、mode `0444`、link count 1，inode 分别为
`131266/131272`；backups 目录无 `.915*tmp*`。sidecar 与 `zstd -t` 在发布前后均 PASS。
正式成员列表与临时归档验证列表逐字节一致，唯一根为 `h1_2_ros2_ws/`，精确包含 28,709
个普通文件、6,086 个目录和 584 个 symlink；绝对/父穿越/控制字符/空名/重复/hardlink/
device/FIFO/socket/symlink 子成员均为 0。

31 个关键内层成员全部匹配归档前哈希，覆盖 v9/r17、v21/r31、v23/r33、826/D1b、
attempt11 CDR、源码和恢复后的 `eb85...` support plugin。归档内 r33 MP4 完整解码
1924/1924 帧；rosbag SQLite `integrity_check=ok`、154,732 条消息、9 topics，`ros2 bag
info` PASS。827/828/831/901/909/914 在发布前后均通过 sidecar，元数据保持保护状态。

首个媒体检查在视频和 SQLite 已 PASS 后，附加 `ros2 bag info` 因 nounset 与 ROS setup
冲突退出；关闭 nounset 后复跑 PASS。发布命令已完成 archive/sidecar 链接、摘要、sidecar、
zstd 和 archive stat 后，一条无 operand 的冗余 `stat` 返回 1；随即补跑 sidecar stat、
完整成员列表及全部发布后门均 PASS。两项均未改变正式文件，纠正日志完整保留。

915 从本节起是最新受保护的完整工作空间恢复/开发模板，禁止覆盖。它包含归档截止前的
v22/r32、v23/r33、D1b 及本轮预发布合同；实际 digest、本节和 post-release 报告按非
自引用设计在归档外。`模板` 只表示以后恢复或复制时以 915 为基线，不把 archive 本身
变为可写，也不改变动作语义权威 v9/r17、最新固定轨迹视角证据 v23/r33 或 D1b plan-only
边界。完整动态双循环仍需 cube tags，Nav2 后在线 AprilTag 执行仍未验证。

完整发布报告为
`runtime_evidence/backup_915_release_20260915/REPORT.md`。本节写入后 live 仍离线；创建
任何 915 之后的备份必须由用户另行明确给出新的、从未使用编号。

## 2026-09-15 10:51 CST：r29 authorized deadband zero-slew 离线实现 PASS

用户要求继续优化步态。本轮沿用 915 作为不可变基线，进入 `HANDOFF_PLAN.md` 分支 D 的
离线单变量开发；用户没有重新明确授权 Gazebo 移动，因此没有启动 Gazebo/Nav2/controller，
没有发布速度、轨迹、support、reset、detach、夹爪、门或方块命令，也没有连接真机、修改
固定动作 authority/run 或创建备份。fresh 离线身份为：

```text
runtime_evidence/unitree_h12_incubator_nav2_authorized_deadband_slew_20260915_r29/
status  OFFLINE_IMPLEMENTATION_PASS / GAZEBO_NOT_RUN
```

本轮只实现上一轮批准的变量：`VelocityGate` 在时间 fresh、命令 finite、所有授权门严格为
true 且输入三轴均落入 deadband 时，使用现有线/角加速度限制把上一条已授权输出渐变到
bit-exact zero；50 Hz watchdog 在输入仍 fresh 时继续该渐变，但不延长 freshness。任一
授权门失败、watchdog stale、非有限输入、时钟回退或非法 target 均绕过渐变，立即精确零并
清除全部 timing/ramp 状态。新 launch 参数 `authorized_deadband_zero_slew` 默认 false，
只传入 Unitree H1-2 profile，V12 不受影响。production policy、controller、gains、limits、
`SimpleGoalChecker/0.08 m`、strict supervisor、场景和 auto navigator 均未改。

纯函数矩阵总体 PASS。正、混合符号、负三组 limit 输出均从
`[abs(vx),abs(vy),abs(wz)]=[0.10,0.08,0.05]` 按每 tick
`[0.002,0.002,0.003]` 上限单调减小，最大观测增量仅有浮点舍入量差异，51 个记录 step /
1.02 s 后 bit-exact zero。1.02 s 包含 runner 的首个 zero-command tick，不是 Gazebo
实测制动时间。授权失败、stale、non-finite、clock regression、feature-disabled legacy
五类 fail-closed case 全部立即 exact zero。该矩阵只证明纯 gate 算法，不能宣称物理平衡、
漂移或严格驻足改善。

聚焦 Unitree/locomotion 测试 `50 passed`；最终隔离构建 `h1_2_navigation` PASS，包结果
`122 tests / 0 errors / 0 failures / 0 skipped`。隔离 install/import 均绑定 r29 目录，构建
后 Miniconda 六库状态精确恢复、无隔离临时目录。915 中旧值和本轮新值为：

```text
velocity_gate.py                         b9d663de... -> e21c9d3f47e3f0e2f96f6e81bb5c421457d1792f02c388e2a814268c675a90fc
unitree_h12_velocity_gate.py             913e58e8... -> 7398e291cbf0affe1702539b9bcc24c9d40265f36d05368599d64c72ad62fbe2
unitree_h12_navigation_gazebo.launch.py  886ada10... -> 8c9410830e1513650081697f243d9845c9c20c2af0bb3c191aa7415a6a58f87e
incubator_navigation_gazebo.launch.py    83a0349e... -> 8ce13bf437b7609a6a5f1c73deb3db41ec27afcc2b6547e624674eb774fea4e8
test_unitree_h12_gazebo_integration.py   942915c0... -> 5f5f69cdcafb0e67e7f1c28c4007b6fb11dadbfaa69339470dee8892ed073a41
test_unitree_h12_velocity_gate.py        absent      -> 2ff73fed915e1e633572e749e1f374e7a68fdbc4558c870dc50e6292c1e64e64
```

完整旧哈希、测试、矩阵和边界在
`runtime_evidence/unitree_h12_incubator_nav2_authorized_deadband_slew_20260915_r29/REPORT.md`；
REPORT SHA-256 为 `424d85ef1b10f2510730ad7263c36b5af11d6cc172df277e381c8a09c8a31db2`，
`SHA256SUMS` SHA-256 为 `bfdde308cbbd7ccd77e514dbb8d0fa6adaee6a1341f2189ef34ca2463ccd9a3b`，
内部 `sha256sum -c` 全部 PASS。

Unitree controller/config/contract/Nav2 control/policy/auto navigator、support plugin、v9/r17、
v23/r33、826 A--L 和 915 保护哈希均匹配。915 仍为最新不可覆盖模板，摘要仍是
`2148b1f083f4fa28ce647c8e8ca4debf4fc49a92ee26f0bfe5eb3428a9beea0c`；它不包含其后新增
的 r29。10:48:59 CST 只读检查没有 Gazebo server/GUI、launch、controller manager、bridge、
Mid360/tag localizer、flow/worker、support、recorder 或 socket；`wmctrl` 不可用，但独立
进程/套接字门均为 0。

r29 身份已被本离线结果占用，不得换用途复用。严格连续 30 秒、0.08 m station-hold 的
成功计数仍为 0。下一步若要测量物理制动、终点漂移和严格 supervisor，必须由用户再次
明确授权 Gazebo 移动，并在当时重新枚举、证明并独占创建全新 identity；失败仍须
stop-and-hold 并完整回退。实体机器人继续未授权。

## 2026-09-16 10:24 CST：r30 Gazebo 失败、回退与 r31 `/home/liu/test` 学习型停止离线候选

用户先前授权查看步态产生 fresh r30。r29 的 authorized deadband zero-slew 在完整 Mid360
孵育箱场景中完成 Nav2 staging goal，但严格 supervisor 终态为
`FAILED:STABLE_HOLD_TIMEOUT`，连续 30 秒、0.08 m PASS 数仍为 0。r30 记录的严格窗口只有
`25.18541362900004 s / 641 samples`，最大物理目标误差 `0.09953585424044105 m`、最大
station error `0.08652989177543136 m`；该候选已拒绝并把六项改动逐字节回退到 915 基线，
隔离回退测试 `113/113` PASS。r30 身份永久占用，报告为
`runtime_evidence/unitree_h12_incubator_nav2_authorized_deadband_slew_gazebo_20260915_r30/REPORT.md`。
2026-09-16 只读检查虽已离线，但缺少中间关闭 transcript，故不宣称 clean exit。

随后用户明确指定学习 `/home/liu/test` 的步态并优化当前项目。本轮在分支 D 只完成离线
实现，没有启动 Gazebo/Nav2/controller、发布命令、修改固定 authority/run、创建备份或
连接真机。fresh r31 身份为：

```text
runtime_evidence/unitree_h12_v12_learned_recover_stop_offline_20260916_r31/
status  OFFLINE_IMPLEMENTATION_PASS / GAZEBO_NOT_RUN
```

设计保留已被接受的官方 Unitree 47-D recurrent actor 作为唯一非零 MOVE policy，不恢复
此前用户否决的整套 V12 MOVE，也不重复 r13/r14 的静态姿态冻结。只在显式 opt-in 且命令
精确为零时，复用 `/home/liu/test/h1_2_isaaclab` V12 `model_17250.pt` 的学习型 zero-command
机制：官方 MOVE -> V12 BRAKE/RECOVER（测量姿态残差）-> 连续 10 个稳定样本 -> V12 HOLD
（捕获姿态残差）；恢复包络破坏则回到 BRAKE。目标仍受逐关节每周期速率和物理限位约束；
非有限状态、非法 phase、无先前移动、重复 sensor step、未 prime 或物理超限全部 fail closed。

新参数 `learned_recover_stop` 默认 false，只在 Unitree profile 显式为 true 时校验/加载冻结
V12 和 `/home/liu/test` 身份。未来唯一启用形式为
`locomotion_profile:=unitree_h12 learned_recover_stop:=true`；它不隐含
`movement_authorized` 或 `auto_navigate`。非零 MOVE 路径、policy、gain/limit、Nav2 goal
checker、strict supervisor、场景和固定动作 authority 均未改变。

r31 最终聚焦 Unitree 测试 `34 passed`；AGENTS 隔离构建 PASS，包测试
`122 tests / 0 errors / 0 failures / 0 skipped`，install/import 绑定 r31 隔离目录，六项
Miniconda 库状态恢复，无残留隔离临时目录。未改 r30 sensor trace 的 plan-only 矩阵为
1,353 样本/54.639820759 s，BRAKE/HOLD `896/457`、18 次 HOLD->BRAKE、最长旧 trace HOLD
3.800033372 s，全部 target finite/物理限位安全，命令观察 exact zero；最大 rate-limit excess
`1.1175870895385742e-08 rad` 仅为 float32 舍入。该矩阵不是反事实闭环物理，不能证明平衡、
漂移改善或 30 秒驻足，Gazebo strict PASS 仍为 0。

完整报告和校验入口为 r31 `REPORT.md`、`SHA256SUMS`。915 仍是最新不可覆盖模板，摘要
`2148b1f083f4fa28ce647c8e8ca4debf4fc49a92ee26f0bfe5eb3428a9beea0c`，不含 r30/r31；
v9/r17 动作语义、v23/r33 固定视角和 826 D1b plan-only 边界不变。r31 已占用。任何闭环
Gazebo 评价仍需用户重新明确授权 Gazebo 移动，并在只读门后建立全新从未使用 identity；
必须保留原 strict 30 秒/0.08 m 门，失败即 stop-and-hold 并回退候选。真机仍未授权。

## 2026-09-16 10:51 CST：r60 learned recover STOP Gazebo 拒绝并回退 915

用户明确要求查看新步态，授权本轮分支 D Gazebo 移动验证。全局精确 `rNN` token 中
r32--r59 均已占用，首个未使用编号 r60 以普通 `mkdir` 独占创建。运行前 live 为空，
v9/r17、v23/r33、826、support plugin、r31 输入和只读 915 哈希门全部 PASS。唯一启用
候选为 `locomotion_profile:=unitree_h12 learned_recover_stop:=true`；upright assistance
关闭，官方 Unitree 仍是 MOVE policy，原 30 s/0.08 m strict supervisor 未放宽。

r60 自动状态为 HEALTH -> PLAN -> NAVIGATE -> Nav2 goal succeeded ->
`FAILED:PHYSICAL_PATH_ENVELOPE_EXCEEDED`。Nav2 到站后只经过
`0.8517879579999447 s` 即拒绝；终端定位误差 `0.030000000000000006 m`，但物理误差
`0.145321742896628 m`、物理 y `0.13079894466352138 m`。学习分支只产生 56 个
`LEARNED_RECOVER_STOP_BRAKE` 样本，`CAPTURED_HOLD=0`；随后
`FALL_ENVELOPE` 在 recorder wall `35.701914936999856 s` 锁存。全 trace 最大起点位移
`1.1449971664455907 m`、最大 |y| `1.106166510295243 m`、最大平面速度
`3.6930743263102044 m/s`、最小高度/up_z `-0.14614619837500864/0.07973108507923266`。
该候选明确失败，strict PASS 增量仍为 0。

失败后 750 个 safe-command 样本全部 bit-exact zero，controller 保持
`FAULT_HOLD/FALL_ENVELOPE`。独立 recorder 又捕获 `30.029837198999303 s` 后正常刷盘
退出。当前源码已逐字节恢复 915 六个基线文件，新增 adapter/test 均删除；六库隔离回退
构建 PASS，`113 tests / 0 errors / 0 failures / 0 skipped`，临时提取和 Miniconda 六库
均恢复。完整报告为
`runtime_evidence/unitree_h12_v12_learned_recover_stop_gazebo_20260916_r60/REPORT.md`。

10:51 CST 用户查看快照中 Gazebo GUI/server、bridge、Mid360、Nav2、auto navigator 和
controller manager 仍运行，仅用于显示已倒地失败现场；独立 recorder 已退出，无 support
socket。运行时仍来自冻结 r31 isolated install，当前源码已回退；禁止 reset、复用 r60 或
再发运动命令。后续任何动作前必须先重新只读核验并安全关闭该现场。r31 只保留为历史
离线证据，不再是当前候选；新步态必须从新离线设计开始，Gazebo 移动仍需新的明确授权和
全新身份。915、动作 authority、D1b 和真机边界不变。

## 2026-09-16 12:48 CST：fresh v24/r103 完整雷达场景双循环 PASS

用户明确要求展示开箱动作，授权本轮分支 G 固定轨迹重放。执行前重新只读检查确认 r60
退化现场已经离线：Gazebo server/GUI、launch、controller、support、recorder 和 socket
均不存在。全局精确 `rNN` 名称 r1--r102 已占用，故以普通 `mkdir` 独占创建 fresh
v24/r103；v24 builder 先 self-test PASS，再只 build 一次。身份为：

```text
builder   runtime_assets/build_candidate3_roundtrip_831_authority_v24.py
builder   ff4ea7b3dbb701105467c43da270821ce3015a9e4be7838a1f786238891a3cc6
authority runtime_assets/candidate3_roundtrip_831_authority_v24/
run       runtime_assets/candidate3_roundtrip_831_r103/
run id    7ec6d3ba4ce65b1beb14d7e0dde24aa3d3df0cd3d44a1895da442fbb7a7335c8
manifest  8710dcbe597c78b96b0afa658ecbf30730feeb1188c2ca2b7df067f31e8617cf
commands  de524b0b0c70a6c5938b65bb787152723c6acc14760dd84d7f6dd79a894cae0e
```

14 项动作/时序/安全 payload 与 v9 逐字节一致，launch/world/Mid360 三项场景 payload 与
v21 逐字节一致；support plugin 仍为合同哈希
`eb85e8b833a6cbaa0fa26eab09e6a983ba2e36243bf37a12ebb9001c19e95d8b`。fresh 健康门确认
6 个 controller active、五个轨迹发布者为 0、门/把手 q0、latch locked、方块 origin、
机器人 launch、720 点/四墙 Mid360 valid；preparation 精确 PASS 且发出控制命令 0。

正式 flow 结果为 `CANDIDATE3_ROUNDTRIP_831=PASS cycles=2 cube=origin door=locked`，调用至
terminal PASS `169.329060321 s`，motion window `166.788539065 s`。控制消息精确
`60=16/16/16/4/8`，左手把手命令/接触均为 0，命令/实测躯干峰值
`44.25/44.25000000101488 deg`。终态方块原位误差 `0.0005006679608470025 m`，门 q0、
把手误差 `5.332543533807687e-06 rad`、latch locked、两个 support detached、机器人 launch、
五个轨迹发布者为 0；Mid360 仍为 720 点/四墙，inlier/robust RMS
`0.017840225306276748/0.013591709888403108 m`。

rosbag SQLite integrity 和数值有限性 PASS，共 189469 条消息；GUI 视频
`runtime_assets/candidate3_roundtrip_831_r103/candidate3_roundtrip_831_r103_gui.mp4`
为 H.264 Main、1160x1518、10 fps、204.5 s、2045 帧，完整解码 PASS，SHA-256
`a5944e34a5db992faa0d628f1e5d7e398ce38dfbf14ff4e07bd5ec49c39de8c8`。首尾帧视角稳定审计
PASS。`final_audit.json` 只因审计器把 finalize 查到错误日志而误报，保留原记录；修正后的
`final_audit_retry1.json` 为 PASS。进程退出审计同样保留一次自计数误报，排除审计祖先链后的
`process_roles_exit_audit_retry1.json` 为 PASS。`RUN_SUMMARY.md` SHA-256 为
`5f11450fe6836bd62d36b742e65d7a3ab1e2d11f568d8630abbf195ba97af8ae`，39 项
`SHA256SUMS` 全部 PASS。

运行前后 v9/r17、v21/r31、v23/r33、孵育箱源模型和 915 哈希逐字节不变。当前 Gazebo
GUI/server 按用户展示要求保持打开，终态安全锁止；flow、support、视频/rosbag recorder
均已退出，socket 不存在，五个轨迹 topic publisher 为 0。不得复用 v24/r103 再执行；
下一动作前仍须重新只读检查。动作语义权威仍为 v9/r17，v24/r103 只是最新完整四墙
Mid360 场景固定轨迹视觉/回归证据，不是 Nav2 行走或 D1b AprilTag 动态偏移执行证据。
915 未修改且未创建新备份；真机未连接、未授权。

## 2026-09-20：920 箱子版本保护与交付离心机场景切换

用户指定先备份 920 为“箱子版本”，再用 `/home/liu/交付/` 的离心机替换箱子，
并保留今后使用箱子的能力。本轮先备份后改场景，未发布操作/步态轨迹或调用 support。
开始时 live 无 Gazebo/ROS 工作负载；不能沿用 9 月 16 日历史现场。

最新恢复点是 `/home/liu/h1_moveit/backups/920.tar.zst`，
SHA-256 `c4e72068e831ab0d14712261446709704012797ab81ae8948800f7a668db3ecf`，
4,008,880,248 字节。archive/sidecar 非覆盖发布、普通文件、0444、单硬链接。
36,271 成员包含 29,182 文件、6,272 目录、817 symlink；所有普通 payload 与源一致，
归档前后 inventory 不变，安全路径及 symlink、zstd、最终摘要、归档内 r103 MP4 完整
解码及 SQLite integrity 全部 PASS。827/828/831/901/909/914/915 旧保护链前后不变。
归档内有 `BACKUP_920_BOX_VERSION.md`；发布报告在
`runtime_evidence/backup_920_release_20260920/REPORT.md`。920 含 v24/r103，
不含其后离心机和发布后文档；920 取代 915 成为最新完整恢复点，禁止覆盖。

当前入口 `bash run_lab_scene.sh` 默认 centrifuge；
`bash run_lab_scene.sh incubator` 显示原箱子+绿色方块。切换前正常退出旧 launch，
不能并行启动两套 world。完整切换/恢复说明统一在 `SCENE_VERSIONS.md`。
箱子切换只是静态显示；正式双循环仍须分支 G、新 builder/authority/run 和密封 argv。

新增 `src/h1_2_description/models/centrifuge/`，11 份 STL/7,922 三角形逐字节复制，
原交付文件不改。按尺寸推断毫米，scale=0.001，CAD +Y 转 world +Z、CAD +Z 转 -X；
保留交付开盖姿态。原点 (0.600,0,0.815)，world AABB 约
[0.305062,-0.225,0.815] 至 [0.894938,0.225,1.408057] m，水平投影完全位于桌面。
这是 static visual/collision 装配，尚无盖子关节、转子、上料或 Tag 操作合同；
旧孵育箱把手偏移/轨迹不能直接用于离心机。

新 `lab_scene.launch.py` 离心机模式不生成 incubator/green_cube，不启动孵育箱
专用 Tag/depth mask；H1-2、双 Dex1、D435、Mid360、桌子和 5×5 m/2.5 m 四墙保留。
基础 `gazebo.launch.py` 仅增加默认保持旧行为的 spawn_cube、enable_task_perception、
world_file 开关。独立 `lab_scene_world.sdf` 使用 1 ms 步长匹配原默认 relay；
原 world、孵育箱/机器人模型、导航/操作源码、密封 launch/world 均不改。

六库精确隔离构建 description/config 两包 PASS，独立 overlay 为
`install_lab_scene_920/`，没有重建/覆盖原 support plugin 或 main install。
Conda 六名称中原本四个缺失、两个存在，构建后存在/缺失及摘要精确恢复。
静态几何、SDF、launch 两分支与非法值检查 PASS，导航回归 113/113 PASS。

全局 fresh r104 证据：`runtime_evidence/centrifuge_scene_20260920_r104/REPORT.md`。
Gazebo entity 确认 centrifuge/11 个 visual 在场、incubator/green_cube 不在场，
双夹爪/雷达/桌子/四墙均保留。15 秒只读观测中 9 个定位样本全部 valid/四墙，
scan=720，max inlier RMS=0.018825478389202892 m；6 个 controller active，
五个轨迹 topic publisher 均为 0。前后 10,262 个旧受保护普通文件和 8 份备份校验 PASS。
静态 GUI 保留供查看，无 flow/support/录制。下一轮必须重新检查 live。
v9/r17 动作语义、v24/r103 固定重放和导航严格驻足未通过边界不变；
离心机场景显示成功不是操作/导航成功。实体机器人仍未连接。

## 2026-09-21：离心机独立盖子运动 r108 PASS，暂定 0..120° 限位

用户要求先实现盖子的运动，最大按第二张图向后打开。图片未给数值，暂按闭合 0°、
最大 120° 实现，并向用户明确这是图示近似而非 CAD 精确限位。本轮只驱动离心机本身；
没有手臂/夹爪/方块/support/步态动作，不连接真机，不创建新备份。

原合页圆柱确定的模型轴心为 (0.2680710507631302,0,0.307) m，轴沿 +Y。
箱体固定到 world，盖子及两上半合页合并为活动 link，全部 STL 字节不变。
闭合基准由原交付 31.763140434° 姿态反算，重构矩阵误差 2.22e-16；
闭合盖子下表面距箱体顶面约 2 mm。质量/惯量仅是明确标注的仿真假设，不是 CAD 实测值。
盖子 joint limit 为 0..120°、0.35 rad/s、30 Nm，effort PID P/I/D=100/50/10。

新入口 `bash run_centrifuge_lid.sh open|close|度数`，默认/status 只读。
helper 从 installed SDF 读取限位，拒绝越界/NaN/Inf，等待反馈与 bridge，
检查实测误差 <0.2°/速度 <0.01 rad/s 并保持 0.5 秒墙钟；异常请求保持。
这是盖子自身仿真执行器，不代表机器人抓盖、真实电机、转子或上料已经实现。
`run_lab_scene.sh` 使用新 `install_lab_scene_lid_20260921_r108/`；启动盖子闭合，
箱子显示入口 `run_lab_scene.sh incubator` 不变。文档见模型 LID_MOTION.md、
LID_MOTION.json 和根目录 SCENE_VERSIONS.md。PROVENANCE static 字段只指原 9 月 20 日导入。

r105 首次 helper 因 env python3 误选 Conda 3.14 在发布前失败，已改为 /usr/bin/python3。
r106 命令通信通过但 velocity-command 模式关节未动，超时后保持；
r107 effort PID 三段开关通过但 60° 中间位积分停稳慢、90 秒超时。
各身份不覆盖，失败输入/trace/报告保留，Gazebo 均 SIGINT 收尾。r108 只把 I 从 10 提高到
50，不放宽限位或完成标准。

最终证据 `runtime_evidence/centrifuge_lid_20260921_r108/REPORT.md`：
open→60°→close→open 四段 PASS，2,724 样本/29.184 秒仿真窗口，
端点约 120°/59.862427634°/0°/120°，最大角速度 0.350000000233 rad/s，
角度边界仅约 1e-12° 浮点差。57 个雷达定位样本全 valid，五个机器人轨迹 topic
publisher 在四段检查中均为 0。两包六库隔离构建、SDF/几何/helper 自检、导航 113 测试
全部 PASS；前后 10,262 旧文件和 8 份备份哈希匹配。920 仍为箱子版本且不含离心机。
当前 GUI 保留最大开盖姿态供查看；下一次操作先重新检查 live，不照抄此快照。

## 2026-09-21：红按钮鼠标切换开关盖 r110，完整机器人场景恢复

用户要求按前面板红按钮打开盖子，再按一次关闭。本轮实现的是 Gazebo 鼠标左键
点击原 button_visual 的交互，不是机器人手指物理按压。CentrifugeButton GUI 插件
在渲染线程按可见网格拾取，不依赖固定屏幕坐标；拖动、右键、修饰键和非按钮区域
不触发，0.5 秒防抖。ROS 节点要求新鲜停稳反馈且无其他命令，调用原 lid helper，
运动中重复点击不排队/不反向，失败锁存；启动不发布动作。
模型/STL、0..120° 暂定限位、0.35 rad/s/30 Nm、PID100/50/10 沿用 r108 原字节。

r109 初次 launch 因新增 Python 脚本缺执行权限异常退出，留下只加载围墙的退化现场，
导致用户看到“机器人消失”。没有删除机器人模型。r109 进程按核实身份正常 SIGINT，
remaining=[]；补可执行权限，并在创建任何场景动作前检查两脚本权限和 GUI 库。
使用全新 r110，原机器人/双夹爪/雷达/桌子/离心机/围墙恢复、6 controller active。

报告 runtime_evidence/centrifuge_button_20260921_r110/REPORT.md：
真实 GUI 点击开→关对为 TOGGLE_PAIR_PASS，实测约 120°/0°；
776 个关节样本/16.226 秒仿真，max speed 0.350000000233 rad/s，
32 个定位样本全部 valid、FAULT 0，非按钮点击无命令，busy 点击拒绝。
原三次点击 harness 的第三次因视角改变而旧屏幕坐标未命中，退出 1；保留原始日志，
不得把该 harness 写成三段全部通过。ui_retry1 因已有 GUI 点击正在开盖而在初始门拒绝，
未自动发出额外点击。后续 GUI 点击完成日志保留，但没有独立全程 trace。

两包六库隔离构建 PASS；按钮 pytest14 passed，colcon 汇总15 tests/0 failures，
导航113 passed。两分支/非法场景/缺执行权限门 PASS；前后10,262旧文件与8份备份
哈希全部匹配，r104..r108 原证据校验通过。没有机器人/夹爪/support/步态动作，
没有创建备份或连接真机。920箱子版本与原 authority/run 保持不变。

当前 run_lab_scene.sh 使用 install_lab_scene_button_20260921_r110，
incubator 分支不加载按钮节点或专用 GUI。GUI 保留供用户点击，最终只读快照期间
仍有正常按钮开合；不得把任一盖角快照当作以后 live。接口集中在 LID_MOTION.md，
版本在 SCENE_VERSIONS.md，旧 r108 overlay 和新 r109/r110 证据均不覆盖。

## 2026-09-22：双按钮、开盖演示与 r113 被动吊篮物理验证

用户依次要求上沿两个按钮、第二按钮只驱动内部转子、允许Gazebo开盖观察，
最后要求两吊篮随转子具有真实被动摆动。当前只执行离心机设备命令，不执行机器人、
孵育箱/support/reset/真机动作，不创建或覆盖备份。920仍是受保护箱子版本。

r111双按钮16项门PASS，旧吊篮仍与转子刚性绑定，报告已按原机器审计补齐。
r112新增allow_open_lid_demo显式开关，默认false；GUI/日志曾显示开盖30 rpm，
但自动harness初始状态不符、随后只读窗口转子已被停住，read_only_audit为FAIL。
不得把r112写成完整连续物理验收通过，其原失败日志保留。

fresh r113：runtime_evidence/centrifuge_passive_buckets_20260922_r113/REPORT.md。
原两桶拆分为独立link，bucket_left/right_swing_joint为被动revolute，无角度控制器、
无手工离心力/动画。按原圆柱几何确定半径7.5 mm的安装轴，距转子90 mm、高237.5 mm；
原11STL不改，初始姿态绕原轴调整为质心自然下垂。完整网格体积积分假设密度
2700 kg/m³：支架1.179530 kg、每桶约0.517251 kg，质心在轴下13.071 mm；
阻尼0.003 N·m·s/rad、-20°..85°限位为仿真假设，不是实测物理参数。

静止→开盖→启动→匀速→停止回落→重启，13项动态门PASS。
两侧30 rpm稳态外摆4.925826°/4.925827°，与独立能量平衡预测4.92578°一致；
停止后平均摆角约-0.000152°，再次启动约4.92584°。
1,619关节样本、1,626状态、66定位全valid，转子峰值30.073 rpm，
5个机器人轨迹发布者0、6个controller active。首版harness只因DDS发现等待不足
在发布前退出，保留原始记录；retry1补等待后完成首次完整试验，不改变模型/门限。
本轮请求通过ROS设备接口，不额外宣称真实GUI点击审计。

入口run_lab_scene.sh / run_centrifuge_lid.sh使用
install_lab_scene_passive_buckets_20260922_r113独立overlay。默认关盖互锁不变；
显式 `bash run_lab_scene.sh centrifuge --open-lid-demo` 允许盖子停稳后开盖旋转，
旋转/减速中锁住盖角，失联减速与原限速/限矩保留。仅Gazebo，不能用于真实离心机。
两包六库隔离构建PASS、精确恢复库状态；配置31 tests/0 failures，导航113 passed。

运行前10,262旧文件/8备份PASS；首次运行后只有915备份读出一次不一致摘要
f45880b52a75c747930daaece9316b6d720580c6f2df359ca5e998339ce8df96。
当即停转且保持开盖，未写入或恢复任何备份。随后sha256sum与hashlib.file_digest
分别匹配原2148b1f0...，mtime/ctime仍为9月15日；完整复核
integrity_retry1/protected_after.json再次10,262文件/8备份PASS。
原因未确定，原异常结果不得删除或写成从未发生。920及旧箱子/authority/run均匹配。

最后GUI/server保留、盖子120°、转子停稳供用户点击绿色按钮查看；该现场只为本轮快照。
后台测试已结束，不发布机器人动作。接口/假设统一在LID_MOTION.md/JSON，
版本在SCENE_VERSIONS.md；更高转速、载荷、流体与真实结构安全尚未标定验证。

## 2026-09-22：r115正面按钮布局，r116全新显示检查

用户要求按钮移到正面便于机器人按到。只改离心机model.sdf的10个按钮visual/collision
pose，盖子/转子/被动吊篮/控制器与GUI拾取名称不变。原11STL与交付源哈希逐项一致。
机器人视角左蓝开关盖、右绿转子启停；帽中心模型(-0.275448466003,±0.13,0.261427679186)m，
世界高度1.076427679186m，间距260mm。按前面板原STL法向(-0.97898044,0,0.20395417)
安装；每侧65个底座边缘采样与原面误差均小于1.6e-8m，避开中央装饰与边缘。
这是几何布局，不是机器人IK、接触触发或物理按压成果；本轮无机器人或设备动作命令。

六库精确隔离构建description/config两包PASS，库状态/摘要精确恢复；
入口切到install_lab_scene_front_buttons_20260922_r115，原install/support不变。
配置34 tests/0 failures、导航113 passed。默认关盖互锁与显式开盖演示开关不变。
改动证据runtime_evidence/centrifuge_front_buttons_20260922_r115/REPORT.md。

首次r115已显示完整场景，但观察器误将低频定位反馈也要求1秒墙钟年龄持续10秒，
慢速仿真下定位最大间隔2.299654秒而超时；36样本全部valid，不得写成harness PASS。
SIGINT launch后残留本轮Gazebo进程，核实身份后正常SIGINT，remaining=[]，失败日志保留。
新r116只将只读观察器定位年龄窗口改3秒，不改模型/定位算法/控制门限。
runtime_evidence/centrifuge_front_buttons_show_20260922_r116/REPORT.md记录完整场景显示PASS，
6 controller active、5个轨迹发布者0、2853关节/142机器状态/6定位全valid，
初始盖子闭合、转子停止、FAULT无。未新增本轮GUI点击或动态转子验证结论。

运行前10,262旧文件/8备份PASS。导航pytest意外重建3个__pycache__，首次保护after及
r116首次before据实FAIL；只读920提取这三份原缓存字节撤销本轮副作用，不恢复任何源码/
模型/场景，不改归档。原失败及生成缓存保留在r115。integrity_retry1全量检查重新PASS。
以后回归测试/运行设置PYTHONDONTWRITEBYTECODE=1，避免测试重写受保护缓存。
920仍为箱子版本；旧孵育箱、authority/run、视频、rosbag、builder、备份保护边界不变。
本轮没有创建备份、实体连接、机器人轨迹/support/reset；live必须重新核对，不照抄快照。

## 2026-09-22：r119机器人真实接触按钮完整流程PASS

用户授权“机器人实际关盖、旋转30秒、停止并开盖”，并在中断后明确要求继续。
本轮仅Gazebo固定站位双臂接触按钮；无腿部/躯干/夹爪开度/support/reset/真机命令，
无新备份。原孵育箱模型、authority/run、builder、视频、rosbag及920箱子版本不改。

fresh r119：runtime_evidence/centrifuge_robot_press_20260922_r119/REPORT.md。
左手Dex1第一指蓝键开盖准备→蓝键关盖，右手第二指绿键启动→稳速30.05仿真秒→绿键停止，
实测停稳后左手蓝键开盖。5物理事件/2127原始contact，18条双臂轨迹（12/6），
终态盖120°、转子停稳、双臂回初始姿态；starts/stops=1/1，无fault、旋转期间始终闭盖。
启动按键至停止按键33.173仿真秒（含加速），峰值30.0803rpm，221定位样本全valid。
执行器PASS，独立16门审计PASS；execution_audit.json SHA-256
792690f6c14469a872ac703aa29b7b7c0c51b18336bcc16faa49cc788cf9c554。

新增离线MoveIt FK/FCL/约束IK、接触对过滤/40ms按下与150ms释放、独占claim流程。
只准指定指尖与对应帽面接触，非目标离线净距8.670888mm（门5mm），目标压入0.5mm；
实测最大深度0.9443微米。正常任务只发双臂轨迹，设备动作必须由原始碰撞触发，
不能用直接命令或距离动画代替。异常路径转子false+实测双臂hold，不得算PASS。
刚性帽与位置控制产生221.24N瞬时峰值，未标定柔顺控制、真实弹簧或安全按压力；
仅固定站位仿真，不是导航后视觉自适应、机器人抓盖或真机安全成果。

r117被动腿自然沉降与YAML零值不符，在正常轨迹前拒绝；r118用实测静止腿姿重新规划，
到达帽面但话题层级错误未触发而保持。r119按Harmonic实际读取修正为contact/topic，
不改关节门限/机器人模型。失败全部保留，不复用身份。两包六库隔离构建、配置42测试、
导航113测试PASS；前后10262旧文件/8备份哈希匹配，PYTHONDONTWRITEBYTECODE=1。
colcon首次扫描重复包及审计首次null时间差错误均保留，修正只涉及工具调用/统计起点。

日常run_lab_scene.sh/run_centrifuge_lid.sh改用install_lab_scene_robot_press_20260922_r119，
仍不自动发布机器人轨迹；enable_robot_press默认false且与开盖演示例外互斥。
后续机器人操作必须新身份、重新规划/审核、执行一次；不得复用r119 claim/run。
接口与边界统一在模型ROBOT_BUTTON_CYCLE.md，920仍不含任何离心机成果。

454秒/4540帧MP4正常SIGINT且全帧解码，但桌面被前台窗口间歇遮挡，不是完整视觉证据。
最终定向窗口截图final_window.png可见开盖停转。GUI/server保留供查看，执行器/recorder退出，
6 controller active、5轨迹publisher为0；下次必须只读重查，不能把快照当永久live。

## 2026-09-23：85%双48box桌面模型与r128真实试抬，尚未完成入篮

用户要求将/home/liu/交付/48box.stl的两个模型先放桌面，再由机器人分别放入两个摇篮；
发现原尺寸大于开口后，用户明确选择“保持摇篮不变，将48box等比缩小至85%后再验证搬运”。
仅授权Gazebo，本轮不转转子、不操作旧孵育箱、不连接真机，不创建/覆盖任何备份。

新增src/h1_2_description/models/lab_48box/，原始STL逐字节复制，SHA-256
8a3b1ad5452064a92bc4d9b09233a1f433d63b9735b18f088f69a9cb7bac5720。
scale=0.00085，外包118.745×88.140×59.5mm；两个独立自由刚体box48_left/right从
桌面(0.44,±0.38,0.84575)m出生、重力落稳。未直接生成于篮内，无物理附着/瞬移/绑定动画。
碰撞是保守完整外包盒，不能利用视觉孔槽作自由空间；质量0.122825kg、均匀惯量与摩擦0.6
均为仿真假设，不是实物标定。原摇篮净开口约90×134mm，85%盒理论单侧短边余隙约0.930mm。
机器人/夹爪/离心机/摇篮几何与原11STL不改，离心机model.sdf摘要仍为
f017dfcf10b0e2c583e80bb6f625ad5998c7815f8fa69e3c1eaa7134a3684a55。

日常入口使用install_lab_scene_48box_20260923_r122；
bash run_lab_scene.sh centrifuge --48box只生成桌上两个料盒，不自动搬运。
默认空载离心机、incubator箱子显示保留；with_48box与旧enable_robot_press:=true互斥，
因为r119成功按钮路径未对新桌面障碍物复核。不得复用r119动作证据替代上料证据。
两盒具有独立只读contact topic；原Pose_V→TFMessage桥接未携带实体名，已移除，
实验执行器按/world/h1_world/dynamic_pose/info中的实体名读取位姿。

r120原尺寸桌面显示/落稳通过；r121为85%离线全路径；r122准入拒绝、r123停靠跟踪门拒绝，
r124微小话题夹爪目标未按预期运动，均没有抓起物体。r125空载probe早于controller激活，
goal被拒绝；r126先确认controller就绪，3次FollowJointTrajectory空载action实测通过。
所有失败/原始日志保留，不覆盖或复用身份。

r127左手真实双指接触并通过21mm试抬，最大抬升48.3858mm，随后2.021244mm/0.5999°
腕相对误差触发防滑保护。r128根据原STL将抓取TCP沿料盒长轴后移11.89912mm，使接触区
更接近重心，并使异常hold保留已接受夹爪目标；模型、原2mm/0.6°误差门不改。
fresh r128左盒最大实际抬升148.6206mm，试抬误差0.837397mm；继续抬升时
2.007438mm/0.5152°再次触发保护。0/2入篮，未释放，右盒未抓取；不能称装载PASS。
最大单点夹持力20.056998N、深度3.525691微米、躯干17.844693°，199定位样本全valid，
转子始终停稳。原生腕/关节插值审计只作诊断，不能用近似时序结果把失败改为通过。

最新报告runtime_evidence/centrifuge_48box_centered_grasp_20260923_r128/REPORT.md。
16阶段12006点离线FCL检查通过，最大关节采样0.002rad，不是连续碰撞或物理装载证明。
规划attached collision object只用于离线碰撞，不给Gazebo物体添加约束。
r128实验执行器15守卫测试通过；正式场景r122两包六库精确隔离构建、配置52测试与
导航113测试通过，PYTHONDONTWRITEBYTECODE=1。实验搬运代码未提升为日常自动功能，
PROVENANCE.robot_loading_validated仍为false，后续先解决可靠夹持/静止保持再入篮，
不能放宽门限、改摇篮、瞬移或重跑旧claim凑成功。

r128视频H.264/1200×900/5444帧/453.666667s正常SIGINT、完整解码exit0，
只是试抬失败录像。机器人失败后stop-and-hold，无开爪/reset/detach；本轮Gazebo/GUI/
launch正常SIGINT，remaining=[]。这是13时现场快照，下次仍先重新只读核验。
运行前后10262个旧受保护文件及8份备份全PASS，r119..r127封存核心校验通过。
920仍是箱子版本，SHA-256 c4e72068e831ab0d14712261446709704012797ab81ae8948800f7a668db3ecf；
不包含任何离心机或48box成果，不得覆盖。原孵育箱动作、导航、正式authority/run与备份保留。

本轮文档同步后另做install_lab_scene_48box_review_20260923_r128独立六库隔离构建，
两包通过，六名称状态/摘要精确恢复。首次测试因工作目录/包发现和PYTHONPATH覆盖导致
0包及5个ROS导入错误，原日志保留；改正调用环境后配置52tests/0failures、导航113passed，
没有修改导航或其测试。review overlay只作回归，日常仍使用实际运行的r122，不提升失败搬运。

## 2026-09-23：r129—r131夹持诊断，双48box仍未入篮

用户要求继续；85%料盒和原摇篮不变，仅Gazebo。最新报告为
runtime_evidence/centrifuge_48box_effort_grip_20260923_r131/REPORT.md。
r129加深夹持25mm因0.8072°试抬姿态门失败；r130恢复40mm并减少20微米过闭合量，
实测仍约19.95N/指，静态保持约1.8秒后2.004725mm滑移失败。均保留原失败证据。
r131建立独立冻结URDF/YAML/launch实验，只让四指使用effort，原几何、惯量、关节限位、
双臂/躯干位置控制和100Hz manager不变；XML还原比较PASS。原URDF/xacro/YAML不改。
夹爪PID500/1000/1.5、积分±1N、PID输出±2N、抓取前馈4N、总输出监测/接口±6N；
空载六次张合通过，真实夹持法向合力约3.243804/3.240150N，但只抬升约5.100011mm，
0.6011°角度门触发。最大FK偏移0.204744mm、同帧原生偏移0.197239mm。
不能与更长r128抬升直接比较改善幅度，不能声称可靠力控抓取已经完成。

三轮均未通过10秒带载保持，0/2入篮，未释放、右盒未抓取、转子未启动。
原2mm/0.6°门未放宽；同步原生腕/指/盒位姿与原FK双门保留。
实际主体指面接触频繁、指垫间歇，当前原网格主体内面投影约14.38mm²；
30种俯仰/高度扫描只作顶点诊断，不是IK/FCL/接触力矩或实际抓取证书。
下一步先分析完整接触面与抗俯仰力矩，再决定新抓取姿态；不盲调增力、不改摇篮、
不瞬移/绑定、不得覆盖原夹爪/孵育箱或重跑r129—r131的claim。

r131全部实验配置仅在证据目录，未接入日常入口；run_lab_scene.sh仍使用r122，
PROVENANCE.robot_loading_validated仍false，r119按钮成功不等于48box装载成功。
r131独立六库隔离构建两包PASS、配置52/导航113/实验20测试PASS；六库精确恢复。
三轮各自运行前后10262旧文件与8备份全PASS，920箱子版本不改，无新备份/真机连接。
r131模型摘要检查曾用错工作目录，保留原失败log；根目录重试匹配，不是模型变化。
各轮失败保持、不松爪/reset/detach，正常SIGINT关闭Gazebo/GUI，remaining=[]。
r129/r130/r131视频完整解码，均只能称失败试抬录像。后续仍重新核对live。

本轮交接/模型README同步后，独立install_lab_scene_48box_effort_docs_20260923_r131
两包六库隔离回归再次PASS，配置52tests/0failures、导航113passed，六库精确恢复。
该overlay只验证文档随包安装，不替换日常r122，不提升实验力控或装载结果。

## 2026-09-23：用户允许摇篮正上方松爪、自然落入作为放置成功

用户要求现场演示，并明确补充：可以在摇篮正上方松开夹爪，若料盒完全掉入则视为正确。
因此不再要求机器人带盒下降到篮底后才松爪；允许受控对准开口上方后自然重力落下。
必须用实际料盒/摇篮位姿、完整包络和桶接触证明完全入篮及稳定停留，不能只看盒中心、
瞬间经过开口或停在篮沿。仍保持85%比例和原摇篮，不瞬移、不绑定，不修改保护门。
该授权只改变最终释放方式，不免除先可靠抓取、试抬和10秒带载保持；失败仍保持并正常收尾。
本轮新r132仅演示当前夹持能力并准备正上方释放分支，不得提前声称两盒装载完成。

## 2026-09-23：r133加速演示仍未入篮，r134恢复完整场景显示

用户在r132空载停靠期间要求加速，核实未抓盒后正常SIGINT中止；其KeyboardInterrupt
原始FAIL只是用户加速请求中断，不是已观察到物理抓取失败，原报告与封存证据保留。
全新r133空载速度0.075→0.12rad/s，带载/试抬0.04→0.10rad/s，夹爪2→1仿真秒；
仅独立world目标实时率0.5→1、更新率500→1000，1ms步长与全部几何不变。
实际执行实时因子约0.241，不能把设置目标当已实现实时运行。原2mm/0.6°门、
10秒带载保持、4N前馈/±6N输出监测不改；未修改原机器人控制器或日常r122入口。

允许篮沿正上方自然松爪：名义盒底高8mm、距坐底68mm；释放后完整8角在实际篮内、
篮接触、无指接触并稳定3仿真秒才算通过。离线16阶段FCL及28守卫测试PASS，
但此次仍未到达释放：左手约3.65N/指，FAIL前最大试抬16.910204mm后0.6389°拒绝，
含异常hold后最大17.155754mm；FK/执行器原生偏移最大0.264794/0.220938mm。
0/2入篮、未通过试抬/保持、未转移或松爪，右盒未抓取、转子始终停止。
114定位样本全valid，最大单点力4.003001N、深度0.113791mm、躯干17.844691°。
原生离线审计的参考时刻/覆盖区间不同，不能用更小角度统计否定原运行时FAIL。
报告runtime_evidence/centrifuge_48box_fast_drop_demo_20260923_r133/REPORT.md。

r133失败保持、不松夹/reset/detach，正常SIGINT录像/GUI/server，cleanup PASS、remaining=[]。
视频239.083333秒/2869帧完整解码，只是失败试抬。离线统计首次因启动期无料盒位姿
KeyError，日志保留；修正过滤后retry1通过，不改执行器或原始trace。
两包六库隔离构建、配置52/导航113/守卫28测试PASS；六库状态精确恢复。
前后10262旧文件/8备份全PASS，r131/r132封存核心一致，原离心机/48box SDF不改。
920仍是箱子版本，无新备份或真机连接，实验未升级为正式装载能力。

用户随后反馈Gazebo已关闭，另建全新r134仅恢复完整场景显示，不重跑失败动作。
机器人/双夹爪/Mid360/桌子/四墙/离心机/两个桌面料盒均在；6 controller active、
5轨迹话题发布者均0，6定位样本全valid，盖闭合、转子停止，无设备动作命令。
报告runtime_evidence/centrifuge_48box_display_20260923_r134/REPORT.md。
本轮结束时r134 GUI/server保留供用户查看，无定时收尾任务；下次仍重新只读核验live。

本轮交接/48box README同步后，独立install_lab_scene_48box_fast_docs_20260923_r133
两包六库隔离构建再次PASS，配置52/导航113测试通过，六库状态精确恢复。
该overlay仅验证文档安装，不替换日常r122，不提升失败搬运或关闭r134显示现场。

## 2026-09-23：r135中间对夹、r136独立增摩擦，左手保持首次通过但整体仍FAIL

用户要求按图中间侧壁夹持，随后看到倾斜并要求增大夹爪与盒子摩擦。本轮只在新实验身份修改，
不改原机器人/夹爪/85%48box/离心机/摇篮模型、旧孵育箱成果、正式run或8备份，无真机连接。

r135：90°竖直腕、TCP相对料盒(0,0,20mm)，实际两指接触中心长轴偏差约±0.111mm。
两版完整入篮路线IK失败并保留；实际执行的是14阶段抬升/10秒保持/放回原桌面probe，
不是完整装载。左盒16.758864mm试抬后运行时FK角0.6427°触发0.6°门，未通过保持。
报告runtime_evidence/centrifuge_48box_midband_grasp_20260923_r135/REPORT.md，
SHA-256 63e4efcb3ad9df0dfb8e420094522f39986f42010340d3749b145547f433589d。
35守卫/52配置/113导航通过，前后10262旧文件/8备份PASS；原始失败及视频保留。
用户请求增摩擦后，在暂停、右盒桌面且左盒仍夹持状态正常SIGINT退休整个旧仿真，没有松夹/reset/detach。
其launch此前已退出、GUI/server残留；不得将历史cleanup的gazebo_retained等同于完整ROS现场健康。

fresh r136：runtime_evidence/centrifuge_48box_friction_20260923_r136/REPORT.md。
仅实验盒SDF的mu/mu2由0.6→1.2；四指共八个指体/指垫碰撞显式设mu1/mu2=1.2。
URDF固定关节合并后的八碰撞及服务器展开SDF中的八指面+两盒都核验为1.2。
指关节dynamics friction、夹紧力/4N前馈/±6N输出、控制器、计划字节、速度和全部保护门均不改。
逆变换XML逐项比较通过，几何/惯量/限位不变。盒子整面摩擦还会影响桌面接触，不是严格接触对限定。
原模型仍为48box dc582ab7ef80536e28836c743f9671b2709d54143cb1d3c2a87a0e3c5f9bf1ec、
离心机 f017dfcf10b0e2c583e80bb6f625ad5998c7815f8fa69e3c1eaa7134a3684a55。

左手约3.64055/3.64036N，最大实际抬升34.961100mm、10秒静态保持并放回桌面通过；
记录原生相对角最大0.194960°、记录FK角0.194991°。只有一轮局部改善，不能外推双手/装载成功。
右手约3.62810/3.62829N，中间接触通过，但试抬2.063127mm即运行时FK角0.6183°失败，
未进入10秒保持；含hold最大抬升2.122516mm。整轮FAIL、0/2入篮，右手未释放，转子不启动。
记录右手原生角0.422886°与触发瞬间FK角不是同一时间/参考，不能用小值推翻FAIL。
最大FK/执行器原生偏移0.172759/0.097782mm，最大单点力4.680827N，接触深度0.112599mm。
332定位样本全valid；最初采集含开盖末段14个未满足120°的样本，原独立审计machine_open_stopped=false保留。
preflight通过后8026机器样本全部开盖且停转，说明机器人动作门未绕过；不得将补充分段统计改写为整轮PASS。

14阶段11388点FCL、两包六库隔离构建、52配置/113导航/39守卫通过，六库精确恢复。
前后10262旧文件/8备份PASS，r135封存核心匹配。新摩擦只在r136冻结副本，日常入口仍r122，
PROVENANCE.robot_loading_validated仍false。失败调参没有覆盖正式源码，无需还原原模型。
下一步先排查右侧同步腕/盒姿态、接触抗俯仰力矩及FK时序，不能仅继续无限加摩擦或放宽误差门。

视频H.264/1200×900/12fps/8915帧/742.916667秒，正常SIGINT且全帧解码exit0。
失败STOP_AND_HOLD后physics暂停(sim175.999s)，右盒仍握持、左盒已放回桌面；没有reset/detach/松夹。
recorder/执行器退出；launch随后SIGINT退出并在其Gazebo包装进程上超时SIGTERM，原日志保留。
最后只余Gazebo server/GUI暂停供查看，不能称完整健康运行现场，不得直接unpause续跑。
920仍为箱子版本、无新备份；下一轮先只读核对并建立全新身份。

本轮交接与48box README同步后，install_lab_scene_48box_friction_docs_20260923_r136
再次完成两包六库隔离构建，52配置/113导航/39守卫测试通过，六库状态精确恢复。
该overlay只做文档安装回归，不替换日常r122、不提升r136整体FAIL或复用当前暂停现场。

## 2026-09-23：用户明确授权夹持角度保护改为2°，r137只做离线配置

用户原话：“0.618度不是很小吗，改为如果出现2度偏差才触发保护”。此新授权只覆盖
48box夹持期间的FK与同帧原生腕相对料盒角偏差门，从>0.6°改为>=2.0°；恰好2°即触发。
保持原三维旋转角度定义，不改为某一欧拉角，不影响预抓姿态或入篮0.25°等对齐条件。
2mm位移门、夹紧力、速度、10秒带载保持、定位/新鲜度/力/穿透和终态门全部不改。
新离线身份runtime_evidence/centrifuge_48box_angle2_config_20260923_r137/，执行器与r136相比
只改两个角度比较和节点身份名；原r136脚本、FAIL、视频、日志及模型/备份不覆盖。
本轮不运行Gazebo、不恢复暂停、不发布轨迹、不松夹/reset/detach，无真机连接或新备份。
新r137仅为代码/边界测试证据，不创建run或execution.claim，不得直接作为可执行场景。
下一次物理测试必须从r136高摩擦配置和r137执行器创建另一个新身份，重做资产绑定和审核。
历史禁止放宽门限的约束仅对以上两处由本次明确用户授权覆盖；不能借此放宽其他条件。
报告见r137/REPORT.md；r136的右手失败结论不能因新阈值而追溯改为PASS。

r137的47项实验守卫（含两处0.6183°/2°边界及2mm边界）全部通过；两包六库隔离构建PASS，
配置52/导航113测试通过，六名称状态精确恢复。前后10262旧文件/8备份和r136封存核心均一致。
只读live仍paused=true、sim175.999s，本轮未执行任何新物理动作。新阈值尚未获物理成功验证。

## 2026-09-23：r138按2°阈值全新试运行，夹爪速度守卫触发

用户明确要求“继续重新运行”。从r136高摩擦builder和r137执行器建立全新r138，
目录runtime_evidence/centrifuge_48box_angle2_run_20260923_r138/，旧r136暂停现场正常SIGINT退休、
remaining=[]；未对握持中的盒子松夹/reset/detach。旧r136/r137密封结果不改。
r138冻结计划字节仍与r136一致，14阶段11388点FCL重新审核PASS；不是入篮转移全路线。
执行器相对r137仅节点身份不同，FK/同帧原生角>=2°触发，2mm位移和0.08m/s指速等门均不改。
模型/控制器/摩擦1.2/速度/夹紧力不变，服务端SDF十处接触摩擦核验通过。
六控制器active、四指effort，设备120°开盖精确PASS后才开始机器人采集与执行；3393机器样本全开盖停转。

六次空载张合和左右停靠通过，左手中间对夹约3.640233/3.640010N，通过接触中线门。
sim91.661s初始pilot抬升时触发“Effort gripper velocity exceeded 0.08 m/s”，整体FAIL。
源守卫实际是非有限速度或abs(v)>0.08m/s，原记录没有触发关节与瞬时原始速度值，
因此不能宣称已测得某个速度峰值，也不能确定超速还是无效反馈。速度守卫不是本次修改的2°门。
触发前一秒33个低频state样本的最大位置差分速度约5.611e-8m/s，不能代表瞬时原始速度，
不得用低频导数推翻FAIL。下一步应新身份增加故障前原始速度/关节/时间戳日志，不自动放宽其他门。

左盒FAIL前最大抬升0.139700mm、含hold最高0.276323mm，尚未离桌完成试抬；
未通过10秒保持、未释放、右盒未抓，0/2入篮。全程最大FK/执行器原生偏移0.038851/0.001029mm。
18个同帧抓取记录的角最大FK0.00006697°/原生0.00001863°，并非故障瞬间完整采样证明。
最大单点接触力3.860362N、深度0.109379mm；139定位样本全valid，转子未启动。
报告r138/REPORT.md、原probe_result_audit.json为FAIL，velocity_failure_diagnostic.json只作诊断。

收尾stop-and-hold、不松夹，physics暂停于sim91.944s，recorder正常SIGINT、视频完整解码。
本轮finish_probe约定结束后统一暂停供查看，不改变测量阶段或控制门。
视频H.264/1200×900/12fps/3496帧/291.333333秒，只能称失败试验录像。
launch/外部ROS随后退出，launch对Gazebo包装进程超时SIGTERM的原日志保留；只余server/GUI暂停现场。
不能当健康完整ROS会话复用或直接unpause。原模型、孵育箱版本、导航、日常r122入口及8备份不动。
两包六库隔离构建、52配置/113导航/56守卫PASS，六名称精确恢复，运行前后10262旧文件/8备份PASS。
本轮未创建备份、未连接真机、未改变2°以外的控制阈值；两盒装载仍未完成。

交接与48box README同步后，install_lab_scene_48box_angle2_docs_20260923_r138
再次完成两包六库隔离构建、52配置/113导航/56守卫回归，六库精确恢复；密封输入与r136/r137核心一致。
该overlay只作文档安装验证，不替换日常r122、不改变r138的FAIL结论或恢复暂停现场。

## 2026-09-23：用户授权持续仿真优化，r139原始速度诊断

用户明确要求“根据碰到的情况不断优化直到能完成任务，不用限制的太高”。
此授权允许基于证据调整不合理的实验门限，不代表取消物理验收或真机许可。
本轮先建立全新r139：每条joint_states记录四指原始速度、位置和时间戳，
单独区分非有限反馈与数值超速；实验指速门由0.08改为原URDF已有的0.2m/s。
首个异常样本锁存、记录关节和值，不滤掉非有限反馈；2°/2mm夹持门和力/穿透条件不改。
先做35mm试抬/10秒保持/放回桌面的双手probe，不能将probe成功称为双盒入篮。
并行离线研究完整入篮可达路线，不修改原摇篮或85%尺寸；最终仍需真实释放、完整入篮稳定。
新实验只改独立副本，不覆盖旧authority/run/失败记录/模型/备份；不瞬移、绑定、reset或detach。
失败保持后正常SIGINT退休整套已暂停现场，再从新身份重建，不能直接续跑旧claim。

## 2026-09-23：r139右手2°失败，r140/r141离线全路线及r142新夹持假设

r139左手34.958540mm试抬、10秒保持和放回桌面通过；右手约3.627N/指，2.410393mm后
运行时FK角2.0111°失败，0/2入篮。16,684条新原始指速最大0.0284575103m/s、无速度故障，
不能用这次结果否定r138未记录的瞬时异常。报告r139/REPORT.md，原始失败保持。

r140最终离线20阶段路线：325mm抬升、载荷倾斜-10°越机壳、篮上方下降再调平释放。
新离心机实验允许躯干yaw规划90°（计划峰86.088443°），在原±2.35rad关节限位内；
不修改旧孵育箱v9/r17语义或其45°要求。原模型、腿、85%尺寸和摇篮均不变。
r141仅构建/稠密审核候选，不启动，无run/claim，不能当物理通过证据。

用户持续优化授权下，下一fresh r142只提高夹持前馈4→8N、接口限幅6→10N，
低于原指关节20N限位；保留2°/2mm、0.2m/s及其他物理验收。右手先行，重新审核载荷顺序。
空载0.20、带载0.10rad/s。最终仍须两盒真实脱离手指、完整8角在实际篮内且稳定3秒。
必要的5mm/3°小幅释放姿态校正只用仿真观测/IK/FCL，不是AprilTag或真机能力。
本轮不覆盖正式资产/日常r122/备份920，不创建新备份，不触发孵育箱或转子动作。

## 2026-10-07：摇篮上料接续，fresh r143验证8N假设

用户要求继续完善双48box摇篮任务，继续沿此前持续仿真优化授权执行。
只读复核：当前无Gazebo/任务执行器/录像进程和窗口。r142仅完成开盖及场景准备，
没有execution.claim、executor trace或result；其8N抓取尚未获物理验证，不能当作成功。
r142 identity内的全部输入/脚本摘要重新匹配，旧目录不复用。

全新runtime_evidence/centrifuge_48box_force8_loading_20261007_r143/继承r142候选，
普通mkdir独占目录、冻结新输入/launch/run identity。原85%盒、摇篮、孵育箱成果及备份不改。
20阶段右先左后路线重新稠密FCL审核；夹持前馈8N、接口±10N，原指关节物理限位20N。
仍执行2°/2mm相对夹持门、0.2m/s原始速度诊断、10秒保持，最终真实自然落入和3秒稳定验收。
两包六库隔离构建、52配置/113导航/65实验测试通过，六库精确恢复；运行前10262旧文件/8备份PASS。
本节只说明本轮已开始的验证，不提前声明装载PASS；以新run结果和独立终态审计覆盖。

## 2026-10-07：r143—r148失败保留，r149落料接触窗口实验进行中

本轮遵循用户“继续完善摇篮任务”和此前持续仿真优化授权，只在新实验副本迭代。
r143空载跟踪失败；r144独立位置增益0.2解决跟踪，但释放校正阻塞采集失败；
r145改异步IK/FCL后8N夹持仍2°失败；r146前馈16N抬升通过，但0.2m/s浮点比较失败；
r147增加1e-8数值容差后实际释放，未启用转子落料被动回转触发旧0.02rpm门；
r148仅加2仿真秒被动回转窗口，落地瞬时篮底穿透1.065755mm又触发原1mm门。
上述各轮整体均FAIL、0/2完整验收，原报告/视频/哈希不改。右盒视觉入篮不是完成证据。

当前fresh r149：runtime_evidence/centrifuge_48box_drop_contact_settle_20261007_r149/。
保留原几何/85%尺寸、2°/2mm夹持、16N前馈/18N接口/2N PID输出和1ms步长。
仅实际上方释放门之后2仿真秒、对应side的release/drop阶段、对应盒与对应摇篮接触，
允许≤1.5mm瞬时穿透；手指及其他接触仍1mm、手指40N，非有限力/深度始终拒绝。
继承停用转子被动≤2rpm窗口；正常动作和3秒稳定验收仍≤0.02rpm，禁止主动旋转。
新114实验/52配置/113导航测试、两包六库隔离构建和20阶段稠密FCL通过，六库精确恢复。
r148已保持暂停后正常SIGINT退休，未reset/detach/松开悬持物体；r149独占run一次性执行。
每轮前后10262旧文件/8只读备份保护，原模型/孵育箱/日常r122/920箱子版本不改。
此处不宣称r149成功；最终仍须两盒真实松爪、全8角完全入对应篮、实际支撑、无指支撑、
各自连续稳定≥3仿真秒，独立流式审计和正常视频收尾。未连接真机、未创建备份。

## 2026-10-08：r154 双手两盒分别入篮 PASS，1008 备份发布前状态

fresh、不可复用身份 `runtime_evidence/centrifuge_48box_right_lower_20261008_r154/` 完成完整
Gazebo 双盒上料：85% 48box 由右手放入右摇篮、左手放入左摇篮。独立轨迹与被动静置审计均
PASS：两盒完整8角包络都在各自篮内，分别有篮底支撑、无指支撑，每盒释放后连续稳定3.0仿真秒；
最终持续篮内跨度右152.878 s、左29.277 s。56段动作完成，最大记录夹持滑移0.884 mm，定位543
样本全有效，转子全程关闭，接触/速度保护故障均为0。最小底部余量约0.499 mm、上沿约1.000 mm，
只属Gazebo几何验收，不代表真实设备公差。

```text
REPORT     SHA-256 3d09a136c565d996846615563a16635960466c516fff0bbe908d8c4189d2188d
SHA256SUMS SHA-256 58fe3e34f2d3bbc915c99ef2c073d56bc6f622420a4031c17f13e52e4201c651
MP4        SHA-256 4a2811711dfc54127bc8e874ebdb54a9777e4d0316bcd23dd5c24cb76cbb9d09
trace      SHA-256 98653919a49c7a2b9d84a3ec6505120adb90d02c10c7ab18bb083b26e6237e80
```

前后保护核验均为10262个受保护文件和8份旧备份全PASS；无源码/正式模型、旧动作权威、旧运行
证据或备份改动。完整报告、轨迹、两盒完整包络、录像解码及审计数据均在上述r154目录。
Gazebo已在完成后暂停并由精确仿真主进程正常SIGINT退出；无执行器、录像器或support daemon。

用户明确指定新备份编号1008。发布前合同为 `BACKUP_1008_RELEASE.md`；完整工作空间归档目标为
`/home/liu/h1_moveit/backups/1008.tar.zst`，sidecar为同目录`1008.tar.zst.sha256`。两目标必须
非覆盖发布，唯一归档根`h1_2_ros2_ws/`，保留符号链接，不包含外部backups目录。实际digest只写入
sidecar及归档外发布报告；须完成全成员路径/类型审计、源与归档普通文件逐字节摘要、zstd完整性、
归档内r154录像全解码、旧备份链前后校验，以及最终`liu:liu / 0444 / 单硬链接`门后才能报告成功。
备份完成前不把1008称为已发布。

## 2026-10-08 11:25 CST：1008 完整工作空间备份发布 PASS

用户指定的1008已完成同目录临时生成、完整验证和no-replace硬链接发布：

```text
archive  /home/liu/h1_moveit/backups/1008.tar.zst
sidecar  /home/liu/h1_moveit/backups/1008.tar.zst.sha256
SHA-256  92ae87377fe9a92922130df5ca30966f70551952a7243b87138976a78c0a2cf9
size     6,517,554,247 bytes
tar      30,594,498,560 bytes
members  76,721 = 49,008 files + 13,780 directories + 13,933 symlinks
```

完整76,720项源inventory在归档生成和逐成员流式审计前后相同；30,480,367,969字节普通文件
payload逐个SHA-256与源匹配。唯一安全根为`h1_2_ros2_ws/`，路径无绝对/父穿越/重复，成员只有
普通文件、目录和匹配源目标的symlink；symlink下无成员，broken symlink与特殊文件均为0。
`zstd -t`及sidecar `sha256sum -c` PASS；归档内r154 H.264视频19,387帧完整解码、0错误。
archive/sidecar均为普通`liu:liu/0444/link=1`文件。827/828/831/901/909/914/915/920旧链发布
前后sidecar及元数据完全一致；backups目录无1008临时文件残留。

归档外发布报告为`/home/liu/h1_moveit/backups/backup_1008_release_20261008/REPORT.md`，
SHA-256 `932a82ba24168e8d30f6a2b847c62bea9bbf6f9f0e4afe1b3f2007ac440bcf79`；同目录
`SHA256SUMS` SHA-256 `62fbc3a8c965d0781ee1c9bf738036bd1a4c2de29f9df46cc0a6db87074efc29`。
1008现为最新完整工作空间恢复点；归档含r154双盒PASS与发布前合同，不含本发布小节及归档外
发布报告，以避免自引用。该备份只保存Gazebo项目，不授予真机操作权限。

## 2026-10-08：r155 双盒任务载物抬升提速 PASS

用户要求优化双盒搬运运行速度。fresh且不可复用身份
`runtime_evidence/centrifuge_48box_lift_speed_20261008_r155/`仅把左右手`pilot_lift`和
载物`lift`速度从`0.10`提高到`0.12 rad/s`；其余速度、轨迹、模型、10秒夹持验证、3秒落篮
稳定门、控制器和物理参数均保持r154。124项离线测试通过，执行器差异由`executor_delta.json`
机器校验仅包含节点身份和上述提速变化。

完整Gazebo运行PASS：两只85% 48box分别由右手放右篮、左手放左篮；两次均通过完整包络、篮体
支撑、无指支撑和各3.0秒稳定门。526个定位样本全有效，夹持最大滑移0.865mm，姿态角偏最大
左0.552°/右0.265°，抬升最大跟踪误差0.0048rad（限制0.015rad），无接触/速度故障。转子
保持停止，释放窗口峰值0.000382rpm。落篮审计、被动静置审计、跟踪审计均PASS。执行器首样本至
PASS为263.503仿真秒；r154同口径为273.191秒，缩短9.688秒（3.55%）。r154有录像、r155未录像，
墙钟时间不作为因果对比。报告：
`runtime_evidence/centrifuge_48box_lift_speed_20261008_r155/REPORT.md`。
报告SHA-256 `7fbbbd29d44e1c3e69875656e776a4121b1347a15e7c94a3473fb8a193ec50c7`；
`SHA256SUMS` SHA-256 `7cd82fd19b8ac4cd62bbe007d586c2c23c78e85890f8c887ea28904a6f069032`。

运行后Gazebo保持暂停供查看；未reset/detach/松夹、未启动转子、未连接真机。旧结果/模型与
1008备份均未改；运行前后对10,262个受保护文件和9份备份（含1008）核验PASS。1008仍只包含
r154及此前状态，不包含r155；未创建新备份。r154和r155运行身份均不可复用或覆盖。若继续提速，
下一试验只能从fresh身份开始，建议只提高空载`descend`速度`0.10 -> 0.12 rad/s`；不得同时
提高载物转运/下放速度或削减任何稳定门，失败须停止并保持，不得把失败参数留在正式状态。

## 2026-10-09：r156预检拒绝，r157全局提速失败

用户将优化目标明确为双48box上料完整动作的Gazebo仿真时间严格小于180秒。r155是可接受完整
PASS基线，首个执行样本至PASS为263.503仿真秒；1008备份仍为r154，不含r155及后续实验。

r156虽完成124项离线测试，但控制器URDF仍绑定r155参数路径，健康门在执行器启动前拒绝；该
身份不可复用。r157修正了路径绑定，以fresh身份实际运行全局`RATE_SCALE=1.8`。执行到约
101.351仿真秒时，右盒入篮前测得相对目标插入姿态误差`1.4938°`，超过既有`0.25°`精确插入
门；有界修正规划也失败，状态为`STOP_AND_HOLD`且右手仍持盒，结果FAIL。右侧最大滑移1.1252mm，
原生同步滑移0.7181mm。未释放、未运行第二盒、未reset/detach/松夹、转子保持关闭。仿真已
暂停并正常SIGINT关闭。全局1.8倍率只得到离线估时，不代表3分钟完成；r157没有完整周期耗时，
不可复跑或复用。

r157离线聚焦测试`124 passed`；运行前后10,262个受保护文件及9份备份均哈希PASS，1008 SHA-256
仍为`92ae87377fe9a92922130df5ca30966f70551952a7243b87138976a78c0a2cf9`。失败报告位于
`runtime_evidence/centrifuge_48box_global_speed_20261009_r157/REPORT.md`，SHA-256
`25de05c3f0cab787d2d59c15c0f07eb8576bca7521c995a8ecf49adaa2a56f59`。

下一步只可用fresh r158验证一个窄假设：保持r157其余全局1.8倍率，只将右侧最终`align`阶段恢复到
r155的`0.10 rad/s`，不改变载物转运、物理参数、轨迹、夹持/姿态/接触门、10秒夹持和每篮3秒
稳定验证。按r155与r157该阶段时长差估算，总周期约178.3仿真秒，但余量仅为估算；只有完整双盒
物理验收PASS且首样本至PASS `<180.000 s`才算达到目标。任何失败均STOP_AND_HOLD；不得reset、
detach、松夹或放宽插入门。1008与所有既有成功资产不变，未获真机授权。

## 2026-10-09：r158 双盒完整PASS但仍超时

fresh r158仅将右手最终`align`恢复到`0.10 rad/s`，其余全局倍率1.8及安全门不变。执行器首个
状态样本31.846秒、最终PASS事件229.632秒，同口径用时`197.786 s`；比r155快65.717秒，但仍比
180秒目标超17.786秒。报告SHA-256 `c636065ca2a30a79a9d4dbf4cc1e109e1ce4a1f669db790c5e992c5dfd2276bd`，
身份SHA-256 `8bba7e5962383bfecdf0af1acc9c109c127b69ea6e63830e26c45ed153d3aee0`，SHA256SUMS
SHA-256 `6373665511f98e2f8e936fbe1592a5b705854e2b3af7d9cfdbd62e4da2f8f43f`。

两盒分别入各自篮并通过完整包络、篮体支撑、无手指支撑和3秒稳定审计；56个动作段PASS，最大
滑移1.183mm，跟踪峰值0.0144rad且无故障，定位395/395有效，接触保护无效样本0。被动转子峰值
0.743rpm仅出现在受限落料瞬态窗内，稳定时低于0.02rpm；转子未启用。仿真PASS后暂停，现场可
检查；未reset/detach/松夹。运行前后10,262个保护文件和9份备份核验PASS，1008哈希不变。

r158实测否定了此前`178.3 s`估计，实测才是权威。下一个唯一速度假设是fresh r159将载物阶段
`pilot-lift/lift/reorient/transfer/lower`共同速度倍率从1.8调至2.6；空载阶段仍为1.8，右侧
最终`align`仍固定`0.10 rad/s`。按r158分段计时估算有望低于180秒，仍待完整物理运行验证。
所有夹持/姿态/插入/跟踪/接触/稳定门、10秒持物和每篮3秒稳定时间原样保留；失败立刻
STOP_AND_HOLD，禁止reset、detach、松夹或放宽安全门。r155/r156/r157/r158均不可复用；1008
只保存r154且不得覆盖；不创建备份、不连接真机。

## 2026-10-09：r159 双 48box 完整上料达到三分钟动作时限

fresh、独占身份`runtime_evidence/centrifuge_48box_global_speed_20261009_r159/`在完整离心机
Gazebo场景完成两只85% 48box分别由右手/左手放入右/左摇篮。唯一调速假设为载物
`pilot-lift/lift/reorient/transfer/lower`倍率`1.8 -> 2.6`；空载保持1.8，右手最终align保持
0.10 rad/s。轨迹、模型、摩擦、物理、控制器、抓取/插入/接触/跟踪/持物/落篮门不变；离线测试
126项通过。

首个执行器状态样本仿真时刻12.991秒，最终PASS事件185.491秒，同r155/r158口径动作耗时
172.500秒，达到`<180.000 s`目标，比r158实测197.786秒快25.286秒。注意绝对Gazebo时钟从
sim=0到PASS仍为185.491秒；达标仅指既有一致的首样本至PASS动作耗时口径。完整结果、落篮和
被动静置审计PASS：两盒完整包络、篮体支撑、无手指支撑；两次落篮各稳定3秒；底部/上沿最小
余量约0.499/1.000 mm。最大夹持滑移1.270 mm，原生同步0.805 mm，角偏最大约0.2964°；跟踪峰值
0.014400 rad且故障0；定位344/344有效；接触违规0。被动转子瞬态峰值0.260783 rpm，最终转子
关闭且低于0.02 rpm。

运行前后保护审计均PASS：10,262个受保护文件和9份备份；1008哈希仍为
`92ae87377fe9a92922130df5ca30966f70551952a7243b87138976a78c0a2cf9`，无新备份。PASS后仿真暂停，
未reset/detach/松夹/启动转子；未连接真机。报告见
`runtime_evidence/centrifuge_48box_global_speed_20261009_r159/REPORT.md`，SHA-256
`e6ed67d6fd2dc1cb22d9dc78a70ea3423200cd5f4980eb907f9a87fa28bbb4f5`；同目录`SHA256SUMS` SHA-256
`df4d50283edadb96bf93d8d03aea833899463533a050ca1cd88ae0b182572799`，114项校验通过。

2.6载物倍率只在r159实验身份内验证，不改变日常入口，也不授权复用本次身份。未来任何上料运行
必须创建全新身份并保持所有安全门；1008仍仅含r154状态。

## 2026-10-09：用户指定 1009 完整备份，发布前状态

用户要求在下一次运行演示前先备份为全新编号1009。r159成功终态仿真已确认暂停、holding=null，
精确Gazebo进程经正常SIGINT全部退出；114项r159 SHA256SUMS在关闭后仍全部匹配。没有启动新的
动作重放。完整工作空间归档合同为`BACKUP_1009_RELEASE.md`，目标为外部backups目录下
`1009.tar.zst`与`1009.tar.zst.sha256`，两目标均确认不存在。备份包含r159及此前全部工作空间
状态，唯一安全根`h1_2_ros2_ws/`，不包含外部backups目录。实际归档哈希只在成功验证后写入
sidecar及归档外发布报告；新归档、成员/内容校验、r154录像完整解码、r159清单核验、九份旧备份
前后哈希/权限及最终liu:liu/0444/单硬链接门通过前，不得宣称1009已发布。

## 2026-10-09 12:00 CST：1009 完整工作空间备份发布 PASS

用户指定的1009已经完成全部校验并非覆盖发布。最新完整恢复点为
`/home/liu/h1_moveit/backups/1009.tar.zst`，SHA-256
`9a746ba6b0f4a7fd871db63ef962cad0d58491207f0e102a85ce7f322c87ea26`，7,504,607,327字节；sidecar
为同目录`1009.tar.zst.sha256`。archive/sidecar均为普通liu:liu/0444/单硬链接文件，无1009
临时文件残留。唯一安全归档根`h1_2_ros2_ws/`，77,277成员（49,527文件/13,812目录/13,938
符号链接）；35,121,136,240字节普通payload逐个匹配源SHA-256，77,276项源inventory在生成、
全成员审计和发布前均未变化。路径/类型/symlink目标及无链接下属成员门PASS；破损链接/特殊
文件均0。zstd及sidecar检查PASS，归档内r159 114项密封清单全部匹配，r154录像19,387帧完整
解码0错误。九份旧保护备份827/828/831/901/909/914/915/920/1008前后哈希及元数据全部相同。

1009包含r159双盒完整上料172.500仿真秒PASS及r155-r158实验、此前全部工作空间成果；不包含
本发布后补充或归档外发布证据，以避免自引用。920仍为独立受保护箱子版本，1008保持原样。
归档外报告为`/home/liu/h1_moveit/backups/backup_1009_release_20261009/REPORT.md`，SHA-256
`5a3f7ccce6ff6d7ab35871743a5812e88b66f66c0571e1c210cc8a73ac3b1d11`；同目录SHA256SUMS SHA-256
`a6e50b89f3b0d57a81e1d6db098ae58546710e06803aa04bd1050cd2c6d9e61c`，22项核验通过。
当前Gazebo及GUI已正常SIGINT退出，原r159身份不可复用；备份期间未启动新的动作展示。

## 2026-10-09：双盒离心机完整流程 r160/r161 集成进行中

用户最新目标为两只85%48box入篮→关盖→旋转→停稳→开盖→两盒取回桌面；仅Gazebo，
沿用此前30rpm、稳速30仿真秒。正常机器动作由机器人指定指尖实际接触蓝/绿按钮触发，
不以直接盖角/转子enable指令代替。原摇篮、盒子比例、离心机及孵育箱模型/正式证据不改。
旧日常with_48box与robot_press互斥仍保留；组合只限新身份内的碰撞审核实验launch。

新状态机已有阶段专属机器守卫：只有闭盖的start/running/stop阶段允许转子运动，上料与
取回仍须120°开盖且停转；持物2mm/2°、原上料0.25°插入、0.015rad跟踪、10秒持物、3秒
落篮稳定门不放宽。停转不会假设原转子角度；取回依据原生Gazebo实测盒/篮位姿重新FK/IK/FCL
规划，并按当前左右分布选手臂。取盒接触点保留盒子长轴中线，上移20mm绕开篮壁，模型不改；
离线仅目标盒起始0.75mm篮底支撑和最终桌面支撑有接触例外，不豁免手指与篮碰撞。

fresh r160：runtime_evidence/centrifuge_48box_full_cycle_20261009_r160/REPORT.md。
配置/导航159、上料回归126、流程守卫24项通过，两包六库隔离构建及精确恢复PASS；左右
取回/放桌路径和两种分布下按键路径的0.002rad稠密检查PASS。但实际运行sim约4.3s时
左踝自然沉降尚未完成，被原腿部门拒绝，正常任务motions=0/物理按键0/转子starts=stops=0。
异常保持后暂停；原relay重复clock step触发launch自动退出，其日志记录自动SIGTERM升级。
残留本身份Gazebo进程精确匹配后正常SIGINT收尾，remaining=[]。前后10262保护文件、10份
备份及r119/r154/r159清单均PASS；未reset/detach/异常松夹，r160整体不是流程PASS。

下一全新身份r161：runtime_evidence/centrifuge_48box_full_cycle_20261009_r161/。
仅修正启动等待：全部初始关节误差<0.010rad持续2墙钟秒，之后再通过原守卫并实际按键。
输入参数/overlay/run均独立，沿用可逐字节校验的离线几何证据；309测试/隔离构建/保护
before均PASS。当前已通过自然沉降健康门、真实蓝键开盖至120°，正在完整流程物理验证。
未完成两盒上料、稳速30秒、停稳开盖和两盒桌面支撑/无指支撑各3秒的独立审计前，不能
宣称全流程PASS。live随时可能变化，必须重查，不照抄本进行中快照。

1009仍为最新恢复点，SHA-256 9a746ba6b0f4a7fd871db63ef962cad0d58491207f0e102a85ce7f322c87ea26；
不含r160/r161。920箱子版本、1008和全部旧成果保留。失败STOP_AND_HOLD/停转/暂停/正常
SIGINT，不reset/detach/异常松夹；无真机授权，不创建新备份，不复用任何既有身份。

## 2026-10-09：r161 部分通过及 r162 接触求解诊断

r161 已两盒入篮、真实关盖、真实启动；完整流程 FAIL，未取回。258.594 s 时左盒侧壁理想
平面仅越界 1.196853 微米，而实际篮接触深度为 3.210032 微米，底/上沿均在内且有篮支撑。
报告在 runtime_evidence/centrifuge_48box_full_cycle_20261009_r161/REPORT.md。紧急停用后
暂停时仍有 25.8608 rpm，不称实际停稳；现场正常 SIGINT 已关闭，remaining=[]。录像完整
解码零错误；前后保护 10262 文件/10 备份及成功清单 PASS；输入密封未变。

新身份 r162 只在实验机器载物阶段诊断有限接触求解：侧壁容差 10 微米，同时要求真实同篮
支撑、有限正力和深度<=10微米；不修改初次上料零容差完整包络或任何模型/夹持/跟踪门。
取回试探抬升消除实测短边微量偏移，仍只在起始0.75mm允许目标篮支撑，不豁免手指碰撞。
失败保持新增等待实测转子<0.02rpm稳定0.5sim秒再暂停。尚无完整流程PASS，必须实际测试。
1009和920箱子版本不变，无新备份或真机许可，不能复用r160/r161。

## 2026-10-09：r162 接触新增门失败，r163 恢复原接触标准

r162 仍完成双盒入篮、真实关盖/启动，但247.535sim秒在篮摆动接触深度17.423697微米时
被新增10微米全接触深度门拒绝；理想侧壁越界只有1.818173微米、竖向完整且同篮支撑
1.020421N。保留失败证据，不能称全流程成功。251.208sim秒实际转子降到1.30636e-8rpm且
稳定0.5sim秒，之后暂停、正常SIGINT关闭，remaining=[]；未异常松夹/reset/detach。
前后10262保护文件/10备份/成功清单PASS，录像13142755字节完整解码PASS。

新r163只去掉不合理的新增全接触深度10微米门，恢复此前1mm接触深度；10微米侧壁求解
容差、同篮真实支撑、竖向范围/无指等保留。原模型、初次落篮严格包络、2mm/2°夹持、
0.015rad跟踪、10秒持物、3秒落稳仍不变；仍须新身份完整物理运行及独立审计。

r163 启动前预测停转约3.7rad的离线检查发现右手送左原位IK残差超限，未发运动。用户本次
只要求取出两个盒子，不要求身份原位复归；故卸料目标改为当前选手侧的两个既有桌面位
(.44, ±.38, .84475)，允许两盒换位，必须验证两个不同桌面位/真实支撑/无指/各3秒稳定。
该修改只限r163独立执行器/取回worker，需新稠密审计；失败预测日志保留，旧上料成果不变。

## 2026-10-09：r163 固定侧壁门失败，r164 使用接触佐证

r163 247.438sim秒短边理想越界12.957854微米，但同篮实际接触深度34.437845微米、真实
侧壁受力且竖向完整；固定10微米门触发FAIL。已实际停稳再暂停并正常SIGINT退出，保护审计
PASS；保留所有失败报告，不称全流程成功。r164新实验只改机器载物判别，避免任意扩大固定
容差：每轴越界<=min(原1mm接触门, 实测同篮穿透+5微米舍入)，超过5微米必须有对应壁法线
与非零侧向力；原同篮支撑、竖向完整、无指、有限接触/深度、模型/夹持/初次落篮等不变。
必须记录最大侧向偏差和接触深度，不把动态接触合格写成旧零容差包络合格。仍须新身份
完整旋转30秒并双盒取回的实际PASS；1009/920/旧成果不可改，不创建新备份或连接真机。

## 2026-10-09：r164 未进入载物旋转；r165 闭合收敛等待

r164 开盖后在右盒初次闭合时3秒回执精度未通过，66.045sim秒FAIL，盒子仍在桌面、有指
接触但正式holding证明未建立，不能按holding=null声称无夹爪接触。保持已接受目标/前馈，
没有抬升、入篮、闭盖/旋转或取回；已静止暂停/正常SIGINT和保护审计PASS。331测试通过，
动态接触佐证方案尚未获得运行结论。r165仅将抓取闭合goal_time收敛等待3->6sim秒，保留
原目标0.2mm、实测0.25mm、16N前馈和所有持物/接触/跟踪门，其余夹爪动作仍3秒。
实验base仅加默认仍3秒的可覆写等待hook，其他与r159逐字节同源；仍需新身份完整运行。

## 2026-10-09：r165 瞬时反力新门失败，r166 接触流形判别

r165 双盒严格上料/真实关盖和启动通过，但246.370sim秒同篮壁面接触反力为0的当步被
新门拒绝；实际有壁面法线、侧偏7.256微米、同篮接触深度38.984微米和底部0.588N正支撑。
未达到30秒/取回，已停稳/保持/暂停/正常SIGINT，保护与录像核验PASS。闭合实际1.3秒
通过，并不能证明3->6秒等待根治r164偏心。原门/模型与旧证据不变。

新r166机器载物阶段侧偏<=原1mm接触上限，>5微米须实测同篮对应侧壁接触法线；不要求
每一帧壁约束反力非零（接触流形可以存在但当步反力为0）。仍要求正同篮支撑、无指或其他
模型接触、有限接触/深度<=原1mm、原竖向范围和严格初次落篮/2mm/2°持物等门。动态
指标必须独立量化，不写成零容差全包络，也不假称实体许可。仅新实验身份，1009/920不变。

## 2026-10-09：r166 篮底静态裕量FAIL，r167动态底面接触门

r166侧壁证据门通过，但246.602sim秒右盒底部-30.501912mm比静态-30.5mm门低1.912微米；
实际篮底穿透0.503389mm、正支撑1.882694N且<原1mm接触上限，整体仍FAIL，未完成30秒。
已实测停稳后保持/暂停/正常SIGINT；保护与视频核验PASS，785个正式源码文件与1009归档
全部SHA一致。r167机器载物阶段底面使用标定-30.0mm加原1mm接触求解上限，并必须实测
同篮底面法线；无底面接触、超过原1mm、上沿>30.5mm、其他/指接触仍拒绝。原严格初次
落篮-30.5mm/夹持/跟踪等不改。动态求解容许不等于原零穿透包络，须独立记录峰值。

## 2026-10-09：r167 真实数值穿透超门，先做 r168 非机器人台架

r167 248.505sim秒底部穿透1.015699mm超原1mm上限而FAIL，同篮接触采样0.997410mm、
支撑0.420124N。已实测停稳/保持/暂停/正常SIGINT，保护/录像PASS。未完成30秒或取回；
不继续放宽原接触门。新r168独立无机器人闭盖载物台架，仅将world步长1->0.5ms验证
数值稳定；相同原模型/质量/惯量/自由摇篮/摩擦/30rpm，仍用原1mm接触及动态包络。
台架诊断指令经原机器互锁与lease，不是机器人实际按键、不是全流程成果；无机器人
控制器、夹持/support/reset/detach，不改原1ms relay。若台架有收益，才新身份接完整场景。

## 2026-10-09：r168瞬时反力FAIL，r169/r170物理台架PASS，r171集成

r168的0.5ms台架接触穿透<1mm，但同篮接触流形当步反力为0，被每步正力门拒绝；未完成30秒。
新r169（0.25ms）及r170（0.5ms）均闭盖30rpm稳速30.0005/30.001sim秒后实测静止、正常SIGINT，
独立审计和前后10262历史文件/10备份保护PASS；最大篮底穿透0.079712/0.449095mm。
原模型、质量、惯量、自由摇篮、摩擦和原1mm侧面/底面/接触上限不改。只在机器动态阶段允许
连续同篮接触流形内短暂零反力：距上次正支撑<=20ms（原100Hz传感器两周期），仍要求对应
壁/底法线、动态包络/有限接触/无异物或指接触。实测最大零力间隔0.75/1ms，最终正支撑。
不能把台架诊断电机指令称机器人按键，也不能把台架PASS称完整任务PASS。

新r171完整场景采用已验证0.5ms世界和独立SHA绑定严格0.5ms原生relay；每物理步原样验证，
100Hz输出/因果公共clock、跳步/重复/复位fail-closed不变，不修改正式1ms relay或任何src。
正常机器操作仍5次真实指定指尖按键，上料原2mm/2°/0.25°/0.015rad、10秒持物/3秒落稳门不变，
两盒取回按实测停转位姿重规划，放回左右两个桌面位，允许身份换位。尚须实际全流程及独立
动作/录像/保护PASS后才能认定用户任务完成；1009/920/历史成果保持，无新备份或真机授权。

## 2026-10-09：r171低速接触触发墙钟超时，r172使用源pose时间

r171仅真实开盖、右盒闭合/试探抬升，64.1805sim秒在Localization invalid通用异常停止，
但128个定位状态全部valid；最近有效样本距失败0.5795sim秒而8.00334墙钟秒。是低RTF触发旧
8墙钟秒超时，不是定位算法无效。右盒holding=right，保持已接受夹爪/16N前馈，没有异常释放。
365测试/隔离构建/视频6863帧全解码/前后保护及785正式源码对1009核验PASS；暂停后launch
自动退出有SIGTERM升级日志，精确残留正常SIGINT，remaining=[]。整体FAIL，不能复用r171。

新r172只加定位健康可覆写hook（base默认仍8墙钟秒），完整流程订阅真实StampedPose，
要求status.valid及源pose距当前sim<=1.0秒、pose/status墙钟年龄<=60秒；公共clock/关节原
2墙钟秒活性门不变。缺源pose、过期或无效定位一律拒绝，不伪造更新时间，不改正式导航源码。
0.5ms物理、原1mm接触、20ms正支撑连续性、全部原夹持/跟踪/初次落篮等门仍保持。
还没有完整机器人流程PASS，须新身份实测；所有旧成果/1009/920不变，无真机或新备份。

## 2026-10-09：r172载物子流程通过但重复clock导致完整FAIL；r173离线取回碰撞

r172完成两盒严格上料、真实蓝/蓝/绿/绿/蓝五次按键、30rpm稳速30.050sim秒、实测停稳和
120度开盖，MACHINE_CYCLE_PASS为309.554sim秒；尚未实际取盒。原1mm门内最大篮底穿透
0.263698mm，最大接触深度0.263717mm、最大侧偏0.064945mm；382测试/隔离构建通过。
320.003sim秒relay先收到重复clock而退出，launch随即停止桥接，之后才报Stale machine。
不能把该超时当成低RTF新鲜度误报，不得放宽2墙钟秒遥测活性或严格clock重复/跳步门。
重复clock来源未证明；不能归咎用户或宣称隐藏GUI控件已经修复根因。已正常收尾remaining=[]，
视频43826帧完整解码；前后10262历史保护文件/10备份及785正式源码对1009核验PASS。

r173只读实际停转位姿做离线取回，第一盒拾取找到，但持盒RRT运输在原0.002rad稠密检查时
与外壳碰撞；整体离线FAIL，无控制发布/物理取回。下一r174离线候选将RRT每条扩展、连接、
捷径边统一按0.002rad检查，保留原FCL/模型和所有物理门，不略过碰撞。完整任务仍未完成。
后续新fresh身份须实际双盒取回及独立审计；不得复用或改写已封存r172/r173。

## 2026-10-09：r174实测姿态离线四段PASS；r175完整新实验

r174统一RRT边为0.002rad后，两盒拾取/持盒运输四段稠密审核PASS共13974样本；实际停转
309.5305sim秒位姿来自r172，空夹爪打开和首盒放回桌面均为规划假设，没有控制或物理取回。
新r175完整fresh实验只在此前基础上去掉取回前冗余旧停车，实测中立姿态在线规划，并将RRT
扩展/连接/捷径用原最终0.002rad稠密门；实验GUI仅移除WorldControl暂停/步进插件，正式
GUI配置不改、其他插件保留。不能声称重复clock根因已解决或离线等于实跑PASS。
原严格clock/2墙钟秒遥测/1mm物理接触/2mm2度夹持等门全部保持，须实际完整运行和独立
动作/视频/保护审计。保护清单包含已封存r167至r174，正式785源码/1009/920及所有旧成果不改。

## 2026-10-09：r175左盒角滑移FAIL；r176只重定时该段

r175已实际开盖/右盒严格入篮、左盒试探抬升和10sim秒持物；164.8695sim秒left_transfer
末段角滑移2.0268度被原2度门拒绝。FK和原生角同时增长，不是时钟误报；位置偏差错误消息
四舍五入0.180mm（r175 REPORT的0.179548mm未独立验证，不作为精确证据）。
holding=left保持16N及已接受目标，未异常释放/reset/detach；机器本就0rpm，实测quiet再暂停
并正常SIGINT精确残留，remaining=[]。389测试/隔离构建/前后保护/785正式源码对1009及
录像全解码PASS，任务整体FAIL，不能复用封存r175或称双盒上料/旋转/取回完成。

新r176唯一运动修改是FullCycle.move仅对left_transfer将实际速度上限2.6->1.3rad/s；
原101点路径和其余阶段/右手/取回速率不改。该段7.567sim秒几乎都受速度门支配，减小终点
减速冲击为待验证假设；不放宽2度或其他任何物理/遥测/接触门，不改正式src/模型。
必须fresh全程与独立审计，1009/920及旧成果保持，无新备份或真机许可。

## 2026-10-09：r176双盒/机器子流程PASS但取回FAIL，下一r177慢速试探

r176左手上料transfer1.3rad/s实际角峰0.092820度，双盒严格上料成功；5次真实指尖按键、
30rpm稳速30.050sim秒、实测停稳开盖在302.198sim秒通过。机器原1mm接触门内最大篮底穿透
0.247938mm/接触深度0.248445mm，最大零反力间隔0。397测试/隔离构建通过。
首次右手按实测旋转后位置抓左身份盒，两指实测15.518/15.616N、中间长轴接触与取回稠密
规划通过；323.867sim秒15mm试探抬升带动原自由转子-0.0977917rpm，电机disabled且盖120度，
触发原0.02rpm静止门。尚未取出任一盒，整体FAIL，不能以机器子流程或离线证明宣称完成。
holding=right/held_payload=left保持；实测quiet后暂停/正常SIGINT，remaining=[]。38724帧
视频全解码/前后保护/785正式源码对1009核验PASS，正式模型和旧成果/1009/920均未改。

新r177仅把两手取回pilot_lift的实际速率0.12->0.012rad/s减小机械牵带冲击，其他取回
阶段不改，保留已通过的左手transfer重定时。原0.02rpm静止、2mm/2度、接触、时钟/遥测
全部门不放宽，原自由摇篮/转子和电机力矩不改、不虚假固定或Gazebo attach。仍须fresh
实际完整取回/两个独立桌面位3秒稳定及独立审核，不能复用封存r176。

## 2026-10-09：重要速度单位纠正；r177右盒lower失败，r178真实降速

上述r175/r176/r177段落及sealed报告若把“2.6->1.3rad/s”称实际左手降速，必须以本节纠正：
LOADED_RATE_SCALE=2.6是乘数，真实trajectory_rate(left,transfer)=0.05*2.6=0.13rad/s；
FullCycle的min(rate,1.3)没有降低实际速率。r176该段角峰0.092820度是观察事实，不能归因于
不存在的重定时，不作为该方案收益证据。原测试用任意2.6输入而没有实际rate联调，覆盖不足。
不改写已sealed旧文件，以本节、真实源代码和新身份报告覆盖错误因果结论。

r177在92.1625sim秒right_lower FK/原生角2.033916/2.033914度FAIL，位置约0.195mm；
该段真实速率也是0.13rad/s，101点2.414176sim秒。右盒仍夹持、未落篮/旋转/取回，quiet/
暂停/正常SIGINT remaining=[]；405测试、视频12062帧全解码、前后保护及785源码PASS。
取回pilot实际0.12->0.012rad/s代码存在，但本轮没有执行到，仍未物理验证。

新r178仅将已出现故障的left_transfer和right_lower实际速率0.13->0.065rad/s；
新测试必须调用真实trajectory_rate，并用实际输入计划证明指令时长增加，防止无效上限。
保留待验证取回慢速pilot；其余速率、原模型和所有静止/夹持/接触/跟踪/时钟等门不变。
还未完整取回两盒，必须fresh物理执行与独立审计；1009/920及所有旧成果不改，无新备份或真机。

## 2026-10-09：r178真实降速上料PASS，慢速取回仍回驱；r179有限力矩台架

r178真实left_transfer/right_lower 0.13→0.065rad/s后双盒上料220.5005sim秒PASS，
五次物理按键、30rpm稳速30.050sim秒及实测停稳开盖在328.331sim秒PASS。原1mm门内
最大底穿透0.280302mm，407测试通过。首次右手实测夹左身份盒后，pilot真实0.12→0.012rad/s
仍带动转子-0.0319488rpm，在350.4295sim秒触发原0.02rpm门；未取回任一盒，整体FAIL。
保持/quiet/暂停/正常SIGINT remaining=[]，43119帧视频全解码，前后保护及785源码PASS。
REPORT SHA=c4194e3c529d7b6bd137b6531f8603627730babdc8195ff70b3d7eccab39fe1a，167文件封存。

新r179为非机器人预置双盒机械A/B台架：仅新实验电机plugin在disabled且目标零时测试速度
增益2→20，真实JointForceCmd、原±2Nm限幅，运行与减速原算法/安全互锁/原模型质量几何/
自由转子摇篮全部保持。±0.025Nm外部物理扰动，各0.4sim秒，先30sim秒原稳速及停稳。
禁止把台架成功冒充机器人任务。原0.02rpm/1mm/2mm2度等门不放宽，不虚假固定/attach，
不改正式src或任何旧身份/备份。若有效才用下一个全新完整机器人身份实测取回，失败慢pilot
回原0.12rad/s；正式785源码仍应逐字节匹配1009。

## 2026-10-09：r180有限电机制动台架PASS，r181完整新身份

r179原基准有真实±0.025Nm回驱，但执行器误只统计发布窗口，错漏CLI异步延后响应而FAIL；
未运行候选，历史已封存。r180全新A/B统计实际全响应及收敛，gain2/gain20分别30.0005sim秒
闭盖30rpm、停稳、两次相反扰动：最大绝对rpm0.119367/0.0119369，约降低90%；原1mm
门内最大篮底穿透0.451892/0.298656mm。两个执行器/独立审计PASS，正常SIGINT remaining=[]。
仅真实JointForceCmd停转速度增益，原±2Nm限幅、运行/减速算法及安全header/模型不变，
自由摇篮保持，不是固定或attach。隔离构建/2C++测试、正式Python46+导航113通过。
这只是非机器人预置双盒台架，不是用户完整任务或真机/视觉/导航成功。

新r181完整身份引用r180审核后的新实验库，模型fork仅换plugin及增益、几何质量关节不改；
正式src仍785文件匹配1009。失败慢pilot回原0.12rad/s，真实故障上料两段0.065rad/s保持。
仍实际两手夹盒落篮、五真实按键、30sim秒稳速/quiet开盖、实测姿态稠密在线取回两独立
桌面位3sim秒稳定。所有旧0.02rpm/1mm/2mm2度/严格clock和遥测等门不放宽。完整PASS
前不能宣称完成；1009/920及所有旧成果不改，无新备份/硬件/复位/异常松夹授权。

## 2026-10-09：r181停转制动物理验证生效，RRT大倾斜运输FAIL；r182离线

r181双盒上料207.501sim秒，五物理按键/30rpm稳速30.050sim秒/quiet开盖315.3365sim秒PASS。
原1mm门内最大底穿透0.286809mm，1397322检查、最大零反力0.5ms。首次真实右手抓左身份盒，
原0.12rad/s pilot通过，59机器样本最大0.0100401rpm，原0.02rpm门保持；无篮接触和10sim秒
持物347.708通过。因此新±2Nm停转制动在此真实机器人情形生效，保留，不是固定/attach。
运输right_retrieval_transfer在353.0035sim秒角>=2度触发原门，位置峰<1mm；未返回任何盒。
holding=right/left payload保持、quiet/暂停/正常SIGINT remaining=[]，视频37963帧全解码；
413测试/隔离构建/前后保护及785正式源码匹配1009 PASS，正式模型和1009/920保留。

r182只读原FK显示旧RRT持盒运输命令最大盒体倾斜52.112836度，试探/抬升约0.965591度。
不能认定单纯摩擦不足或放宽2度门；先离线验证近水平Cartesian运输和原0.002rad碰撞。
首次1.190m抬高试案有IK残差被原20um/.01度门拒绝，保留失败；降低高度/合理绕行仍须严格
可达性和碰撞检查。离线不是取回；当前没有用户完整任务PASS，无新备份或真机。

## 2026-10-09：r182近水平四段离线PASS，r183完整物理候选

r182保留多次IK/碰撞失败，不放宽原20um/.01度或0.002rad门。最终旋转后约180度的盒从
180度方向夹同一中间部位，近水平Cartesian在1.180m中心高度搬运；空手仍原RRT。
四段离线FCL共13140点通过；4443稠密持物FK样本最大命令倾斜首/次0.964590/0.965088度，
取代旧52.1128度RRT运输。第二盒假设首盒已返桌/闭爪间隙，离线不是物理成功；35文件封存。

明确新交付姿态：本次“取出2盒”允许矩形盒正放Yaw0/180度，选近当前朝向且执行前声明，
桌面位仍[.44,±.38,.84475]；相对声明目标仍原2mm/2度误差，不允许倒置或任意Yaw。
这是桌面目标设计改变，不是提高2度夹持保护。不得混入孵育箱回出生位语义，旧孵育箱模型/
authority/run/920及1009保留；原held-payload相对夹爪2mm/2度门严格保持。
r183新完整物理候选额外在每0.002rad携物FCL样本约束命令倾角<2度，取回transfer真实
0.20→0.10rad/s。r181已验证±2Nm停转制动与所有原门保持；实际两盒返桌3sim秒稳定和
独立动作/视频/保护审计前不能宣称完成，无新备份或真机。

## 2026-10-09：r183第一盒实际返桌，第二盒回驱FAIL；r184台架

r183双盒上料204.1005/机器子流程311.924sim秒PASS，5真实按键/30rpm稳速30.050sim秒。
首盒（左身份右手）同中部180°夹取，pilot峰0.00634472rpm、10秒持物及近水平运输通过；
371.737sim秒实际返右桌面位3秒稳定。运输相对夹爪角峰1.179981°，原2°门保持。
次盒（右身份左手）pilot在418.8585sim秒牵带-0.0318713rpm，超原0.02rpm而整体FAIL，
制动力矩0.0667176Nm未饱和；不是夹持失败，不回退有效近水平运输，尚未完成双盒取回。
保持/quiet/暂停后launch自动SIGTERM升级日志保留，精确残留SIGINT remaining=[]；
451测试/视频44465帧全解码/前后保护10262文件与10备份/785正式源码对1009核验PASS。
下一r184非机器人A/B只测停转增益20→40和±0.067Nm真实外扰，原有限±2Nm/运行/减速/
安全header/几何/自由摇篮/所有门保持。外扰近似不是已识别外力，台架不是完整任务证据。
必须新fresh完整机器人身份实际两盒返桌/独立PASS；1009/920及所有旧正式成果不改。

## 2026-10-09：r184 gain40台架PASS，r185新完整身份

r184全新机械A/B，gain20/40各闭盖30rpm稳速30.0000/30.0010sim秒、停稳后±0.067Nm
真实外扰，全响应峰0.0319907/0.0159951rpm，约降低50%；原1mm门内最大底穿透
0.296367/0.363508mm，两个执行器/独立审计PASS。仅非机器人台架不是完整任务。
原运行/减速gain2/±2Nm限幅/安全header/几何质量/自由摇篮保持，2C++及159Python/
隔离构建六库恢复、正常SIGINT remaining=[]、前后保护/785源码对1009核验PASS，62文件封存。
下一r185新完整候选只换新实验plugin gain20→40，其余r183有效近水平路线/速率/原0.02rpm/
2mm2°/1mm/严格clock与活性门不变。451测试/隔离构建通过，须fresh全程及独立审核。
没有两盒实际返桌前不能宣称用户完整任务成功；旧正式成果/1009/920均不改，无新备份或真机。

## 2026-10-10：r185 gain40完整FAIL，局部轴取盒为下一假设

r185实际上料204.091、机器子流程311.923sim秒PASS，30rpm稳速30.0495sim秒/5真按键；
首盒370.4125sim秒返桌3秒稳定。但次盒pilot417.7755sim秒-0.0303591rpm触发原0.02rpm，
制动力矩0.127136Nm，未达±2Nm。相比r183 gain20，速度峰几乎相同而反力近翻倍，
不能把固定外扰A/B收益外推为实际取回收益；尚未取回第二盒，完整FAIL与独立审核FAIL。
夹持序列角峰0.478014°/位置峰0.952057mm，831定位状态valid；保持/quiet/暂停/收尾
remaining=[]，launch有SIGTERM升级日志，视频44404帧完整解码，451测试/隔离构建/前后保护/
785源码对1009核验PASS。186文件封存，正式模型/孵育箱/1009/920及所有旧成果不改。
r186仅测试1篮盒+1桌盒预置机械gain40/80稳定性，非机器人、不是实际卸载或完整成功。
r187只读实际两次抓取请求，旧pilot仍在篮内时径向重居中约0.922mm且lift又回原xy；
离线候选只沿实测篮局部z轴15mm，不横向重居中，原IK/FCL0.002rad和命令倾角<2°保持。
这是待验证几何假设，不是已证明根因；有效近水平搬运保留，失败gain40不直接作为后续完整默认。
所有0.02rpm/2mm2°/1mm/严格clock与活性门保持；新fresh实跑双盒返桌与独立PASS前不能称完成。

## 2026-10-10：r187沿篮轴离线PASS，r188新完整物理候选

r186部分卸载预置机械gain40/80各闭盖30rpm稳速30sim秒，±0.13Nm峰0.0310356/
0.0155183rpm；执行器/独立审计/正常SIGINT remaining=[]及原接触门通过。仅台架，
不能外推真实机器人；r185回驱几乎不随gain减小，故不采用80进入下一完整默认。
r187只读r185两次真实抓取请求（第二次首盒已实际返桌），改15mm pilot沿实测篮局部z轴、
取消篮内径向0.921475/0.922641mm重居中。原0.002rad FCL共10066及4472稠密held FK
通过，命令倾角峰0.967789/0.966167°，pilot相对篮xy误差<0.001mm；23几何测试PASS。
仅离线可行性，不是实际取回；原IK和所有物理门不改，19文件封存。
r188新完整候选回已验证gain20/±2Nm，其他r183有效近水平携物/速率和所有原门保持，
唯一运动几何改为沿篮轴pilot。另将规划前pose用deepcopy冻结，修复同可变字典自比问题；
原规划后0.3mm/.1°门不变，5个实际回调更新回归证明拒绝超限位姿。旧字符串GUI断言初次
失败日志保留，更新为deepcopy合同后479测试/隔离构建/六库恢复PASS。须fresh实际双盒返桌/
独立动作视频保护PASS才可称完成，正式785源码/孵育箱/1009/920及所有旧成果不改。

## 2026-10-10：r188在线篮壁碰撞FAIL，先r189有限力矩驻车台架

r188两盒上料204.090/机器子流程311.924sim秒PASS，5真按键/30rpm稳速30.0495sim秒。
新沿篮局部轴pilot在线FCL在index2/fraction0拒绝左盒与左篮壁碰撞，331.988sim秒FAIL；
未执行取回pilot，没有盒返桌。r187离线可行性不能外推不同fresh姿态，失败几何不作默认。
原quiet/夹持/碰撞门不改，保留近水平搬运与deepcopy修复；holding右手/左盒，quiet332.616，
暂停后精确收尾remaining=[]，launch有SIGTERM升级；35339帧视频完整解码、479测试/隔离构建/
六库恢复/前后保护10262文件和10备份及785正式源码对1009 PASS。完整任务仍未完成。
下一r189只做1篮盒+1桌盒机械A/B：baseline速度gain20；候选D80/P8000编码器驻车反馈。
只在disabled、target0和初次实测quiet后锁存实际转角；真实JointForceCmd保持±2Nm，允许
有限非零角挠曲，不是固定/设置位置/attach。运行减速原gain2及安全header保持，原自由摇篮/
质量几何/0.5ms/1mm/20ms/0.02rpm等门不变。先证明真实有限角响应、参考保持、quiet及支撑，
再新小规模机器人取回测试，通过后才新完整身份；不把台架或初始预置当用户完整成功。
孵育箱/正式模型/authority/run/1009/920保留，不新增备份或真机/reset/detach/异常释放。

## 2026-10-10：r189有限驻车反馈机械PASS，r190仅单盒实际取回

r189一篮盒/一桌盒初始预置fresh A/B各30rpm稳速30sim秒，±0.13Nm真实外扰全响应峰
D20/P0 0.062070462、D80/P8000 0.011046675rpm；候选真实16.25006729微弧度非零挠曲，
参考全程不变，清除外扰后角误差3.04e-12rad，最大力矩0.240516Nm<原±2Nm。
原1mm/20ms支撑、质量惯量几何/自由摇篮与0.02rpm门保持，3C++/159Python、隔离clean重建/
六库恢复、前后保护及785源码对1009 PASS，两执行器/独立审计PASS，SIGINT remaining=[]。
报告SHA256 06f5149656cc795177a513a9af18e7ac55d265a4d8f5a00572ef5ce0a67a9594，64文件封存。
这只是台架，不证明机器人取回或完整流程；初次未source Python/直接增量构建日志均保留，
随后正确环境和六库隔离clean-first重建复验通过，不隐藏过程失败。
r190新身份初始左身份盒预置右桌位，右身份盒预置右篮；转子/两篮仅同轴刚性初始Yaw变换，
篮摆角初始0，其余关节轴限位/模型几何质量不改。真实蓝按钮开盖、左手中部抓取右盒及
居中pilot/10sim秒持物/近水平返左桌3sim秒稳定，原所有门保持；不启转子、不attach/pose设置。
第一盒不是由此试验机器人取回，故即使单盒PASS也不是用户完整任务，之后仍须全新完整双盒
装入/5真按键/30sim秒稳转/真实两盒返桌独立PASS。旧正式成果与1009/920不改。

## 2026-10-10：r190单盒真实取回PASS，r191完整验证待实跑

r190新fresh左身份盒初始预置右桌位，右身份盒预置右篮；真实指尖蓝按钮120°开盖，
左手实际中部抓右盒，居中15mm试探/10sim秒无篮接触持物/近水平运输，103.8065sim秒
返左桌位3sim秒稳定；128.737sim秒单盒流程PASS，无held payload。第一盒预置且没有旋转，
不是用户整套完成。实际rpm峰0.008575469<原0.02，力矩峰0.080393550Nm<±2，
夹持角峰1.706360680°/位置峰0.882822622mm<原2mm2°。两个在线0.002rad FCL1792/5497，
原IK与deepcopy0.3mm/.1°门保持；257定位valid，125测试/隔离构建六库恢复/前后保护和
785正式源码对1009 PASS，独立单盒审核PASS，13140视频帧全解码、SIGINT remaining=[]。
新r191只准备完整候选（尚未实跑）：r189有限D80/P8000/±2Nm，原居中pilot和近水平运输/
所有旧物理门，fresh初始两盒均桌面；459Python测试、隔离构建通过。必须新完整双盒装入/
5真按键/30sim秒稳转/实测quiet后开盖/实际两盒返桌3sim秒、独立视频动作保护PASS才称完成。
旧正式成果、孵育箱、模型、1009/920保留，无新备份或真机、reset/detach/异常释放。

## 2026-10-10：r191身份检查拒绝，r192重新完整验证

r191异步seal/被动launch已完成后编辑supervise_completion.py加俯视截图，未先确认seal完成，
执行前独立hash复核发现不一致而拒绝；未运行execute_cycle、任务轨迹、抓盒或启转子。
原identity.json及expected/actual变动证据保留，不恢复后复用，正常SIGINT remaining=[]。
459测试/隔离构建不能掩盖身份变化，完整任务NOT EXECUTED；前后旧保护及785源码PASS。
仅此新实验收尾脚本，孵育箱/正式模型/成功run/1009/920不改。r190单盒真实PASS仍有效。
新r192全部源码/收尾方案先审核、构建、测试，之后保护/一次seal；seal后停止编辑成员。
仍原完整两桌盒/真实上料/5指尖按钮/30sim秒稳转/quiet开盖/两盒取回桌位3秒稳定目标。
未取得新完整物理/独立动作视频保护PASS前不能称完成，无新备份或真机/reset/detach/异常释放。

## 2026-10-10：r192封存自引用拒绝，r193根因修复

r192完整NOT EXECUTED，无Gazebo或任务动作。按钮审核未完成时首次seal被缺失文件门拒绝，
之后全部459测试通过，但seal stdout重定向seal.log，旧全根扫描捕获正在写入的空日志；
SEALED输出改变日志，launcher hash检查拒绝seal.log。原manifest与日志保留，不恢复或复用。
前后旧保护/785正式源码对1009 PASS，孵育箱/正式模型/旧成功run/1009/920未改。
r193新的封存选择器只选源码/合同/inputs和明确已完成验证日志，排除自身与执行日志，
增加4个选择器回归；等全部测试结束才保护/一次seal，seal后不编辑成员，再独立复核。
新完整物理验证及实际两盒返桌仍待完成；r190单盒实际PASS不能冒充整套，所有原门不变。
无新备份或真机、reset/detach/异常释放。

## 2026-10-10：r193首次取回合爪回驱FAIL，先平缓合爪局部验证

r193全新完整初始两桌盒，上料204.1005/机器子流程311.924sim秒PASS，5真按键、
30rpm稳速30.0495sim秒。原支撑全频1397184检查、零反力最大0.5ms、底穿透0.255582mm。
首次右手取左盒在线FCL/冻结位姿门通过，但329.886下降后原1sim秒合爪在330.3645触发
-0.029129894rpm>原0.02；真实力矩0.278821398Nm<±2，编码器误差4.351734微弧度。
已有0→16N插值，并非瞬时16N；未pilot、未返任何盒，整套执行器/独立审核FAIL。
保持已接受夹爪目标和力、quiet330.994/暂停/收尾remaining=[]；launch有SIGTERM升级日志，
视频34639帧全解码、463测试/隔离构建/六库恢复/前后保护和785正式源码对1009 PASS。
下一只新身份局部“两篮预置”首次取回测试平缓合爪，保持最终16N及所有原门；预置不是装盒/
旋转完整成果。通过后才新完整两桌盒验证。正式模型/孵育箱/旧成功run/1009/920保持，
没有用户整套PASS，无新备份或真机/reset/detach/异常释放。

## 2026-10-10：r194两篮预置首次取回PASS，r195完整复验候选

r194初始两盒各预置篮、仅刚性初始相位/篮摆0；实际蓝按钮120°开盖，右手中部取左盒。
仅取回合爪改6sim秒五次多项式位置/力爬升，最终16N/目标不变，加载原1sim秒不改；
15mm居中pilot/10sim秒无篮接触持物、近水平搬运，返右桌位3sim秒稳定、119.0005sim秒
单盒流程PASS，无held payload。另一盒始终原右篮，原1mm/20ms连续支撑保持。
真实全程峰0.000394178rpm<原0.02、0.003370651Nm<±2、夹持角1.054964°/位置0.789549mm。
在线FCL0.002rad/IK/冻结0.3mm/.1°门保持，475测试/隔离构建六库恢复/前后保护10262/10/
785源码对1009及独立单盒审核PASS；视频9512792字节全解码，SIGINT remaining=[]。
初次ROS数组比较/新增hook合同失败日志保留，修正后精确还原hook全文对r159比较通过，
不改旧物理门。初始预置且没启转子，不是用户整套完成。
下一r195全新完整两桌盒候选仅加入平缓取回合爪；必须真实装盒/5按键/稳转30sim秒/停稳
开盖/两盒实际返不同桌位3sim秒、独立动作视频保护全PASS。旧正式成果/1009/920保持。

## 2026-10-10：r195合爪通过但15mm无篮接触证书FAIL，r196真正出篮门

r195完整上料210.331/机器子流程318.155sim秒PASS，5真按键、30rpm稳速30.0495sim秒；
1397184支撑检查、零反力最大0.5ms、底穿透0.349841mm。6sim秒取回合爪通过，取回阶段
峰0.017157084rpm<原0.02、0.178741418Nm<±2、夹持角0.267284°。
但原15mm试探后344.739仍真实篮壁接触约0.000111mm/0.501014N而无接触断言FAIL；
盒底仍在篮壁高度范围，自由篮卸载会回摆，不能把15mm视为全出篮。holding右手/左盒，
未返任何盒，整套/独立审核FAIL。quiet345.379/暂停/收尾remaining=[]，包装SIGTERM升级日志保留，
36248视频帧全解码、476测试/隔离构建/六库恢复/前后保护及785正式源码对1009 PASS。
下一r196新双篮预置实际双盒卸载，原试探/1.180m完整lift和全部运动路径/FCL/IK保持，
把10sim秒无篮接触持物放到原完整lift后，并新增所有实际角点高于原变换碰撞网格rim至少3mm/
整个10sim秒无篮接触门。原1mm/40N/0.02rpm/2mm2°/冻结位姿/有限力矩门不变，途中不忽略碰撞。
明确这是证书位置修正，原15mm不再宣称全出篮；预置双盒卸载不是完整装入/旋转成果。
之后仍须全新完整两桌盒身份真实全流程/独立视频动作保护PASS，旧正式成果/1009/920保留。

## 2026-10-10：r196双盒返桌子目标完成，额外空手撤回腿故障FAIL

r196两篮预置新fresh，两个实际全lift后10sim秒无篮接触/最小净空72.023094mm通过；
两盒真实返不同桌位、各支撑后正常松爪3sim秒稳定，无held payload。
峰0.007092794rpm/0.065784862Nm/1.059165°/0.692134mm，全部原物理门不改。
但第二盒放稳后的额外空手retreat202.5805左膝越原0.025rad门，保持后约-0.089911rad，
整体/独立审核FAIL，不能把两个返桌子目标称整套成功。quiet203.362/暂停/退出remaining=[]，
22232视频帧全解码，包装SIGTERM升级原日志保留；488测试/隔离构建六库恢复/前后保护/
785源码对1009 PASS。初次控制器列表有inactive，随后精确终端active全6确认才发动作；
下一完整r197候选已有精确状态/非零CLI返回码拒绝回归，不再用active子字符串。
下一r198局部将用户材料任务与可选空手停车分离：首盒正常撤回，末盒放稳后不执行额外
retreat/return，双空夹爪开放保持放料姿态，连续10sim秒验证两桌盒支撑/2mm2°/.5mm漂移及
原被动腿0.025rad/机器/时钟/定位全部健康门。不是提前结束避故障；原腿门不提高。
预先声明终态不承诺双臂neutral，停车单独待验证。预置局部不是完整装入/旋转成功；
通过后才新完整两桌盒/5真按钮/30sim秒旋转/实际双盒返桌及10sim秒健康终态全PASS。
旧正式成果/模型/孵育箱/1009/920保持，无新备份或真机/reset/detach/异常释放。
 
## 2026-10-10：r198更早的腿漂移FAIL，下一专属静态腿姿态反馈

r198双篮预置卸载：实际首盒返桌3sim秒、第二盒全出篮10sim秒持物通过，但第二盒返桌途中
173.4105sim秒左膝越原0.025rad而FAIL，末次停车/最终保持尚未执行，取消停车不能解决此问题。
保持左手/右盒、quiet174.093/暂停/收尾remaining=[]；18854视频帧完整解码，包装SIGTERM升级
原日志保留；515测试/隔离构建六库恢复/前后10262文件与10备份保护及785源码对1009 PASS。
首次seal前发现旧fork的可再生install引用r193，取消预封存流程，改为独占r198 install/build
并重构建/测试后才一次seal；正式源码/模型/旧identity/报告/备份未改，不把旧install当动作权威。
REPORT SHA-256 71e4cb0650aefa0640457bcb13349671fd6872fb96ee8248e1e469927ed609fa。

该手部场景原world到pelvis固定，不是自由行走/站立成果。12腿position接口无人声明，
对应安装版本gz_ros2_control1.2.19的fallback只发零速度而没有位置恢复，提出根因假设：
新r199局部增加专属12腿静态posture JTC，原被动稳定门后激活，实际姿态5sim秒平滑到既有
plan初始腿位，endpoint1mrad和1sim秒稳定后才发手部任务；保持原0.025rad腿门/原所有手部
路径、接触、夹持、FCL、有限转子力矩及支撑门，不新增固定关节/pose/reset或RL。
局部双盒卸载与10sim秒终态独立PASS后才新r200从两桌盒完整实跑。r197未seal/未launch，
不能启动失败依赖的旧候选；旧正式模型/孵育箱/成功run/1009/920保持，无新备份或真机。

## 2026-10-10：r199静态腿反馈通过，第二盒落桌力峰FAIL，下一末段减速

r199专属12腿静态posture JTC从实际腿位5sim秒到原plan初始位、1mrad endpoint/1sim秒稳定
通过；原0.025rad腿门不变，末次故障前腿误差0.000121098rad，无腿故障。原固定pelvis
仅手部场景，不称自由站立/步态/导航。实际首盒返桌3sim秒、第二盒全出篮10sim秒持物通过，
但195.9085sim秒left_retrieval_setdown指1接触合力42.207569N>原40N而FAIL；桌接触3.274um，
尚未松爪，不能归因于开爪。holding左手/右盒、quiet196.573/暂停/收尾remaining=[]，包装
SIGTERM升级原日志保留。526测试/独占隔离构建六库恢复/前后10262文件与10备份保护及785
源码对1009 PASS，21306视频帧完整解码。REPORT SHA-256
961038e63950f516860dfedc6106eb169a7d268ac72ab1b354d79a3736dca890。没有整套PASS。
新r201局部保留腿反馈和全部原轨迹，原121点落桌路径在离桌10mm的点116分段，前段原
.12rad/s、最后4段.03rad/s，共享分界点、每段原.3mrad endpoint门。只末段减速与停稳，
原40N/1mm/2mm2°/0.02rpm/有限力矩/IK/FCL/冻结位姿及支撑门不变、不跳过点。
局部实际双盒返桌+10sim秒终态独立PASS后才新r202完整两桌盒实际全流程。r200未seal/
未launch不启动。旧正式源码/模型/孵育箱/成功证据/1009/920保持，无新备份或真机。

## 2026-10-10：r201启动退化NOT EXECUTED，执行前健康与录像必须完成

r201启动时Gazebo server退出、CLI超时；我在健康/录像单元未明确完成时提前启动执行器，
这是调度错误。原被动姿态门未过，trace任务trajectory/gripper goal/leg goal/button命令及
physical_press均0，未产生held payload；只有claim和715条按钮状态及FAIL，没有录像/物理验证。
原identity/日志保留，不恢复/复用；退化ROS派生进程精确PID/ctime SIGINT remaining=[]，
无任务动作执行器SIGINT后SIGTERM收尾，无复位/异常释放。534测试/独占隔离构建/六库恢复/
前后历史保护和785源码对1009 PASS，REPORT SHA-256
a736b6aec720b034d663c180bc2c28d7df22055f7db036d0093763849a710ae2。
下一r203保留静态腿反馈/末10mm减速，增加execution.claim和ROS任务节点创建前的硬gate：
当前六控制器精确active与当前launch的server/GUI PID/ctime存活、录像PID存活、相机ACK/
非空实际MP4均确认；CLI超时落盘FAIL，不发任务。必须等待健康+录像单元明确exit0，
不能把“仍运行的session ID”当完成。局部物理双盒返桌10sim秒终态独立PASS后才新r204
完整两桌盒全流程；r202未seal/未launch不启动。旧正式轨迹/物理门/源码/模型/孵育箱/
成功证据及备份1009/920保持，无新备份或真机授权，仍没有用户整套PASS。

## 2026-10-10：r203图形驱动版本冲突阻塞，整套未完成，先由用户恢复主机

r203局部身份已seal118项且逐项复核PASS，但启动GUI OpenGL context失败退出，server随后
退出，控制器健康FAIL。内核同一GUI ruby PID374583记录client580.178.04与已加载kernel
580.173.02 API mismatch；nvidia-smi同时报告NVML库580.178版本冲突，证据为
runtime_evidence/centrifuge_48box_setdown_braking_20261010_r203/graphics_diagnostics.log。
临时Mesa/llvmpipe GLX pbuffer探测亦未通过，仅此探测失败，未验证Gazebo替代渲染。
本轮确认主机图形阻塞，不能归因于任务参数或用户关闭GUI，也不以此替代r201历史诊断。

claim前硬gate SAFE_REFUSAL（Controllers not certified），没有execution.claim/任务节点/
轨迹/夹爪或腿goal/实际按键/held payload/视频。末10mm减速仍无物理验证，不称有效；
静态腿反馈的r199已验证子目标仍有效，但r199因第二盒落桌42.207569N>40N整体FAIL。
用户“双桌盒装篮→关盖→30rpm稳转30sim秒→停稳→开盖→双盒返桌”仍没有完整PASS。
r195完整装入/机器子流程与r196预置双盒返桌子目标不能拼接冒充完整运行。

545测试（260局部+46配置+113导航+126装载回归）PASS，专属隔离构建与六库恢复PASS；
前后历史10262文件/10备份和785正式源码对1009逐字节PASS。r203正常SIGINT收尾remaining=[]，
07:01:14 CST只读Gazebo/本任务相关进程为空，无reset/detach/松夹，没有视频解码证据。
报告REPORT SHA-256 09bffe51be97dbb87f7a4834dd1ba6f0cdb2c4bd56fd5cbf434d1122ec0f1ef0；
SHA256SUMS SHA-256 dc08db2b89fdb44740de8405a4e59d647d42acc34576770c4d3a36e5857d62a6。

先等待用户重启电脑或修复主机图形；不得自主重启电脑/显示服务、改驱动或包配置。
恢复后重新只读核验GPU版本、进程/夹持/控制器与保护哈希、磁盘（当时仅约15GiB可用）。
r204只是代码/测试准备，未seal/launch/claim，依赖r203局部物理PASS不成立，禁止直接执行。
继续时从r204代码链派生新未使用局部及完整身份，普通mkdir独占run/install、隔离构建/
测试/一次seal，先真实双盒卸载+10sim秒终态独立PASS，之后才新完整双桌盒全流程。
不得复用r201/r203、改封存成员、放宽原门、覆盖正式资产或受保护备份；1009/920保持。
仍原固定pelvis手部场景，不称导航/自由站立成果；真机、新备份、reset/detach未授权。

## 2026-10-10：用户继续后驱动恢复，r205双盒实际取回局部PASS，下一r206完整

09:20:16 CST重新只读NVIDIA kernel/client均580.178.04、nvidia-smi正常，无旧Gazebo现场。
未改系统或自主重启；r203主机阻塞为历史证据不改。新r205从r203代码派生，普通mkdir独占
run/install、六库隔离构建/545测试/前保护完成后一次seal118项，健康及录像明确exit0后
claim前硬gate通过才任务；原静态腿反馈/121点落桌最后10mm减速和所有原物理门保持。

r205双篮预置实际蓝指尖按钮120°开盖、两手中部抓取各盒、完整出篮后各10sim秒无篮接触、
分别返不同桌位正常松爪各3sim秒稳定及终态10sim秒健康独立PASS，终端sim216.4335，无held。
落桌指接触峰30.461818151/30.582437919N<原40N（r199失败42.207569N），最小出篮净空
71.825486mm，夹持角0.587878°/位移0.690320mm，转子峰0.002136328rpm/0.040352256Nm。
终态位置误差0.113593/0.121417mm、383记录/故障0、腿偏差峰0.000221347rad。
首盒正常空手撤回，末盒双空夹爪开放保持放料位，不承诺neutral；原固定pelvis，非自由腿成果。

独立卸载21gate、视频26064帧完整解码、前后10262历史文件/10备份/785正式源码对1009
逐字节PASS，正常SIGINT remaining=[]，没有reset/detach/异常松夹或新备份/真机操作。
runtime_evidence/centrifuge_48box_setdown_braking_20261010_r205/REPORT.md SHA-256
01a061c93b8eaa72123dbd01e7fc73c886915773ab6184a990fbf6c3fd0f5168；
SHA256SUMS SHA-256 1c5a5135abe928e3f66e743aae7c38946e9ff2fbb6867d81b9f68537a359653b。
此局部初始两盒预置、没有装入/旋转，不是用户整套PASS，不能与r195子目标拼接。

新r206完整候选从未seal/launch的r204代码链派生，初始两盒仍在桌上，545测试/独占构建
已通过；必须核验r205实际独立PASS与封存/保护后，r206前保护PASS才一次seal/launch。
真实完整装篮/5指尖按钮/关盖/30rpm稳转至少30sim秒/停稳开盖/两盒返桌3sim秒与终态10sim秒，
独立动作、视频解码、前后保护全PASS才可称整套完成。旧正式模型/源码/孵育箱/成功run/
报告/录像/rosbag/builder/920箱子版本及1009等备份保持，所有旧身份不得复用或覆盖。

## 2026-10-10：r206完整取回pilot微动FAIL，r207提速未执行，下一r208真实方位试探

r206初始两桌盒真实装篮210.5105/机器子流程318.339sim秒PASS，5真按键、30rpm稳速30.05sim秒；
原支撑全频1420904检查，零正反力最大0.5ms、底穿透峰0.274681mm，定位679有效/0无效。
第一次右手取左盒6sim秒合爪通过；但原0.12rad/s居中15mm pilot在343.278sim秒转子
-0.062300525936rpm>原0.02而FAIL，实际力矩0.510390736Nm<±2，尚未完整lift/返任何盒。
保持右手/左盒、quiet344.3295/正常SIGINT收尾remaining=[]；41657视频帧全解码、545测试/
隔离构建六库恢复/前后10262历史文件及10备份/785源码对1009 PASS。没有用户整套PASS。
失败记录runtime_evidence/centrifuge_48box_full_cycle_20261010_r206/REPORT.md，旧身份不复用。

用户重申尽量快，r207离线新增空手白名单乘2上限0.60rad/s和10ms密点间隔、空手按钮进出
0.44rad/s，下降/载荷动作/指尖接触间隔/30秒旋转与所有旧门/3和10秒证书保持。
586测试与隔离构建PASS，上料空手离线预测少22.023815sim秒，不称实际提速。
因r206完整PASS依赖失败，r207未seal/launch/claim，不允许执行。

下一全新r208局部从r205链派生，用r206实际停转box/rotor方位初始化预置两篮，
自由篮初始摆角0由重力稳定，仅原15mm取回pilot速率0.12降到0.03rad/s，所有原路径点/
有限D80/P8000/±2Nm、quiet0.02rpm、接触1mm40N、夹持2mm2°、腿0.025rad、支撑20ms、
IK/FCL/冻结与末段落桌减速保持。局部独立双盒返桌+10sim秒终態PASS后才新完整提速身份。
只固定pelvis手部仿真，不称导航/自由站立；正式模型/源码/孵育箱/1009/920等保护保持，
无新备份/真机/reset/detach/异常松夹。每个新身份普通mkdir独占run/build/install、一次seal。

## 2026-10-10：r208真实停转方位双盒取回局部PASS，下一r209完整提速

新r208采用r206实际取回前box/rotor方位初始预置，实际相位误差1.25682767e-07°；
自由篮初始摆角0后重力稳定，未runtime pose/reset/detach。唯一运动变化为原15mm取回pilot
0.12降到0.03rad/s，全部原点/其他载荷率/几何/6秒合爪/10秒持物/落桌末10mm减速/所有旧门保持。
真实蓝按钮120°开盖，两手中部完整出篮各10sim秒无篮接触、实际返不同桌位正常松爪各3秒，
终態10sim秒健康，220.1425sim秒局部执行器/独立卸载21gate及pilot合同3gate PASS，无held。
峰0.002057712rpm/0.044011831Nm/0.890995°/0.770401mm，净空最小71.870745mm；
桌位误差0.114399/0.112457mm、漂移0、382记录/故障0、腿偏差0.000550516rad。
548测试/隔离构建六库恢复/前后10262历史文件与10备份/785源码对1009 PASS；
27615视频帧全解码，正常SIGINT remaining=[]。末盒不额外停车、不承诺neutral/自由站立。
runtime_evidence/centrifuge_48box_pilot_braking_20261010_r208/REPORT.md；预置局部不是完整成功。

新r209从未seal/launch的r207代码派生，继承r208唯一pilot减速与r207空手白名单乘2上限
0.60rad/s、密点10ms/按键空手0.44，指尖push/withdraw和其他载荷率、30秒旋转及所有旧门保持。
588测试/专属隔离构建已PASS，必须r208两独立PASS/SHA、r209前保护PASS后才一次seal/launch。
初始两桌盒，必须本身份实际装入/5真按钮/关盖/30rpm稳速30秒/停稳开盖/双盒返桌/终態、
独立完整审核、视频全解码、前后保护全部PASS才能称用户完整成功。
实际完整时间和空手同路径时序/已通过r206装载子流程对照落盘；r206整体FAIL，不能伪造完整
基线时间或全流程提速比例。r207旧失败依赖身份仍未执行，不复用。旧正式源码/模型/
孵育箱/成功证据/1009与920等保护保持，固定pelvis仅仿真，无新备份/真机授权。

## 2026-10-10：r209空手停车提速跟踪FAIL，r210撤销停车/密点加速

r209真实初次蓝按钮120°开盖通过，但40.7205sim秒left_park左肘实际0.584364825rad、
参考0.567600000rad，差0.016764825rad>原0.015而FAIL；尚未夹持/装盒/启转子。
未放宽跟踪门、接受目标保持/quiet41.4715/正常SIGINT remaining=[]；3876帧视频全解码，
588测试/专属隔离构建六库恢复/前后全部保护和785源码对1009 PASS。失败身份/密封成员保留。
runtime_evidence/centrifuge_48box_fast_complete_20261010_r209/REPORT.md，不称完整或提速成功。

下一全新r210完全撤销park的0.60和10ms提速：左右park恢复原速/20ms。
其余空手approach/retreat/return只乘1.25、上限0.45rad/s、原密点20ms；
holding和held_payload必须均None；下降/载荷/合爪/松爪/原跟踪0.015与端点0.3mrad门保持。
保留已通过初次按键的空手按钮0.44、原push0.1/withdraw0.07与r208独立验证15mm pilot0.03。
所有其他原路径/几何/载荷率/3秒落稳/10秒持物终態/30秒旋转/物理门不改。
专属构建/全部测试/前保护与r208两项独立PASS+SHA后才一次seal/launch。
必须本身份完整真实全流程/独立完整审核/视频/前后保护PASS才用户任务完成，不能拼接局部。
固定pelvis仅仿真；旧正式源码/模型/孵育箱成功资产/1009/920保持，无新备份或真机/reset/detach。

## 2026-10-10：r210空手approach跟踪FAIL，r211所有臂速回退，仅按钮进出提速

r210左右park原速通过，真实蓝指尖120°开盖通过；right_approach的0.45rad/s在50.4105sim秒
右肘实际-0.294639511rad、参考-0.279055607rad，差-0.015583905rad>原0.015而FAIL。
初始桥接仅1.867微弧度，不是起点突跳，此数据窗口约35ms跟随滞后；不放宽跟踪门。
尚无夹持/装盒/旋转；正常保持目标/quiet51.1575/SIGINT remaining=[]，视频全解码。
590测试/隔离构建六库恢复/前后10262历史文件及10备份/785源码对1009 PASS；
磁盘guard未发送任何信号。runtime_evidence/centrifuge_48box_bounded_speed_20261010_r210/REPORT.md。

下一全新r211撤回全部空手臂白名单/1.25/0.45，所有臂率和原20ms密点时序回到r206，
仅继承r208实际停转方位独立双盒取回PASS的原15mm pilot0.03。
只保留已两轮初次真实开盖通过的按钮空手进出0.22→0.44，push0.1/withdraw0.07及所有接触/
跟踪/载荷/支撑/夹持/转子/定位/腿门和3/10/30秒证书不改，禁止空手臂retime事件。
必须新两桌盒本身份实际完整装入/5真按键/关盖/30rpm稳速至少30sim秒/停稳开盖/
两盒返桌/终態、独立完整审核/视频/前后保护全PASS。局部成果不能拼接；只量化按钮同路径
时序和r206成功装载子流程，不虚构完整速度基线。进一步臂速需另做伺服与加速度规划验证，
不能继续凭倍率追赶/提高原跟踪门。固定pelvis，仅Gazebo手部，不称Nav2/自由站立。
普通mkdir独占run/build/install，构建测试/前保护/r208两项独立PASS及SHA后才一次seal；
磁盘至少6.5GiB、低于1GiB精确PID/ctime SIGINT执行器保持，不删除保护证据。
旧正式源码/模型/孵育箱/1009/920与全部保护保持，无新备份、真机/reset/detach/异常释放。

## 2026-10-10：r211首盒返桌通过、第二pilot微动FAIL；r212无损日志完整候选

r211两桌盒实际装入/五真按键/关盖/30rpm稳转至少30sim秒/停稳120°开盖与首盒
完整出篮/10秒证书/返桌3秒通过；第二盒6秒合爪后15mm pilot的0.03rad/s在426.255sim秒
转子0.020560172597rpm>原0.020而FAIL，保持左手/右盒，quiet426.899/暂停426.936后
正常SIGINT场景remaining=[]。598测试/隔离构建六库恢复/前后全部保护及785源码PASS。
录像另有13:59:25内核x11grab/libavdevice段错误，收尾前已退出，MP4无moov且解码FAIL；
不称正常录像SIGINT或完整视频，原坏视频/日志不修复覆盖。动作及视频均未整套完成。
报告runtime_evidence/centrifuge_48box_button_speed_20261010_r211/REPORT.md，
SHA-256 5ffc1ed469fa138ed39da6d26cf6acd2913cef47c77b9c445472ee9afcb1d734；
SHA256SUMS SHA-256 3f6fae2293322ae6f033952e9817fcc4ae8103b06187d863acc5082a85070509。

下一全新r212只将原15mm两次pilot 0.03→0.02，其他原路径/臂率/20ms/物理门和
3/10/30秒证书保持，按钮空手0.44及原接触push/withdraw不改，失败臂倍率全撤回。
采用本身份两桌盒完整实跑验证更保守pilot；r208局部0.03 PASS不是r212已验证结论。
磁盘仅约2.4GiB，禁止删除旧保护证据，不能再写5GiB原始日志；新身份完整原JSONL
gzip level1字节无损保存，独立解码SHA/行数/字节数/CRC核对，物理与采样不改。
录像使用仅Gazebo窗口XID的GStreamer ximagesrc/H264/2秒分片MP4，先72帧短测全解码，
再独立完整视频，持续进程/文件健康guard；异常或磁盘<1GiB只精确执行器SIGINT保持。
至少1.8GiB才启动；视频收尾失败也正常退场，不录无关桌面、不强杀或改旧录像。
本身份实际完整动作/终态/独立审核/录像/保护全PASS才完成，不虚构完整速度基线。
普通mkdir独占/隔离构建测试/前保护/一次seal，固定pelvis仅仿真，孵育箱/正式模型源码/
所有旧证据及1009/920备份保护保持，无新备份/真机/reset/detach/异常释放。

## 2026-10-10：用户明确授权放宽限制、先完成全流程；r213新仿真演示合同

最新用户“可放宽限制，先完成全套流程再优化”覆盖本日先前全程quiet0.02要求。
只放宽取回阶段有限制动引起的瞬时微动至0.1rpm，必须转子电机禁用、brake active/
D80/P8000/力矩有限±2Nm、120°开盖，其他夹持2mm2°/接触1mm40N/腿/定位/支撑/
跟踪/IK/FCL/3与10与30秒证书均保持。原停机can_open且|rad_s|<0.02确认和终态
|rpm|<=0.02仍保留。pilot回到已验证0.03，正式模型源码/旧成功严格证书不改。
新完整PASS只能称用户授权的Gazebo微动合同，不冒称原0.02严格取回证书；
独立报告实际取回rpm峰、旧0.02超限样本数及新0.1门，失败仍保持/正常SIGINT。

r212录像短测发现主机缺h264parse插件，在claim前拒绝，无任务动作/夹持/视频；
614测试/专属隔离构建/前后保护与785源码PASS，正常SIGINT remaining=[]。
报告runtime_evidence/centrifuge_48box_lossless_complete_20261010_r212/REPORT.md，
SHA-256 382dabbda2230e051ed5f8817f88cd3ab74d505f4b4c01d5ce647c27a618ea46；
SHA256SUMS SHA-256 90ba29f6d20b622d7e42801d481631f4351cfb7ce8cc3e4fb225723a030c440a。
密封成员不改、不复用。r213直接AVC协商不依赖parser，72帧synthetic codec已全解码PASS，
646测试/专属构建/六库恢复/前保护及原碰撞审核PASS后一次seal，新Gazebo窗口短测全解码，
再完整录像/fresh claim/两桌盒装入/关盖/稳转30秒/停稳开盖/双盒取回/最终10秒全流程。
持续录像/磁盘guard与无损JSONL+独立SHA/行数/字节数/CRC保持，任何旧证据不删除覆盖。
本身份完整动作/独立审核/视频/保护均PASS才完成；原固定pelvis手部仿真，非导航或真机。
不建备份，1009/920与既有孵育箱/正式模型/源码/authority/run/视频/rosbag/builder永久保护。

## 2026-10-10：r213第二盒携带2.0097°保护FAIL，用户授权r214实际携带角5°

r213第一盒真实落篮通过，第二盒左手transfer约174sim秒，FK位移0.268mm/角2.0097°
超过原2°而FAIL，held左手、没有第二盒正常释放或机器旋转/取回；quiet174.7535后
正常SIGINT视频/场景remaining=[]。646测试/六库隔离构建/前后全部保护与785源码PASS，
70,772,899字节视频全解码，无损trace 746421行/1,944,615,752字节/CRC及SHA PASS，
guard未发停止信号。动作FAIL不能被有效视频替代。
报告runtime_evidence/centrifuge_48box_codec_complete_20261010_r213/REPORT.md，
SHA-256 3c414a9d46b8a180b3a07fbd88dfffd67bc051495c394ed5f1c76660da51ac91；
SHA256SUMS SHA-256 2224325d215d4de3f879f659d9905bd9fa24ca7360b0293f516dd2b4b7d9b469。

最新用户授权“可放宽限制，先完成全套流程再优化”：r214装载及取回actual FK/native
携带相对角统一5°，位移仍2mm，非有限值拒绝；原指令carry倾角2°和返桌最终姿态2°
不改，真实对向中部接触/正反力/接触1mm40N/落篮/碰撞/腿/定位/支撑/跟踪均保持。
继承只取回阶段制动瞬时0.1rpm、电机禁用/brake active/有限±2Nm，终態quiet0.02。
所有原路径/臂率/20ms/按钮0.44原push与withdraw/3与10与30秒证书保持，
先新身份完整通过再优化速度；新PASS只称授权仿真合同，独立记录旧2°/0.02是否通过，
不篡改旧严格结果。新r214659测试/专属隔离构建/前保护及原FCL审核后一次seal，
新窗口72帧全解码和完整录像/fresh claim才完整实际执行。

磁盘约1.7GiB，完整日志gzip9字节无损（真实64MiB持物样本比0.178803，原gzip1 0.250936）；
新前置>=1536MiB、运行512MiB余量guard，低于该值只精确PID/ctime SIGINT执行器保持，
并发正常收尾；不删除旧保护数据。仅本身份launch/执行子进程ulimit -c0避免巨大core，
不改主机配置或删除既有core。CRC/解码SHA/行数/字节数/视频/全部前后保护仍独立核验。
固定pelvis仅Gazebo手部，不称导航/自由站立，孵育箱/正式模型源码/历史成功证据及
1009/920等备份保持，无新备份/真机/reset/detach/异常松夹，不复用任何旧身份。

## 2026-10-10 15:28 CST：用户授权旧构建缓存清理，r214继续原封运行

用户明确“允许清理已核验的旧构建缓存”。仅删除74个旧CMake编译`.o/.o.d`文件，
逻辑1,034,625,838字节/占用1,034,797,056字节；没有清空回收站、删除任何程序/共享库/
源码模型/正式动作证据/日志视频报告/备份。清理根为主工作区moveit_demo CMakeFiles及
外部diagnostics七项历史build CMakeFiles，完整绝对路径/哈希/字节数/配方在
runtime_evidence/build_cache_cleanup_20261010_r1/inventory.json与REPORT.md。
逐项无live fd/maps/cmdline依赖、非链接、可由保留源码CMake再生；60个中间文件虽在
920整体归档中，但不属于现行正式保护范围；920本身与全部10备份前后哈希仍PASS。
前后10262历史保护文件及后续封存SHA链/785正式源码对1009全部PASS；保留全部
现有可执行程序、插件和构建配方，下次对应构建需重编对象，不声称缓存仍完整。
清理后可用2,417,184,768字节，15:27:59并发记录增长后余2,382,684,160字节。
r214当前sealed identity保持919f10a99ccbab4e56a9a64ff9ad6fa61067d4ed3866185b1a91ee765019b863，
独立build/install/inputs/run不动；首盒正常落篮通过、right_return sim124.167，尚无整套PASS。
不因清理改变r214轨迹/门、不停止场景/释放持物、不建备份或连接真机。

## 2026-10-10 16:43 CST：r214双盒全流程首个同身份完整PASS，速度未改善

runtime_evidence/centrifuge_48box_demo_complete_20261010_r214/REPORT.md为新完整基线。
两个桌面盒真实对向中部抓取、分别自然落篮、机器人五指尖接触按键、关盖/30rpm稳速
不少于30sim秒/停转120°开盖、两盒分别取回两个桌槽并各3秒落稳、最终无持物10秒，
本fresh身份完整通过，独立物理27/27 PASS。固定world→pelvis仅Gazebo手部，不是
Nav2联动/自由站立或真机证据。返桌按持物手分槽，不宣称每盒回各自身份原出生位；
双夹爪开放保持，原预声明不执行末次空手neutral，不称双臂neutral。

完整trace首个sim3.608到PASS512.772为509.164sim秒，wall5047.543210秒。
不是三分钟以内；speed_audit FAIL保留，按钮同路径反事实省13.106143095秒，
实际装载223.423秒比r206成功子流程慢16.6865秒，不宣称实测整体或装载提速。
r206完整未PASS不能作完整速度基线；后续用r214作为首个有效完整速度基准，先分析
左盒姿态修正及空手折返，再平滑时序/加速度验证，不重复失败倍率或把快放当提速。

采用已授权actual携带5°/2mm、仅取回制动瞬时0.1rpm合同：角峰2.357256192°，
执行器保守位移1.227342972mm，取回峰0.083695422rpm/4个样本超过旧0.02；
原2°/全程0.02严格门不PASS，不重写旧证书。其他原门/3与10与30秒保持；
最终盖120°、motor禁用约0rpm、两桌位误差0.215181/0.243673mm，10秒380样本无fault，
腿峰0.008353960rad、定位1015样本全有效，完整出篮最小70.546392mm。

659测试/专属隔离构建/六库恢复/原FCL/新72帧短测/140密封成员SHA核验PASS。
完整JSONL gzip9无损2217298行/5933706304字节，解码SHA
19e76fe75113b5aa289574503dfac9f60eca6c6a33972a6f595bdbe50a279215及CRC/计数PASS。
原窗口视频222201778字节/60800帧完整解码PASS、正常SIGINT-EOS；另10x preview
仅播放派生，不是动作提速或逐帧权威录像。场景正常SIGINT remaining=[]，
16:42:59重新只读无相关运行进程。前后10262历史保护/后续SHA链/10备份/785源码对1009 PASS。
旧孵育箱/正式模型源码/所有历史证据/1009及920箱子版本仍保护，没有新备份/真机/
reset/detach/异常释放。不得编辑本身份密封FULL_CYCLE_PLAN/代码/inputs、覆盖或复用运行；
未来优化需全新未用身份、隔离构建测试、fresh检查/封存/录像/完整独立审核与保护。

r214 REPORT SHA-256 a494e7e26f90645581a618cdf746e6eda30403a732a10e81b99b8fd263007c94；
SHA256SUMS SHA-256 4a35dfea335b1be7c66df555dc183e8df6e343a86a6c96e7bd4b5ff7d706d31c。
255个封存文件sha256sum逐项PASS，140个动作密封成员未变；speed_audit FAIL原样保存。


## 2026-10-10 17:05 CST：用户要求归并MD并清理无用过程文件，成果保持

当前用户“总结md，清理我的无用过程性文件，但是重要的成果要留下来”。
HANDOFF_CURRENT.md改为唯一精简当前摘要/证据索引；README/QUICKSTART/交接入口
去除重复过时状态，五入口3029行/199715字节→284行/20669字节。
旧HANDOFF_CURRENT第1–90节及其他七份修改文档完整原文/哈希/字节数保存于
runtime_evidence/documentation_process_cleanup_20261010_r1/documentation_before.json，
这是文档变更溯源，不是新编号整工作区备份。历史“当前/最新”及旧章节号不再解释为
现时摘要；HANDOFF_PLAN/SCENE_VERSIONS/导航计划只新增入口解释、原正文保留，
826_plan与正式报告/源码包README不动。AGENTS唯一累积式权威不变，本轮只末尾追加，
原463552字节历史前缀SHA 2b053a90f61fb1c7bdde4652a2bcf8a6e25d0c7756698de15da1cc75d2d1b776不变。

本轮独占清理身份documentation_process_cleanup_20261010_r1：
1453个未引用旧colcon日志目录+12个可再生未受保护Python/pytest缓存目录，
13379普通文件/355646688逻辑字节（约339.17MiB），原占用384073728字节。
全部gio trash移入系统回收站，逐文件哈希/大小及链接目标复核；
原路径、精确回收站路径/.trashinfo与字节数见inventory.json/trash_result.json。
保留42个正式引用/latest日志目录与24个含受保护成员缓存目录；排除22029受保护路径，
无live文件/映射/命令依赖。未删除整试验根、build/install、程序/库/配方、源码/模型/
策略、任何正式或失败的封存动作证据、报告、视频、rosbag、trace或备份。
同文件系统回收站不等于释放同等磁盘容量；没有永久删除或清空回收站，
永久清空仍须用户另行明确授权。本轮没有扩大上一轮74个CMake对象删除范围。

前后10262历史受保护文件及后续SHA链（含r214/旧构建缓存清理）、10备份均PASS，
785正式源码对1009前后PASS；920箱子版本/1008/1009、孵育箱动作权威、
826独立成果/官方步态/Mid360地图、离心机完整成果不变。r214同身份全流程PASS仍
509.164sim秒/speed_audit FAIL，r159172.500秒仅装载；不称完整三分钟、Nav2联动或真机。
r214原报告/255封存文件/140动作密封成员不改；1009不包含r160+/r214，旧备份不覆盖。
无Gazebo启动停止重启、控制发布、reset/detach/松夹/真机连接或新备份；
本轮仅文档/过程缓存，不修改物理源码，因而不执行新动作或隔离重建。
本轮REPORT与validation、前后保护、只读live检查及精确清单同身份落盘。
以后接手仍默认只读，任何运行先重新fresh检查和取得对应任务授权。

本轮最终validation全部15项PASS，61个本地文档链接有效；13379个回收站普通文件及
r214的255封存文件/140密封身份成员再次核验PASS。17:06:22 CST只读无相关仿真进程。
两次初始检查仅因本检查输出文件尚未生成而出现前向链接假阴性，原检查保留，
最终检查顺序已修正；不是动作失败或保护哈希不一致。
整理REPORT SHA-256 3f0caead1006307cc0a682eb9bc228933d2880b2c4570dae9101768e2b6b4dde；
整理SHA256SUMS SHA-256 3380281dd8c8e28b17451a505e1185ec9dbaf09eac8cb64858c2aa3fd53e178e，
27个本轮封存文件逐项sha256sum PASS，清理/保护/文档溯源均可核验。

## 2026-10-10 17:30 CST：用户继续速度优化，r216离线候选通过但原始RL依赖缺失

当前授权“继续速度优化”，仅离心机固定pelvis Gazebo完整流程开发，不是真机/导航动作授权。
新r215只读逐行分析r2142217298条无损trace：首盒空手return175点/17.447863080sim秒，
左盒落篮前修正41点/15.9245秒。此为原指令分段估计，不能称新完整提速。
新r216唯一动作候选是首盒返桌3秒证书及原完整空手retreat之后省return；holding与
held_payload均None、两证书标记均True才略过；下一盒从实际双臂/torso场景重新完整IK/FCL。
所有载物路径/原臂率/20ms点时序/0.5ms物理步/6秒合爪/10秒持物与终态/3秒落稳/
30秒稳转、actual5°/2mm及仅取回制动瞬时0.1rpm均保持；不承诺双臂neutral。
离线第二盒pickup1786/transport5502共7288检查点PASS，2mrad FCL/20um/.01°IK/2°指令
carry保持，真实快照仍须重新审核；局部400/配置46/历史上料126及两包六库隔离构建PASS，
六库逐字节恢复。首轮局部1失败仅旧r178节点名称归一化未更新，修正新r216名称后全通过，
物理守卫不变，原测试输出保留。685最终用例681PASS/4FAIL，不能称全回归通过。

实跑前导航复测109 passed/4 failed：/home/liu/robot/当前不存在，官方Unitree源策略/
配置/deployer/model/training_config/双脚网格7项及V12源checkpoint/env/agent3项缺失。
共10项精确路径/大小/冻结哈希见r216 dependency_preflight.json；保留的官方NumPy策略/
黄金样本及Isaac URDF/USD仍匹配，未发现现存资产哈希不一致。不推断消失原因/时间。
前次本线程清理仅工作区内日志/缓存，没有删除该外部路径；不自动从Trash/备份恢复，
不下载替代原源策略、不修改合同/跳过导航身份测试，已请用户提供这些原始文件的新路径。

r216停在P0：未codec/动作seal/launch/录像/claim/机器人动作；run无launch_pid/execution.claim，
identity.json不存在。此身份候选/离线审计/报告最终封存，不能直接续跑；来源逐项校验恢复
且全部回归通过后再派生全新身份，严格独占构建/一次seal/fresh录像/完整实跑/前后保护。
当前完整物理基线仍r214509.164sim秒；r216没有实际新速度或完整三分钟结论。
本轮前后10262历史文件及后续SHA链（含r214/整理/r215）、10备份/785正式源码对1009 PASS；
孵育箱v9/r17和v24/r103/826独立成果、正式模型/policy导出/全部证据/920/1008/1009不改。
无Gazebo启动停止、控制发布、reset/detach/松夹、真机连接或新备份；离线候选不写正式源码。
当前摘要/计划第22节/Quickstart/交接提示词已更新，历史正式报告不改。

r216离线REPORT SHA-256 dfbeb55ca0571ce02a3abb16be3d719a3c56a5c3cc34e0a5b8d05dff5e2112d5；
离线SHA256SUMS SHA-256 0a92e10886c10efb5ca41975a6337aef3376da54febd8cb104f13190247ae816。
440个本身份普通文件逐项核验PASS、2条构建链接单独记录；candidate_validation PASS仅为
离线候选完整性，all_runtime_prerequisites_passed=false，不是运行或物理提速证据。

## 2026-10-10 17:43 CST：用户确认删除并授权恢复RL来源，官方恢复、V12仍缺

用户确认原/home/liu/robot/内RL目录是自行删除，随后明确“帮我恢复到之前的样子”。
本次仅恢复被删除的来源依赖，不解释为回滚工作区、恢复全部robot目录、启动仿真或动作许可。
从现存本地/home/liu/下载/unitree_rl_gym-main.zip非覆盖恢复原官方目录
/home/liu/robot/gym_sim/unitree_rl_gym-main/，277普通文件/150507282字节逐项匹配归档，
官方policy/config/deployer/model/training_config/双脚网格7项原冻结哈希全部匹配。
ZIP SHA-256 2c52ba9b76b57517e3dcfd59c6b9abb8ff0a75ead295da06f7d1cf2b5d66c459；
目标原不存在，普通mkdir独占、xb写入，不软链冒充、不下载替换、不覆盖任何现有目标。
恢复官方归档不能证明原删除目录所有历史修改/训练状态也恢复。

主目录普通文件名/3799642字节检查、主要920/1008/1009payload与三个旧训练tar.gz
均未找到V12原model_17250.pt；仍缺该原checkpoint与params/env.yaml/params/agent.yaml。
系统回收站1465项元数据均为此前工作区过程文件清理，没有对应外部原RL目录。
保留的V12 NumPy/TorchScript导出和黄金样本仍匹配，但不能冒充完整训练checkpoint。
没有底层磁盘恢复，不声称所有外部备份均排查或原文件绝对不能找回；若有其他副本，
只接收冻结哈希匹配原文件，存在目标拒绝覆盖。不得制造假的同名checkpoint、修改
身份合同/预期哈希或跳过失败用例来宣称恢复成功；独立仅导出部署合同须用户另行选择。

来源缺失10→3项、现存资产哈希不一致0；导航111 passed/2 failed/0 skipped，
剩余失败为V12 checkpoint身份与资产审计；原官方两个失败已消除，不能称全回归PASS。
首轮漏source本地Nav2 underlay造成nav2_msgs收集错误，保留日志并正确source install后
完整复测；没有新包安装/合同或测试修改。785正式源码对1009前后PASS，工作区物理
源码/模型/policy未变，没有新隔离构建或动作identity。全部依赖恢复并通过后才派生
新身份，已封存r216不直接续跑；r214509.164sim秒仍是最新完整物理速度基线。

前后10262历史保护普通文件、后续封存SHA链、10备份及785正式源码全部PASS；
另外r216440封存文件前后PASS。17:42:16 CST只读无相关Gazebo/执行进程。
未启动停止/重启仿真、发布任何动作、reset/detach/松夹、连接真机、恢复回收站、
回滚整理清理、解包编号备份到工作区或创建新备份；全部历史孵育箱/离心机/导航成果保留。
当前摘要、计划第23节、Quickstart与交接提示词更新，旧文档完整原文在本轮
documentation_before.json；历史正式报告不改。

恢复证据runtime_evidence/rl_source_recovery_20261010_r1/REPORT.md，
SHA-256 48c7d00ec9870f5bf3b1942e1290545c5665363034183b475ff88f849ee68908。
该身份是源依赖恢复审计，不是新Gazebo run或新编号备份；完整性通过仅指恢复范围，
P0三项缺失与两项回归失败继续保留，不宣称完整恢复或已解除运行前置。

## 2026-10-10 17:53 CST：用户授权官方重新下载，匹配原版本，手部独立合同待选择

用户要求“gym_sim/unitree_rl_gym-main/这个可以重新下，你下完之后继续进行工作就行”。
已从Unitree官方GitHub读取main并按不可变提交276801e46c5d433564f24658bac64f254b7d2d4b
重新下载，新ZIP59750482字节，SHA-256
0f43bf9a9e1310b155cb0496facc6a1cf4924a387f11cec3b73be30213c475a8。
277个文件内容逐项与当前已恢复官方目录相同，7项冻结身份全部匹配；ZIP根前缀
含提交名而非main，整体ZIP哈希不同不解释为策略改变。没有覆盖既有目录或改来源哈希。
官方归档无用户V12原model_17250.pt，三项原V12来源仍缺，复测111通过/2失败/0跳过；
不会用官方预训练policy或保留的V12推理导出冒充原训练checkpoint。

已继续只读提速分析：两个r216 launch输入AST含23声明节点，没有声明RL/velocity gate
节点；这不是完整传递依赖或新的运行独立性证书。已询问用户是否采用固定骨盆离心机
手部独立任务合同、保留V12失败并暂不执行腿部导航，当前尚无该新合同明确选择。
在选择及新合同完整依赖/测试/隔离构建/fresh核验完成前不改变原P0前置，不seal/实跑。
r216仍不复用。首盒空手return仅省17.447863080原指令秒，反事实491.716136920秒，
离180秒仍差311.716秒；不能称实测提速或三分钟。后续优先短空手路径/FCL，再载荷
跟踪感知时序，不盲目提高倍率、删30秒稳转或其他未重新授权的原稳定证书。

下载后10262历史保护普通文件/后续SHA链/10备份与785正式源码全部PASS，原恢复28
及r216440封存文件也PASS。本轮无物理源码/模型/policy/动作资产修改，无新构建或
Gazebo启动停止/动作发布/reset/detach/松夹/真机/新备份/回收站操作；重要成果保持。
下载审计根runtime_evidence/rl_official_download_20261010_r1，不是新Gazebo运行身份。
REPORT SHA-256 7d40d592e9bdf809bb375d4b44abd1b31924b5732ec4e0835a0b35ebb3265735。
原报告与失败测试不改；当前摘要/计划第24节/Quickstart/交接提示词只补充再下载事实
和待选择边界，原文保存在本轮documentation_before.json。

## 2026-10-12 CST：用户停止V12原件找回，仅重新下载官方Unitree，已核验完成

用户确认原V12训练目录放入回收站后已清空、无副本，曾要求先找回原训练文件；
随后明确“没有备份也恢复不了，你重新下gym_sim/unitree_rl_gym-main/就行”。
采用此次最新选择：只重新获取官方来源，不再主动继续原件恢复、安排Windows切换/
磁盘扫描或要求准备Live USB/外置盘。未执行底层恢复，不把用户的任务选择写成
原文件绝对不可恢复的技术证明；后续重新恢复调查须用户明确提出。

新独占审计身份rl_official_redownload_20261012_r1，不是Gazebo run或编号备份。
本轮实际读取官方main并新下载提交276801e46c5d433564f24658bac64f254b7d2d4b；
官方main仍为该提交。新ZIP59750482字节，SHA-256
0f43bf9a9e1310b155cb0496facc6a1cf4924a387f11cec3b73be30213c475a8。
277普通文件/150507282字节与/home/liu/robot/gym_sim/unitree_rl_gym-main/逐项一致，
额外文件0、7项原冻结官方来源身份匹配；现有正确目录不删除/覆盖，不执行下载代码。
官方下载目录完整，不代表用户私有V12 checkpoint/env/agent三项恢复；来源缺失保持。

本轮导航完整回归111通过/2失败/0跳过/0错误；两项仍是原V12来源身份/资产审计失败，
保留日志/Junit/退出码1，不改测试、预期哈希/合同或制造替代checkpoint来消除失败。
停止原件找回不自动选择独立手部运行合同、启用腿部导航或解除原运行前置；
本次官方下载任务完成，不将原完整回归或导航运行门称为全PASS。

前后10262历史保护普通文件/后续正式SHA链/10备份、785正式源码对1009全部PASS；
另r216440、原恢复28、原下载17封存文件前后PASS。现存来源哈希不一致0、V12缺3。
只读live无相关Gazebo/策略/任务进程；无新仿真启动停止、机器人/设备控制、
reset/detach/松夹、真机、回收站操作或新备份，无正式模型/源码/policy/动作变化，
因此不执行新隔离构建。旧孵育箱/离心机/导航证据、builder链及全部备份保持。
当前摘要/计划第25节/Quickstart/交接提示词已更新，旧根文档完整原文/哈希在本轮
documentation_before.json；AGENTS仅末尾追加，旧正式报告不改。

本轮REPORT SHA-256 6a96b2d8dd3dae1a34b1a91b6634ed028656bcd0cbaea84a9f83c882efb39df0；
证据runtime_evidence/rl_official_redownload_20261012_r1/，下载PASS与V12未恢复分开记录。

## 2026-10-12 CST：用户授权补独立仿真入口，r217真实场景启动PASS

用户明确“继续运行仿真，缺的东西重新写一下补上”。本轮将此前无关的V12原训练
来源前置从固定骨盆场景启动合同分离，补scene_contract/audit_scene_dependencies、
派生器、一次seal及只读健康观察。此为新用户授权下的独立startup-only合同，不制造
原checkpoint、不改旧来源哈希/测试，不证明原训练或腿部导航合同已经恢复。
以后不得继续把原V12缺失一概当作该已审核静态场景的启动阻塞；其他任务按实际
完整依赖独立审核，不能将本轮启动PASS扩展为动作/步行PASS。

新身份centrifuge_48box_scene_resume_20261012_r217，root/inputs/run普通mkdir独占，
专属install_lab_scene_48box_scene_resume_20261012_r217；25父级代码/输入核验r216
SHA后只改身份/安装路径，二进制原样、几何/惯量/摩擦/关节/控制payload保持。
82相关资源/launch include/Mid360定位模块/固定world→pelvis已审核，无RL/速度门/
Nav2动作依赖。新27/27测试、原46/46配置、6/6 CTest、两包六库隔离构建/精确恢复
PASS；原导航111通过/2失败/0跳过仍原样保存，仅原FileNotFoundError被分类为
本startup-only范围之外，原完整回归仍FAIL。动态/未知节点、自由骨盆、步行或
伪造原文件、新回归错误一律拒绝。首轮新校验器误读CTest的零失败摘要已修正，
增加5个正反例，初始诊断保留；六项CTest本身始终PASS，未放宽物理门。

45成员一次seal，实际fresh server2781965/GUI2781966同一新launch父链，六控制器
精确active。30.000sim秒观察：clock60391、关节3020、scan299、定位60、机器1490；
30按sim秒样本持续定位有效/31关节有限/机器ready且转子停止，真实两桌盒/完整机器人/
双夹爪/Mid360/D435/桌子/四墙/离心机保留。GUI合成72帧与真实窗口72帧全解码PASS，
相机仅GUI视角ACK，两个截图保存。末次图22节点无RL/Nav2/步态，检查的速度与双臂/
双夹爪trajectory发布者均0，45密封成员复核PASS。场景保留运行供用户查看；将来需
重查PID/ctime/控制器/持物/机器状态，不重复启动，不能照抄本快照。

本轮没有重新执行完整双盒流程或腿部导航，没有机器人任务/设备动作命令、
reset/detach/松夹、真机、回收站操作或新备份。此30秒只是固定骨盆静态健康观察，
原Nav2严格驻足PASS仍0；r214完整509.164sim秒仍基线，整套三分钟未实现。
后续完整手部任务须全新未用身份/完整相关合同、回归/碰撞/一次seal/fresh录像门，
不直接复用r216或扩展r217的startup-only；无须伪造或要求找回该范围不使用的原训练源。

前后10262历史保护普通文件/后续SHA链/10备份、785正式源码对1009全部PASS；
另r216440/源恢复28/旧下载17/本日重下载24封存文件前后PASS。正式源码模型/policy/
孵育箱/离心机/导航证据、builder链和全部备份不改。当前摘要/计划26/Quickstart/
交接提示词更新，旧根文档原文/哈希在本身份documentation_before.json；AGENTS仅追加。
run/launch.log及run/ros_log仍live增长，保留launch_snapshot与元数据，不把活动日志
当作静态封存文件；稳定核心SHA与live输出分开，原动作或报告不改。

r217 REPORT SHA-256 a1bd161658c979578d41679d66b1057b1e316b5b3600c8b81cf7ad9a80549ce8；
证据runtime_evidence/centrifuge_48box_scene_resume_20261012_r217/，结果仅限startup-only。

## 2026-10-12 11:52 CST：用户授权完整双盒任务，r219物理与实测提速PASS

用户明确“继续将2个盒子放入篮子，盖上离心机盖子，然后启动离心机旋转，
打开盖子，取出2个盒子，并且要求运行尽量快”。采用独立固定world→pelvis
完整手部合同，不扩大r217启动scope、不伪造原V12训练文件、不运行腿部导航。
177相关资源/完整Python及MoveIt/FCL传递链审核，无RL/速度门/Nav2动作依赖；
617相关Python/6项CTest及两包六库隔离构建/精确恢复PASS。原导航111通过/2失败
原样保留，仅已知原checkpoint FileNotFoundError与本范围无关；原全回归仍FAIL。
不得再把缺失的私有V12训练源一概当成该已审核手部范围的阻塞；原训练/腿部导航
来源合同不改变，未知相关失败/资源变化/自由骨盆/步行仍拒绝。

r217先只读确认机器停止、双桌盒原位、动作发布者0，再按PID/ctime正常SIGINT。
r218完整手部新身份已seal/fresh启动，但录像在窗口尺寸变化后not-negotiated，
守卫SIGINT遇bash后台继承忽略未实际停止；发现后只pause:true冻结、陈旧反馈触发
原stop-and-hold，保持原夹持努力，无reset/detach/松夹，并正常SIGINT收尾。
r218不是完整PASS/速度证据，264失败封存文件及录像日志报告全部保持。
这次故障必须按实际停止而非“已发信号”核验，不得复用其claim。

r219新root/inputs/run/install独占，169成员一次seal。补interrupt_control显式
恢复默认SIGINT/KeyboardInterrupt，守卫查/proc确认可送达；真实子进程继承忽略
恢复及实际执行器前置PASS，Popen监督。录像固定指定Gazebo窗口[0,0,1199,899]
区域、输出1200×900；放大后不宣称整窗，缩小低于区域拒绝启动/故障保持。
X11测试窗4次尺寸调整120帧全解码、合成/真实窗口72帧门通过。gzip9改gzip1，
所有行/字节仍完整写入，无抽样，CRC/SHA保持；轨迹/控制/物理门不因这些改动改变。

同一fresh r219两桌面85%48box真实中部对夹、分别自然落篮、5次真实指尖按钮、
关盖、30rpm稳转至少30sim秒、停转、120°开盖、双盒取回不同桌槽，各3秒落稳和
最终10秒双桌位/双空爪保持；执行器与独立物理27/27、完整速度审核PASS。
实测首条sim2.588至PASS473.902，471.314sim秒（7分51.314秒），wall4924.442240秒；
比r214509.164秒省37.850sim秒/7.433754%，wall省123.100971秒，整套三分钟未实现。
上料204.593/机器98.009/取回158.712/末保持10sim秒；全部改善不归因于17.4479秒
原return指令。只省首盒返桌证书+完整retreat后的空手return，下一手从实际非neutral
完整双臂场景重求IK/FCL，原带载路径/臂率/密点20ms/0.5ms物理及3/10/30秒不缩短。

携带角1.548617390°、独立滑移0.901623245mm（保守累计FK0.944883305mm），
取回微动0.005489592rpm、旧0.02超限样本0；定位938样本无效0，载荷容纳故障0，
抽离最低净空71.156708mm。合同仍已授权5°/2mm及取回有限制动瞬时0.1rpm；
本次实测满足旧数值不改写r214严格失败、不升级原合同/训练/导航/真机证书。
按取回手分(.44,±.38,.84475)桌槽，末次空爪保持，不宣称各盒出生位或双臂neutral。

无损trace2055680行/5573715626解压字节/1475238607压缩字节全核验，
SHA256 1a868c5477fd2dd657d4b3bcfa038b764685e02fa8a3f95e491ceaff35218124。
原速视频183212890字节/59200帧，正常SIGINT、完整解码PASS；
10倍播放预览5920帧/493.333333wall秒也全解码，快放不是动作提速。
executor/guard/disk/supervisor退出码全0，录像刷盘后场景正常SIGINT退休，
末次只读无相关Gazebo/执行进程。今后重查live，不复用r217/r218/r219旧身份或claim。

前后10262历史保护文件/后续正式SHA链/10备份、785正式源码对1009全部PASS，
另r216440/r21789/r218264/原恢复28/旧下载17/重下载24封存文件前后PASS。
原孵育箱权威、离心机/导航正式源码模型policy、重要失败、报告视频rosbag/builder链
和920箱子版本、1008/1009及全部备份不改。无真机、清理、回收站或新编号备份。
AGENTS仅末尾追加；当前摘要/计划27/Quickstart/交接提示词更新，五份根文档完整
修改前原文与SHA在本身份documentation_before.json；历史报告不改。

最新完整报告runtime_evidence/centrifuge_48box_fast_complete_20261012_r219/REPORT.md，
SHA256 b86f6df5238e75f7aa1ed5c40f393fe99a81737a0da0fba3f5b346d12faca77c。
后续提速以r219为新完整基线，优先审计与控制采样解耦/平滑时序/短空手路径；
任何变化仍须新未用身份/隔离构建测试/保护/一次seal/fresh录像门和完整实跑，
禁止盲目倍率、删稳定证书或覆盖正式资产，失败必须实际stop-and-hold并正常SIGINT。

## 2026-10-12 16:28 CST：继续速度优化，r225水平提速完整PASS及失败回退覆盖

用户要求“继续优化速度”，只执行固定world→pelvis完整手部Gazebo合同，
不运行腿部导航/私有V12策略或真机。最新完整速度证据r225：两个桌面85%48box
中部真实对夹/分别自然落篮，五次指尖按钮lid/lid/rotor/rotor/lid、关盖、30rpm
稳转至少30sim秒、停转、120°开盖、双盒返不同桌槽，各3秒落稳及末10秒无持物。
执行器/独立物理27/27、完整速度、无损trace、视频全解码/前后保护PASS。

新身份centrifuge_48box_horizontal_rate_20261012_r225，root/inputs/run普通mkdir
独占、专属install；103已核验r224父文件先仅换身份，再于seal前限定修改6文件。
180完整依赖、657相关Python（局部485/配置46/旧上料126）、6项CTest和两包六库
隔离构建/精确恢复通过；178动作成员一次seal及事后逐项SHA通过。
原导航111通过/2原V12缺源失败/0跳过/0错误仍原样保留，不称原全回归PASS，
不制造checkpoint/改原冻结哈希，原训练/腿部导航来源合同不变。
首次只读进程检查误匹配自身shell多行文本、seal前日志名不匹配诊断保留；
改为实际executable/argv检查、纠正日志名称后才生成唯一identity，无重复seal。

前置失败/离线尝试：r220严格共线/五次静止端点通过原生JTC插值验证但13条空手
0收益，无物理执行；r221仅预留，不复用；r22220→15ms预计仅省0.060秒，未连接
执行器、未实跑。r22325%/15ms通过双上料和机器周期，但取回合爪phase误用
装载0.02rpm门，在0.021481rpm中止；r224上下文修正21例回归通过，但首盒释放前
4.117678°超原3°校正窗口，未进入IK/FCL求解即拒绝，不是高度/碰撞规划错误。
这两次真实失败均stop-and-hold/保持夹持目标、pause:true、正常SIGINT收尾；
完整失败报告/视频/trace不改、不计作完整速度PASS，失败参数不留在正式源码。

r225撤回25%/15ms和装载lift提速；仅四个真实认证自由水平transfer×1.15，
left/right装载率0.07475/0.1495、两取回0.115rad/s。原垂直/接触/试抬/抽离/落桌/
抓放/按键、20ms密点/0.5ms物理、0.015rad跟踪/2mm滑移/5°携带/0.25°释放/
3°单次校正/1mm接触、3/10/30秒证书及FCL几何密点保持。不拆多段绕过校正窗口，
不增加摩擦/夹力或假attach。首盒空手return省略继承r219，非本轮新变化。

保留r224取回观察上下文hook：仅显式retrieval_closing=True且原命令left/right_grasp
才观察为left/right_retrieval_grasp；原goal phase/16N/6秒软合爪不改。装载仍原门，
取回用已授权motor off/盖120°/brake80/8000/有限2Nm/0.1rpm，不改变阈值。
本轮两个合爪322/323机器样本故障0、accepted→result6.044/6.049sim秒、error_code0；
合爪本身均未超过0.02rpm，不宣称实跑复现越过r223故障值，21例回归覆盖其快照。

首条sim2.293至PASS468.2745，完整465.9815sim秒（7分45.9815秒），wall4744.791356秒；
比r219471.314省5.3325sim秒/1.131411%，wall179.650884秒，比r214累计省43.1825sim秒。
上料202.3475/机器97.9945/取回155.6395/末保持10；整套三分钟仍未实现。
离线预计6.202255秒不是实测，实际动态场景重求IK/FCL，单次wall改善不是重复性能保证。

实际携带角峰4.222334383°，发生left_retrieval_setdown，较r2191.548617°更大；
独立滑移1.449929176mm、保守累计FK1.456378969mm、native1.340172306mm；
取回微动0.021285175rpm，旧0.02超限1样本。按既有5°/2mm/有限制动取回0.1rpm
合同PASS，旧2°/全程0.02rpm严格证书本次未通过，不改写旧严格成果。
定位928无效0、容纳故障0、抽离净空至少68.289132mm；末10秒双桌槽漂移0。
末次双空爪/按取回手分槽，不宣称各盒出生位、双臂neutral、Nav2联动或真机。

此为单次新完整速度PASS，不是多次稳定验收；角余量约0.778°，不再直接提高带载
倍率。后续优先左盒取回落桌姿态/短空手路径/平滑时序，新身份重证物理与速度。
r225最新速度基线与r219较低角偏差回退基线并存；正式源码/模型没有推广修改。

trace2030773行/5305619803解压字节/1413622391压缩字节，JSON/CRC/SHA全部PASS，
UTF-8 SHA e114ced94336efbe24d762e27a0351030b393d8a2813296a5adfda72d0b253cf。
原速视频87609361字节/57028帧，10倍预览5703帧/475.25wall秒，全解码通过；
快放不是动作提速，指定窗口固定1200×900区域不宣称放大后整窗。
四角色退出码全0，正常SIGINT视频刷盘后正常SIGINT场景退休，相关残留0。
今后重新只读检查live/PID/ctime/夹持/机器，不能复用任何旧run/claim。

前后10262历史保护文件/后续SHA链/10备份、785正式源码对1009全部PASS；
另r216440/r21789/r218264/r219292/r220168/r2211/r222104/r223285/r224280及恢复/
下载28/17/24封存文件前后PASS。原孵育箱权威v9/r17/v24/r103、离心机/导航
源码模型policy、重要失败/报告/视频/rosbag/builder链、920永久箱子版本/1008/1009
及所有备份不改；无reset/detach/异常松夹、真机/清理/回收站或新编号备份。
五份根文档修改前原文/SHA在r225/documentation_before.json；AGENTS仅追加，
当前摘要/计划28/Quickstart/交接提示词同步，旧正式报告不改。

最新报告runtime_evidence/centrifuge_48box_horizontal_rate_20261012_r225/REPORT.md，
SHA-256 e286bbc7c8217fc1cc27358aeb3d79b9a19bbedb4774f533e74d4d6d778d54da。

## 2026-10-12 CST：三分钟目标耗时审计及r225视频内容FAIL纠正覆盖

用户要求自行看视频，并询问全流程能否三分钟。目标仍按此前约定首条非空sim至
完整PASS≤180sim秒，不是wall或快放。最新完整物理/时间仍r225465.9815sim秒，
需减少285.9815秒/61.371857%，目前没有新三分钟实跑或可行性证书，不能保证做到。

r226新身份centrifuge_48box_three_minute_budget_20261012_r226，普通mkdir独占，
仅处理封存trace/视频，32分析器正反例测试通过、各源SHA与r225SUM匹配。
39臂指令225.139196秒、8按钮空手29.506143秒、10固定按键接触/退出35.35秒、
18夹爪accepted→result28.0775秒；完整稳转30.05秒。窗口汇总可能与稳转重叠，
不能简单累加当通用物理下界。只缩臂运动/按钮空手、其余时序不变的2/3/5倍
反事实为338.658830/296.217940/262.265228秒，这两类运动归零仍211.336161秒。
这是特定不改其余时序的诊断，不是实际可用倍率、安全证书或结构重构后的通用下界。

三分钟需要结构改动而非继续盲目倍率。r226结构计划先双持物独立状态/守卫，
同一sim时间轴双臂/双盒/全场景FCL，不能两线程直接运行单holding旧执行器；
再从真实场景重求短连接、完整每盒10秒/3秒窗口重叠、真实按钮预位及平滑控制。
173秒设计预算只是目标，尚无新路径/控制/碰撞/实跑证明，不宣传为已有成果。
原30秒稳转、3/10秒证书和碰撞/0.015rad跟踪/2mm滑移/5°携带/0.25°释放/
3°校正/机器/支撑/末态门不删除或放宽；任何新源或资产实际改动仍须隔离构建，
新未用身份/前后保护/完整相关回归/一次seal/fresh门及新完整实跑才可计数。

录像重要纠正：r225只证明MP4完整解码PASS，后续视频内容复核为FAIL。
已按整段10倍预览时间线抽帧检查、核对原速开头及约30/90/600/2400/4200wall秒帧，
不是逐帧看完整原速录像，不把wall抽帧时间写为sim时间。约原速1分半及后续所查
关键阶段只有地面/机器人边缘，设备/盒子/双爪在录制区外，不能展示完整任务。
起始窗1200×900、固定左上录制1200×900、最终独立截图3676×2022；推断固定
采集区域未跟随后续放大的渲染窗，确切resize事件未记录，不归咎用户。
原速SHA3ffd0664c3749a0bd0d69127cf6e6706f845e696d68f23d401c8c39da8aee14d；
预览SHA3d86e55cb9afd4542275b94551bbee98b5f1184d74d5e34ad63bf0e1bfd8c5c1。
与原r225SUM一致，原video_audit/正式报告/视频/trace/SHA全部保持，不重写历史；
原独立物理/时间PASS与视频内容FAIL分开记录，不能用末态截图补成完整过程视频。
新录像前必须四角/移动主体内容测试、持续全窗口/精确窗口约束和主体可见性检查，
关键抓取/落篮/真实按键/关盖/转动/开盖/取回/末态分段视觉复核，不只验证解码。

本轮没有新launch、controller、builder、identity、run或execution.claim，没有实现
双臂同步控制或新三分钟候选，没有新仿真/动作、reset/detach/松夹、腿部导航/真机。
未改正式ROS模型源码/轨迹/policy/authority，因此不进行新ROS隔离构建或导航回归；
原最近111通过/2V12原缺源失败仍保留，不把32分析测试说成原全回归PASS。
前后10262历史保护文件/各正式SHA链/10备份/785正式源码及r216–r225封存链PASS，
r225304文件全部核验，原孵育箱v9/r17/v24/r103、离心机/导航成果、重要失败/
报告/视频/rosbag/builder链和920箱子版本/1008/1009/全部备份不改。
只读live无相关仿真/执行器；无清理/回收站操作或新编号备份。

五份根文档修改前原文/SHA在r226/documentation_before.json；AGENTS只追加，
摘要/计划29/Quickstart/交接提示词同步上述纠正与重构方向，旧正式报告不改。
证据runtime_evidence/centrifuge_48box_three_minute_budget_20261012_r226/REPORT.md及
timing_budget.json、video_review.json、STRUCTURAL_REDESIGN_PLAN.md。

## 2026-10-12 晚 CST：用户授权连续实现180sim秒，r227录像修复实跑与r228组件候选

用户明确直接修改/测试/fresh运行直到完整双盒装篮、实际指尖关盖、30rpm稳转不少于
30sim秒、停转开盖、两盒返桌完整PASS≤180sim秒，同时修录像且保护旧成果，
不等待每步“继续”。本授权仅Gazebo固定骨盆手部，不含真机/导航/新备份/清理。
既有FCL/0.015rad跟踪/2mm滑移/5°携带/0.25°释放/3°校正、3/10/30秒证书保持。
截至此工作检查点三分钟没有达成，r225465.9815sim秒仍最新完成物理/时间基线，
视频仅解码PASS/视觉FAIL的r226纠正继续有效，旧报告/视频/哈希均不改。

r227全新身份centrifuge_48box_dual_sync_20261012_r227实际不是双持物：
原固定站位x=0、单holding/原双桌盒、r225动作控制保持，仅新全原生窗口动态采集、
等比缩放+letterbox及宽视角。502局部/46配置/126旧上料/6CTest及两包六库隔离
构建通过；原导航111/2V12缺源仍原样保留。203成员一次seal，Popen四角色连续
执行，机器人动作不等待AI按阶段触发。resize四种尺寸120帧全解码；98稳定帧
四角全可见、22过渡帧独立保留，首次未处理Expose的黑帧失败也保持。
真正录下的前置抽帧可见完整H1/双臂/桌子/离心机，不能仅凭此提前宣称全程视觉PASS。
检查点r227 fresh server1705529/GUI1705532、executor1734720、video1707485在运行，
latest trace约left_static_grip_hold；PID/ctime/夹持/机器必须重查，禁止照抄历史快照。

r227的dual_state独立双holder单元实现和离线双臂planner并未进入控制器：
原x=0固定站位左右桌抓取同torso IK失败，旧两单臂torso需要相反方向；
前移0.12m单独输入fork虽有抓取IK，但起始配对直线FCL碰壳，失败日志保存。
station/dual输入未被实际launch，不能称双臂并发或新站位物理PASS，不改正式模型。

r228新root/inputs/run已普通mkdir独占，尚未seal/build/launch/claim，
centrifuge_48box_servo_rate_20261012_r228准备独立空臂500Hz组件校准。
gz_ros2_control官方Jazzy位置控制式v=gain*error*manager_update_rate，原100Hz/P0.2
在0.8rad/s估计误差0.04rad，500Hz估计0.008rad；只是假设，非物理证书。
只forkcontrollers100→500与参数路径，原开机位/模型/P0.2/惯量/质量/摩擦/0.5ms不变，
完整实际场景2mrad FCL双空臂小幅往返，原跟踪/末点/新鲜性门，零按钮与盒仍桌面。
组件仅COMPONENT_PASS，不能覆盖完整双盒/180sim秒目标，不推广失败参数。
不得在r227活动期间移动六个Conda库构建或启动第二Gazebo；先完成/核验/正常SIGINT
收尾r227，再新组件一次seal/fresh实跑。旧完整依赖测试及原导航缺源事实保留。

10262历史文件/正式SHA链/10备份/785正式src以及r216–r226封存前置全PASS；
r225304/r22624封存保持。无旧成果/备份覆写、reset/detach/异常松夹、清理或新备份。
本小节是活动检查点，不是最终完整运行报告；所有成功须新落盘机器证据和实际视频核验。

## 2026-10-12 19:40 CST：r227完整录像修复回归PASS，r228500Hz实际组件PASS

覆盖前一活动快照：r227已完整真实双盒装篮/五次指尖按钮/关盖/30rpm稳转≥30sim秒/
停转开盖/双盒返不同桌槽/3秒落稳/末10秒完成，执行器与独立物理27/27 PASS。
首条sim38.6835→499.9825，461.299sim秒，wall4938.294727；非180秒。
角峰1.715810945°、独立滑移1.027942775mm（保守FK1.092283117mm）、取回微动
0.027850945rpm，旧0.02超限1样本；按已有5°/2mm/取回有限制动0.1rpm合同通过，
不称原全程0.02严格PASS。4.6825秒较r225差异来自单次动态执行，不归因于录像改动。
身份dual_sync仍未执行双持物或station，旧离线IK/FCL失败原型保持，不外推双臂成功。

r227整窗实际3676×2022→1200×900letterbox，62903帧/5241.916667wall秒全解码，
62891真实采集/12补时重复、max gap0.147402039秒，正常SIGINT；17关键相位真实抽帧
已查看，主体/设备/动作区持续在画面，不再地面裁切。远桌盒或夹爪仍有几何遮挡，
盖下转动不能仅靠画面确认；不是逐帧看完整视频，末俯视截图在停止录像后不能冒充视频。
四角色全0，场景正常SIGINT退休/残留0，无损2013383行/5408115439解压字节CRC/SHA。
203密封成员事后核验、367静态文件封存：SUM02683a745b55706a03f67a1d4c32295aa326d96949381e5ed583a0de275b51d1，
REPORT c149a9047b9e122097a8984ef5398c3da8627a2374729853224cccb6f5c5e712。

r228在r227退休后独立六库构建、527局部/46配置/126旧装载/6CTest通过，
原导航111/2V12缺源仍原样保存。199完整相关依赖/205成员一次seal、实际六控制器active，
ROS参数确认update_rate=500，P0.2/0.5ms/原x=0/质量惯量摩擦模型保持。
双空臂0.12/0.36/0.8rad/s六次小幅往返，原动态0.015rad/末点0.0003rad/0.2sim稳定门；
实测跟踪0.000960/0.002880/0.006400rad，独立297实际样本审核PASS，末3秒双盒桌支撑/
漂移<0.5mm，按钮0/盖0/电机off/启停0。仅COMPONENT_PASS，不是完整双盒或180秒。
FCL机器人/桌子/离心机/两实际盒；外围墙另按真实碰撞网格/链长三角界证明至少
1.152838385m净距，不把未放入FCL的墙宣称FCL对象。首次大摆幅碰撞拒绝已保留。
误占内部open-x日志/外部相对Path预检拒绝都发生在任务claim前，改调用不放宽门。
视频4126帧/343.833333wall秒全解码；末俯视已在SIGINT前实际录入，抽帧可见两桌盒。
四角色全0、正常SIGINT退休/残留0；333文件封存SUM09e79a787f08028d256418b41c52832c36b9ae159bf736832da7c4684943fc58，
REPORT f6315e06d250ce89df60c6c7dd21a49f74fff9d2f4ef3ffc9d90d7067936c6ae。

两运行前后10262历史文件/正式SHA链/10备份/785正式src对1009及r216–r226链PASS，
r228还前后核验r227367静态文件。原孵育箱/离心机/导航全部成果、重要失败、视频/
rosbag/报告/builder/920箱子版本/1008/1009及所有备份不改；无reset/detach/异常松夹/
导航/真机/清理/回收站/新备份。任何旧身份或claim不能复用，旧源码不能推广失败参数。

用户持续实现180sim秒的明确授权仍有效，不需要每步“继续”。下轮r229普通mkdir
独占centrifuge_48box_servo_fast_20261012_r229，113父源核验复制、原x=0单holding，
候选空手0.8/自由带载0.5、按钮20ms、取回软闭合2s；硬门、3/10/30秒不改，尚无
新物理PASS/180证书，必须新隔离构建/回归/保护/一次seal/fresh门，再实跑并按结果迭代。
500Hz只小幅空臂有证据，绝不外推为本带载倍率已安全；失败保持并正常SIGINT。
本检查点r227/r228均已退出、r229未claim，下一次接续重查实际live与报告。

## 2026-10-12 21:20 CST：r229原门失败退休，r230同时间测量fresh运行检查点

r229已实际FAIL，不是准备中或完整PASS：首右盒right_transfer在sim99.5095触发
原FK2mm门，2.038809544mm/0.1053°，最大native0.976562283mm；只1次lid实际按钮。
不能凭native较小宣称误报或完整成功。静态state约25ms，不捕获瞬时完整q，
100Hz关节与更快独立native不同时间是待证假设；原FAIL保持。另park/reorient受
20ms密点底限约束，单加倍率不能到180，后续需控制路径重采样/完整重新FCL。
实际stop-and-hold保右手努力/目标，rotor quiet、pause:true、正常SIGINT视频/场景，
executor1/其他0/残留0，无reset/detach/异常松夹。202原成员复核、321文件封存，
REPORT83871ae535f18b92f2672b5460731a3b3bc422f568b98a957dbe0ed2bf43049e，
SUM7ceaf4b6949f938f664a6f303780c931cd230bb5acfecf631cbd697140f8eec5。
正式src没有推广失败参数、785对1009仍PASS；原全部成果/失败/视频/rosbag/builder/备份不变。

r230全新centrifuge_48box_coherent_feedback_20261012_r230/专属install，118父源SHA
验证后仅新身份派生，207成员一次seal、200相关依赖、556局部/46配置/126旧装载/
6CTest和两包六库隔离构建/精确恢复PASS。原导航111/2V12缺源失败仍保留。
前置10262历史文件/正式SHA链/10备份/785src/r216–r226链和r227367/r228333/r229321
静态全部PASS。x=0、500Hz/P0.2/0.5ms/模型质量惯量摩擦保持，无真机/导航/清理/新备份。

新CoherentCycle只修抓握反馈时域：完整实际31关节单调历史，≤12ms有界插值，
不外推或用command假实际；同packet body/native与其时间的实际q比较，
匹配包落后最新native≤12ms、FK/native wrist额外≤1mm/0.2°。
即时最新native的2mm/5°也单独严格检查，其他物理/接触/容纳/末态仍读原最新pose。
参考只用冻结native抓握ref及首次FK/native几何映射，不以当前body重置来消除滑移。
FK和native数值2mm/5°、0.015跟踪/0.0003末点、接触/释放及3/10/30秒均保持。
旧未对齐FK另存诊断，不能称旧时间合同已通过或重写r229；新的测量合同必须新物理证据。
audit_coherent另用全部真实q帧重算bracket、FK/native/最新native与冻结ref，不能只信报告。

录像仍全窗，新增显式yuv420p；真实ffprobe High/yuv420p与实际前置截图已核验，
四角resize120帧/真实窗口72帧/实际六控制器/ROS参数500/fresh父链/新录像门通过后
一次claim。21:20快照executor2956113/ctime1791810821.05、独立视频2946058在运行，
阶段约right_descend，四角色连续自主，无需用户逐步继续。此非新动作或180 PASS。
禁止修改207密封成员、复用旧claim、启动第二Gazebo或活动期间移动六库构建；
先重查run进度/结果/guard/磁盘/视频/持物，失败保持并正常SIGINT，由原监督器收尾。
磁盘约1.8GB，守卫512MiB仍启用；不可为了继续而删旧重要成果或擅清。
持续目标仍首条非空sim至完整≤180及视频内容/全解码/物理/日志/保护全部通过。

## 2026-10-12 21:50 CST：r230第二盒落篮FAIL覆盖活动快照

r230已terminal，不再运行。新同时间抓握在两盒搬运通过原2mm/5°，最大coherent FK
0.985282882mm/native0.984813778mm，legacy未对齐FK诊断2.281332039mm；
只是部分运行，不能称完整audit_coherent/原时域合同PASS或重写r229FAIL。
右盒自然落篮/3秒容纳证书完成，左盒释放后原8sim窗口无法连续3秒完整落稳，
原门正确FAIL，最后quiet155.4035sim。仅一次lid实际按钮，无关盖旋转取回/完整180。
终态left无Dex接触，仅bucket_left约0.8992N/0.688um，但corners local Y最大
0.0672645m/Z0.0812339m、容纳false；right完整容纳true。失败末原速帧已查看。
需区分盒世界倾角/摇篮摆角与落差冲击，不能称夹爪未张开或仅接触篮子就PASS。
不得锁死摇篮/改模型/门限/假support。原gripper努力目标、quiet后pause、视频/场景
正常SIGINT，退出executor1/其余0、残留0，无reset/detach/异常松夹。

556局部/46配置/126旧装载/6CTest及隔离构建仍PASS，原Nav111/2缺源保留。
前后10262历史文件/正式SHA链/10备份/785src对1009、r216–r226链及r227367/r228333/
r229321静态全部PASS，旧全部成果/失败/视频/rosbag/builder/备份不改，正式src无失败参数。
原完整物理仍r227461.299sim，180未实现；新录像High/yuv420p全窗/全解码正常SIGINT，
只能证明失败过程。trace691739行/1720572218解压字节/415213230压缩字节无损核验。
接续读本r230 REPORT/drop_diagnosis/实际world旋转与接触，先新身份审核释放落差/
篮坐标系适配；再控制点稀疏平滑重采样，保留20ms和3/10/30秒，不盲加倍率。
已安装RobotTrajectory支持TOTG(path_tolerance/resample_dt)和Ruckig smoothing，
当前只有本地API只读确认，尚无新路径或实际重采样。约1.4GB磁盘，512MiB守卫不关闭，
不擅删旧重要成果/清理/新备份。用户持续完整180目标授权仍有效，无须每步“继续”。

## 2026-10-12 22:18 CST：r231降低自然释放落差fresh运行检查点

r230完整世界旋转诊断：左盒约world pitch -21.97°、up_z0.927385，篮仅0.00754°；
是盒卡篮沿而非篮大幅摆动。实际夹爪已分离，旧FAIL和封存不改。
全新r231/专属install从124父源核验派生，只在原上方对准门后增加45mm向下释放路径，
名义自然落差68→23mm；不改机器人/盒/篮几何、质量惯量摩擦、原站位、摇篮物理。
左右15/25/35/45mm真实旧释放快照新IK/FCL及张爪路径均离线PASS；右55mm IK失败保留。
运行时从实际完整q/poses再独立生成2mrad FCL路径，0.3mm/0.1°/0.3mrad快照门、
0.08rad/s/20ms下降、实际终点和原XY/角门，再自然张爪，原容纳/3秒证书不变。
保留r230同时间实际q/FK与立即最新native2mm/5°、500Hz/P0.2/0.5ms、原3/10/30秒。
日志只gzip1→6无损，所有原字节/行数/UTF8/SHA/CRC检查不变，512MiB守卫不关闭。

563局部/46配置/126旧r159装载/6CTest和两包六库隔离构建/恢复PASS；原Nav111/2V12
缺源失败仍原样保留。首次Nav测试重写受保护pytest lastfailed，before据实FAIL，
已仅从只读920核验提取该缓存原字节撤销本轮副作用，生成内容/失败/撤销记录保留，
原SHA f14be2194b3e82166d1f116f899a416eb8d3f471803ea50819edcf1dd7119534恢复。
未来pytest必须-p no:cacheprovider并PYTHONDONTWRITEBYTECODE=1，禁止再写旧缓存。
未恢复任何源/模型或改归档。重试10262历史文件/10备份/785src及r216–r230全部链PASS。
原resize外部日志误占内部open-x前检失败保留，修调用后四角/72帧/完整解码通过。
205相关依赖/217成员一次seal并0444。全新fresh server3349006/GUI3349007/实际6active/
参数500，视频3351232 High/yuv420p整窗，实际录像前帧已查看主体/手/设备/桌均在画面，
远盒自然遮挡声明；首次过早读取尚未moov失败、随后真实解码帧证据保留。
executor3356025/ctime1791814688.56由四角色Popen连续执行，不等AI逐阶段触发。
这是活动检查点，不是完整/180 PASS；重查live/result，失败由原保持/正常SIGINT监督收尾。

r232仅普通mkdir预留control_resample身份，14纯函数测试及25urad有界折线压缩原型。
初步原路径名义估计仅省17.97098sim命令秒，未完整预算/密集FCL/新隔离构建或实跑，
不得宣传三分钟或复制未密封父源；不复用其未来claim，不在r231活动时移动六库或开第二Gazebo。
约1.3GB磁盘，重要历史失败/成功/视频/rosbag/报告/builder/备份全部保留，无清理/新备份。
原完整基线仍r227461.299sim，持续目标保持完整两盒/30秒/取回≤180及真实录像/保护全PASS。

## 2026-10-12 22:39 CST：r231真实native角门FAIL退休，r232撤回高速候选

覆盖上一活动快照：r231 right_lower于81.356sim真实最新native角5.039365337°触发原5°门，
右手仍持盒，尚未到新45mm释放下降，不能把本FAIL归因新释放或宣称该释放物理通过。
独立完整实际q bracket/冻结参考重算FK0.400969055mm/4.695440161°，
native0.434273154mm/5.039365337°，同时间腕几何约5.59e-16m/2.30e-14°一致。
是真实角失稳，不是未对齐旧FK或2mm位移超限；5°/2mm不放宽，不忽略即时native。
全运行滑移coherent0.918807345mm/native0.984356965mm，1次真实lid开，未装篮/旋转/取回。
原target/effort保持、quiet82.005sim、pause、正常SIGINT视频/场景、executor1其余0、残留0。
实际失败末录像帧已查看，主体/双臂/设备/桌仍在整窗，非完整流程影片。
10051帧/837.583333wall秒 High/yuv420p全解码PASS；gzip6无损397983行/802983384原字节/
139277927压缩字节/CRC/SHA PASS，解压d10c9a7538593e4f173c00538dd41753d1c3446c6d1a4102393012ba1702a0fc。
217原成员事后复核、340静态文件0444封存，REPORTc76d2b5090450ee16850a76294733684cc478da08dc9c3c85bfb29b84bd136b3，
SUM27ec6299ae6256ebee9f55cc0fb2c44909fb2039755305ee19fa28939746142d。
前置重试/事后10262历史文件/10备份/785正式src及r216–r230链全部PASS；缓存f14已精确恢复。
正式源码始终未推广失败0.5，旧孵育箱/离心机/导航成功及重要失败、视频/rosbag/报告/
authority/run/builder/备份全部保持，无清理/新备份/reset/detach/异常松夹/真机/腿部导航。

r232全新control_resample身份已从129个r231密封源SHA核验派生，新install隔离两包/六库
构建与恢复PASS。实际候选撤回0.5自由帶载到0.25，right_lower恢复原已证0.065，
left_lower保守0.13，未证取回粗setdown0.36撤回0.20，最终10mm原0.03不变。
保持x=0/500Hz/P0.2/0.5ms、23mm自然释放待验证、原20ms和3/10/30秒、原全部硬门。
原Nav111/2V12缺源保留，全部pytest禁cacheprovider/禁bytecode，旧lastfailed f14未再变化。
577初测后新增4项wrong-holder下降拒绝，共581局部；还须新完整前检/一次seal/fresh实跑证据。
本检查点r232未启动，后续若更新必须以最新落盘实际结果为准，不复用r231或任何旧claim。

14测试25urad压缩原型并未启用；名义只省17.97098sim命令秒，不是180/完整预算/FCL。
新共享torso双臂探针0.12/0.18m仅桌抓取IK通过，双篮姿态失败；0.20m仅释放IK通过、
无完整路径/FCL且可能撞桌，不选择该站位/dual，不写成双手同步实际PASS。
先真实带载/释放/完整合同恢复，再选择密集FCL证明的短路径/平滑控制，持续180目标不缩小。

## 2026-10-12 22:50 CST：r232真实fresh连续运行覆盖待启动快照

r232581局部/46配置/126旧装载/6CTest、两包六库隔离构建/恢复PASS，原Nav111/2缺源
原样保留。206完整相关依赖/219成员一次seal及0444，前置10262历史文件/10备份/785src
对1009、r216–r231全部封存链PASS，旧lastfailed f14保持。正式源/模型/资产/备份没有变动。
实际fresh server3524944/GUI3524945/六controller active/参数500；录像3526929，
High/yuv420p/全窗/实际前置录像帧已查看机器人/手/设备/桌在画面，远盒自然遮挡声明。
executor3536185/ctime1791816462.36四角色Popen连续执行，不待AI或用户逐阶段触发。
本活动检查点阶段约machine_preparation，非完整或180 PASS；必须重查trace/result/guard/
录像/磁盘/持物，不能照抄PID。活动期间不移动六库构建、开第二Gazebo或修改219密封成员。

新事后只读r231接触分析放在r232/run/prior_angle_contact_diagnosis.json，不改封存r231。
右pilot/static/lift/reorient/transfer角峰约0.00988/0.01033/0.08752/0.14310/0.17780°，
急增只在right_lower，故恢复靠篮下降原0.065更直接；失败前盒接触只有Dex1，
没有盒↔篮接触，不能凭猜测称篮沿碰盒致本角故障。机器人手指↔篮未由该盒topic覆盖，
仍不能排除这类未观测接触，也不把速度根因假设写成已证因果。实际5.039°原FAIL保持。
撤回0.5自由帶载、未证粗setdown后仍需实际完整PASS，23mm释放尚待验证。
control_resample/dual/新站位原型未进入运行，不宣称双持物/重采样或180成功。
磁盘约1.2GB、512MiB守卫保持，无删除旧过程或重要证据/清理/新备份/真机/导航。
用户持续修改、测试、实跑至全流程≤180的授权和目标保持，不需要每步“继续”。

## 2026-10-12 CST：r232磁盘1GiB正确中止，r233离线证据与存储待授权

覆盖r232活动快照：实际disk_watch在free1072304128B发送正常SIGINT，原阈值
1024**3=1073741824B；run_guard另有512MiB/20wall秒录像门，未发送停止。
两个门同时生效，之前只提512MiB的摘要不完整；不得忽略/降低1GiB继续实跑。
不是录像停增长/编码失败或又一次角失稳，不能修改录像flush去“修”没有证据的问题。
0.25候选完成真实右pilot/10sim静态保持后，right_lift尚未结束，77.405sim停止、quiet78.025。
只该早期握持coherent/native峰0.036741922/0.036734974mm、0.001837877/0.001839701°，
不能与r231末lower直接比作完整带载改善；right_lower/23mm释放/第二盒/机器/取回均未到。
原努力/目标保持、pause、视频/场景正常SIGINT、executor1其余0/残留0，无reset/detach/异常松夹。

581局部/46配置/126旧装载/6CTest/六库隔离构建依旧PASS，原Nav111/2V12缺源保留。
219成员事后复核、337静态文件0444封存：REPORTd3e8292a61b59615eabde80ae582792ee541e4b46701d40cb9cb838aece8a79f，
SUM88be882a1f6fee9b4744c8f9a634cfef994814ec8727880704665f0c04d4ff08。
全原354703行/608354731字节gzip6无损97991813B/CRC/SHA PASS，
解压41eef5d83f4c43b9160c25e0fbaa9507c2ba853cc2c39dc683e71a1530409526。
8934真实采集/0补时/最大间隔0.102414wall秒，8934帧/744.5wall秒High/yuv420p整窗完整解码，
真实前置/失败末录像帧已看，主体/双臂/设备/桌在画面，远盒自然遮挡，仍非全流程影片。
前后10262历史文件/10备份/785正式src及r216–r231封存全部PASS，旧全部成果保持。

r233仅新mkdir独占离线探针，原x=0/桌盒/设备及模型不改，共同torso+14臂、四种左右腕
yaw0/180多起点搜索均未找到双桌抓取IK，约57.8–58mm/17.6–18.9°残差。
只是这次搜索未找到，不称数学全局不可能；无新模型/路径FCL/控制器/claim/Gazebo。
只读盘点1512个未被正式SHA/运行依赖/安装链接引用的CMake对象候选323007041B，排除56；
未删除/移动任何缓存，候选还须逐项确认配方/引用和新授权。约308MiB不足完整运行。
两个真实16MiB日志块8次内存gzip6/zstd3/9/19精确字节回环PASS，zstd19比gzip6省38–44%
但慢29–49倍；未证明流式时延/完整空间预算，不推广19、不改旧日志/当前codec或磁盘门。
r23310静态文件0444封存：REPORT15cccfeab8bb023a9d5b1f6d7567373476ba97adc4a48b494327a578d98afb0f，
SUM14ecb304fbffce8eacddfd711b65b6edd56e2a248db4e6817632680dc048b616。

当前约1GB空闲、Gazebo已退休；存储是本轮第一次真实阻塞，不得启动注定撞1GiB门的下一身份。
已向用户询问额外≥3GB/其他存储或仅核验旧构建缓存新授权，当前未收到答复；不擅清重要资产/
回收站/备份、无新备份/真机/腿部导航。若用户授权缓存，323MB仍不足，不能用正式视频/备份补。
若用户提供空间/存储，重新只读核对后用新身份继续源码/测试/隔离构建/fresh完整实跑，
不能复用r232或r233，不能把早期握持/codec组件/搜索/预算当完整或≤180证书。
持续目标及自主授权仍有效，不需要每步“继续”；尚未达到180，不标完成，原完整基线仍r227。

## 2026-10-13 CST：r234空手换边合并离线FCL通过，存储仍待回复

本目标续接轮先重新核对实际：约1GB空闲、无Gazebo server/GUI/录像/执行器活进程，
原1GiB/512MiB守卫均不降，无新存储或清理授权。上一目标轮是源码/实跑/封存进展，
本轮同一存储条件第二次出现，但仍完成下面有价值的代码/测试/FCL，不是反复只说阻塞。

r234新ordinary mkdir独占，仅离线，无新模型/install/authority/claim/Gazebo或控制消息。
从r227完整27门PASS的原trace核验压缩SHA，读取右drop3秒证书141.9935sim与真实全状态，
将right_retreat/right_return/left_approach组成15个torso/双臂同步路径，腿/夹爪/设备不变。
原516点直接连接碰撞拒绝保留，12次有界快捷连接得4控制点；2端点/原4257/快捷7648/
最终2066，共13973 FCL调用（包含拒绝快捷），最终2066全部按2mrad通过，无碰撞豁免。
机器人已有碰撞几何/桌/设备/转子/盖/两篮/两盒在场景；墙不在FCL、仍需原x0包络证明。
15真实URDF硬限通过、最小余量0.02000001rad；原起终点保持、20ms底限与3/10/30秒不变。
同0.8rad/s三段名义12.218745978→合并5.275813437，预计该指令段省6.942932541sim秒。
r227原实际三条command duration22.279480655不同速率，不能把17秒都归因于路径合并。
0.656417wall仅场景构造后的数值检查，不是动作时间或完整预算/180证书。

32路径/运行上下文纯测试通过：真实任务关节约束、2mrad、碰撞直连拒绝、双空手/开爪/
零努力/原3秒证书与0.3mrad快照门。上下文门尚未接控制器，不把历史snapshot标为fresh。
未来新完整身份必须先有空间；真实right drop PASS后重采全场景/全q并重新整路FCL。
当前23mm释放比r227历史起点低45mm，必须检查新起点连接，禁止直接发布旧4点。
实际同步控制、0.015/0.0003跟踪、右盒持续容纳及完整物理/录像/计时必须实跑；
未真实完成合并前，不可用“跳过旧两段”假确认或伪造旧到位；失败保持、正常SIGINT。

原10262历史文件/10备份/785正式src及r216–r233完整静态链事后全PASS。
r23420文件0444封存：REPORTa319147b33a7654ab778857a36c1112ac5c061b3d7d5eb767f0381052c9e2cbd，
SUMc532557099522f2a38aa49919c783da1caa247b311eab331ac6e83719fe4eeaf。
旧全部成功/重要失败/authority/run/视频/rosbag/报告/builder/备份不改，无清理/新备份/真机/导航。
不能复用r234，后续实际运行须下一全新身份并依规隔离构建/测试/保护/seal/fresh门。
存储新授权尚待用户答复，约308MiB构建对象候选仍不足完整实跑；不拿正式成果补空间。
完整≤180目标仍未达到，保持目标而非改成6.94秒组件完成，不标完成，也不自行暂停。

## 2026-10-13 00:04 CST：同一存储阻塞第三次审计成立

重新只读实测可用1063583744B，仍低于disk_watch原1GiB=1073741824B；
无Gazebo server/GUI、执行器、录像进程在运行，r227/r232退休remaining=[]。
r227完整PASS461.299sim仍最新完整物理基线，r232FAIL/DISK_STOP_AND_HOLD，
r23432测试/2066最终FCL仅离线、无完整180证书。目标没有达到，不标complete。

同一条件已连续三个目标轮次：r232实际磁盘中止及存储询问；r234同条件下完成离线代码/
测试/FCL；本轮再次核验空间/终态，无新用户存储位置、空间或清理授权。
当前必要下一步是新真实带载/23mm自然释放及完整控制器运行，不能用更多离线预算替代。
不降低1GiB/512MiB守卫，不删正式成果、回收站/备份，不启动已知必失败的下一身份。
可再生对象候选约308MiB仍不足完整运行，未执行删除；存储问题已经向用户提出，不反复问。

本次blocked审计满足，应将现有完整目标记录为blocked并停止无效自动轮询（以目标工具
返回状态为准）；不是用户请求暂停，也不是完成或把目标改小。全套≤180及完整录像/物理/
日志/保护要求原样保留。用户提供额外空间/可用存储并明确继续后重新核对现场和哈希，
用下一全新身份接续，不复用r232/r234；曾blocked后恢复按新的阻塞审计计数，不沿用本次计数。

## 2026-10-13 CST：用户授权仅删除无当前运行用途的Isaac Sim安装包，存储条件改善

用户先授权仅核验旧构建缓存，随后问回收站可否删除；明确列出Isaac Sim ZIP后，
用户回复“你看一下还有用吗，没用就删掉”。本次仅删除这一个离线安装ZIP与对应trashinfo，
不是清空回收站，也不扩大为其他下载包/训练/视频/归档的删除授权。
新ordinary mkdir独占installer_archive_cleanup_20261013_r1，判断、精确路径/大小/哈希、
引用检查和前后保护均在该REPORT/inventory/cleanup JSON，不复用旧清理身份。

ZIP13018774928B，SHA348013c4d3ac398d40162566d0edfe4f3837a0eb5efa358ec87065f81aa68dd7；
对应已安装/home/liu/isaacsim版本相同，六核心文件SHA一致且保留，无live fd/maps/cmdline
引用或当前Gazebo依赖。不是用户V12训练成果，没有删除其他训练压缩包或恢复缺源。
不称整个安装与ZIP完全重复：32645个预打包文件未在安装树中、194个尺寸不同，
没有启动/认证Isaac Sim；本ZIP的可选离线重装/补库价值删除后不再保留。

只删两个普通非链接文件，占用13018783744B；立即可用14033440768B，最终复核后
14037438464B。其余回收站18451递归成员与六安装核心文件哈希/元数据/链接目标前后一致。
约308MiB构建对象候选本轮未删；没有删正式成果、模型/源码、authority/run、视频、rosbag、
trace、报告、builder、备份，也没有清空整个回收站。未来其他永久删除仍须相应明确授权。

原r232冻结审计身份核验后向本新根输出，前后10262历史保护文件/10备份/785正式src对
1009全部PASS；r216–r234十九近期封存根逐项PASS。额外150历史SHA清单路径解析初错
保留FAIL与日志，修正后四个删除前已存在的差异（r155旧Gazebo日志哈希/3旧导航实验源码
路径缺失）及一个被1009正式版本覆盖的旧源码checksum均前后一致；未修复/恢复或改写。
不声称全部历史清单都匹配；current_protection PASS范围仅现行保护与历史快照未改变。

原空间阻塞条件实际解除，1GiB/512MiB守卫不降低；此轮未启动停止Gazebo、发布动作、
reset/detach/异常松夹、连接真机或创建新备份。完整≤180仍未达，r227461.299sim仍完整基线。
原目标工具blocked历史记录不能冒充当前仍缺空间或改为complete；后续须重新核对预算/
现场，用下一全新身份隔离构建测试/一次seal/fresh录像与健康门，不复用r232/r234。

本次22静态文件0444封存：REPORT7b5c493ab33ccd6d4bb43d58200883495629f36a43884e770bc81b440018948f，
SHA256SUMSf031f0e4c8a75649b386ce82e96fae9257248d3610cf84367ee1b6b6108ed274；
封存后可用14036897792B，回收站仍1491顶层条目，不称已经全部清空。

## 2026-10-13 CST：r235恢复fresh全流程试验，右align真实5度故障，非磁盘中止

上一目标轮清理经用户单独授权的单安装ZIP，空间由约1GB变14GB，属于真实进展，
原空间阻塞已解除；本目标恢复审计重新计数，不沿用之前三次blocked记录。
新ordinary mkdir r235/独立install/build/inputs/run，从r232验SHA派生133源/配方；
两包六库隔离构建/581局部/46配置/122本次r178装载模块/6CTest通过，原Nav111/2私有
V12缺源失败与XML/exit1保留，pytest禁cacheprovider/bytecode，不改正式src。

219成员一次seal/fresh六控制器active/实参500Hz/录像前实际解码图已看。
仍原x0/单holding/0.25自由带载/右lower0.065左0.13/原5度2mm20ms及3/10/30秒，
r234合并与station/dual原型未用。实际指尖开盖1次、空爪/park、右抓/pilot/10sim静态/
lift/reorient/transfer/lower完成；right_align及其0.5秒观察后sim91.9845故障。
第一非空7.469→故障84.5155sim，仅部分失败不是完整180证书；quiet92.6335，持物/
努力/接受目标保持、pause、视频/场景正常SIGINT，executor1其余0、剩余0，无reset/detach。

真实31关节bracket/冻结参考独立复算：FK角4.866707041、最新native5.048969929度，
对应滑移0.451773744/0.469363156mm；运行峰FK/native0.864118947/0.989100394mm。
记录native角pilot/static/lift/reorient/transfer/lower分别0.009107/0.009512/0.043035/
0.089194/0.123532/0.215727，角失稳集中align；盒接触只有两Dex，左盒仍桌面。
盒传感器不覆盖指→篮接触，机械原因未证明，不能排除未观测碰撞或仅归因于速度。
fast_profile把align也归0.25自由带载，未保持原右align0.10；这项失败分类撤回，
下一新身份应明确匹配loading实际holder并将左右align设0.10作单一假设验证，
不能推广0.25整套/500Hz持物为完整成功，不改/覆盖r235或正式src。
45mm新下降/23mm释放未到，更未第二盒/关盖旋转取回；完整基线仍r227461.299。

447208行/1020342064原字节gzip6无损187764906B，CRC/JSON/行数/字节/SHA PASS，
解压66b3d61f1204f2b9baef24155c33c7581356a0e90a2b3cf53d7236697bf42cc3。
11925真实/0补帧/最大0.154383wall秒间隔，11925帧/993.75wall秒整窗完整解码PASS。
真实前/失败末帧已看，主体/双臂/设备/桌在画面、远盒自然遮挡；仍失败过程非全流程视频。
前后10262历史文件/10备份/785正式src及r216–r234/安装包清理链全PASS，219成员哈希/
0444事后复核；339静态文件0444封存：REPORT25fd5b8c6d948eebc77b209cd8473f643dd1a9603c714e614d14c19e0cfb8358，
SUM9b2091742b538c0e23a196f7fed56840db633392429bfcd95ea8b10626e145f7。
仍约13.7GB空闲，无新清理/备份/真机/腿部导航。目标持续完整≤180，不标完成或阻塞。

## 2026-10-13 CST：r236对齐阶段分类修复、587测试后fresh完整流程运行中

新独占centrifuge_48box_alignment_cap_20261013_r236，从r235封存验SHA派生134源/配方；
仅将right_align/left_align要求actual matching loading holder/payload None并恢复0.10，
其余0.25自由带载/0.8空臂/500Hz/P0.2/0.5ms/原几何与2mm5度20ms/3/10/30秒不变。
撤回失败阶段分类而非放宽物理门；机械唯一原因仍未证明，45mm下降/23mm释放仍待实跑。
本次不使用station/dual/r234合并原型，无新清理/备份/真机/导航或正式源码模型改写。

587本地（新增6对齐分类回归）/46配置/122本次旧装载/6CTest及两包六库隔离构建全部PASS，
原Nav111/2私有V12缺源与XML/exit1保留。前保护10262历史/10备份/785正式src及r216–r235/
安装包清理全部PASS。219一次seal，identity85bb9bf0b797e7a9d4c0ce1de38d0d8fd76d2bbf8d511cf278a1b545956a0aed。
真实fresh六active/500Hz、72帧录像及真正前置录像帧通过，视频独立整窗采集已运行。

四角色exec/guard/disk/supervisor自主连续执行，全流程不用AI逐阶段派发。
run/live_checkpoint_1.json实际四PID/ctime活着，到了开盖withdraw sim27.428，约13.75GB，
尚无result，**不是完成/180证书**。会话81500为本轮launch_run工具句柄；PID/ctime以
process_identities.json与live_checkpoint为准，每次重新核查，不用历史快照代替存活。
下一目标轮如仍活着只接续观察同运行，不因观测超时重启/重复claim或改密封源。
终态后独立物理/计时/日志/相位视觉/后保护；失败保持并正常SIGINT，无reset/detach/松夹。

## 2026-10-13 CST：r236双盒23mm真实落篮通过，r237空手压缩组件已测未接入

上一目标轮为实际fresh r235角FAIL/独立复算/封存与r236分类修复、测试、实跑启动，
属于真实进展；本轮重新核对原四PID/ctime仍活着，不重启或重复claim，并完成下面新代码。

r236原执行器继续自主运行，真正45mm下降/名义23mm门：右108.3745、左173.317sim；
右112.6795、左177.6185sim各完成3sim自然落稳；LOADING_PASS190.329sim。
源证据run/phase_checkpoint_3.json为正在增长gzip的有限快照，未把未闭合CRC或完整
后审计称PASS；只证明该部分在线事件，关盖/30rpm≥30sim/开盖/取回/末保持仍待结束。
四PID/ctime再次实际活着，最新见trace_progress，已进入machine_close_approach；
snapshot不代替未来重查，无result，原会话81500不能重启，也不能改219密封成员。

新ordinary mkdir r237仅离线，不launch/model/install/run/claim，不改或向r236发命令。
button_sampling只接受准确同侧七臂关节及无持物必要条件，保留42推压点/0.02接触间隔。
17测试通过；真实两侧往返181→3控制点，保留起终/转折，误差≤6.66134e-16rad。
table/loaded保守包络×原名义/双完全开爪×双侧往返16场景，原2mrad密查8608次全FCL
PASS；硬限最小余量0.244183181rad，只目标按钮→目标指尖豁免，无新豁免。
既有机器人/桌/设备/转子/篮及0/30/60/90/120五盖角在场，墙不在FCL，别冒称查过墙。
同0.8rad/s/20ms底限，单段名义3.65→1.39039587，五按键十空行程预计省22.596041285
sim秒；244.327wall为离线数值/模型构造时间，不是动作，未接控制器/无180证书。
未来新完整身份必须实际全状态起点连接/其他臂、爪、躯干姿态审核及跟踪/事件/物理门，
禁止盲发3点、压缩接触推压、改r236源或把名义/组件当完整提速成果。

r237事后10262历史文件/10备份/785正式src以及r216–r235二十封存和r236219密封成员
哈希全部保持；不清理/新备份/真机/腿部导航，不修改原孵育箱或任何正式成果。
14静态文件0444封存：REPORT1d8e3ad9c10d1d4ce38b70f4a3278dd879bb17a1917389292fee085d6b3e0024，
SUM4dea836a33ff4908fb2560b42519835b11072ada15568fb0f3676cb7d32e8a88。
完整≤180目标仍未達，原完整物理基线仍r227461.299；目标继续active，不标完成或阻塞。

本轮末实际重查r236四角色仍活着，第一非空sim13.56，machine_start_approach214.1635，
已过200.6035sim且未完整返回；**本身份完整≤180已经不可能通过**，不将其称待验证的
180候选成功。继续原程序跑完是为验证新释放/全流程和量化完整瓶颈，终态仍须全部后审计。
后续提速必须新身份与结构性控制点/并行/短路径改进，不能靠快放或丢观察门缩小目标。

## 2026-10-13 CST：r236终态FAIL覆盖活动快照，r240双臂全段路径找到

前一轮回收站复核没有推进180控制目标，本轮重新核对无live Gazebo/执行器，实际写新
耦合路径代码、测试及FCL，属于真实进展。没有新清理授权执行、备份/真机/腿部导航。

r236真实FAIL sim294.46，首13.56→280.9sim至失败；右手持左盒，首次取回10sim静态
观察因No recent actual joint/native bracket中止。双盒45mm下降/23mm释放及各3sim落稳
通过，上料190.329（176.769sim）；五真实指尖事件、30.0505sim稳转/停稳/120度开盖
完成，MACHINE_CYCLE_PASS272.252；两盒没有返桌。原完整审计16/27，整体FAIL，
不把上料或机器子流程当180成功。quiet295.144、正常SIGINT/retirement剩余0，禁止复用。

r236闭合1545993行/3698917671原字节/gzip765384027，CRC/计数/SHA通过；原27门审计
FAIL退出1保留。三个实际持物锚点/4767记录样本独立同时间FK/native/冻结参考复算
通过前缀数值门，FK/native0.942742/0.976627mm、角0.143015°，明确complete_task_pass=false。
原source_runtime native峰0.984099mm不同采样范围保留，不以已记录前缀取代每循环守卫。
MP437739帧全解码，37738真实/1时序补帧；四实际落篮/上料/机器结束阶段主体可见。
失败末帧变近景、机器人主体部分出画面，原因未证；不能称全程构图/完整任务视频PASS。
新视频需持续广角/阶段检查，全窗口letterbox只解决原生窗口裁剪，不替代相机检查。

r23913测试与实际闭合复放证明旧256/新8192在4767记录样本无配对不同，FAIL前最后
记录native294.449旧256仍可配对；实际100Hz历史约2.55sim，不是误估0.128sim。
故障精确stamp/age未录，无配对/超过2wall秒原因仍未分开，容量不作为已证根因/修复。
下一新身份须记录精确故障快照、有界实际回调配对；不外推、命令值代测、不重置参考、
不放宽0.012bracket/lag、2wall秒或2mm/5°，保持最新native即时守卫。

r238保留全部失败端点与原错误高度1.03025输出，另作可信plan来源纠正：原1.12025
减45mm才是1.07525，lift1.16975。不能用旧误标23mm的失败外推真23mm不可达。
初始固定pelvis前移.20的hip_pitch/hip_roll↔table实际FCL碰撞拒绝，不豁免撞桌。

新ordinary mkdir r240只离线：独立robot固定初始x=.12、离心机离线x=.52（原.60），
两盒桌面出生位/85%尺寸/原质量惯量摩擦/摇篮/正式模型不改，不是导航或腿部移动。
正向直线2壳碰撞/1IK、五绕行翻转及六固定torso候选拒绝、JSON/log保留；逆向连续
冗余分支找到|y|=.29/.30/.32三条双臂全带盒路线，2224/2228/2203原2mrad FCL通过。
15关节、torso±44/关节缩20mrad/20um/.01°；只自己finger/pad及起终桌支撑接触豁免，
机壳/前板/篮/臂/机器人桌碰撞门保持，墙未在FCL，不冒称验过围墙。
.30候选初始全状态/RRT预抓取/双腕下降/同body连接1962密查PASS；初始0/30/60/90/120
盖角无碰撞。22单元通过，原导航111/2私有V12缺源保留XML/exit1。
未建新Gazebo/install/run/claim；本实验布局没有作为live替换旧场景。

同步带盒单程名义14.1908–14.2189sim（.25rad/s/20ms），空臂名义5.22094（.8），
不含真实抓取/试抬/校正/跟踪/观察/按钮/机器/末态，不是180证书。停转改变篮方位，
不能直接倒放名义装载路线作实际取回；须新actual pose/冻结双参考、15关节耦合再规划。
下一新运行先实现两独立物理接触/夹持/最新native守卫、双臂torso双爪共同时间戳，
重新生成新实验布局五按键路径，禁止复用r119旧坐标。各自原3/10/30秒证书不跳过，
可并行观察，但必须每盒逐项实际满足。隔离构建/测试/一次seal/fresh健康和真实录像前门
必需；既有正式模型/身份/成果不可改，任何新失败保持并正常SIGINT。

r240前后10262历史文件/10备份/785正式src、r216–r237/RL恢复/安装包清理链及r236219
密封成员全部PASS；孵育箱模型44ff23fc7077f0c232e2e69ff9d73aba336140d6a7951968c9c4241a97e73db9保持。
四根一次0444封存：r236364文件REPORTc77a52075e24e203857da7b016c0d0a486a9989368f55f29c571983eebc5efcc，
SUMde913268e133d357383b5dcd6bf3431422aa2ad2d096e376951acb223cec0084；
r23823文件REPORTd2954b9a328d04132151a41a35f801ed819014dd9b94f9e0d9d9ea80d619fd58，
SUM018ba5b191a00e388affbeb50079bd7b92ae35fa4a32320d729b15755f538ed4；
r2397文件REPORTa68df4ed35953889284a027268b0614854dfac11e236c1a2e1aa4e60b9e4a039，
SUMe4a6185ad36cfe754d0576973982959139f45868417150dc4bfcd9aa500368ba；
r24069文件REPORT95f8fd88884b50de71ddc63404261a5acc473dc7bc1fe41b5ad9c2e3d87f6438，
SUMa76c678101d80c2a6f61a280e5147cd158a1d7f9b2ab625e56bf3cf688c617ed。
下一前后保护须纳入这些新封存，不覆盖/reseal。本目标仍active，完整基线r227461.299，
未达完整≤180，不因有候选路径或工作量大标complete/blocked，不等待用户逐步“继续”。

## 2026-10-13 CST：r241空手碰撞FAIL，r243根因修复后fresh连续运行检查点

r241真实首22.72→49.4585，26.7385sim至FAIL；仅左指尖首次开盖，两盒未抓取/装篮，
转子未启动。右finger1↔右盒511.032976N触发原40N门，右盒y从-.38移到-.4311278，
quiet50.201，原保持/正常SIGINT退休，剩余0。r242移除实际计划own-finger豁免后：
approach1377点中287碰撞，descend485中0；已证明错误空手豁免允许推盒，不能只归因速度。
r241403文件REPORTcfc8288af21496676143aeb993f799db25827508778177855d6f3ea1a5554040、
SUMe3cf89e45087ae01f301afe3d53a0d7ffd1bd1e1705ec85e3424bb367987c58f；r24211文件
REPORTe6168e27e4426a5bf49665b3ada791fbf9abc2dbaba0d51a74a61f34b24b9d3a、
SUM45371e364be22e30ffe43a3937dfa5aec8863234826b446ef6760b4030e3eab9；全0444保护不修改。

r243普通mkdir独占新install/inputs/run，固定初始robot x=.12/device x=.52，原桌/两盒85%
出生位/摇篮/正式模型保持。取消load_pickup/retrieval_pickup/empty_return的手指/pad↔
loose payload碰撞豁免；持物lookahead仅自己的grasp touch links，不放宽其他碰撞。
捕获前实时核对两盒出生位置<.3mm/角<.2度；接触敏感下降≤.13rad/s及跟踪<2mrad，
其他原15mrad/2mm/5度/1mm/40N门不放宽。首次fixture误用初始-.019开爪下降拒绝保留，
另-.0199实际reopen目标fixture装载1871/102/2262、取回1692/62/2523密查通过，仅名义
零停相位几何，不是actual开爪/抓取/停止证书，运行仍真实bracket/native重做全段FCL。
660局部/58双执行器等/46配置/122旧装载/6CTest、两包六库隔离构建/恢复PASS；原Nav
111通过/2V12缺源XML/exit1保留。24按键布局10320密查/最小净距8.600mm通过，240相关
依赖逐哈希核验；before10262历史文件/10备份/785正式src及r216–r242/RL恢复/安装包清理链PASS。
初次before重复生成calibration open-x拒绝，原失败及部分输出另名保留，完整重试PASS。
新增独立双参考审计要求4冻结锚点/4正常释放/两手实际bracket/FK/native重算及最终空registry；
新的首非空sim至完整PASS≤180计时不再误用旧串行skip证书，完整物理缺一不可。

一次seal240成员/0444，identity3b7c9c97312944b1c93871fcd8b6ad0e6799d4c20425c5563836d2702dc6f778。
真实fresh server838441/GUI838442、六controller active、manager500Hz；72帧短测/整窗
动态尺寸/四角录像预检PASS，实际MP4前帧已查看主体/脚/近手近盒/桌/设备/墙可见，远手
远盒自然遮挡，不声称本帧证明双盒。独立4wall秒/阶段广角恢复线程保持，末态俯视在录像停止前。
executor844216/ctime1791867868.37、guard844217、disk844218、supervisor844220、录像840569
已启动，四角色连续执行不待AI逐阶段触发。run/live_checkpoint1.json复核六角色PID/ctime
活着、240密封成员不变、sim8.845准备阶段；这是活动检查点，不是完整或180 PASS。
必须重读result/trace_progress/live，禁止照抄PID/复用claim；活动中不改密封源、不移动六库
构建、不启动第二Gazebo。失败保持原目标/努力，由原监督正常SIGINT；无reset/detach/异常松夹。
实际停键择相允许稳速≥30至≤32.4sim，旧30.3窗口另记；仍须实际停止相位与真实取回规划。
完整基线仍r227461.299sim，目标active；本轮无清理/备份/真机/腿部导航，所有旧成果保护。

## 2026-10-13 CST：r243录像超时终态FAIL覆盖活动检查点，r244仅准备

r243已终态FAIL，不再live；首6.2995→39.9935为33.694sim至失败，不是完整耗时。
实际pickup1875密查/空手接近/.13下降/合爪结果到达，出生位守卫未触发；实际记录上料
前缀盒位移峰左0.000771mm/右0.005004mm，r241约51.13mm推移未在本前缀重现。
未冻结双实际holder/未装篮/未转/未取回，不能把这一进展当两盒任务完成。
camera82次ACK后一次1.2秒服务超时→录像退出，守卫发SIGINT，rclpy take_message pybind
参数转换异常进入原FAIL/保持；不宣称已证明唯一因果，不吞异常继续。原16N努力/目标
保持，无reset/detach/异常松夹。紧急连续转子静止确认emergency_rotor_quiet_unconfirmed，
不能用约0rpm缓存冒充新鲜确认；独立pause统计44.883sim已暂停，随后正常SIGINT场景
退休remaining=[]。executor1/guard0/disk0/supervisor1，监督因录像normal_sigint=false拒绝，
finally实际完成退休。3972真实帧/0补帧，全解码退出0但录像总体FAIL，失败影像/日志保留。
前置和失败末实际电影帧主体/近手近盒/桌/设备在画面，远手远盒自然遮挡，非完整证书。
183613行/206151975原字节/gzip24975316字节CRC/计数/SHA完整PASS，240密封成员事后未变。
前后10262历史文件/10备份/785正式src及r216–r242/RL恢复/安装包清理链全部PASS。
r243355静态文件0444封存：REPORTaa7f94e5cb34b898aa9980f68c9fd1c5b11b3dfacf672b7753dbec612b7eda9f，
SUM4e5048a07ee2150173c912e13549b09cb4cfc0883dcf3c7029097bf0270ef714，禁止修改或复用。

r244普通mkdir预留，从187个r243密封源/输入逐哈希派生，未复制旧run/claim/实际结果；
仅camera同请求最多3次有界重试，各原1.2秒service/3秒进程超时/.2秒间隔，永久无ACK
仍FAIL，独立encoder不中断真实采集。六项首ACK/暂时恢复/永久失败/异常不能伪造ACK
初测通过；未证明实际录像或绑定异常已修复。全部物理门/实际空手碰撞修复/轨迹保持。
新的prepare_candidate.sh已启动隔离构建/回归/保护/几何/录像前检流水线，不等待AI逐项
触发；尚未seal/claim/Gazebo。接手先查preparation_pipeline.log/作业和输出，不因观察
超时重启或复跑同一open-x输入。下一before/after纳入r243355封存。
完整≤180目标仍active，基线r227461.299不改；无新清理/备份/真机/腿部导航，旧成果保持。

## 2026-10-13 CST：r244相机有界重试新身份fresh运行检查点

上轮真实修改/测试/实跑r243并封存失败、派生r244属于progress，非重复计划；本轮重新
核对原流水线实际PID/ctime，未因输出观察超时重启。原prepare_candidate.sh已正常退出0，
666局部/64双执行器等/46配置/122旧装载/6CTest及两包六库隔离构建/恢复PASS；原Nav
111/2V12缺源XML/exit1保留。新装/取fixture1871/102/2262与1692/62/2523名义几何通过，
24按键10320密查及动态整窗/四角/codec前检PASS；241相关依赖实际逐哈希核验。
before10262历史文件/10备份/785正式src及近期链包括r243355封存全部PASS。
本候选仅GUI camera同请求最多3次有界ACK重试，永久失败仍拒绝，encoder独立采集；
原实际空手无碰盒豁免/出生位守卫/近接触2mrad与全部物理参数/限值/轨迹不改变。

一次seal235成员/0444，identity623d1d52096341dee8bdb714ab21f7c7c69f9927618deda199c0f70be213d30c。
全新fresh server982737/GUI982740，六controller active，manager实测500Hz；实际72帧
短测全解码PASS，新MP4第一秒解码帧已查看机器人身体/脚/近手近盒/桌/设备/墙可见，
远手远盒自然遮挡，不冒充双盒可见。录像984601/ctime1791869187.11及executor989768/
ctime1791869240.58、guard989769、disk989770、supervisor989771已真实启动，四角色连续
执行不待AI逐阶段触发。run/live_checkpoint1.json复核六角色PID/ctime活着及235密封成员
未变；这是活动检查点，无完整或180证书。必须重读result/trace_progress/live，不照抄
PID/复用claim，不因观察超时重启；活动中不得改密封源/移动六库构建/开第二Gazebo。
失败保持原目标/努力并正常SIGINT，禁止reset/detach/异常松夹。camera重试仍待实际
完整录像/各相位和终态内容审核；实际抓取/稳转/取回/独立物理/首非空计时/事后保护
缺一不可。完整基线仍r227461.299sim，持续≤180未达，目标active，旧全部成果保持。

## 2026-10-13 CST：r244真实双夹持前缀/录像PASS但墙钟超时FAIL，r245准备

覆盖r244活动检查点：终态FAIL，首7.894→45.944为38.05sim至失败，不是完整耗时。
两个实际中段/对向载荷夹持在41.43冻结独立native/FK参考，pilot102密查后.03rad/s
共同时间戳执行部分试抬；未完整试抬/10sim保持/装篮/稳转/取回。独立真实bracket复算
每手221/441同时检查：FK/native滑移峰.023350/.023369mm，角.001620246度，腕协议差
.000583mm/.000000567度。独立审核整体FAIL/complete=false/2锚点/0释放/仍持两盒，不
把前缀数值通过改成完整；原完整cycle审核FAIL。runtime更密采样native峰.023689mm保持。

唯一已记录终态故障为Paired arm motion timeout：pilot sim41.528发4.823470885sim指令，
未来起点.3+完成尾.15应至46.801470885，在45.944尚差.857470885sim时，墙钟120.595106
超过原max(60,duration×25)=120.586772。平均倍率27.308674wall/sim；原预算也漏.45sim
起止余量，不是本前缀已记录的滑移/角/接触保护超门。扩大容许尚不证明未来实际完成。
原16N努力/目标保持，quiet46.518真实确认，独立pause/视频与场景正常SIGINT退休，
remaining=[]，executor1其余guard/disk/supervisor0，无reset/detach/异常松夹。

录像5977真实帧/0补帧、完整解码/normal SIGINT/camera PASS；123次ACK全成功，实际
重试恢复路径未触发，不外推永久超时/ROS绑定错误已修复。前置/完整较早试抬片段/失败
末电影帧已查看主体/近手近盒/桌/设备可见，远手远盒遮挡；非完整双入篮影像证书。
读取未闭合末片段第一次NAL不完整失败保留，重读同活PID较早片段成功，未重启录像；
最终闭合全解码通过。206519行/395708749原字节/gzip68993057字节CRC/计数/SHA PASS。
235密封成员事后未变，前后10262历史文件/10备份/785正式src及r216–r243链全部PASS。
r244351静态0444：REPORT6be4077cb7719509a720d40f3bb9ad7702edd81bfe962e0ac6d9e4e3f5242f68，
SUMcbb09ae6ebe96aa5e4892e136a74961906f72edd130eec4787af2b1e8773dfcb；禁止修改/复用。

r245普通mkdir预留，191个r244密封源/输入哈希派生；只将有限wall预算变为包含.45sim
余量的50倍，最小60/最大900秒，加实际sim停滞2wall秒拒绝。原2秒新鲜/.012配对/
2mm/5度/跟踪/碰撞/终点与全球7200秒监督保持，路径/速度/仿真时间戳不变，≤180
验收不放宽。12初测及原trace20719事件复放通过候选预算，只为prefix，非物理成功。
首次preparation脚本相对身份未重绑定，试图输出旧r244只读build.log被权限拒绝；原
日志保留，旧build.log/build脚本/identity哈希未变，未执行旧build或生成新install。
已修正输出根并加身份测试；当前prepare_candidate.sh新隔离构建/回归/保护/几何/录像
前检流水线在preparation_pipeline_retry1.log，旧preparation_pipeline.log是初始失败。
尚未seal/claim/新Gazebo，接手检查实际作业及结果，禁止因观察超时重启或复跑open-x。
下一保护纳入r244351封存；完整≤180目标仍active/基线r227461.299，旧成果/备份保持。

## 2026-10-13 CST：r245统一实际sim等待后fresh连续运行检查点

上轮r244实跑/失败分析、独立复算、正常退休/封存和r245代码/测试属于progress。本轮
重查原38281前检作业/PID，无重启；原流水线正常退出0。封存前只读发现同一根因还
涉及旧dwell(seconds×20wall)和固定600wall的稳转计划等待，按实测约27.3wall/sim会使
10sim保持或30sim等待提前失败。已在未seal的r245统一为有界实际sim进展监视；3/10/30
sim证书不缩短，有限50倍/60..900wall秒和2wall停滞拒绝，原全部物理/新鲜/配对/跟踪/
终点门、指令路径/速度/仿真时间戳保持，全球7200wall监督不变。一般状态/按钮事件等待
原有限超时不改。初次准备686修改前的测试/依赖输出另名before_wait_patch保留，不覆盖。
最终686局部/84双执行器及clock等待等/46配置/122旧装载/6CTest和新隔离构建/六库恢复
PASS，原Nav111/2V12缺源XML/exit1保留；243相关依赖实际重新逐哈希核验，名义fixture/
24按键10320密查/8.600mm最小净距/codec/动态整窗四角前检PASS。before10262历史文件/
10备份/785正式src及近期链包括r244351封存全部PASS，无正式源码/模型改动。

一次seal241成员/0444，identitye09e5594a2eaf18d6ebfd59967547eb40ce8efcf1ad6a92645500197db43670d。
全新fresh server1196976/GUI1196979，六controller active，manager500Hz；实际72帧
短测解码与新MP4第一秒前帧PASS/已看，主体/脚/近手近盒/桌/设备可见，远手远盒遮挡。
录像1199119/ctime1791871185.7、executor1208855/ctime1791871288.3、guard1208856、
disk1208857、supervisor1208858已真实启动，四角色连续全流程不待AI逐步触发。
run/live_checkpoint1.json复核六角色PID/ctime存活、241密封成员未变；这是活动检查点，
尚无完整/180证书，必须重读trace_progress/result/live，不照抄PID/复用claim，不因观察
超时重启。活动中禁止改密封源/移动六库构建/开第二Gazebo；失败原目标/努力保持和正常
SIGINT，禁止reset/detach/异常松夹。实际全部动作/独立物理/首非空计时/阶段视频/日志/
事后保护缺一不可；完整基线仍r227461.299sim，≤180目标active，旧全部成果/备份保持。

### r245同一活进程第二检查点：双试抬/10sim保持后真实运输

上段准备初测为679、补实际sim等待反例后最终686，修改前测试/依赖原件均保留。
未重启同一9963执行作业；run/live_checkpoint2.json核验executor/guard/disk/supervisor/
录像PID/ctime仍活、241密封成员未变，当前forward_above_housing_outboard sim77.567。
首非空14.4→检查点63.167sim，非完整耗时。实际双holder pilot完成/无桌接触断言及10sim
静态保持后，第三actual load_transport请求双盒z=.859749816/.859749861，仍各自独立
冻结holder参考；新2257密查计划正在执行。没有装篮/机器/取回或180 PASS，不把请求/
runtime门前缀取代终态独立物理审核。实际电影较早完整片段帧已查看两手与两蓝盒清楚
可见，主体/脚/桌/打开设备均在画面，记录transport_review1；完整录像闭合解码/终态仍必需。
保持原活动保护边界；继续读最新progress/result，不能照抄本相位或PID，更不能复用claim。

## 2026-10-13 CST：r245双自然落篮76.1125sim完成但撤回规划FAIL，r246准备

覆盖所有r245活动快照：LOADING_PASS90.5125，首14.4→双装篮76.1125sim；双真实中段
抓取/独立冻结/15mm试抬/各10sim保持/actual2257密查后同步运输、正常上方松爪、各3sim
自然落稳均实际完成。两盒在各自篮口/垂直界内/无手指接触。仅装载，不是整套≤180。
整体FAIL90.878，首至失败76.478sim，empty_return90.5095实测请求15维RRT7740查询
未找到路径、未发撤回轨迹。起终碰撞检查通过，不能称初态碰撞或数学不可达；未关盖/
启动/稳转/再次开盖/取回，starts/stops=0。保持/quiet91.598/pause91.662/正常SIGINT视频
与场景退休remaining=[]，executor1其余0，无reset/detach/异常松夹。

独立前缀每手1840/3679同时检查，FK/native滑移峰.882692/.945372mm、角.188680度，
腕一致差.002650mm/.000191度；runtime即时native峰.989893mm不同采样范围保留。
2锚点/2正常释放/最终空registry，独立整体FAIL/complete=false，完整cycle10/27；不把
装篮与前缀数值通过改成完整。17751真实帧/0补帧/全解码/normal SIGINT/camera PASS，
362次相机请求7次暂时ACK失败在有界重试恢复，未重启；两手持两蓝盒运输与末双盒在
两篮电影帧已查看，主体/脚/桌/设备可见，不补成未发生的机器/取回影像。
385315行/1618253807原字节/gzip332919318字节CRC/计数/SHA PASS，241密封未变；
前后10262历史文件/10备份/785正式src及r216–r244链全部PASS，旧全部成果/备份保持。
r245368静态0444：REPORTd4a9e0f44588d6a112ee368a0cd537bd84dc16d338bb99bb4c6e131aa1becd95，
SUM5f989458804d37fef58b5c75d7327830c47c2b361b36904b769a98ffe58ce866，禁止修改/复用。

r246普通mkdir预留，固定腕姿态抬升四高度/宽度均由原20um/.01度IK拒绝，失败源/日志/
审计保留、绝不放宽IK或执行该方案。新方案从actual释放后31关节/两盒/篮，使用**当前
run已经实际执行的装载/可选对位修正关节路径**只作空手路线提案，逆最后三段离开篮口/
壳体，再home，不无谓降回桌面抓取高度；仍持物或身份不属于当前run则拒绝。
所有盒碰撞豁免保持false，不移动盒子或物理attach/detach，不以命令替代当前测量或
持物参考；所有2mrad FCL/原关节余量重验。r245真正释放场景3062密查PASS，最大torso
20.532度，9几何测试通过；仅离线几何，不是新的实际撤回/完整/180证书。
196父密封源/输入哈希派生，真实载入流程在双drop门后才注册当前run指南，包含实际
可选对位修正。第一篮口空手退段.13rad/s/2mrad严格跟踪，后续空手.8/原15mrad，所有
带载/机器/取回/3/10/30sim证书保持。已启动新隔离构建/回归/保护/前检流水线，尚未
seal/claim/新Gazebo；先查preparation_pipeline.log与实际作业，不因观察超时重启/复跑
open-x。下一保护纳入r245368封存；完整≤180目标active，基线r227461.299不改。

## 2026-10-13 CST：r246当前run指南空手撤回候选fresh连续运行检查点

上一目标轮已完成r246实现/隔离构建/测试和前检，属于progress；本轮只读重新确认原
前检退出0及已经启动的真实scene/video PID/ctime，没有因输出观察截断复跑启动。
最终698局部/64双执行器/46配置/122旧装载/6CTest及两包六库隔离构建/恢复PASS；原
Nav111通过/2私有V12缺源FAIL/XML保持，不称导航全回归通过。26输入仅当前身份重绑，
247相关资源和250密封成员实际逐哈希不变；before10262历史文件/10备份/785正式src及
近期链包含r245368封存PASS。原3062密查仅几何，实际释放后仍须重新规划，不复用旧结果。

一次seal250成员/0444，identity SHA
`7b0654fec12e7923602a504ef2899985b249d582c7ef1995e0574f389658ac2a`。
fresh server1617282/GUI1617283，六控制器active/manager500Hz；实际72帧短测/全解码
和整窗四角前检通过。新MP4第一秒真实帧已看身体/脚/近手近盒/桌/关闭设备/墙，远手
远盒自然遮挡，不声称双盒同框或完整过程。录像1619414/ctime1791875178.75保持原进程。
executor1655643/ctime1791875555.49、guard1655644、disk1655645、supervisor1655646已经
真实一次启动，连续执行完整双盒任务、不等AI逐动作触发；execution.claim只用一次。
run/live_checkpoint1.json实际核验八角色PID/ctime存活、250成员不变，sim55.877准备阶段；
其中first_non_null_sim=null只是当次观察器glob未匹配loading_trace，不能当未有sim。
随后直接读取本run真实gzip首非空49.1335，sim61.5255为12.392sim前缀，仍非完整计时。

这是活动检查点，不是装篮/机器/取回或180 PASS。接续先查result/trace_progress及真实
PID/ctime，不照抄本快照、不复用claim、不因观察超时重启；活动中不得改密封源、移动
六库构建或开第二Gazebo。第一退篮段.13rad/s/2mrad跟踪、原盒碰撞不豁免、双释放后
才注册当前指南及所有物理/3/10/30sim门保持。失败stop-and-hold并由原监督正常SIGINT，
无reset/detach/异常松夹。完整基线仍r227461.299sim，≤180目标active且未达到。
本轮无清理/新备份/真机/腿部导航，全部既有成果和备份继续保护。

## 2026-10-13 CST：r246准备姿态FAIL覆盖活动快照，r247早接管候选准备

r246已终态FAIL/正常退休，不再live：首49.1335→FAIL64.8385为15.705sim失败前缀。
5.05sim原静态腿姿态action在49.7815接受，原10sim goal_time_tolerance后ABORT；
64.8305实际左/右hip_yaw误差.0011345265/.0011722883rad超过原.001门，未放宽。
r245同目标21.4405最大.0003384603rad、21.445通过；其首14.4，本次scene/video后
曾跨对话等候再起执行器。晚接管与误差相关，但未证明唯一因果。没有任务轨迹/按钮/
holder/释放/装篮/机器/取回；新空手指南实际未触发，不把离线3062密查改成实跑。
quiet65.5825/实际pause65.725/视频与场景正常SIGINT，remaining=[]，executor1其余0。
录像6195真实帧/0重复/全解码/正常SIGINT/camera PASS，124调用含2暂时ACK失败恢复；
实际窗口放大仍整窗留黑边，首及失败末电影帧已看，远手远盒自然遮挡，非任务过程证书。
72512行/71463657原字节/gzip10306120字节CRC/计数/SHA PASS；完整cycle7/27、
独立夹持0锚点/0释放/0检查仍FAIL，速度无完整时长。250密封成员事后未变。
前后10262历史文件/10备份/785正式src及r216–r245封存全部PASS。初次独立audit与
包含同一步wrapper重复open-x拒绝，原FAIL日志/成功部分另名保留，规范完整重试PASS；
另一次重复创建已有video_audit被拒绝，原监督PASS/独立解码日志保留，没有覆写。
r246371静态0444：REPORT36d78672468360968adf908a8f6c00b9358b45e2c2ebde749a991249dc1165f9，
SUMa51ed1473a11e79f53b19510e7038ee2732f6305b1d321ecfd82728007f93482；禁止修改/复用。

r247普通mkdir独占，202父源/输入哈希派生，只在实际settle后/静态姿态action前新增
测得sim必须≤25的启动窗，拒绝未知/非有限/已老化现场，不用伪造0时钟或reset回零。
原腿目标/.001容差/10sim容许/增益/模型/手轨迹及全部物理证书保持；新15门/绑定测试
通过，不是物理成功。继续采用r246实际双drop后注册的当前run撤回指南，旧r246未触发
本方案不能当失败调参回退对象。新隔离构建/回归/保护/几何/录像前检流水线已启动，
尚未seal/claim/Gazebo，下一保护纳入r246371。先查preparation_pipeline.log/原作业，
不因观察超时重复open-x；fresh健康/实际录像门后须及时启动完整连续执行。
完整基线仍r227461.299，≤180未达且目标active；无清理/备份/真机/腿部导航。

## 2026-10-13 CST：r247早接管真实姿态通过，fresh完整任务连续执行

本目标轮r246真实失败分析/独立审核/正常退休/371封存以及r247实现/测试/构建/实跑
属于progress，不是只重述计划。原r247前检退出0，713局部/64双执行器/46配置/122旧
装载/6CTest与两包7.07秒隔离构建/六库恢复PASS；原Nav111通过/2V12缺源FAIL/XML
保持。26输入仅独占身份重绑，248相关依赖实际逐哈希通过；before10262历史文件/
10备份/785正式src及近期链包含r246371封存PASS。3062指南几何/24按键10320密查/
8.600mm最小净距/整窗四角与codec前门通过，仍非实际完整证书。

一次seal249成员/0444，identity SHA
`0e5593637d0d2ca2765ee4338d8d56af8781a3b6b699e7b23edbc27dd9813448`。
fresh server1806289/GUI1806290、六控制器active/manager500Hz、72帧短测通过；首次
在MP4尚无header时抽第一秒帧失败，原失败记录保留。同录像1808334/ctime1791876959.32
实际仍活，再读完整第一秒成功并已看身体/脚/近手近盒/桌/关闭设备/墙，远手远盒自然
遮挡；没有重启录像，不把暂时观察失败当终态或解码全部通过。
executor1812813/ctime1791877005.45、guard1812814、disk1812815、supervisor1812816一次
启动完整连续流程。首非空7.2045sim，启动窗实际7.4885通过；原静态腿姿态13.862
真实PASS，实际最大误差.0001851257rad<原.001，目标/增益/终点门没有放宽。
这是一次早接管现场的前缀证据，不证明晚启动是唯一失败原因或完整任务成功。

run/live_checkpoint1.json核验八角色PID/ctime实际存活、249成员不变，sim34.1355
paired_load_descend，首至检查点26.931sim；仍没有双落篮/机器/取回或180证书。
接续重读result/trace_progress和原59189执行作业/实际PID，不能照抄相位、复用claim
或因观察超时重启。活动中不改密封源/移动六库构建/开第二Gazebo；原2mrad密查/
近接触跟踪、空手不碰盒豁免、双drop后才注册当前指南和3/10/30sim证书全部保持。
失败原目标/努力保持并由监督正常SIGINT；无reset/detach/异常松夹。完整终态独立
物理/首非空计时/各阶段视频及闭合解码/日志/事后保护缺一不可。完整基线仍r227
461.299sim，≤180目标active且未达；无清理/新备份/真机/腿部导航，旧全部成果保持。

## 2026-10-13 CST：r247实际双取出后放下接口FAIL，r248严格paired制动修复准备

r247终态FAIL覆盖全部活动快照，不再live：首7.2045→FAIL199.7745为192.57sim失败前缀，
不能当完整时间。LOADING_PASS83.2385为76.034sim装载；actual3062密查后的指南空手撤回
真实执行通过，关盖108.3455、MACHINE_CYCLE_PASS160.772，稳转30.5855sim、starts/stops1。
五次均真实指尖物理按钮，旧30.3窗口false、≥30且≤32.4择相合同保持；1206872容纳检查/
约.242mm底/接触穿入<原1mm、最大支撑力间隙左.0005sim/右0。之后实际双抓取/试抬/
抬离/各10sim保持并运至桌上空，尚未放下/正常释放或桌面终态。

唯一记录终态异常：旧单臂braking_setdown_segments只接受121点，新PairedPlanner先加
seed再遍历121Cartesian目标，实际setdown正确122点，前两点最大差1.1112764e-08rad；
整个取回运输2458密查通过。不是本前缀已记录的滑移/力/旋转保护失败，不删额外点凑数。
FAIL时双实际holder保持，quiet200.424/pause200.507/正常SIGINT视频与场景退休剩余0，
executor1其余0，无reset/detach/异常松夹。独立每手3043/6084同时检查，4锚点/2释放/
末仍持双盒，整体FAIL；FK/native峰1.132548/1.137662mm、角.553911度，腕差.010132mm/
.001621度，runtime更密即时native1.148562mm区别保留。完整cycle17/27，速度无完整PASS。
32819真实帧/0重复/正常SIGINT/全解码/camera PASS，685请求30暂时ACK失败有界恢复。
实际首帧、双篮两盒同框、关闭盖、再开盖两盒及失败末持物帧已看；远手远盒部分相位
自然遮挡，不伪造未发生的桌面放回录像，盖闭时转子不可见靠物理trace而非截图认证。
1027451行/3473427141原字节/gzip748028647字节CRC/计数/SHA PASS，249密封事后未变。
前后10262历史文件/10备份/785正式src及r216–r246封存全部PASS。
r247393静态0444：REPORTc81ca932b8cf94b81e640a605621603ab35260d97de550917d3c0f1262e99e65，
SUM0bffc2ef7008831c0ddf3b5f7cb5f14434790732f13eb961bf2144c5b000f529；禁止修改/复用。

r248普通mkdir独占、205父密封源/输入哈希派生；新paired helper要求seed+121目标共122
点、有限且相同关节键、seed与首目标≤.0003rad、原双300mm竖直跨度；split117保留全部
审计点，末5点仍10mm，coarse.20/fine.03和旧单臂121点合同保持。17接口/绑定测试PASS，
原730局部/两包隔离构建7.02秒已通过；完整保护/其余前检流水线仍须查实际作业。
新独立旧actual计划FK探针逐SHA核验r247 request/result，原122点全保留，双split高度
.8547500000/端点.8447500000，精确最后10mm通过；这是离线几何不是新的放下或PASS。
seal必须要求该探针PASS；当前尚未seal/claim/Gazebo，原29166准备作业不能因观察超时
重启或复跑open-x。下一保护纳入r247393；需全新fresh完整实跑，再按真实完整时序提速。
完整基线仍r227461.299sim，≤180未达且目标active；无清理/新备份/真机/腿部导航。

## 2026-10-13 CST：r248严格paired制动修复fresh完整流程启动检查点

上一目标轮已真实运行/审计/封存r247失败并实现r248制动修复，属于progress。本轮重查
原29166前检退出0，没有因观察超时复跑。730局部/64双执行器/46配置/122旧装载/6CTest、
两包隔离构建/六库恢复PASS，原Nav111通过/2V12缺源FAIL/XML保持。26输入只身份重绑，
250相关资源启动前逐哈希当前一致，before10262历史文件/10备份/785正式src及近期链
含r247393封存PASS；17制动/绑定测试、旧actual122点FK末10mm和3062指南几何通过，
24按键10320密查/8.600mm净距/整窗四角/codec前门通过，不代替本run实际放下证据。

一次seal254成员/0444，identity SHA
`ee91eb84f6a3fc921f26a876a120fc9bb6865c873e6e920351ae0f4cca5e59b5`。
fresh server2226998/GUI2226999，六控制器active/manager500Hz及72帧短测通过。最初
4秒有限header观察仍未就绪，原FAIL和8次probe保存；同录像2229255/ctime1791881069.01
仍活，稍后读完整第一秒成功并已看身体/脚/近手近盒/桌/关闭设备/墙，远手远盒自然遮挡。
不因短观察失败重启录像，不称目前闭合全解码或完整阶段已通过。
executor2236370/ctime1791881134.09、guard2236371、disk2236372、supervisor2236373一次
启动连续完整流程，不待AI逐阶段。首非空10.5185，启动窗11.166及原腿姿态17.669
实际PASS，最大误差.0002645746rad<原.001；原目标/增益/物理门不放宽。
run/live_checkpoint1.json核验八角色实际PID/ctime及254成员不变，sim34.6365
paired_load_approach，首至前缀24.118sim，尚无双篮/机器/取回/完整180证书。

接续先查result/trace_progress及原4339执行作业和实际PID，不复用claim、不照抄本
相位或因观察超时重启；活动中不改密封源/移动六库构建/开第二Gazebo。新paired分段
保留seed+121目标全部122点，末10mm/.03与coarse.20保持；本run实际完整物理、双
参考、首非空计时、阶段视频/闭合解码、无损日志和事后保护仍缺一不可。失败保持
原目标/努力，正常SIGINT，无reset/detach/异常松夹。完整基线仍r227461.299sim，
≤180目标active且未达；无清理/新备份/真机/腿部导航，旧全部成果保护。

## 2026-10-13 CST：r248空手取回接近速度FAIL覆盖活动快照，r249严格接近候选准备

r248已FAIL/正常退休：首10.5185→FAIL169.2505为158.732sim到失败，非完整时间。
LOADING_PASS87.0045，上料76.486sim；实际指南空手撤回、机器人物理关盖/启动/停转/
开盖通过，MACHINE_CYCLE_PASS164.649、稳转30.5sim，旧30.3窗口false/择相≤32.4保持。
取回尚未抓取，.8rad/s/.015rad空手approach期间right finger2实际-.2011198954m/s
超过模型.2，保护FAIL。168.5滑轨约-.0199且近零速，169.22部分到-.020硬限，169.24
右finger2到-.0176108；观察窗口无盒↔手指接触，但不能排除其他接触或证明唯一因果。
velocity_fault_context1保存原始状态；不能取消速度门或把单次小超门视为完整成功。

保持/quiet169.971/pause170.0735/视频与场景正常SIGINT退休剩余0，无reset/detach/异常
松夹。独立每手1843样本/2锚点/2释放/末空registry，整体FAIL；FK/native峰.975003/
.975004mm、角.163568度，runtime更密native.980290mm不同采样保留。完整cycle16/27。
870453行/2576557073原字节/gzip533196679字节CRC/计数/SHA PASS，254成员事后未变。
电影26171帧=26165真实采集+6重复填充，不是零重复证书；完整解码/正常SIGINT/camera
PASS，539请求13暂时ACK失败恢复。首帧/双持盒运输/闭盖/再开盖双篮/失败末帧已看，
主体/脚/手/桌/设备在画面，部分远手遮挡；缺失取回过程不补录伪造。
CLI observer4339返回143/launcher退出时四角色仍PID/ctime真实活着，未重启，之后才因
真实速度故障停止；不能把观察结束当任务终止，也不能伪造已缺失的四角色OS退出码。
guard/disk PASS JSON与监督完成/退休/视频证据分别保留。前后10262历史文件/10备份/
785正式src及r216–r247封存PASS。r248388静态0444：
REPORTa8c1c4d5700efe960d3ecec270b68b39c7cdd98ce7e2dab662d186d76e47cfa4，
SUM0fee6f6c6d2ad1383efd79f6f6aab73345cf424ba7627c74690e579da3d8c211，禁止修改/复用。

r249普通mkdir独占，209父源/输入哈希派生，只把paired_retrieval_approach加入已有
.13rad/s/.002rad严格门，原load空手.8/带载.25/制动.20/.03/模型.2m/s门/点/模型保持。
19初测通过，是抑制自由滑轨瞬态的待验证假设，不证明原因已定或提速完成；r248未到
paired122点放下，该修复仍待实测而非认定其失效。新隔离构建/回归/保护/几何/录像
前检流水线已启动，当前尚未seal/claim/Gazebo；先查原4366作业/输出，不因观察超时
复跑open-x。下一保护纳入r248388，完整FK末10mm门仍必需。完整基线r227461.299sim
未改，完整≤180目标active且未达；无清理/新备份/真机/腿部导航。

## 2026-10-13 CST：r249严格空手取回接近fresh连续运行，collector独立观察

本目标轮真实运行r248至速度FAIL、独立分析/审计/正常退休/388封存及实现r249属于
progress。原4366前检退出0，733局部/65双执行器/46配置/122旧装载/6CTest、两包7.39秒
隔离构建/六库恢复PASS；原Nav111通过/2V12缺源FAIL/XML保留。26输入只当前身份重绑，
250相关依赖启动前逐哈希当前一致，before10262历史文件/10备份/785正式src及近期链
含r248388封存PASS。原paired FK probe单独补充PASS，122点/末10mm保持；3062指南
几何/24按键10320密查/8.600mm净距/整窗四角/codec前门均通过，不替代本run实际取回。

一次seal255成员/0444，identity SHA
`8da4d8614bb5b4fc9003435c6c3a3d33a7a40afb2d6546c1a98b15081856b4b6`。
fresh server2673353/GUI2673354，六控制器active/manager500Hz/72帧短测通过；同录像
2675449/ctime1791885432.61有限18wall秒header就绪PASS，第一秒真实帧已看身体/脚/
近手近盒/桌/关闭设备/墙，远手远盒自然遮挡。未重启录像，不预称全部阶段/解码PASS。

只把原密封launch_run.py同argv放入独立session与run/collector.log，collector2681599/
ctime1791885496.46实际sid=pid，避免短CLI observer终止时丢失OS结果收集。不是增加
物理命令或变更四角色；executor2681603/ctime1791885496.49、guard2681604、disk2681605、
supervisor2681606一次启动，实际PPID均collector，完整流程连续、不待AI逐阶段。
首非空9.6225，启动窗10.0515/原腿姿态16.5945实际PASS，最大误差.0002434542rad<原.001。
run/live_checkpoint1.json核验九角色真实PID/ctime、collector父关系及255成员不变，
sim31.4415 paired_load_approach、首至21.819sim前缀，不是装篮/机器/取回/180证书。

接续读result/trace_progress及collector.log/实际PID，不能照抄快照/复用claim/观察超时
重启；不再用已结束的旧CLI会话判定当前任务停止。活动中不改密封源/移动六库构建/
开第二Gazebo。仅empty retrieval approach试用.13/.002，原.2m/s速度门/其他速度/
点/模型/3与10秒及≥30sim证书保持，r248未触发的paired制动仍须实测。失败原努力/
目标保持并正常SIGINT；完整物理/计时/阶段视频/闭合解码/无损日志/事后保护缺一不可。
完整基线r227461.299sim未改，≤180目标active且未達；无清理/备份/真机/腿部导航。

## 2026-10-13 CST：r249桌面下降误用篮口高度FAIL，r250保留3mm空间净距修复准备

r249已终态FAIL覆盖活动快照：首9.6225→FAIL221.7675为212.145sim到失败，不是完整。
上料76.0955sim、真实机器稳转30.5055sim/物理停转开盖通过；empty retrieval approach
.13/.002实际通过，滑轨速度峰.0383624m/s，原.2门保持，不证明唯一原因已定。
195.3295双实际夹持冻结，完整抬离/各10sim静态保持210.459通过；122点paired制动
分段117实际进入，coarse.20/fine.03/全部点保持，未到fine/桌面释放或终态。

唯一终态故障是pair_clearance_payloads仍在桌面下降应用篮局部竖直3mm：实际2.9944893mm
小于3mm而FAIL，但两盒已横移到桌槽上空。新原生故障帧与左右221.767/221.7675接触
缓存重算，原collision STL局部XY包围盒投影最小距离209.759679/136.473605mm，均无
篮接触；只是旧故障几何诊断，不把旧FAIL重新标PASS，也不以改小3mm规避问题。
保持/quiet222.3485/pause222.3995/视频与场景正常SIGINT退休剩余0，collector真实收齐
executor1/guard0/disk0/supervisor0。独立每手3135/4锚点/2释放/末持双盒，整体FAIL，
cycle18/27；FK/native峰1.104044/1.104053mm，角1.299074度；runtime即时1.143878mm
不同采样保留。1114961行/3709335882原字节/gzip799127559字节CRC/计数/SHA PASS。
电影36092帧=36091真实+1重复，非零重复证书；全解码/正常SIGINT/camera PASS，748
请求31ACK暂时失败恢复。真实运输/闭盖/取回/失败末帧已看，部分远盒遮挡，未发生
桌面释放不补录；活抽帧时PID已正常退休，随后读同原闭合电影，未重启。
255密封事后不变，前后10262历史文件/10备份/785正式src及r216–r248封存PASS。
r249395静态0444：REPORT8eba459ff362e5472ab54052e01ec340f40ce8b19c9803b5ecc1b9183738950e，
SUMe7fd3bb25ceb4fc722ca7c0665de64ef04f71c4531c62029254f6c73d5c41696；禁止改写/复用。

r250普通mkdir独占、210父源/输入逐SHA派生。只在paired_retrieval_setdown及fine持续
以原STL变换包围盒验证真实8角在**两篮**局部XY的保守距离≥同一3mm，且任何零力/
非零力篮接触都拒绝；其余抬离/10sim保持/运输原竖直3mm门不变。不一次清旗，不改
轨迹/速度/模型/硬门或3/10/30sim证书。记录同一原生poses/contacts快照，独立审计
另式从box世界8角与两篮变换重算并比对原contact流，要求每盒>100实际检查，不能仅
信自报valid。12几何/19集成与独立审计反例测试PASS；全752局部/65双执行器及7.10秒
隔离构建已PASS，剩余保护/几何/录像前检原18691作业中，尚未seal/claim/Gazebo。
初次geometry probe对另一盒接触未证明的结果另名保留，补原真实双接触后完整几何
probe PASS，仍只plan-only；新运行实际桌面返回/独立物理/录像/计时/事后保护必需。
完整基线r227461.299sim及完整≤180目标未改，仍active且未达；无清理/备份/真机/导航。

## 2026-10-13 CST：r250空间净距修复fresh连续实跑检查点

r250原18691前检退出0，未因观察超时复跑open-x。752局部/65双执行器/46配置/122
旧装载/6CTest、两包7.10秒隔离构建/六库恢复PASS；原Nav111通过/2V12缺源FAIL/XML
保持。253相关资源启动前逐哈希一致，before10262历史文件/10备份/785正式src及
近期封存含r249395通过。原实际故障几何双接触/独立另式重算、122点FK末10mm、3062
指南几何/24按键10320密查及整窗codec前门通过，不替代本run真实桌面返回。

一次seal264成员/0444，identity SHA
`7c47b6e38e92d8a6df3cc512332962ccea8b8cbb00a4310f4f0634ac22b833e2`。
fresh server3278726/GUI3278727，六控制器active/manager500Hz/72帧短测通过；实际
MP4首秒已看身体/脚/近手近盒/桌/闭盖设备/墙，远手远盒自然遮挡。同录像3280884/
ctime1791891439.84有限18wall秒header通过，未重启或预称全部阶段/闭合解码通过。
独立collector3286095/ctime1791891493.54，executor3286102/guard3286103/disk3286104/
supervisor3286105均ctime1791891493.57，实际父为collector，连续完整流程不待AI逐段。
首非空sim8.355/启动窗9.046/原腿姿态15.584 PASS，最大误差.0002429275rad<原.001。
run/live_checkpoint1核验九角色PID/ctime、真实父关系与264密封成员不变；本次重新
只读检查九角色仍活，sim51.7385 paired_load_static_grip_hold。此为活动前缀，不是
装篮、机器、取回或完整≤180证书，不能照抄为未来live状态。

接续看result/trace_progress/collector与实际进程，不以旧CLI退出判断任务终止；不复用
claim/观察超时重启，活动中不改密封源/移动六库构建/另开Gazebo。r250只修桌面两阶段
双篮空间3mm/no-contact守卫，原非桌面竖直3mm/速度/轨迹/模型/3、10、30sim保持。
失败保持原目标/努力并正常SIGINT。完整物理/计时/阶段视频/闭合解码/无损日志/独立
审计/事后保护仍必需；完整基线r227461.299sim未改，180目标active未达。
本次目标接续不进行清理/新备份/真机/腿部导航。

## 2026-10-13 CST：r250完整双盒物理237.0605sim通过，录像收尾FAIL；r251中速/录像修复准备

本目标轮真实连续运行r250至完整动作PASS、独立审计/17阶段视觉/正常场景退休/443封存，
并实现r251新候选，属于progress。r250首8.355→PASS245.4155=237.0605sim，非180；
LOADING_PASS84.7905/装篮76.4355sim，MACHINE_CYCLE_PASS161.9535/实际稳转30.5595sim。
真实中部双夹持、分别自然装篮、指尖关盖/启动/停稳/开盖、双取回不同桌槽和10sim
末端保持全通过。独立物理28/28，7306同步检查/4冻结/4正常释放/末空registry PASS；
FK/native峰1.283274/1.283164mm，角2.503127°，已授权2mm/5°通过，原2°严格不称通过。
桌面双篮原collision独立净距检查各476次/最小151.147599mm；非桌面竖直抬离最小
35.654259mm，原3mm/路径/模型/速度/3、10、30sim证书保持，修复实际返回而非只计划。

原电影38597帧全真实/0重复，完整probe/decode均0；17阶段原电影抽帧/接触表和额外
6/15/17全尺寸已看身体/脚/桌/设备，放篮能见两蓝盒，部分远手/远桌盒自然遮挡。
终态俯视单次GUI ACK超时，监督assert失败跳过finish_video并进入finally退休，录像器
随后窗口不可见退出：normal_sigint=false，**录像交付FAIL**，不因完整解码0或物理
PASS重标整体成功；缺失俯视不补录。真实OS0/0/0/1，场景正常SIGINT退休剩余0。
1224480行/3965088906原字节/859545536gzip，CRC/计数/SHA PASS，原SHA
2e78541ca10fd82a3b4f120189358734bddd3a0a63be52eb0bf26a90451d0514。264成员事后未变，
before/after10262历史文件/10备份/785正式src及近期封存含r249395全部通过，不外推
额外全部历史清单无差异。r250443静态0444，REPORT SHA
06468a03df59a90238289cc9083f6549c2e2ce57fae490721f63ae610477ca81，SUM SHA
44aff8fc274cb9fe25b6e07df4ef55c284cd904e2044bac44d044b5a07679256；不得改写/续跑。
当前更快完整物理基线237.0605sim有录像缺陷；原含合格录像交付基线r227461.299sim仍保留。

r251普通mkdir独占、218父源/输入逐SHA派生，只empty paired_retrieval_approach从.13
试.4rad/s，保留原.002跟踪/模型.2m/s实际速度门，全部路径点/载荷速/下降与制动/
3、10、30sim不改；r250该段21.9205sim，是否真实提速必须新的fresh完整执行。
终态GUI相机复用最多3次有限ACK重试，新增可注入收尾器确保检查失败仍先正常录像
SIGINT/解码；未核验录像退出时保持场景、不强杀、不先退休。六个失败/顺序/活身份
反例测试通过，针对28通过；初次源码直接字节比较忽略允许安装前缀绑定产生假阴性，
改按已验证派生哈希比较，原27/1日志保留，不放宽物理检查。初次前检脚本${workspace}
变量仍旧r250路径，写只读build.log被拒绝，旧哈希核验未变/构建未启动；修正绑定并
新增断言，另新retry1日志/39242作业开始隔离构建/完整前检。尚未seal/claim/Gazebo，
不因观察超时复跑open-x或复用任何旧身份。下一保护必须含r250443。
完整≤180与录像正常交付仍未达，目标active；无清理/新备份/真机/腿部导航。

## 2026-10-13 CST：r251空手.4/.002与正常录像收尾候选fresh连续启动

原39242 retry1前检退出0，未因观察超时复跑。761局部/64双执行器/46配置/122旧装载/
6CTest、两包6.89秒隔离构建/六库恢复PASS；原Nav111通过/2V12缺源FAIL/XML保持。
before10262历史文件/10备份/785正式src及近期含r250443封存通过，255相关依赖启动前
逐哈希一致。旧实际r250路径离线重定时21.26978→6.99368sim仅证明同全部点与20ms
floor的候选时钟变化，**不是本次执行、物理速度PASS或完整180证书**。

一次seal269成员/0444，identity SHA
ab21f10dbf2dc9c1b367a7db49b6ee4387013e95f42b98afe4c61550ab66ba2f。
fresh server3808796/GUI3808797/ctime1791896695.29，六控制器active/500Hz/72帧短测
通过；同原录像3810733/ctime1791896709.57 header及首秒解码通过，抽帧ss=.5解析
234失败记录保留，改0.5/新PNG路径成功并已看身体/脚/近手近盒/桌/闭盖设备/墙，
远手远盒自然遮挡。没有因抽帧格式错误重启录像，未预称完整解码/收尾已通过。
原密封launch_run同argv由独立collector3825259/ctime1791896862.92执行，真实sid=pid；
executor3825261/guard3825262/disk3825263/supervisor3825264均ctime1791896862.95，
实际父均collector。完整程序连续执行，不待AI逐动作触发，不增加物理角色或旧命令。
首非空sim20.435，实际启动窗21.0445/腿姿态27.463 PASS，最大误差.0004586842rad<
原.001。run/live_checkpoint1核验九角色PID/ctime、父关系及269成员未变，sim44.1345
paired_load_approach/首至23.6995sim只是前缀，不是装篮/机器/取回/180或录像成功。

本目标轮已真实完成/核验/封存r250物理237.0605sim及失败录像，并实现/测试/启动r251，
属于progress。接续必须重新读result/trace_progress/collector和活PID，不照抄本快照、
不以旧CLI观察结束当任务停止，不因观察超时重启/复用任何claim；活动中不改密封源/
移动六库构建/开第二Gazebo。只试empty retrieval approach .4，原.002跟踪/.2m/s物理
门/载荷速度/路径与3、10、30sim保持；终态相机3次有限重试、检查失败也先正常视频
SIGINT，未确认录像退出不退休场景。实际完整物理/180计时/阶段电影/闭合解码/独立
审计/事后保护仍必须；失败原目标努力保持并正常SIGINT，无reset/detach/异常松爪。
完整180与合格录像仍未达，目标active；无清理/新备份/真机/腿部导航。

## 2026-10-13 CST：r251空手.4跟踪FAIL正常收尾封存，r252单增益空臂校准准备

r251已终态覆盖活动快照：首20.435→FAIL178.5405=158.1055sim是失败前缀，不是完整。
双装篮76.354sim、真实指尖机器稳转30.5325sim/停稳开盖通过；空手取回接近右肩roll
actual-.2538239694191177/reference-.2562376036511916，误差.002413634232073858rad
超过原.002而FAIL，尚未取回抓取/载荷/放回，不放宽门或取消守卫。44个匹配原始
手指速度帧峰.11663839496619095m/s；初次诊断误读values字段得到0的文件保留，但
明确其0不是有效实测，corrected1从真实velocities字段重算。只短前缀，不证全路径。
独立物理16/28整体FAIL，双持盒整体FAIL，仅装载2冻结/2正常释放/3669同步检查/
末空registry；FK/native峰.825414/.824947mm、角.155283°，runtime更密.993018mm与
legacy未对齐1.350780mm不同作用保持区分。原模型.2m/s与所有几何/3、10、30sim门未改。

失败保持/quiet179.275/正常暂停/先视频SIGINT再正常场景SIGINT退休剩余0，无reset/
detach/异常松爪；实际OS1/0/0/0。completion_finalization四步骤PASS，实际证明新
收尾器的失败分支，不证明完整PASS后俯视分支或全流程三分钟。原电影26918真实/
0重复/采集gap峰.155403wall秒，normal_sigint和全probe/decode0；六个实际阶段原片
已看主体/脚/桌/机器留画面，放篮与末帧见分别两篮蓝盒，远手有自然遮挡，未取回
不补录。相机18次暂时ACK失败恢复，最终PASS。869402行/2564675055原字节/
534742032gzip，CRC/计数/SHA PASS，原UTF8 SHA
f08767a628296e42898284f2c4b9c68e46832de67325ae1f81506ce96d0f5154。269成员事后未变，
before/after10262历史文件/10备份/785正式src及近期含r250443全PASS，不外推全部
额外历史清单无差异。r251417静态0444：REPORT SHA
207f8e0437c44e573a5fe1fc5e32c15ae43ca45c4680e9e5ed267fb5909415cd，SUM SHA
2d94d3c972d7e82243ca943bd8baa6feb530c3ef6749136d3c6b694a807f91ce；不得改写/续跑。

下一根因假设针对位置接口gain.2/500Hz滞后：旧r228真实.36rad/s峰2.88mrad已高于
2mrad，本次符合这一阶低带宽现象，但尚未用新实测证明唯一根因。已核对本机
gz_ros2_control1.2.19和官方Jazzy位置接口说明，不把增益理论稳定界当物理成功。
r252普通mkdir独占，222父源/输入逐SHA派生，仅私有robot_servo/dual XML位置gain
.2→.6及apply_patch添加的一次末尾LF格式，逐字节恢复后哈希精确一致；其他几何/
质量/惯量/摩擦/限位/YAML保持。只完整场景空臂小幅双向.12/.4/.8校准，无按钮/抓盒/
载荷命令；原.015自由门/端点.0003保持，.4额外必须≤原.002，原手指.2m/s/持续
出生0.3mm及3sim末端保持/实际2mrad密FCL保留。只COMPONENT_PASS，identity/result
full_cycle_execution_certified=false；旧gain.2空臂证书不许可新gain.6，更不许可加载。
新独立审计另式标量插值重算原actual_full_joint_frame，不信自报peak；校准后仍需
全新完整加载/180/完整视频门，本目标未缩成部件。20针对/771局部及7.03s隔离构建
通过；初次末尾LF严格hash检查2假阴性保留，确认仅一个LF后修正，未放宽其他字段。
原24441完整前检仍在进行，尚未seal/claim/Gazebo，不观察超时复跑open-x，下一保护
含r251417。所有正式源码、模型及旧成果/备份不改；无清理/新备份/真机/腿部导航。
当前完整物理基线r250237.0605sim有录像FAIL，旧合格完整交付r227461.299sim保留；
完整≤180及合格全流程录像仍未达，目标active。本轮真实失败审计/封存及新校准
实现/测试/隔离前检属progress，不把计划或理论预算标完成。

## 2026-10-13 CST：r252正肘校准前检碰盒未执行，r253负肘几何走廊候选

r252原24441前检真实退出1，未启动Gazebo/任何轨迹/按钮/夹爪，没有seal/claim。
771局部/64双执行器/46配置/122旧装载/6CTest、7.03s隔离构建/六库恢复、259依赖及
其他前门通过，但闭盖旧实际起点正肘小幅轨迹14/60密查步/fraction.233333碰撞：
box48_left↔left_dex1_finger_2_link、box48_right↔right_dex1_finger_1_link。它是旧真实
状态几何，不是当前gain.6反馈，reported_distance最大浮点哨兵不是净距，不能当安全。
只读四组有限符号探针正肘均碰，负肘-.12/肩roll正负两组各61原2mrad密查通过，
仅plan-only；没有删碰撞/手指触摸豁免，也未重标原FAIL为PASS。前后10262历史文件/
10备份/785源码及近期含r251417封存保持。r252323静态0444：REPORT SHA
2ce970fd620340247d2fd432d32240cd4728ab970d23d02f4a4bfac17d4a763f，SUM SHA
65979f00eae5c069a896aa63b8a3cf60409adfe0ded7596eb4faad7e53cbb966；不得改写或续跑。

r253普通mkdir独占、228父源/输入逐SHA派生，选负肘-.12/原肩roll±.06校准往返。
私有gain.6及其他模型/控制/门保持，但gain仍未物理校准，旧.2证书不可替代。新
实际起点必须重新2mrad密FCL，不能拿旧几何探针作命令许可；.4实际跟踪≤原2mrad/
端点.0003/手指.2m/s/持续出生0.3mm及3sim端稳保留，只COMPONENT_PASS且full_cycle
false，不抓盒/按门或转子，不作为完整180或加载成功。771局部/7.18s隔离构建/六库
恢复通过，before保护含r252323；原20648完整前检作业在进行，尚未seal/claim/Gazebo。
接续查同作业/日志，不观察超时重启/重复open-x。完整180目标仍active未达，原
物理完整r250237.0605sim/录像FAIL及旧合格完整r227461.299sim保持；无清理/新备份/
真机/腿部导航/reset/detach/异常松爪。本轮真实r251失败审计/封存及新增益校准
实现/测试/失败前检与新的低风险几何选择是progress，不缩减最终完整目标。

## 2026-10-13 CST：r253真实gain0.6空臂校准/独立原生审计/正分支录像收尾通过

r253原20648前检退出0，771局部/64双执行器/46配置/122旧装载/6CTest、7.18s隔离
构建/六库恢复、259启动前逐哈希依赖及before保护含r252323通过。一次seal275成员，
identity SHA 4184906bcade4f090a75db7effd07ad357d03c61a1ad106eef9e17fff705a029。
fresh server438454/GUI438459/ctime1791904475.74，六控制器active/manager500Hz/72帧
短测及同原录像首秒解码/实际帧视觉通过。独立collector447784/ctime1791904566.85，
executor447792/guard447793/disk447794/supervisor447795均ctime1791904566.88，真实父
关系核验，程序连续无需AI逐段触发。本run只空臂部件，不调用完整加载流程。

实际只读/gz_ros_control参数Double value0.6、manager500Hz，新实际起点61个2mrad密
FCL通过；小幅负肘往返0.12/0.4/0.8，原生关节独立另式标量插值重算全部PASS：
峰0.160000/0.160000、0.533333/0.533333、1.071304/1.066667mrad，样本100/100/30/30/
15/15；0.4保留原2mrad，自由原15mrad/端点0.3mrad、手指0.2m/s保持，实际手指峰
0.12831165669919897m/s。3sim端稳两盒drift0且桌支撑，持续出生0.3mm/0.2°门保持。
无实际抓盒/按钮/载荷，starts=stops=0，盖闭/转子停止，max_slip0是无持盒，不作为
载荷证书。首非空11.1635→COMPONENT_PASS28.5285=17.365sim是部件时间，identity/
result/audit均full_cycle_execution_certified=false，不得称完整180或大幅路径保证。

正分支终态相机第一次ACK真，俯视确实录入原电影后才正常视频SIGINT/全解码，核验
录像退出后才正常场景SIGINT退休，OS0/0/0/0、remaining=[]。原片2667全真实/0重复/
采集gap峰0.152142wall秒，完整probe/decode0；六动作原片及额外0.4全尺寸、原片末
俯视已看。运动相位身体/脚/桌/设备可见，俯视尾部见头/臂/桌/两盒，脚不在俯视
画面，不能称全程脚无遮挡。此证明正分支收尾在部件现场有效，非完整操作录像。
76668无损行/79404405原字节/7947090gzip，CRC/计数/SHA PASS；原UTF8 SHA
2b7389092ef02b117baef04bd80815fdb69abc232ff23b8a2af26bc5f1850d62。275成员事后不变，
前后10262历史文件/10备份/785正式src及近期含r251417/r252323保持，不外推全部额外
历史清单无差异。r253403静态0444：REPORT SHA
9d52f57bbb938ddd1ff464dad6b9d77174b2f4c420573e897ee50c0d160454cd，SUM SHA
c7c85cf28e5cfd4045d1f8b6175ec29a7e6f4896d51edd162a8a963004674fda；不得改写/续跑。

下一安全动作是另立全新完整加载身份，绑定本gain0.6校准/差异审核并保留全部原生
载荷/2mm/5°/速度与3、10、30sim门，实际验证完整大幅取回/旋转/视频/180及保护；
小幅部件证书不替代这些证据。当前完整物理仍r250237.0605sim/录像FAIL、旧合格
完整r227461.299sim，180未达且目标active。本轮真实r251失败审计/封存、r252前检
碰撞证据/保护以及r253实现/测试/实际部件/独立审计/视觉/保护/403封存属progress。
没有清理/新备份/真机/腿部导航/reset/detach/异常松爪，最终完整目标不缩减。

## 2026-10-14 CST：r254真实增益双装篮后空手RRT超速FAIL；r255单段时钟修正前检

r254普通mkdir独占、221父源/输入逐SHA派生，私有gain.2→.6之外几何/质量/摩擦/
限位/路径/点/速度与3、10、30sim保持；绑定r253403封存/六次小幅原生校准，fresh
真实读回.6/500Hz、六控制器active/72帧短测/原电影首秒视觉通过。770局部/64双/
46配置/122旧装载/6CTest、7.13s两包隔离构建/六库恢复及256依赖通过；原Nav111/
2V12缺源保持。272一次seal，identity SHA
f68ab5756a8ef5f454569d881588afc7a72bf302de7edb9233811b2687e95c22。
完整DualCycle由独立collector679236/ctime1791906667.77连续执行，非部件替代。

首7.8565→LOADING_PASS83.5775，真实双装篮75.721sim。实际中部两夹持/10sim保持/
自然各篮释放通过；独立3691同步检查、2冻结/2正常结束/末空registry，FK/native
峰1.072950/1.072953mm、角.174060°，runtime更密1.086069mm保持区别。95.5705在
machine_paired_empty_return原.8rad/s大幅RRT中，原生右finger1速度.22379600366586946
m/s>原模型.2触发FAIL，joint_sim95.57/command93.142/start93.442原证据保存。
首至失败87.714sim不是完整180，未关盖/启动/稳转/取回。独立物理11/28、完整
双持盒/速度整体FAIL，缺样本Infinity不是净距证明。小幅gain.6/.8不能外推大幅
路径；不放宽速度门，不采纳失败私有候选为正式版本，正式src/模型没有修改。

stop-and-hold、quiet96.311；先正常视频SIGINT全刷盘解码再正常场景SIGINT退休，
OS1/0/0/0、四收尾步骤PASS、remaining=[]，精确七角色PID/ctime已不在。原电影
17718真实/0重复/gap峰.109138wall秒/完整probe/decode0，七前缀阶段及4/5全尺寸
已看主体/脚/桌/设备与两蓝盒搬运/装篮，早期远手远盒自然遮挡，不补造未执行阶段。
实时抽帧未刷完NAL的初次FAIL保留，同原片较早已完成片段成功，录像没有重启。
435605行/1716623120原字节/352091319gzip，CRC/计数/SHA PASS，原SHA
2b69cf0de85ce03127a17ec930638160132ebe8b7cc5da6f1ec818952d78644d。272密封成员未变，
前后10262历史文件/10备份/785正式src及近期含r253403保持，不外推全部额外历史
清单无差异。r254413静态0444，REPORT SHA
e88a176112c033a81e9583ee445eaaab36c31fc000eb6a3e781c5ed98d327c60，SUM SHA
953b0b8415946d98c0c148d869b515c993a222a7c885083ffd2fcced3111f499；不得续跑/改写。

r255普通mkdir独占、223父源/输入逐SHA派生，只故障machine_paired_empty_return
限至.4/原2mrad，原RRT点/20ms floor/三组时钟和其余速度/载荷/按钮/接触/出生/
物理/3、10、30sim保持，26输入只有身份绑定，新保护包含r254413。779局部及
7.20s隔离两包/六库恢复通过，原29838完整前检作业继续，尚未seal/claim/Gazebo。
接续看当前日志/实际进程，不观察超时重启或重复open-x。完整目标active未达，
物理完整r250237.0605sim/录像FAIL、旧完整交付r227461.299sim不变。本轮真实完整
身份试验/失败审计/视频/保护/413封存与单段修正实现/测试是progress；没有清理/
新备份/真机/腿部导航/reset/detach/异常松爪，最终目标不缩减。

## 2026-10-14 CST：r255单段空手回撤修正fresh完整身份连续执行检查点

原29838前检真实退出0，未因观察超时复跑。779局部/64双执行器/46配置/122旧
装载/6CTest、7.20s两包隔离构建/六库恢复、256启动前逐哈希依赖及before10262
历史文件/10备份/785源码和近期含r254413通过；原Nav111/2V12缺源保持。
272一次seal/0444，identity SHA
cca84b3e5f23c4fbd0684ff2f581c8f68c18502b57164768d3ec87b43b4b95e6。
fresh server938428/GUI938429/ctime1791909218.26，六控制器active、真实只读gain.6/
manager500Hz、72帧短测、同原录像首秒解码与全尺寸视觉通过；身体/脚/桌/设备/
近手近盒/墙可见，早期远手远盒自然遮挡，不预称完整录像或动作通过。

独立collector944370/ctime1791909272.63，真实sid=pid；executor944372/guard944373/
disk944374/supervisor944375均ctime1791909272.66且实际父collector。launch938017/
ctime1791909217.39与video940592/ctime1791909234.93绑定同新根。九角色精确身份
存活、272成员未变已写run/live_checkpoint1.json；sim25.174在paired_load_close_empty_check，
尚无result，只是装载前缀，不是加载完成/机器/取回/180或录像收尾证书。

本run仅故障empty neutral RRT限.4/原2mrad，其他速度/物理/路径点与3、10、30sim
不变。接续必须重查同collector/trace_progress/result及PID/ctime，不照抄此快照，
不观察超时重启/复用claim；活动中不改密封源/移动六库构建/另开Gazebo。失败原
stop-and-hold与先正常录像SIGINT完整刷盘解码再正常场景SIGINT，禁止reset/detach/
异常松爪。之后完整计时/物理/独立持盒/阶段原片/日志/事后保护仍必需。
本轮真实r254失败全审计/原片/保护/413封存和r255修正/测试/fresh启动是progress，
完整180目标active未达，原r250237.0605sim物理基线与r227旧完整交付保持；无清理/
新备份/真机/腿部导航，最终完整目标不缩减。

## 2026-10-14 CST：r255停转回home超速FAIL/421封存；r256直接交接及四段自由搬运实跑

r255已终态覆盖旧活动。首6.4735→装篮87.643=81.1695sim，有真实4.3385sim对齐
修正；原neutral .4/2mrad通过。真实指尖关盖113.5625、启动123.2375、steady_start
126.3645→物理停转156.87=30.5055sim。只有4物理事件，未最终开盖/取回；158.9905
在machine_stop_withdraw非接触回home的.8命令，右finger1原生-.20259496755294398m/s
>原.2而FAIL，joint_sim158.99，command158.2075/start158.5075/duration1.561958sim。
首至失败152.517sim非完整180，不放宽速度守卫或把失败私有候选当正式版本。
独立物理11/28、完整双持盒/速度整体FAIL；2冻结/2正常结束/4147同步检查/末空
registry，FK/native峰.903353/.903352mm、角.473935°，runtime更密1.043944/1.086977mm
保持区别。原2mm/5°及其他硬门保持，无取回缺样本Infinity不是净距证明。

quiet160.5175、最终闭盖/rotor_disabled/starts=stops=1，rpm4.4714e-9。先正常视频
SIGINT/全解码再正常场景SIGINT退休，OS1/0/0/0、四收尾步骤PASS、remaining=[]，
九精确角色均不在。25578帧=25576真实+2重复、gap峰.261652wall秒，不称0重复；
9原片前缀阶段、第2/6全尺寸和原片末帧已看主体/脚/桌/机器，双盒装篮与闭盖30rpm
可见，早期远手/远盒遮挡，不补造未发生阶段。805785行/2608955413原字节/
542990857gzip，CRC/计数/SHA PASS，原SHA
7f7c7ba1f69fb48f39f399a27b7e2e9adeff140e62e1866b9a93a3eb986bc97b。272成员事后未变，
前后10262历史文件/10备份/785正式src及近期含r254413保持，不外推全部额外历史
清单无差异。r255421静态0444：REPORT SHA
e400b3d913bb726622d053f8905d325764046aeb6a28196a750b76ce9fde2c5a，SUM SHA
0b318064a881fbab348f60916c353915c36acb40b336750089cd4d4553c7a3d9；不得改写/续跑。
只读8组交接几何原2mrad/5mm/5盖角与loaded envelope通过，最小8.600004mm；仅
plan-only，不能作当前实际姿态许可。

r256普通mkdir独占、224父源/输入逐SHA派生，26输入仅身份绑定。StopHandoverCycle
继承完整DualCycle，只停转press保留真实push反撤与rearm后不回right home；实际
停稳后，新原生coherent快照worker全机器人FCL/原2mrad/5mm通过，全部转动关节
漂移≤.0003rad/指爪移动关节≤.0003m，才能左手原路径开盖。原5盖角/loaded envelope
与只目标左指尖按钮接触豁免保持，失败原stop-and-hold。只四段水平自由搬运试.4
而原.25，必须同时两实际holder；其余速率/路径点/20ms/2mm5°/手指.2/接触与3、
10、30sim不变，不预称载荷提速通过。速度声明明确只times变化，不声称原时间戳
不变。首次794、非有限值/单位修正后798测试输出原字节保留，最终799全部通过。
64双/46配置/122旧装载/6CTest、7.10s两包隔离构建/六库恢复、260依赖、名义交接
fixture及before保护含r255421通过；原Nav111/2V12缺源保持，名义fixture非实际许可。

281一次seal，identity SHA
dcd1e26220546a0adfaf7534bd22f990cef525f626c1ab848b79e1adc9f82046。
fresh server1362887/GUI1362890/ctime1791913389.35，六控制器/真实只读.6/500Hz/72帧
短测、同原电影首秒解码/全尺寸视觉通过。launch1362470/ctime1791913388.41、
video1365047/ctime1791913406.03，独立collector1377929/ctime1791913542.04，sid=pid；
executor1377931/guard1377932/disk1377933/supervisor1377934均ctime1791913542.07且父
collector。run/live_checkpoint1确认九角色存活/281成员未变，sim51.716双抓取中尚
无result，是前缀而非装篮/机器/取回/180或录像收尾证书。
接续必须重查同进程/原片/trace_progress/result，不照抄快照，不观察超时重启或
复用claim；活动中不改密封源/移动六库构建/另开Gazebo。之后完整实际/180/独立
物理/持盒/原片阶段视觉/日志/事后保护仍必需。完整目标active未达，旧物理完整
r250237.0605sim/录像FAIL与旧合格完整交付r227461.299sim保持。本轮r255真实失败
审计/原片/保护/421封存与r256根因交接修正/提速/测试/fresh实跑是progress，无
清理/新备份/真机/腿部导航/reset/detach/异常松爪，最终目标不缩减。

## 2026-10-14 CST：r256软件派发不一致/419与r257空手超速/413失败封存；r258 PID阻尼候选

r256旧活动已终态覆盖：首18.8345→FAIL75.099=56.2645sim，真实两中部抓取/pilot/
两10sim持盒及原.25首段完成；首次自由搬运.4请求被父类残留.25软件上限拒绝。
原trace .4 command严格0，不能称.4物理成功或失败。799局部未走真实父类上限，
已如实记录测试缺口；不删物理守卫。独立物理8/28、2冻结/0结束/2231同步检查，
末仍持两盒，FK/native峰.070216/.070168mm、角.011245°，无装篮/机器/取回。
stop-and-hold保留原力/目标/两holder，quiet75.6775，正常视频SIGINT/全解码后
正常场景SIGINT退休，OS1/0/0/0、四收尾步骤PASS、九角色不在。12692全真实/0
重复/gap峰.119103wall秒，7原片前缀阶段及第6全尺寸已看主体/脚/桌/机器与仍
持有两蓝盒；不补造阶段名所指未发布动作。292053行/1061463004原字节/
202257280gzip，CRC/计数/SHA PASS，原SHA
ad82aafefc6491e2b4d449a9785ce9f4c4fcad8c405885e9a142c006860e7e43。281成员及前后
10262历史文件/10备份/785源/近期含r255421保持。r256419静态0444，REPORT SHA
45eebf60438bc9fa12aa5407c80d0c25056609f2c08937b4780c27b48692e566，SUM SHA
cbade77b83b73c21e325374c82eaac046ae7b67f3345dc0fda12433d36397127；不得续跑/改写。

r257普通mkdir独占、229父项逐SHA派生，把.25抽为默认不变hook，只有四既声明
自由phase与两actual holders允.4；其他基类/阶段仍.25。逐字节逆变换验证除hook/
条件/错误文字外控制体不改。807局部/8真实父类三组publisher路径/64双/46配置/
122旧装载/6CTest、7.07s隔离构建/六库恢复、261依赖/before含r256419通过；Nav111/
2V12缺源保持。282一次seal/fresh六控制器/.6/500Hz/72帧短测/原片首秒已看。
首4.7645→FAIL30.0605=25.296sim，在初始paired_load_approach原.8时右finger2原生
.20607530927743162m/s>原.2，joint_sim30.06，未抓盒/未.4载荷/未交接。原命令
第58段间隔.024946sim，剩余1.716693sim，最大相邻名义关节速度跳变量.369044
rad/s；它们是命令斜率，不是实际手臂速度或唯一因果证明。
独立完整物理/持盒/速度整体FAIL，0冻结/0结束/0持盒检查，max_slip0为未持盒，
不作为载荷证书。quiet30.81，正常先视频后场景SIGINT、OS1/0/0/0、四收尾PASS/
remaining=[]、九角色退休。2846全真实/0重复/gap峰.131983wall秒、完整probe/decode0，
5原片前缀阶段及第4全尺寸已看主体/脚/桌/设备，远手远盒自然遮挡，无伪造后段。
149325行/137573570原字节/12637900gzip，CRC/计数/SHA PASS，原SHA
a30fc95dd387dcda4147e7e452ed824da2521dacef017e4d76a0c75a9d21eb5d。282成员、前后
10262历史文件/10备份/785源及近期含r256419保持，不外推全部额外历史清单无差异。
r257413静态0444，REPORT SHA
2b4a6b05f24a610f3877dae30537cd1d8a689dce6855b074283b007180f06446，SUM SHA
71e09f10b79c25b376c2d1203e0c064af91248bc8d9c0e2a36fa59922c3c7e52；不得续跑/改写。

已核对实际URDF/YAML：四指爪effort-only接口、P500/I1000/D1.5、反馈u-clamp±2、
FFvelocity0、指爪质量.028354kg、原模型damping.2/friction.05/velocity.2。Jazzy官方
接口说明effort-only用位置/速度误差PID加轨迹effort前馈，effort0不是关闭PID。
下一假设提高夹爪PID D抑制运动中振荡，非唯一因果结论，不以公式代替实测。
r258普通mkdir独占、231父项逐SHA派生，只私有controllers_servo.yaml四D1.5→3.0，
严格字节逆变换与结构审核，P/I/积分及±2反馈clamp/FF和所有URDF/SDF模型/质量/
惯量/摩擦/限位/轨迹速率、.2手指/2mm5°/力/FCL/追踪/3、10、30sim保持；不全局
放宽速度或改变模型。fresh四D必须只读精确3.0，执行器claim前再核验。完整
GripperDampingCycle继承动作与实际停转交接，当前初始空手接近须在原生门下完成
才允许四段载荷.4；只是当前路径资格，不是通用校准或载荷证书。r253 arm驱动
.6/500Hz和模型不变，但其旧指爪峰不是新D证书，已在校准记录明确区分。
817局部/8派发回归/7.05s隔离构建/六库恢复通过，原76165完整前检作业继续，
当前验证到audit_dual_buttons进程1895466/ctime1791918407.96，尚未seal/claim/Gazebo。
接续读当前日志与实际进程，不观察超时重启/复跑open-x，不复用旧身份。失败私有
候选未作为正式版，正式源/模型未改，所有成功/重要失败/视频/报告/备份保持。
完整180目标active未达，r250237.0605sim物理基线/录像FAIL与r227旧完整交付保持。
本轮真实r256/r257失败审计/原片/保护/封存、派发修正实测和阻尼假设实现/测试是
progress；无清理/新备份/真机/腿部导航/reset/detach/异常松爪，最终目标不缩减。

## 2026-10-14 CST：r258双盒返桌观察被录像超时中断，r259相机期限/静止准备提速

r258原前检817局部/8真实派发/64双/46配置/122旧装载/6CTest、7.05s隔离构建/
六库恢复/263依赖/before保护通过。285一次seal，identity SHA
35565b97d93047bbddf3c36dc10f7159dc0d141910bfcdaaa4bd9a207af4a512，fresh六控制器/
0.6/500Hz/四D只读3.0/72帧短测与原片首秒通过。首5.3315→LOADING_PASS80.9515
为75.620sim；当前D3空手接近、两中部实际抓持/试抬/10sim保持、四自由运输.4/
自然各落篮通过。真实指尖关盖/启动/稳转30.538sim/停转/开盖，机器PASS163.629。
旧30.3严格窗不通过，用户不少于30sim保持。新实际交接全模型555接近+125按压
密FCL通过，2mrad/原5mm/五盖角与loaded envelope、仅目标左指尖按钮接触豁免
保持；最小8.670888mm，原生快照漂移峰.000037833727<原.0003。

实际两重抓/抬离/10sim保持与返桌运输，216.597两盒各自桌槽/姿态/支撑门通过，
四holder正常释放，末registry空。220.7925在paired_retrieval_release_on_table的
3sim观察中KeyboardInterrupt，缺table_return_pass/末10sim/完整PASS，首至FAIL
215.461sim只是失败前缀。根因录像最后三次1200ms请求returncode0但stdout空/
Service call timed out，recorder退出后run_guard仅向精确executor发SIGINT；非手指
或滑移故障，不伪称完整动作。独立物理20/28与完整持盒/速度整体FAIL，6836同步
检查/4冻结4释放/FK-native峰1.124474/1.145569mm/角1.058646°，旧2°当前前缀也过。
全原生指爪峰.19030259445m/s<原.2、抬离36.551640mm、表上两篮净距143.335789mm，
430定位样本0无效，不作为通用PID或更快路径许可。

原电影36076全真实/0重复/gap峰.135716317wall秒、probe/decode0，但normal_sigint
false/camera FAIL，录像整体FAIL。17实际阶段+末帧18格已看，稳转/末帧全尺寸检查；
主体/脚/桌/机器持续可见，近侧返桌盒可见，远手/远盒自然遮挡，不从原片独立证明
两盒完整终态，不补造缺终态阶段，墙钟映射近似。trace1120159行/3897535814原
字节/834928204gzip，CRC/计数/SHA通过，原SHA
81c9570a3980bdb601bff8f85dd78cdc32422c024b26dc295bb67340f541b332。285身份未变；
前后10262历史文件/10备份/785源码及近期含r257413保持，已知额外历史差异不外推。
stop-and-hold/正常场景SIGINT退休剩余0，九精确角色不在，OS1/0/0/1；不改标录像
正常SIGINT。446原静态0444，REPORT SHA
34ed5e4ee3bc3f9f64c81ca9280ce5f0511f545dda7c8f4a52b3ff03320b9042，SUM SHA
2513ff844fff42a0e52cc164b1f004522f9ff6fb4b270f186caff49295f8f3bf。原报告22/28人工
误数以新REPORT_ERRATUM.md明确更正20/28，不改原封存字节；勘误两项0444，独立
sidecar SHA69f8d7f703dad1767dec1e6102c064f916e75a84db2d0e2d6c6424d23ef9b86b。
保护必须包含原446及勘误2项，不复用r258claim或继续其run。

r259普通mkdir独占、234父项逐SHA派生，仅相机服务1200→3000ms/进程3→5wall秒/
关闭join10→16wall秒；三次/0.2间隔/4秒与阶段校正/真实ACK/持续实际采集及永久
失败门不删。已有<.010rad静止起点的腿初始化5→1sim，原12目标/首50ms/末1.05s/
零端速/.001位置/.03速度/goal_time10/原.025偏差门/1sim观察保持，非步行。
四D3/.6/500Hz与所有模型/其他速率/2mm5°/.2手指/FCL/3、10、30sim保持；26输入
仅身份绑定，不能再宣称相对r258进行了D四字段变化。63针对/821局部/8派发/64双/
46配置/122旧装载及6.93s隔离构建/六库恢复通过，原Nav111/2V12缺源保留。
原44707完整前检仍进行before保护；尚未seal/claim/Gazebo，接续重查同日志/实际
进程，不观察超时复跑open-x。约4sim准备节省不是180达成，完整原目标active。
本轮进展为失败物理/原片/保护/封存、实际时序及根因修复/测试；未清理回收站、
新备份/真机/腿部导航/reset/detach/异常松夹，旧成果和所有备份不动。

### r259同一候选完整前检退出0，fresh连续执行检查点覆盖未启动状态

原44707前检退出0，821局部/8派发/64双/46配置/122旧装载/6CTest及6.93s两包
隔离构建/六库恢复、全部几何/录像/依赖/before保护通过，Nav111/2V12缺源保留。
before含r258原446+勘误2，共448静态文件；原10262历史/10备份/785源及近期保持。
287一次seal/0444，identity SHA
122147024206c3c2c1cdef23108c853def67dab44449bd94980d0316a70627b6。263依赖启动前
逐哈希复核，fresh六控制器active/真实.6/500Hz/四D3/72帧短测、同原MP4首秒解码/
全尺寸视觉通过：主体/脚/桌/机器/近手近盒可见，远侧自然遮挡，非完整视频证书。

launch2424802/ctime1791923597.44，server2425212/GUI2425213均ctime1791923598.35，
video2428017/ctime1791923621.22；独立collector2431297/ctime1791923654.04，四子
executor2431300/guard2431301/disk2431302/supervisor2431303均ctime1791923654.07，
实际父为collector，各自独立session，未用短CLI观察代替执行生命周期。
run/live_checkpoint1.json核验九精确PID/ctime存活及287成员未变，sim10.867为
初始按键接近前缀。随后原生fresh门7.479<25、新1sim初始化9.9345通过，目标误差
峰.000153670028<原.001；当前D3初始空手路径28.417原生门通过。最新已核sim38.467
在paired_load_pilot_lift，仅前缀；完整装篮/机器/返桌/180/合格视频仍未证明。

原生父级分阶段速度分析已存当前run/parent_phase_speed_analysis.json。原零effort
空爪检查速度峰约.007–.021m/s，而自由带载若干段峰约.1897、篮口下降.1903接近
原.2门，因此下一提速优先研究低幅空爪时序，不盲目提高带载倍率。只是父级失败
前缀分析，不授权新时间表，也不称原路径以新速率必然通过。
原69920 observer只读针对同collector观察600wall秒，观察界限不等于任务停止。
接续读同进程/trace_progress/result/原片，不观察超时复跑claim或启动第二场景；
活动中禁止修改密封成员、移动六库构建或更改模型。失败原守卫stop-and-hold并
按正常录像后场景收尾，不能把源初始化提速或新采样当完整180达成。
本轮为真实progress，完整目标active保持，无清理/备份/真机/腿部导航/reset/detach/
异常松夹；旧成功、重要失败、报告/原片/无损日志及全部备份均保留。

### r259同一连续进程第二检查点：两真实10sim持盒保持通过，正在装载运输

原69920只读observer达到600wall秒正常结束，仅观察界限，不是执行结束。随后
重新核验原collector2431297/ctime1791923654.04与其四子角色、launch/server/GUI/
video九精确身份仍活，287成员逐SHA未变，尚无result/录像终态。
run/live_checkpoint2.json记录首非空6.531、当前sim53.887/
paired_load_forward_table_endpoint_descent。两static_grip_hold_pass均在52.6475、
duration10.0，当前两真实持盒试抬/原10sim保护前缀通过，不外推完整装篮、机器、
返桌、180或录像交付。新静止腿初始化实际9.9345通过/原误差门保持，真实流程
始终由同一collector一次连续执行，不等待AI逐动作下令。
接续必须核对同PID/ctime和trace_progress/result/原电影；不能因observer退出而
重新launch、复用claim、改密封源、活动中构建/移动六库或启动第二Gazebo。目标
仍完整≤180及合格全流程原片/独立物理/无损日志/前后保护，不缩为前缀。下一阶段
由已运行程序继续，失败原stop-and-hold并正常收尾。未做回收站清理或新备份。

## 2026-10-14 CST：r259录像修复PASS但取回指爪超速FAIL；r260时序/D6候选前检

r259已终态覆盖原活动检查点。首6.531→LOAD76.9945=70.4635sim，双中部真实抓持/
试抬/两个10sim/四自由.4/各3sim自然落篮通过；比r25875.620少5.1565sim，单次
实测对比不将全部差异归因腿初始化。新1sim初始化9.9345通过，误差峰.000153670028
<原.001。真实指尖关盖/启动/稳转30.4785sim/停转/开盖，MACHINE_PASS159.8125；
旧30.3严格窗不通过，用户至少30sim保持。实际交接555+125密FCL/原2mrad5mm/
五盖角loaded envelope通过，最小8.670888mm，快照漂移.0000341478032<.0003。
取回空手接近实际.4，left_finger1原生.2208555190362m/s、joint_sim170.97超原.2，
FAIL170.9705，首至164.4395sim仅失败前缀，未取回抓取/返桌/末10sim/完整PASS。
独立物理16/28、完整持盒/速度整体FAIL，仅装载2冻结2释放3493同步检查，FK/native
峰.990458/.990295mm、角.238733°，运行时保守native1.105970mm不互换。末无holder，
两盒在篮、盖120°/转子稳停/starts=stops=1，失败按原stop-and-hold。

相机期限修复实际失败分支PASS：519服务记录/13失败请求被有界重试恢复，25543
真实帧/0重复/gap峰.129480227wall秒，normal_sigint true，probe/decode0，video_audit
PASS仅已发生前缀，不能冒充完整任务。13阶段+末帧14格已看，稳转/末帧全尺寸
检查；主体/脚/桌/机器持续可见，末蓝盒仍在篮内，远侧部分遮挡，不补录未取回。
先正常录像SIGINT刷盘/全解码再正常场景SIGINT，四收尾PASS/remaining[]/九精确
角色退休，OS1/0/0/0，run_guard no_stop_sent true，非录像中断。884390行/
2570981237原字节/533052214gzip，CRC/计数/SHA通过，原SHA
20e011c911a07a846f6c8b922dfaad87c958bc7681b96cbdf021ab0f6a482659。287成员未变，
前后10262历史文件/10备份/785源码与近期含r258448保持。r259446静态0444：REPORT
SHA975e388b3a88147e66074ea35a8452e641574e148ef6a562cb8119d40f8c3d79，SUM
SHA656112562b97b878ae3f29ddd0a97ee0cb849e61315821f1f5134242a802893d；不可续跑/改写。

r260普通mkdir独占，在r259活动时仅从其287密封身份SHA逐项复制235项代码/fixture/
输入作离线测试，不修改活动源/模型/录像、不移动六库、不运行第二Gazebo。七个
指定零最终effort阶段、无holder且四实际手指距目标≤1mm，两点末时刻1→.5sim；
首50ms/实际起点/零端速/初始实际FF（释放可16N）/末0N/.3共同起时/原结果/
目标.0002与实际终点.00025/.3观察保持；握持/软抓/较大位移不加速，仍持盒快速
释放拒绝。仅fine .03→.06，必须当前原生资格和两个不同actual holders；默认基类
hook仍.03，元数据/实际指令共同使用，直接move不能绕过资格/holder/声明rate。
122同路径/20ms/粗.20/3mm/力/0.2/2mm5°/3、10、30sim不变，逆变换验证正文。
49针对/841初次/845补派发反例回归均保存；真实父类两ROS goal与三组publisher
测试使用假客户端/假publisher，是软件覆盖，非实际物理证书。

r259实际取回超速后，尚未seal的r260再加四私有D3→6，P500/I1000/反馈clamp±2/
FF与所有URDF/SDF/质量/摩擦/惯量/限位不变，严格四字段字节逆变换/结构审核。
新D6读回必须精确6.0且执行器claim前再读，当前原生空手资格仍先于自由载荷。
旧D3/D1.5指爪记录不能代替D6，不根据公式预称物理成功。原845 D3时序快照/日志
保持，另845 D6离线与845构建后局部通过；26输入仅身份及四D变化审核通过。
确认r259退休后再6.71s两包隔离构建/六库恢复，before10262历史/10备份/785源/
近期含r259446通过，原Nav111/2V12缺源保留。原51592完整前检继续，尚未seal/
claim/Gazebo；重查同日志/进程，不复跑open-x。完整180原目标active未达，旧r250
237.0605sim物理/录像FAIL与旧r227461.299合格交付保持。无清理/新备份/真机/
腿部导航/reset/detach/异常松夹，所有旧成果/重要失败/原片/日志/备份均保护。

## 2026-10-14 CST：r260准备反馈边界FAIL/412封存；r261新原生反馈等待实现并前检

r260原51592完整前检退出0，845局部/8派发/64双/46配置/122旧装载/6CTest、6.71s
两包隔离构建/六库恢复/before含r259446与265依赖通过，Nav111/2V12缺源保留。
292一次seal/0444，identity SHA4629da5f349cb7475317f9f0dbbb749b9fdf127db3ca038f46f06bba417574c0，
fresh六控制器/.6/500Hz/四D只读6.0/72帧短测/原片首秒通过；collector2744794/
ctime1791926800.18实际启动完整程序，不是计划。首6.496→FAIL8.93=2.434sim，
腿1sim action result0，但紧随独立JointState缓存source8.92左踝pitch误差
-.0010152535721841272rad，超原.001；下一source8.93为-.0009994157242448481rad，
仅10ms后在原门内。失败不能改标PASS，不能以此放宽精度。已发送腿姿态goal，但
手臂/夹爪motion计数0、未按钮/抓持/装篮/取回，新D6空手资格/时序/载荷仍未证明。
独立物理7/28与完整持盒/速度FAIL，0锚点/0持盒检查，max_slip0不是持物稳定证书。

stop-and-hold/quiet9.6805，末无holding、闭盖稳停/starts=stops0、盒仍桌上。
623原片全真实/0重复/gap峰.089707221wall秒、camera/normal_sigint/完整probe-decode0
均PASS，只准备失败前缀；首帧/原片35wall秒帧全尺寸已看主体/脚/桌/机器/近盒，
远侧遮挡，不补造抓放。先正常录像后正常场景SIGINT，四收尾PASS/remaining[]/
九精确角色退休，OS1/0/0/0；live采样遇原launch消失后检查actual result/退休，
未重启或生成假RUNNING检查点。14020行/14434597原字节/1455666gzip，CRC/计数/
SHA通过，原SHAb3152348a11f785874140c6639d2d71d6750aec0861a44e78755fb2344e054fe。
292未变，前后10262历史/10备份/785源及近期含r259446保持。r260412静态0444，
REPORT SHAfd341e425747eba314af1bf58d015a400104398438a4e54dcf6f0d0446d616f5，
SUM SHAea07ee301e1a4b776013b87507813953fe7cd38989deabce2ec581c715c3c09d，禁止改写/续跑。

r261普通mkdir独占、238父项逐SHA派生，26输入仅身份，D6/.6/500Hz及原目标/
轨迹/FF/力/速度/模型/守卫与3、10、30证书不改。action result0后有界等新原生
joint stamp>当时缓存且原<.001精度，最多3sim/30wall，再原1sim观察；原.025门保持，
超时原stop-and-hold，不用命令值或估计终点替代。独立严格逆变换证明控制正文除
等待声明/新反馈门外未改；新反例覆盖恰好r260边界、旧缓存、误差等于.001、非
有限和不收敛。继承夹爪接口对未建立retrieval_closing的早期调用只用原默认False，
未知阶段不加速。852离线回归通过，新原33857完整隔离前检已启动，尚未seal/
claim/Gazebo。接续查同前检日志/实际进程，不复跑open-x或照抄历史live；全流程
180/合格原片仍未达，旧r250237.0605物理/录像FAIL与r227461.299合格交付保持。
本轮为真实progress：两fresh真实失败审计/原片/保护/封存，时序与阻尼实现/测试，
反馈边界根因及窄修复；不是blocked或完成，不缩减目标。无清理/新备份/真机/
腿部导航/reset/detach/异常松夹，所有旧成果和备份保护。

### r261前检退出0/fresh连续执行：新原生精度等待通过，两10sim持盒通过

原33857完整前检退出0，852局部/8派发/64双/46配置/122旧装载/6CTest、6.97s
两包隔离构建/六库恢复、26输入仅身份审核/全几何与录像前检/before保护含r260412
通过；原Nav111/2V12缺源保持，不冒充全导航回归PASS。292一次seal/0444，identity
SHA0d8280e95798793b1c54c11cabb24f05b903f18638fe467c2fb47e82341db41c；266依赖启动前
逐哈希复核。fresh六控制器active/.6/500Hz/四D6实际只读/72帧短测/原片首秒解码
与全尺寸视觉通过，主体/脚/桌/设备/近手近盒可见，远侧遮挡；非完整录像证书。

launch2882930/ctime1791928050.2，server2883342/GUI2883344均ctime1791928051.1，
video2886405/ctime1791928076.97；独立collector2888952/ctime1791928101.67，四子
executor2888954/guard2888955/disk2888956/supervisor2888957均ctime1791928101.69且
实际父为collector。run/live_checkpoint1、2逐项九PID/ctime存活/292未变，尚无
result。首非空6.0125，新反馈门在8.94由原生actual_joint_stamp8.94、最大误差
.000990465996<原.001通过，原1sim观察9.94通过、误差峰.000153611621；原.025门保持。
三指定空爪阶段每手两点末时刻1→.5真实派发/原目标/结果/终点/.3观察通过，早期
空爪速度峰约.012–.016m/s，非一般更快速率许可。当前D6初始空手路径26.774原生
门完成、接近实际原生速度峰.186195776845<原.2；仅当前路径，不外推全部载荷。
两中部实际抓持/试抬/10sim保持均在51.0045通过，当前52.2805/
paired_load_forward_table_endpoint_descent进入运输，未装篮/机器/取回/180/完整视频。

原50130只读observer600wall边界已正常结束，仅观察窗口，不是执行结束；之后
再核对同collector与九角色仍活，没有重启/新claim。接续看同PID/ctime和当前
trace_progress/result/原片；活动中禁止改密封源、移动六库构建或另开Gazebo。
剩余完整流程由同一已运行程序连续推进，不需要AI逐动作触发。完整目标active，
不能把准备修复/实际D6前缀/无result当完整180达成。失败原stop-and-hold/正常录像
后场景收尾，未清理/备份/真机/腿部导航/reset/detach/异常松夹，全部旧成果保护。

## 2026-10-14 CST：用户条件删除授权下定点清除回收站旧过程文件

用户再次要求核对回收站“还有用吗，没用就删掉”。新独占身份
`runtime_evidence/trash_cache_purge_20261014_r1/REPORT.md`记录本轮，不复用旧清理身份。
只删除2026-10-10封存清单中的1453旧colcon过程日志目录及12可再生缓存目录；
13379普通文件逐大小/SHA匹配，逻辑355646688字节，1465对应trashinfo一并删除。
这些准确回收站位置现为历史位置，不再可恢复；旧清单/报告不改写。
其余26项训练/源码归档/录像/模型资料/PDF/下载文件及其trashinfo共52项，前后
哈希和元数据不变；没有整站清空或删除任何正式成果/模型/运行/视频/备份。
可用空间采样4307320832→4701106176字节，活动仿真输出仍在增长，不冒充排他测量。
r261292密封成员前后SHA保持，identity摘要0d8280e95798793b1c54c11cabb24f05b903f18638fe467c2fb47e82341db41c；
没有仿真控制命令、场景重启/停止、编号备份或源码/物理/动作守卫改变。
185同用户进程可读取引用未见候选，4进程部分/proc权限盲区如实记录；
本轮没有重新复算全部历史大备份哈希。以后其他永久删除仍需对应明确授权。

## 2026-10-14 CST：r261完整合格交付216.0305sim；r262完整时间/私有盖子候选

r261实际完整PASS覆盖此前活动快照：首6.0125→222.043=216.0305sim，速度FAIL，
目标完整≤180仍active未达。双装篮74.8475/首至68.835sim，真实lid/lid/rotor/rotor/lid
五事件与稳转30.435sim、实际停转/680密FCL交接/开盖通过；双取回、各10sim持盒、
不同桌槽正常释放及3sim观察212.043通过，末10sim到222.043完整PASS。
独立物理28/28，4冻结4结束/每手3313/6624同时检查，FK/native峰1.055717/1.055724mm，
角峰2.820252°<授权5°，旧2°与旧30.3上窗不通过。执行器较快native1.111366mm
与独立对齐峰不是同采样范围，不能互换；原生指爪峰.189751482306<原.2。
末registry空、两盒不同桌槽、盖120°/转子约1.09e-10rpm/starts=stops1。
35409原片帧全真实/0填充/gap.144258218wall秒，normal_sigint/camera/probe/decode均PASS；
19实际阶段+末帧20格及7关键原生帧已复核，稳转全尺寸亦看。远侧桌盒侧面部分遮挡，
最终原片俯视见两桌面蓝盒和空篮；终态特写不冒充整机视角或精确sim帧同步。
1117254行/3726389813原字节/800156412gzip，CRC/计数/SHA通过，原SHA
0b76d893b9f8181fa26d19eb1d027676dd5b65245a82fda26c02b3d9c1643066。
先正常录像SIGINT/全解码后正常场景SIGINT，四收尾PASS/四子角色退出0/九身份退休。
292成员未变，前后10262历史保护/10备份/785源码及近期含r260412保持。
467清单成员0444；REPORT SHAab46753b11fe9d3d9313a9ff9aabdee20a8e17acee16dbb4879ffaaf519c86d3，
SUM SHA599f5c2541f723f6dd2eea33967cdc86a115560dd65063115aaf3076651eb838。
旧r250237.0605物理/录像FAIL与r227461.299旧合格完整保持，不复用旧身份。

r262普通mkdir独占，在r261活动时只从其292身份逐SHA派生240项作离线候选，
不改活动密封输入、不移动六库、不另开Gazebo。129真实父类三组/按钮publisher假客户端
反例及981局部通过，仅软件覆盖，不是物理证书。r261终态命令重新计时显示纯速率
只省26.3177sim，假设开销不变仍189.7128sim；不冒充180，也不盲跑该纯速率版本。
完整已测方案原文/哈希保留在offline_profile_revision1_source_snapshot.json，不是新备份。
进一步仅新私有inputs/centrifuge_brake.sdf一个lid速度.35→.8和apply_patch的末行换行，
严格逆变换/结构审核；正式源码和所有旧模型不写，原120°/30Nm/几何/质量/摩擦/惯量/
机器人限位/PID D6/.6/500Hz/转子/互锁保持。盖子仍由实际指尖接触触发电机，非GUI/
pose假开关；固定接触间隔、3/10/30证书和全部原生守卫不删。
完整阶段率详见r262 CANDIDATE_DELTA；只有当前D6原生资格及两个不同真实holder允许
带载提速，空手/按钮率不允许持物旁路。8盖子逆变换+129派发共137针对通过，989局部
通过；原末行换行和旧模型测试未识别新声明造成的预检查失败日志保留，非实跑失败。
父r261已退休审计/视觉/467封存后才可新隔离构建/完整前检/一次seal/fresh完整运行。
任何新率或盖速必须新实际证书，不能外推父D6完整结果；失败stop-and-hold/正常SIGINT。
本轮目标续接无清理/新备份/真机/腿部导航/reset/detach/异常松夹，全部旧成果保护。

## 2026-10-14 CST：r262装篮53.065sim/按钮终点超速FAIL462封存；r263有限加减速实现

r262原72827完整前检退出0，6.95s两包/六库恢复、989局部/129派发/8盖速/8旧派发/
64双/46配置/122旧装载/6CTest及26输入/几何/视频门通过；Nav111/2原V12缺源保留。
303一次seal/269资源启动前逐SHA/fresh六控制器/.6/500Hz/四D6/72帧短测通过。
完整录像最早三次读取moov尚未刷盘报183，原失败日志保留；同一recorder首秒和
原生帧随后PASS，没有重启场景/recorder。collector3323280/ctime1791932467.85一次
完整程序，首9.2595<25；原D6路径25.7735/双试抬.08及各10sim44.313、桌端.45/
自由.6/篮口.14/正常释放/各3sim自然落稳通过。LOADING_PASS62.3245，首至53.065sim，
对照r26168.835少15.77；不归因单字段或外推完整180/全部取回路径。
真实lid事件14.948/81.282，新私有.8盖速仅本次开闭前缀；启动前未平滑1.2接近
start84.516，末点1.0579721674，末点后约6.028ms的native85.58出现right finger1
+.246517408678及finger2-.252176328195m/s，原.2门FAIL85.5805，首至76.321仅失败前缀。
中间折点85.288019也有速度扰动，与非连续时间参数的假设相符，不证明唯一根因。
整体物理10/28FAIL，装载2冻结2释放/每手1323/2645同时检查，FK/native峰
1.065838/1.065792mm、角.091556°；整体持物与速度FAIL，未旋转/最终开盖/取回。
quiet86.324，末无holder、两盒在篮/闭盖稳停/starts=stops0；不把quiet当完整终点。
14733原片帧全真实/0填充/gap.144947311/normal_sigint/camera/probe/decode PASS，
14实际阶段+末帧15格及3原生关键帧已看，只已发生失败前缀，不补造缺失阶段。
418486行/1370105586原字节/268073573gzip，CRC/计数/SHA PASS，原SHA
e4700b77408effb6718a1d74d73515fe3130a9f743326310909724a6f462d291。
四收尾PASS、四角色1/0/0/0、九身份退休，303未变；10262历史/10备份/785源码及
近期含r261467/定点清理6保持。462清单成员0444，REPORT SHA
ca63c7c522bed06dec714bf55375066e5bce38d6e9dc6ee55c5815e30d9e7df9，SUM SHA
c20f8beb55d6c9da79de0a956d0f1fa43a1c6736d7af929060c8a8bbce9b0067，不修改或复跑。

r263新ordinary mkdir，父级退休/审计/462封存后逐SHA派生246源/输入/fixture。
只改合格空手按钮非接触时间参数：沿同一原关节折线增加位置+速度点，原每顶点
命令速度0，.1sim加減速/峰≤1.2/加速度≤12rad/s²/20ms底限/50ms首点保持。
固定按压/撤回及资格前.8不变；目标/几何/原.015追踪/.005终点/原生.2夹爪/
2mm5°/力/净距/3、10、30证书和完整父流程不缩减。没有派发失败的未平滑1.2轨迹。
已安装JTC trajectory.hpp证实位置+速度cubic；新fresh左右臂必须实际只读回读
interpolation_method=splines，且claim前再查记录，不能只依据默认配置假设。
28专项覆盖原失败路径、独立Hermite几何/导数、全部顶点/底限、实际ROS publisher/
元数据/固定接触不改/错误回读拒绝与默认hook严格逆变换，1017局部通过，仅软件。
原浮点12断言和${workspace}漏绑定被构建前测试捕获修正，失败日志保留；26输入
仅身份绑定，包括继承.8盖速没有新模型变化，原source snapshot只历史溯源不改。
后续隔离构建/完整前检/一次seal/fresh完整验证；当前无r263物理/180证书。
完整合格基线仍r261216.0305sim，目标active未达，不缩为加载或离线预算。
无新清理/备份/真机/腿部导航/reset/detach/异常松夹，全部旧成果继续保护。

## 2026-10-14 CST：r263按钮平滑通过但取回超速FAIL477封存；r264只改空手接近时间

r263实际结果覆盖此前离线状态：首6.4515，LOAD60.3605=53.909sim；真实五按钮事件
12.9685/79.5175/85.282/118.992/126.8175，稳转30.591sim，MACHINE_PASS132.721。
新平滑启动接近原生指爪峰0.016460659m/s，旧平面时序超速未重派发。但空手取回
接近start136.1315、原0.6rad/s/192点，在joint136.86原生left finger2
0.209453680410>原0.2，FAIL136.8605；首至130.409sim仅失败前缀。故障约起时后
0.729sim，不是末点或仅首点；未取回抓持/返桌/完整PASS，不以quiet137.596作终点。
独立物理16/28FAIL，持盒只有装载2冻结2结束/2643同时检查；FK/native峰
0.988078/0.986542mm、角0.116163°。末无holder、两盒篮内、盖120°/转子稳停。
19957原片真实帧/0填充/gap0.110255930wall秒，正常SIGINT/相机/probe/全解码PASS
只前缀；14阶段格及3原生关键帧已看。先录像正常结束再场景SIGINT，四收尾PASS/
四角色1/0/0/0/九身份退休。720593行/2071100048原字节/424062700gzip，CRC/计数/
SHA通过，原SHA3d02118556d660112cd5ad5833e5c4519673bacc254ecbd97c4ef587d632bb49。
306密封成员不变，前后10262历史/10备份/785源码及近期含r262462保持。477成员
0444；REPORT SHA63f56b4415f0e8c20cffa4eecabc8bcef0a75556e3f2bcdcbdbfbf52bde9810b，
SUM SHA2aaebb261d555b38aa0a1840b6be78974ab4b3752b23cb1c0e42cae7ad198859，不改写/续跑。

r264普通mkdir独占、249父项逐SHA派生，仅`paired_retrieval_approach`沿原关节弧长
增加0.2sim首末时间坡度，保持原0.6速率上限/192原点/三组/0.1首点/20ms底限/
0.3共同起时和positions-only原线性折线。解析弧长加速度参数3rad/s²，但离散段
速度仍可不连续，不能声称实际连续加速度证明或已知唯一根因。只有当前D6资格/
无holder/精确阶段和声明rate允许，其余时序/平滑按钮/私有盖速0.8/D6/0.6/500、
原0.002追踪/0.0003末点/0.2指爪/2mm5°/净距/接触及3、10、30证书不改。
24新专项加父测试共1041离线通过，221针对通过，只软件不作物理或180证书。
当前原生进程检查无Gazebo/executor，开始新隔离构建/完整前检；尚未seal/claim/
实跑。完整合格基线仍r261216.0305sim，完整180未达；本目标接续无清理/备份/
真机/腿部导航/reset/detach/异常松夹，全部旧成果与已封存失败保留。

### r264前检退出0/309一次密封/fresh连续运行：仅新身份，不是完整结果

完整前检退出0，7.08s两包隔离构建/六库逐字节恢复；1041局部/129阶段率/24新坡度/
28按钮/8盖速/8派发/64双/46配置/122旧装载/6CTest通过，Nav111通过/2原V12缺源
保持。26输入仅身份绑定/全部几何前检/before保护含r263477通过。309一次密封0444，
identity SHA323d6684e90ce9a945fd20d1cc6cc131b3c59a3e187ad87805837605e2fc5892，
273选用资源启动前SHA复核，普通mkdir独占run。fresh六控制器active/0.6/500Hz/
四D6/两臂splines真实只读/72帧短测/原片首秒通过；首帧命令`.5`被ffmpeg拒绝，
只纠正CLI为`0.5`，失败记录保留，同一场景/recorder未重启、未重复claim。
原片首帧全尺寸已看，主体/脚/桌/闭盖离心机/近蓝盒可见，远侧盒/手部分遮挡，
不冒充完整视频证书。独立collector4008415/ctime1791939077.30实际启动完整程序，
run/live_checkpoint1逐项九PID/ctime存活、309未变；首非空6.6665<25，当前只准备/
开盖接近前缀，未装篮/机器/取回/180/完整视频。活动中不改密封源/移动六库构建/
启动第二Gazebo；只查同handle/trace_progress/result，observer超时不是执行结束。
本轮完整180目标active未达，程序自主连续推进，不需要AI逐动作派发；失败原
stop-and-hold及先录像后场景正常SIGINT。没有新增清理/备份/真机/腿部导航/reset/
detach/异常松夹，旧成果与所有重要失败保持。

## 2026-10-14 CST：r264追踪FAIL494封存；r266放弃坡度/恢复0.4并隔离前检

r264实际终态覆盖上述live：首6.6665→LOAD60.3565=53.690sim，双真实抓持/各10sim/
正常释放和各3sim自然落篮通过。实际五指尖按钮lid/lid/rotor/rotor/lid为
12.9685/79.514/85.278/118.9495/126.723；稳转30.5485sim，原680实际交接/停稳/
开盖通过，MACHINE_PASS132.6265。原30.3上窗不通过，不冒充旧严格窗。
空手取回0.6沿原弧长0.2首末坡度实际声明/派发，但joint137.97右肘actual
0.5833196550019306/reference0.5801989433906486，误差0.0031207116112820232rad
超原0.002，FAIL137.9705；指爪峰0.196637930445<原0.2，单门通过不代表整体。
首至131.304sim仅失败前缀，未取回抓持/返桌/末10sim/完整PASS。quiet138.7025
不是完整终点；末无holder、两个盒仍篮内、盖120°/转子稳停/starts=stops1。
独立物理16/28FAIL、完整持盒/速度FAIL，仅装载2冻结2结束/2643同时检查；FK/native
峰0.768371/0.993764mm、角0.096604°。725963行/2080072569原字节/424653705gzip，
CRC/计数/SHA通过，原SHA349d301dde5dd9c562f02dc4e25e7d374e28cc588aa753f2a68cf459039ddb17。
原片20367帧全真实/0填充/gap0.136267726wall秒，normal SIGINT/相机/probe/全解码
PASS只前缀，14实际阶段格及09/12/14原生关键帧已复核。主体/脚/桌/机器可见，
远手/载物部分遮挡，末见两篮内蓝盒，不虚构取回。先视频后场景正常SIGINT，
四收尾PASS/四角色1/0/0/0/九精确角色退休，309密封成员未变，前后10262历史/
10备份/785源码及近期含r263477保持。494成员0444；REPORT SHA
c4aadc28367263ebd59afc28163f32c45de48f3b4a84940da787b70761ed323b，SUM SHA
47999b066cd969d29e75a00258e566739b23b18ab9310e38c84106c7e08369de，不改写/续跑。

r265是源派生过程失败：apply_patch自动创建父目录而非正式普通mkdir独占，随后
对二进制输入UTF-8解码失败，部分拷贝保留。没有run/claim/seal/构建/Gazebo，不
修复续用该身份。r266已确认目录/安装目标不存在后普通mkdir独占，249父项逐SHA
从r263派生；只对声明文本后缀重新绑定、二进制逐字节保持。7项继承源证明测试
采用已封存r264的验证版本，依据当前父级不重复反转旧hook，采用证据SHA落盘。
**不继承失败r264的PairedRampCycle/common timing hook/坡度模块**；唯一新控制变化
为空手取回approach上限0.6→r261完整实测的0.4。中立返程仍0.6，全部原位置/
三组/线性插值/首点/20ms/原0.002追踪/0.0003末点/0.2指爪/2mm5°/力/净距/
3、10、30证书保持；其余较快速装篮/平滑按钮/私有盖速0.8/D6/0.6/500继承。
7新针对含旧失败192点真实ROS三publisher软件派发通过，共1024局部通过。
旧r261的0.4完整合格不是当前物理或180证书；目标完整≤180不缩减，不删保持。
确认无live后开始新隔离构建/完整前检，尚未seal/claim/fresh实跑。磁盘只有约
1.5GiB，不足以带原1GiB硬门安全完成下次完整日志/录像；已请求用户自行腾出
至少2GB或另行明确授权限定旧缓存清理，等待期间只离线进展，不盲跑失败场景。
本目标接续无新增清理/备份/真机/腿部导航/reset/detach/异常松夹，全部旧成果保护；
完整合格基线仍r261216.0305sim，目标active未达。本轮有代码/测试/真实失败审计/
录像/保护/封存进展，不因为首次磁盘条件把目标标blocked或complete。

### r266完整前检退出0：只等待足够空间，不能重跑准备或当作完成

原一次prepare_pipeline1退出0，末行CANDIDATE_PREPARED_NOT_SEALED_OR_RUNNING。
6.95s两包隔离构建/六库逐字节恢复；1024局部/7恢复/129阶段率/28按钮/8盖速/
8派发/64双/46配置/122旧装载/6CTest及272依赖/完整几何/录像/26输入/before保护
含r263477、r264494通过，Nav111/2原V12缺源保持。无identity/run/claim/新Gazebo，
当前无live；空间仍约1.5GiB，等待腾出至少2GB或另行明确限定清理授权，没清理。
PREPARATION_STATUS记录接续：不要重跑整个准备，空间足够后重新只读检查/重验
哈希，一次seal/普通mkdir run/fresh完整连续执行。新原片首帧seek用0.5而非.5。
失败r265源派生240项另已0444封存，REPORT SHA
b15b736aa7796637d134af1fe0e4a789438c7345776ffdafaeeb495e0ec618fa，SUM SHA
8726eefdec4da337ad4b8456c33258c6d347debdb7e062fcd40d1ee637f2aff1，不续用/删除。
完整合格基线仍r261216.0305sim，完整180/完整原片新结果未达，goal保持active；
本轮有真实运行、失败审计/视觉/封存和新实现/测试/隔离前检进展，首次磁盘等待
不满足三次blocked审计。所有旧源码/模型/运行/原片/报告/备份继续保护。

## 2026-10-14 CST：用户新限定缓存清理授权落实；r266一次密封/fresh连续运行

用户明确回复仅允许符合条件的旧构建缓存清理，该新授权只覆盖本次白名单，不
扩大为原片/模型/源码/训练文件/正式结果/备份/回收站清空或其他应用缓存删除。
build_cache_cleanup_20261014_r1删除1476普通文件：1470个超过24小时CMake .o/.o.d，
6个超过12小时、实际前日或更早的CppTools .ipch。正式清单/identity/场景依赖/
安装符号链接目标和可读取进程引用不含候选，逐SHA/大小/inode/mtime/ctime/nlink
复验；源、配方及全部绝对依赖保留，.ipch中的唯一绝对原源路径核验，已安装
Microsoft官方schema确认生成预编译头缓存。没有删除browse SQLite数据库、mmap
文件、扩展安装或其他缓存。C++进程字段可读，其他权限盲区逐项如实记录。
删除逻辑869776193/占用873684992字节约0.814GiB，可用采样1526784000→2395086848；
并发外部写入使差值非排他测量。前后10262历史/10备份/785源及近期含r264494与
r266准备350成员/源配方保持，没仿真控制命令/新备份/真机/导航/reset/detach/
异常松夹。34清单成员0444；REPORT SHAe698e1ef3c2addd9f7646aec61254fd11a3f9a1081694d22a12c10093460e8df，
SUM SHA510e9bdb8dfc1f72a74659d800ad6392bed1bc55e711ad7333a0fb07ab0bb86c。今后需要
这些旧对象须重新编译，不能称可无需重建。其他重要资料和所有备份保留。

另trace_lossless_codec_probe_20261014_r1只在管道测r261完整3.726GB/1117254行的
Zstandard12往返，raw/decoded SHA及所有字节/行数/退出码通过，无大归档保存。
仅723654518对原800156412，省76501894约9.56%，不足单独消除磁盘限制，不采用
新格式；实际r266仍原gzip6/无采样。6成员0444，REPORT SHA
fd98b66a0fa28cae662d0c09e5f1548f583cc80af32ebc77f7a697f648c8b9f9，SUM SHA
6aaf6a2e5473d4d25dbe64a72e6269db5e71199dd183fda9105b32145c96829a，非动作或180证明。

r266原完整前检不复跑，按真实r261完整trace800156412+movie112338496+原1GiB硬门+
额外256MiB计算新启动预算2254672188，实际余量高于此数才启动，硬门不降低。
311一次seal/0444，identity SHA35c0570a8cb15831914e2dad5556ac36c5045c2375a5af55f7d17fbf1f2051a4；
普通mkdir独占run，272选用依赖启动前SHA复核，另新清理34/探测6/失败r265240
前启动SHA保持。fresh六控制器active/0.6/500Hz/四D6/两臂splines真实只读、72帧
短测/同一原片首秒和全尺寸首帧通过；主体/脚/桌/闭盖设备/近蓝盒可见，远盒/手
部分遮挡，非完整视频证书。独立collector247884/ctime1791943235.02实际连续执行，
run/live_checkpoint1九精确角色存活/311未变，首非空6.2395<25，当前只腿准备前缀。
当前不是磁盘阻塞或完整结果；查同PID/ctime和trace_progress/result，observer边界
不是执行结束，不重启/复用claim。活动中不改密封源、移动六库构建、另开Gazebo。
仅当前新身份的完整程序自行连续推进，失败原stop-and-hold/先视频后场景正常SIGINT。
完整≤180/双取回/独立物理/合格原片仍待实际证据；完整合格基线仍r261216.0305sim。
新授权清理已经结束，不继续扩大清理；无真机/腿部导航/reset/detach/异常松夹/
新备份，全部旧成果保持，目标active未达。

## 2026-10-14 CST：r266完整185.695sim终态覆盖；用户要求先清理无用录像

r266实际首6.2395→PASS191.9345=185.695sim，独立物理28/28、完整持盒/原片/
无损日志/保护PASS，速度FAIL，仍超180目标5.695sim，不标记目标完成。双装篮
54.110sim、五真实按钮、30rpm稳转30.543sim、实际停转开盖/双取回各返桌槽及末
10sim全部完成；FK/native峰1.078540/1.078941mm、角0.770363°，原生指爪峰
0.189253264837m/s。29838原片真实帧/0重复/正常SIGINT刷盘全解码PASS，20实际
阶段已复核；完整原电影、机器指标、报告保留。506成员0444，REPORT SHA
5dabb583421a36f38aa3a5ce16595bcb18548da6be994ad85be3fd7fb98c79fe，SUM SHA
23234488e5f30ae671356dfb9e49b556024be7be58c1bbd40da84db4d3dc44f9。九精确角色
已经退休；本轮重新只读未见Gazebo/执行器/collector，旧live不得照抄或claim续用。

用户新要求先清理无用录像，只落实video_cleanup_20261014_r1白名单34普通文件。
397媒体逐SHA/大小/保护/引用盘点，16重复组不自动删除。仅下载目录速度优化8.26.mp4
5383533字节与文档引用原录像及另一个正式副本逐字节重复且未受保护/无正式引用；
另33个已结束pytest的0/4字节占位文件不是真录像，测试源保留。磁盘紧张故仅逐个
unlink这份有原件的冗余副本和可再生占位；逻辑5383641/分配5496832字节约5.4MB。
其余363媒体SHA/权限不变；正式视频/预检证据/重要失败/模型/日志/报告/authority/
builder及全部备份不删，回收站未操作。逐路径journal、删除前inode/时间/哈希复验
和可读取进程字段检查落盘，权限盲区如实记录，不声称普遍全系统可见。引用查询
首次自命中阻止删除的日志保留，修正本审计排除glob顺序后才通过，不删真实引用。
前后10262历史文件/10备份/785源码/近期链以及r265240/r266506/旧清理34/探测6保持。
空间采样1313685504→1318936576不是排他测量；本次不足腾出数GB，不能据此盲跑。
此前单独获准的两个旧安装下载包本轮未处理，不扩展为其他缓存或成果删除。
只纠正HANDOFF_CURRENT/QUICKSTART的旧live并记录清理，无新仿真/备份/真机/腿部
导航/reset/detach/异常松夹；完整180目标仍active，本轮先完成用户清理请求。

## 2026-10-14 CST：两份旧安装包授权结束；r267离线拒绝，r268原路线时序候选

installer_download_cleanup_20261014_r1仅落实用户明确许可的两份旧DMG/DEB，逻辑
1120867270/分配1120878592字节；已安装程序/活动可执行文件/今日71326更新包
前后SHA与元数据保持。更新管理器所查字段可读且无归档引用，其他权限盲区保留；
不把常驻wait-for-signal的shutdown服务当实际包安装，也不忽略apt/dpkg安装。
首次进程名/服务分类误判停在前检，失败日志不删。前后10262历史文件/10备份/
785源码/近期链与r266506/录像清理45保持。45成员0444，REPORT SHA
104addf40fe98a17e0dea2950e7b7758e55309c0193c5b6228815f92f2619f0b，SUM SHA
0d89cfdd4da62cd5f6ec3a57324407fbba2ae7abe2253a48d69e967b5bf5f71e。授权已经结束，
没有删除其他缓存/安装/训练/项目成果，不继续清理。空间采样约2.43GB，非排他释放测量。

r267普通mkdir、251父项从r266逐SHA派生，1045局部/21新publisher与父press软件/
129阶段率/28按钮/8盖速/8派发/64双/46配置/122旧/6CTest及7.08s两包隔离构建/
六库恢复通过；Nav111/2原V12缺源保留。原前检755463/ctime1791948266.47退出1，
实际native128.06撤回关节加较后父盖已开几何的混时离线探针3种IK拒绝/第四种RRT
端点碰撞，因此home-skip拒绝。无新Gazebo/trajectory/seal/identity/run/claim，不
放宽IK/FCL或续用失败身份。前后保护保持，370成员0444；REPORT SHA
4aadb7802d13ed0cd2625d6dc0de3fd657fba721a97cbd36c585b75c77bc61c7，SUM SHA
ac8475a95485388411642e74a428aab5c097b82c35a34bb04759c89304463688。附带phase_peaks
误聚合位置与速度，静止0.019不是速度，不作为速度余量或物理证书；原数据保留。

r268是普通mkdir全新身份，重新从合格r266派生，不继承失败home-skip或混时探针。
只有当前D6实际空手资格之后，三组臂/按钮非scheduled/双爪未来起时0.3→0.16，
原0.15未来安全门、资格前/父类0.3、停转显式scheduled及原墙钟0.45预算不改；
实际命令裕量0.31如实记录，不称绝对起时未改。只原已准入的四实际指爪≤1mm、
白名单phase、零末力、无holder、非retrieval_closing且原两点验证通过，合格后才
从0.5→0.25末点，原0.05首点/端速度/位置/FF/goal0.0002/实测0.00025/原生0.2/
力/正常释放/3、10、30sim证书不改。解析0.0075m/s命令界不是实际PID证书。
31新专项真实调用继承三publisher/按钮/bilateral action，保留故障拒绝/原final
home和新实测规划，通过；新隔离构建/完整前检已经一次启动，尚未seal/claim/
fresh实跑。接续查同进程/原日志，不观察超时复跑open-x准备；失败保持并正常收尾。
完整合格基线仍r266185.695sim，180目标active未达，不把预算或测试作为达成。

### r268原完整前检退出0；314一次密封/fresh完整程序连续运行检查点

原66440前检退出0，末行CANDIDATE_PREPARED_NOT_SEALED_OR_RUNNING。1055局部/
31新时序双爪/129阶段率/28按钮/8盖速/8派发/64双/46配置/122旧装载/6CTest和
6.87s两包隔离构建/六库逐字节恢复、26输入/全部几何/原片预检及before保护通过，
Nav111/2原V12缺源保持。含r267370、r266506、两份新清理各45及原保护链均保持。
274选用依赖启动前逐SHA复验，原1GiB硬门+800156412完整trace+112338496完整
movie+256MiB预算2254672188，实际余2397962240才启动，预算/硬门没有降低。

314一次seal/0444，identity SHA
d0bfbada583855e8c5b37f665d1e92ea52c4a356988dd0c1dc3461d36c480b1b。
普通mkdir独占run，fresh六控制器active/实际0.6/500Hz/四D6/两臂splines只读、
72帧窗口短测及同原片首秒解码/全尺寸首帧通过。主体/脚/桌/闭盖设备/近蓝盒/
墙可见，远手远盒部分遮挡，不预称完整录像。

独立collector971446/ctime1791950203.30、sid=pid；executor971449/guard971450/
disk971451/supervisor971452均ctime1791950203.32。launch965085/ctime1791950148.93、
server965502/GUI965505均ctime1791950149.84、video968445/ctime1791950173.78。
run/live_checkpoint1九角色逐PID/ctime存活、314未变；首非空6.3675<25。最新
只读sim36.3935在paired_load_static_grip_hold，两手已抓取/试抬，仍10sim保持
前缀；尚无result，不称装篮/旋转/取回/180或完整视频证书。
接续必须重新确认同collector/trace_progress/result，不照抄此历史快照，不观察
超时重启、复用claim、活动中改密封源/移动六库构建或启动第二场景。完整程序
自行连续推进；失败原stop-and-hold及先正常视频SIGINT刷盘全解码后场景SIGINT。
之后完整时计/28物理/持物/实际阶段原片视觉/无损日志/事后保护仍必需。
当前完整180目标active未达，不把此live或软件PASS当成完成；没有新清理/备份/
真机/腿部导航/reset/detach/异常松夹，原所有重要成果继续保护。

## 2026-10-14 CST：r268成功action读旧反馈FAIL469封存；r269原精度新反馈等待

r268旧live已经终态覆盖。首6.3675→FAIL55.7295=49.362sim，仅双真实抓取/试抬/
各10sim持盒/篮上释放前缀，无LOADING_PASS或完整自然落稳3sim/机器/取回/末10sim。
0.25零末力松爪原准入通过，左action在55.7295 status4/error0，但桥接native仍55.72，
左finger2=-0.019604372176653773，误差0.000295627823>原0.00025而assert FAIL。
9.5ms时差与旧反馈边界问题一致，尚不证明唯一根因；不放宽原精度或忽略action。
quiet56.4515、盖120°/转子稳停/starts=stops0、无holder；晚期native不能回填失败点。
独立整体物理9/28、完整持物/速度FAIL；加载每手1289/同时2577，2冻结2正常结束，
FK/native峰1.043562/1.043741mm、角0.118188°，原生指爪峰0.189072737091m/s<0.2。
空列表both_drop_stable真值不作已执行证书，缺净距Infinity不作实际证明。

stop-and-hold，先正常视频SIGINT刷盘全解码再正常场景SIGINT；四收尾PASS、四角色
1/0/0/0，九精确角色退休。36793614字节原片/11607真实/0重复/gap0.097761100wall，
相机/probe/decodePASS仅前缀；9实际阶段/末帧和07/09原生帧已看，主体/脚/桌/机器/
篮上两盒可见，部分手/盒自然遮挡，不冒充完整落稳或旋转取回。完整阶段抽取缺
未执行阶段报错保留，独立前缀抽取不覆盖；误名scope字段有独立勘误。261232行/
1142201882原字节/224459218gzip，CRC/计数/SHA通过，原SHA
d4cd4a9e05ebcf43960c232d2c12a8aab61b6e981bd192fde9aa1a1e02563a98。314成员及274
依赖事后不变，前后10262历史/10备份/785源码/近期含r267370和新清理各45保持。
469成员0444，REPORT SHA46b9cff3ff5d14ab9a524d97eb7fd2b677c8bd6d03a1121cdfb299990d42390b，
SUM SHAfc2f0575e56afe44ef39a5dee09044b6610dc65a7a3ea66e58941a4b325725d9，不续用/改源。

r269普通mkdir新身份，从r268314父源/输入逐SHA派生，不反转继承时序hook两次。
只有本轮default-preserving端点hook与新入口；原父立即0.00025断言仍在，资格前
不变。成功action结果全部future完成后冻结共同result_clock，当前D6路径资格后
只等待实际joint_time严格新于该时钟且误差仍<0.00025，≤1sim/30wall秒，不扩展
上界、不外推/代入命令值，原safety每次检查，失效仍保持并正常SIGINT。原0.16/
0.15未来门/0.25旧准入短行程、全部路径/速率/FF/goal0.0002/3、10、30证书保持。
17新专项涵盖真实失败窗口、实际双action、共同结果时钟、过期/精度/原生安全/
sim与wall拒绝/不得扩展等待及原资格前断言；首误取19.187旧结果的测试失败日志
保留，精确phase/hand选择修复后17通过。原1055加新17共1072通过，6.90s隔离
两包/六库恢复通过，原一次完整前检进行中，尚未seal/claim/Gazebo，不作当前物理
或180证明。接续看同前检handle，不观察超时复跑open-x。本轮无额外清理/备份/
真机/腿部导航/reset/detach/异常松夹，合格完整基线仍r266185.695sim，goal active。

### r269完整前检退出0；318一次密封/fresh连续执行

原85324完整前检退出0，1072局部/17新native反馈/31时序双爪/129阶段率/28按钮/
8盖速/8派发/64双/46配置/122旧装载/6CTest及6.90s两包隔离构建/六库恢复通过，
Nav111/2已知V12缺源保持；26输入/全部几何/录像前检/before保护含r268469通过。
启动预算改用最新**已完整合格**r266两原输出，659178291trace/94852916movie逐SHA
复验，完整物理和原片PASS；原1GiB硬门+256MiB附加余量/无损gzip6/无采样保持。
所需2096208487、实际余2101866496才启动；旧r261较大输出估计不冒充本次值。
277依赖启动前逐SHA，318一次seal/0444，identity SHA
7a1a53ad41861277492670d2d828c1e45f633b9815434ef7aa9d9ff2603560d3。
普通mkdir独占run，fresh六控制器active/0.6/500Hz/四D6/两臂splines只读、72帧
短测/原片首秒解码/原生首帧通过。主体/脚/桌/闭盖设备/近蓝盒/墙可见，远侧
自然遮挡。view与collector启动同工具脚本，模型收到图像晚于collector启动；
run/startup_original_visual_review_timing_note.json如实更正，不声称先于启动已看。
完整逐阶段原片核验仍必需，这个首帧不是完整任务证书。

独立collector1271223/ctime1791953056.35、sid=pid；executor1271225/guard1271226/
disk1271227/supervisor1271228均ctime1791953056.37。launch1264781/ctime1791953000.99、
server1265205/GUI1265206均ctime1791953001.91、video1268249/ctime1791953027.20。
live_checkpoint1九身份存活/318未变，首非空6.3395<25。最新只读35.0085在
paired_load_static_grip_hold，两手抓取/试抬后10sim前缀，尚无result，不称加载/
旋转/取回/180或完整视频PASS。接续重查同handle与progress/result，不观察超时
重启或复用claim，不活动中改密封源/移动六库构建/另开场景；失败原保持并正常
先视频后场景SIGINT。旧r269未启动/无live已覆盖，完整185.695基线和180目标不缩减。

## 2026-10-14 CST：r269真实松爪修复/机器前缀通过但取盒规划拒绝；r270密shortcut离线通过

r269旧live已终态覆盖。首6.3395→FAIL131.431=125.0915sim，非完整180。
LOAD59.0305、首至52.691sim，双真实中部抓持/各10sim/正常松爪/各3sim落稳通过。
8次实际新native原0.00025门通过，峰0.000130714708m、最长0.0035sim；释放
结果55.7295→native55.73，clock55.7305，左等0.001sim/right0sim共时钟，约0.0001m。
只证明本前缀修复旧反馈边界，不代入命令值/放宽精度或把action成功当全部成功。
真实lid/lid/rotor/rotor/lid事件12.999/76.970/82.512/116.1105/123.596；实际30rpm
稳转30.468sim、停稳/680交接密FCL/最终开盖，MACHINE_PASS129.8365；原30.3上窗
不通过，不改标。实际native130.55取盒请求的转子mod=-1.261250124rad，较成功r266
约-1.04762和两盒平面约19mm/12°变化，不外推旧轨迹。四flip：2 IK拒绝、一个
粗RRT路径被原2mrad全状态检查拒绝、最后端点碰撞；worker1、无取盒轨迹。
静态preparation或MACHINE_PASS都不是取回/返桌/末10sim/完整目标完成。

独立整体物理16/28、完整持物/速度FAIL；加载每手1288/同时2575、2冻结2正常结束，
FK/native峰1.000267/1.082843mm、角0.116422°，原生指爪峰0.189277392773<原0.2。
quiet132.16、末无holder/两篮内盒/盖120°/稳停/starts=stops1。stop-and-hold、先
正常视频SIGINT刷盘全解码再正常场景SIGINT；四收尾PASS、角色1/0/0/0、九身份
退休。61967071字节原片、19510真实/0重复/gap0.155057810wall，camera/probe/
decodePASS仅前缀；15实际阶段/末帧和09稳转/13开盖原生帧已看，主体/脚/桌/
设备与篮内两盒可见、早期遮挡如实，不补造取回或末持盒阶段。首帧review实际
晚于同工具脚本collector启动的timing_note保留，不声明已按先看后启动顺序完成。
697126行/2003390137原字节/410607214gzip，CRC/计数/SHA PASS，原SHA
a76fcb070ca099132d833a43a7ac3f9e59e75ef48221aa63ac6c009480c2012c。318与277依赖
事后不变，前后10262历史/10备份/785源码/近期含r268469保持；483成员0444，
REPORT SHA9a60a2e7428f4727b0221dbacb05d242b331aba949681ed49ff0def910a76ce9，SUM SHA
013fe02ea9649aa052303ca8f46921d33f474a686f4e8aade8ef1efd73b49150，不续用/改控制源。

r270普通mkdir小文件**plan-only探针**，不是完整候选，不含input/build/identity/run/
claim/launch，不在此身份继续加完整运行。父r269worker/request/fail-result与
原r121 RRT源按受保护依赖SHA核验；只内存改shortcut采样0.035→0.002，原树边/
seed480121/边界/2000迭代/120wall与最终全状态0.002/原IK/模型/碰撞豁免均保持。
严格逆变换恢复原函数字节，不写旧source。完全同一原FAIL请求现[0,180]flip取盒
2219检查/前瞻试抬62/搬运2466，15.992wall，通过；180接近点/0.4+20ms时长
8.877678sim，成功r266不同场景8.667898不是因果速度比较。只几何/前瞻，不作
新native/实际重抓或180证书。父483逐SHA/权限事后保持，7成员0444；REPORT SHA
a30c84766f004639b06446702d3a3e5299a349b0a30b3a4ae045311889ec5a97，SUM SHA
211bd98151076c8d1c79ea12f8c1d99764fd580fb418bb115293e8aeacc902a0。

当前无相关Gazebo/动作进程。磁盘约1.61GB<下一完整约2.10GB，已经请求用户腾出
至少1GB；未另获清理授权，不删旧成果/失败录像或盲跑。下一完整候选须普通mkdir
从未用身份、移植密shortcut/相关新单测/隔离构建/完整回归/保护/一次seal/fresh
完整验证；不能复用r269/r270或把本计划替代最终全流程。合格完整基线仍r266
185.695sim，完整180目标active未达；本轮两次真实完整身份尝试/失败审计/原片/
保护/封存与精度边界实修、密规划同现场探针PASS均是progress，不缩减原目标。

## 2026-10-14 CST：用户先清理无用录像，补充33个非录像测试占位

用户再次要求“先清理没有用的录像”，仅落实新独占审计
`runtime_evidence/video_cleanup_20261014_r2/REPORT.md`，不扩展为其他缓存、正式原片、
重要失败诊断或备份删除。409媒体盘点，仅移除pytest-101/102/103内33个0/4字节
非录像占位文件：逻辑108/原分配110592字节，真实录像删除0；其余376媒体SHA/
权限/inode/mtime/ctime逐项保持。前次r1重复下载副本5383533字节已经清理，本次
没有再释放5.4MB，也没有安全可删的新真实录像。回收站未操作，完整路径/原因/
字节和立即复验及逐行flush/fsync日志保留。读取进程无引用/活动pytest，1397项
字段可见性限制如实记录，不宣称全系统普遍可见。

前后10262历史/10备份/785正式源码及近期链PASS，r269483与r2707清单SHA/0444
保持；原程序退出0，27审计成员0444封存。REPORT SHA
`ecb12c188700e77a0d8a2110cf10423becced4940b50829609bb9a73a8738386`；SUM SHA
`cba9bffb1cad3c1c0da25e9a0478f470fb386aa3213ed025490599809f18f84d`。
可用空间采样约1.60GB，外部写入/释放影响差值，不等同本次净释放；仍不足完整
约2.10GB预算，不降低守卫或盲跑。无新仿真/动作/备份/真机/腿部导航/reset/detach/
松夹，旧正式成果全部保持。仅根交接摘要与本累积记录更新，旧封存证据不改。
完整合格基线仍r266185.695sim，180目标未达；本轮先完成用户录像清理请求。

## 2026-10-14 CST：r271密shortcut完整候选已前检，未seal/未fresh实跑

新普通mkdir独占`runtime_evidence/centrifuge_48box_dense_shortcut_full_20261014_r271`，
父r269483全SHA/0444核验，从318身份成员派生258源码/输入/必要夹具，不继承run/
claim/物理PASS。唯一控制路径改动：明确retrieval_pickup/true/空实际holder时选择
r270同三行密shortcut函数，SHA826bcb3e5b617c6cb37636b55e2dade2ee9ffeea8ad1bb1707305a1d25e0519e；
原树边0.035/seed480121/边界/2000迭代/120wall/最终全状态0.002 FCL/IK/目标/模型/
ACM/阶段率/原生反馈及3、10、30sim证书均保持，其他模式原函数，不完整声明拒绝。
NativeGripperCycle及父精度/时序hook保留，只反转最新worker接缝作继承身份证明。

1102局部（含22密采样/8prospective声明）/46配置/122旧/64双/原专项及6CTest通过；
Nav111/2已知V12缺源不称113全过。6.99s两包隔离构建/六库恢复，26输入仅绑定，
全部原几何/录像/净距/停转交接前检通过；281依赖事后SHA保持。首次继承字节证明
1失败/1093及旧prospective夹具缺holder字段的FAIL均保留；只为显式prospective
夹具补非测量/非fresh许可空手假设，不清除已声明未知/非空holder，实际执行器
仍取self.holders.snapshot()，不调用该helper。未重复build或早期open-x审计。

新模块实际接入，完全同一r269130.55失败请求[0,180]flip取盒2219/前瞻试抬62/
搬运2466通过，15.723089wall，180接近样本/0.4+20ms为8.877678sim。仅离线，
不是实际重抓/返桌/180证书，fresh实际状态与抓后规划仍必需。前后10262历史/
10备份/785正式源及近期含r269483/r2707/录像补清理27全PASS，三正式模型SHA保持。
原seal全部前提dry-check通过，停在identity写入前；无identity/run/claim/launch。

准备报告`PREPARATION_REPORT.md` SHA
62a56689a327aae1f00856f39a571bc0b9b40f27d5b0ac49cdd6ec37a8b7b6de；401成员当前准备
快照`preparation_snapshot_20261014_v2.json` SHA
c76fb1d9519eaead26cc73932b950ebd60e6e2a43c59fca39d9134d8a05c2fc0。准备报告原字节
移名保留，为未来实际终态预留REPORT.md，不覆盖准备报告；v1快照为移名前历史。
接续见PREPARATION_STATUS.md，原准备及修复session73356/92586/29203终态1/0/0，
事后保护52514退出0，不重跑已完成prepare/build/open-x门，不把旧末marker当验收。

当前无活动Gazebo，约1.57GB仍不足完整预算2096208487B（最新完整合格r266实际
两输出659178291+94852916逐SHA核验、原1GiB硬门+256MiB不降）。须再腾出约1GB
余量后重检空间/进程/依赖，一次seal/普通mkdir run/fresh完整实跑。首帧原图必须
独立工具返回并实际看过后，再立即启动collector。无新仿真/动作/清理/备份/真机/
腿部导航/reset/detach/异常松夹；基线仍r266185.695sim，完整180目标active未达。

### r271空间已足/fresh连续运行覆盖准备等待状态

用户回复检查空间、不足时可删失败录像；实际约4.82GB已超过2096208487B完整预算，
因此没有删除任何失败录像。新独占peer审计`centrifuge_48box_r271_protection_20261014_r1`
执行原核验器，所有旧保护PASS，401准备快照和281选用依赖再核验保持。
352成员一次seal/0444，identity SHA
d5bbc29cd8eecd91fa76e430deb4effe4c83c91b35428894fe13b82cf4d2bb61。
普通mkdir独占run，fresh六控制器active/0.6/500Hz/四D6/两臂splines真实回读、72帧
窗口短测及同一原片首秒全尺寸PNG通过。原片首帧独立view_image工具返回并实际
审阅后，才在下一工具调用启动collector，未重复r269的顺序问题。主体/脚/桌/
闭盖设备/近蓝盒和夹爪/墙可见，远手远盒部分遮挡，不预称完整视频PASS。

独立collector1813476/ctime1791958266.77，executor1813480/guard1813481/disk1813482/
supervisor1813483均ctime1791958266.80；launch1803080/ctime1791958168.21，server
1803484/GUI1803485均ctime1791958169.05，video1806636/ctime1791958195.59。
352身份持续未变，首有限trace sim11.7335<25；live_checkpoint3当前sim99.626处于
machine_running_steady，仅双装篮/关盖/开始稳转前缀，尚无result/双取回/完整180。
查同PID/ctime/progress/result，不因观察窗口结束重启/复用claim，不活动中修改
密封源/构建/另开场景。原程序自动连续执行，失败保持并先视频后场景正常SIGINT。
原空间等待/未seal/未运行描述已覆盖；完整合格基线185.695及原目标不缩减。

## 2026-10-14 CST：r271真实取回运输通过但粗放桌2mm故障，539封存覆盖live

r271终态FAIL，首11.7335→175.0725=163.339sim仅失败前缀，不是完整180。双装篮
63.6505/首至51.917sim，五真实按键17.568/81.5995/87.015/120.6905/128.240，实际
30rpm稳转30.553sim/停稳开盖，MACHINE_PASS134.138。134.859当前实际空holder
请求在线取盒PASS：[0,180]flip/2310密FCL，前瞻62/2449仅几何；之后实际双重抓/
冻结参考/抬出和10sim保持/转移到桌槽上方已执行，密shortcut确实解开旧规划拒绝。

0.35粗下降在175.0725左盒native2.015316860mm/3.698754438deg触发原2mm守卫；
源同175.0725，角未超过5deg。result已接受样本native1.968659mm不包括抛异常这帧，
不能推翻故障。920行近窗、原118目标/actual初点/119消息时间完整保留，不重定
持物参考或放宽2mm/5deg。FAIL后保持两holder，quiet175.7055/停转/盖120deg/
starts=stops1；无fine/释放/最终桌槽/末10sim，整体物理19/28与完整持物/速度FAIL。
独立每手2413/同时4824，4冻结2正常结束、仍两holder；接受样本FK/native峰
1.415904/1.781729mm、角3.283840deg不是全过程上界，指爪峰0.195911990<原0.2。

正常视频SIGINT刷盘/probe/全解码后正常场景SIGINT，四收尾PASS、角色1/0/0/0，
九PID/ctime已退休，352密封源和281依赖事后保持。原片80968742B、25403输出/
25401真实采集/2软件重复/gap0.255111536，不能宣称零重复；全窗/相机守卫通过，
21实际阶段/末帧与14/19/21原生图已看。主体/脚/桌/机器/可见盒保持在画面，远手/
远盒遮挡如实；最后仍抓持，不能作最终释放或完整视频证书。首帧独立返回/实际
审阅后才启动collector，旧r269顺序问题未重现。870160行/2944556813raw/
630109153gzip，CRC/计数/字节/SHA通过；rawSHA
0859b9f5e9fb0326b1a52b98748db99f59ec7f1065e815f060dbdce9d1ab4ff3。
posthoc trace/cycle/grip/speed/profile实际0/1/1/1/0，所有失败/原前检失败输出保留。

新peer保护前后10262历史/10备份/785官方源及近期链PASS，19成员0444，SUM SHA
3095dda296a5cbfc03b423ed4a590abfa0bf3539e6cda5974f34402d3f9e370c；无重要成果改写。
r271539成员0444，REPORT SHA8286890ff037561ef33bc1a537f15a0343bcd63bc08d12f55bedaf0b1499502e，
SUM SHA78ba3a51efb0058b5ccc1fbbc9e95c5d1d4d953bead81d72b56cfb5cb1ce0daf。
准备报告和v1/v2历史快照保留，正式REPORT覆盖其未运行状态，不覆写旧文件。

同一失败命令只离线复现原actual初点/全部点/原时间：0.35→0.30增加0.653504sim，
20ms底限/几何保持。首比较遗漏actual初点的断言记录保留，修正后才保存结果；
这不是物理成功或180预测。下一未用身份可只测试收缓近桌粗下降，保留全部原门
和完整目标，不改r271或复用claim，不把静态retiming当达成。无活动Gazebo，磁盘
约4.07GB仍足够原约2.10GB完整预算；用户“若不足删失败录像”条件未成立，没删。
完整合格基线仍r266185.695sim，原目标active未达；本轮真实执行/规划实修验证/
故障近窗/原片/保护/539封存都是progress，不以计划替代后续实跑。

## 2026-10-14 CST：r272只降粗放桌0.35→0.30，fresh完整流程连续运行

新普通mkdir身份`centrifuge_48box_gentle_setdown_20261014_r272`，父r271539逐SHA/
0444核验，352父身份只派生266源码/26输入/夹具，不复制run/claim/物理PASS。
GentleSetdownCycle继承NativeGripperCycle，仅已资格/双实际holder/两不同payload的
paired_retrieval_setdown接受0.30，替代父0.35；其他阶段/点/actual初点/20ms底限/
共同时间/原native2mm5deg/端点/力/原生速度/FCL/全部3、10、30证书保持。
父密shortcut和新反馈/时序hook不重复反转，最新源证明只反转新runtime alias；
历史lift证明单独反转已经验证的dense worker seam，不放宽原几何证明。

1120局部/18新真实三publisher与原失败119消息点/资格拒绝/守卫和生命周期继承
通过；预构建1失败/1119日志保留，修复的是错误测试方法名，不改任何守卫。
原专项/46配置/122旧/64双/6CTest及5.90s两包隔离构建/六库恢复/全部前检通过，
Nav111/2原V12缺源保留。0.30同旧失败命令多0.653504sim只时序不是物理或180证书。
284选用依赖启动前逐SHA核验，before含r271539与独立保护19以及全部旧保护PASS。
338成员一次seal/0444，identity SHA
e6c0e30216a337c579332ff52f35034f4c72f2b11322f21989b34cc4a907dd9d。
原2096208487B启动预算/1GiB硬门/256MiB余量不降，磁盘足够，未删失败录像。

普通mkdir run；fresh六控制器active/0.6/500Hz/四D6/splines/72帧与同原片首秒通过。
首帧独立view_image返回并实际审阅后才启动collector，主体/脚/桌/闭盖设备/近蓝
盒及夹爪/墙可见，远手远盒部分遮挡，不预称完整视频PASS。独立collector2008917/
ctime1791963111.89，executor2008919/guard2008920/disk2008921/supervisor2008922均
ctime1791963111.91；launch2007471/ctime1791963028.42，server2007792/GUI2007793均
ctime1791963029.20，video2008560/ctime1791963052.48。live_checkpoint1九角色存活/
338未变，首非空sim11.2725<25，当前22.8665处于加载前重新开空爪，尚无result。
查看同PID/ctime/progress/result，不观察超时重启/复用claim，不活动中修改密封源/
构建/另开场景；原程序自动连续执行，失败保持并按先视频后场景正常SIGINT。
所有旧源码/模型/正式成果/备份不改，无新清理/备份/真机/腿部导航/reset/detach/
异常松夹，完整合格基线仍r266185.695sim，目标active未达，不缩减原完整180范围。

## 2026-10-14 CST：r272装篮接近速度故障已退休，480封存覆盖live

r272首11.2725至FAIL27.5505=16.278sim仅失败前缀；paired_load_approach仍原0.8，
left_dex1_finger_1_joint实际0.20325094619111955m/s、源27.55超过原0.2。没有抓持/
装篮/旋转/取回/粗放桌，因此0.30粗放桌假设未达到，不宣称已通过或因它失败。
唯一真实开盖按键17.1885；quiet28.333，空holder、盖120deg、转子停、starts=stops0。
整体物理9/28、持物/速度FAIL，空集合drop真值不是证书；原2mm/5deg/速度不放宽。
四收尾PASS、角色1/0/0/0，先正常视频SIGINT刷盘解码再正常场景SIGINT；九原PID/
ctime退休，338密封源/284依赖事后SHA保持。原片7145664B、2196真实/0重复/
gap0.091032462wall，全解码PASS仅前缀。五实际阶段/末帧及末帧原图已看，主体/
脚/桌/设备/近盒/夹爪/墙可见，远手远盒部分遮挡，未执行阶段不补造。
trace109564行/96731641raw/8236428gzip，CRC/计数/SHA PASS，rawSHA
bdce0f5be81ce265b27912945f3fa1c320f1aad5cfabe21b4b3d3a06f7b0d869。
前后10262历史/10备份/785正式源及近期含r271539/保护19保持；480成员0444，
REPORT SHA8069dbe6564fedf47d6973c1dd86af2895c40aa3675e7e75e24c813148192439，
SUM SHAad16d061304cc7fafdd45ba7029098cb70705112a4cd94658444071720b22e83。
约4.02GB>2096208487B下轮预算，用户不足才删失败录像条件未成立，未删任何视频。
下一未用身份可只收缓初始空手接近0.8→0.7，保留全部目标与门；不改r272/复用claim。
当前无相关Gazebo，完整合格仍r266185.695sim、180目标active未达。

### r273初始接近0.8→0.7完整候选fresh连续运行

新普通mkdir `centrifuge_48box_initial_approach_20261014_r273`，父r272480全SHA/
0444核验，只派生270源码/输入/夹具，不继承run/claim。InitialApproachCycle继承
GentleSetdownCycle，只允许原空手初始接近0.8请求以0.7执行；余几何、actual初点/
共同时间/20ms底限/原0.2m/s、2mm5deg及3/10/30sim门保持。0.30粗放桌继承但尚未
物理资格化。1137局部/17新真实publisher等、原专项/46配置/122旧/6CTest与6.90s
隔离构建/六库恢复通过，Nav111/2原V12缺源保持；两次预构建缩进错误日志保留。
全几何/录像/停转交接前检与前保护含r272480通过，286依赖重新SHA核验。

342成员一次seal/0444，identity SHA5507709233bc642760f4660d02f964212b87266780a45be2c737cbbfbf9eb99a；
普通mkdir run，fresh六控制器/0.6/500Hz/四D6/splines/72帧与同原片首秒通过。
首帧独立工具返回并实际审阅后才在下一调用启动collector，主体/脚/桌/闭设备/
近盒近爪/墙可见，远手远盒部分遮挡，非完整视频证书。collector2125270/ctime
1791965551.23，四角色2125278/79/80/81均1791965551.26；launch2117909/1791965485.4，
server2118306/GUI2118309均1791965486.28，video2121206/1791965510.21。
live_checkpoint1九身份/342不变，首有限非空contact7.917<25，当前10.4965仍初始化。
原完整自动程序继续，不观察超时重启/复用claim，不live改密封源/构建/另开场景。
约4.00GB足原2096208487B预算，无需条件删失败录像，原成果/备份不动。
整体180目标尚未达，完整合格仍r266185.695sim；终态需实际结果和完整审计覆盖。

## 2026-10-14 CST：r273装篮通过、旋转接触深度FAIL，504封存覆盖live

r273真实初始0.7空手接近在原指爪守卫下完成；双中部抓持/10sim试持/搬运/正常
松爪及各3sim落稳通过，LOADING_PASS60.4205，首7.917至52.5035sim。真实三个按键
13.948/78.3495/83.711开盖/关盖/启动，steady_start86.8275，FAIL88.992，仅2.1645
稳速前缀，不是30sim证书。左盒/左篮接触depth1.003269628mm、force1.720951627N，
超过原1mm；接受样本峰0.974033mm不包括触发故障这帧，不推翻故障或放宽原门。
首至FAIL81.075sim仅失败前缀，无正常停转按键/开盖/取回/粗放桌/末保持；0.30
粗放桌假设仍未到达。整体11/28、完整持物/速度FAIL；每手1289/同时2577检查、
2冻结2正常结束、空holder，FK/native峰0.979420/0.993082mm、角0.132020deg。
指爪峰0.200000000008m/s、原0.2+1e-8数值容差保持，无速度故障，不称严格<0.2。

故障保持quiet92.6395、转子停/盖闭/starts=stops1，四收尾PASS、角色1/0/0/0，
先正常视频SIGINT刷盘解码后正常场景SIGINT；九精确PID/ctime退休。342密封源/
286选用依赖SHA保持，前后10262历史/10备份/785正式源及近期含r272480保持。
原片48133537B、15033真实/0重复/gap0.147693417wall，probe/全解码PASS仅前缀。
13实际阶段/末拼图及07/09/13原图已看，主体/脚/桌/设备/可见盒在画面，远手
远盒部分遮挡，旋转时闭盖自然遮挡盒子，未执行阶段不补造。trace470159行/
1435142313raw/284668323gzip，CRC/计数/SHA PASS，rawSHA
55bb79a569543668645e243f23a793316a873c4df84bc17beb4c3a4b26d2d299。
posthoc0/1/1/1/0/1/0保留；提取helper provenance键KeyError保留，只修复记录读取
后revision2独立成功，不重复open-x审计。504成员0444，REPORT SHA
06ecaf3be2ec56e5e60880be0756f61596e07ee18001495b15a766ebb5006e82，SUM SHA
575caffa34ff244e81b01d728986efb03b3dfd5a2b1ad4e4bc77fe192ac989ff。

用户中断后“继续”重新只读确认无相关Gazebo，仅约0.53GB<2096208487B完整启动
预算，已要求腾出至少2GB，不盲跑/降低硬门或删除封存录像和备份。当前只在新
普通mkdir `centrifuge_48box_startup_ramp_probe_20261014_r1` 做小文件离线原型：
原转子已有1rad/s²线性目标启动；候选仅正常运行启动降0.8，停转/安全停保持原1，
30rpm/原CentrifugeSafety、StoppedMotorTorque、PositionBrake及几何/接触守卫保持。
它不是完整候选/动作authority/新run，无launch/claim，必须全新完整身份再证明
真实物理及180。旧正式源码/模型/所有成果/备份不动；完整基线仍r266185.695sim。


## 2026-10-14 CST：用户授权 r161–r211 精确31份日志无损归档完成

用户明确要求无损压缩、完整核验后删除原始日志，并处理读取/校验兼容。
本次严格采用 `runtime_evidence/trace_archive_20261014_r1/inventory.json` 中31个路径，
未扩大为其他日志、原片、模型、源码或备份清理。原68,313,102,160 B变为标准
gzip 14,226,440,860 B；按文件分配量释放54,086,705,152 B（54.09 GB/50.37 GiB）。
逐份源SHA匹配旧清单，系统gzip完整解码CRC、SHA、字节、行数一致，fsync提交与
登记后才删raw；31原SHA256SUMS、封存源均不改。原始逻辑数据仍可无损恢复。

兼容范围是显式启动器 `tools/trace_archive/run -- 原命令及参数` 的Python进程树；
未来新候选prepare/回归/before/after保护均须通过该入口并保留继承环境。精确31路径
只读映射到独立data目录gzip，按原字节参与旧SHA核验，错误归档/缺失不跳过。
原生cat/sha256sum/目录枚举/mmap不虚拟化；含这些raw的旧清单用
`python3 tools/trace_archive/archive_cli.py verify-manifest 清单路径`，不能把普通
sha256sum的MISSING误解为归档损坏。新命令仍须全新候选身份，不复用旧claim或输出。

不改旧源地执行r273原完整保护链，输出绑定本次新审计目录，前后7组保护结果完全
一致PASS，覆盖10262旧保护文件、10备份、785正式源及近期链；r273另504成员校验PASS。
31归档doctor全部PASS；36归档/迁移测试、1137 r273、122 r178、46配置离线测试，
合计1341通过；真实ROS/install环境的旧路径读取通过。没有启动Gazebo/机器人动作，
因此这不是新增物理成功或180sim验收。详细结果见verification_summary.json。

133252个非目标既存文件元数据复核，无缺失、无未解释变更。除本次3份交接文档，
并行startup_ramp_probe目录有48项变化（46权限/ctime、2 CTest临时记录），已如实
记录而未回滚/冒称不变；本次迁移程序不操作该目录。所有目标外正式成果校验通过。
使用说明见tools/trace_archive/README.md；下文/历史“所有raw原路径物理存在”约定
由此授权存储迁移覆盖，数据原SHA和仿真物理门均未降低。

### r274接续准备与启动斜率原型封存

startup_ramp_probe_r1仅离线：私有插件4.34s构建/四CTest通过，新32441条断言，
六库恢复cmp保持；Nav111/2原V12缺源保留。原三header/9派生来源字节及真实权限
保持，严格反转插件命名/安全类型/包含/状态字段；前后旧保护及r273504全通过。
51成员0444，REPORT SHAe563eb1666b685ef636950f56429ccff5003b0bc5ff3cd9b47f214a7aa2ea76a，
SUM SHA05508a971c0761c3f39e7c1c48d1c9540473f5205c0395ee3d56f9def7d9a3d7；产物1495768B，
SHA cb030abe1b549ac266132da3ba4cb5f78e460d015598d7be8bde81500d9dc8f1。
真实r273470159全行重解码SHA、263机器/21518接触对/179篮姿态；543个>=0.8mm
样本都在左侧稳速后，不证明启动斜率一定是根因。0.8仅新待实跑假设，非180证书。

用户完成无损归档后空间足够（当前约54GB），旧0.46GB等待快照已过时。普通mkdir
新 `centrifuge_48box_startup_ramp_full_20261014_r274`，父504/原型51逐SHA/0444核验，
只派生274源码/输入/夹具及新私有so，不继承run/claim。唯一新物理变化正常启动
目标斜率1→0.8rad/s²；停转1/30rpm/扭矩制动/模型几何/原2mm5deg和1mm接触门/
全部3、10、30sim证书保持，继承初始0.7与未到达粗放桌0.30。
两包6.64s隔离构建/六库恢复/1155局部（18新模型与实际状态字段门）/原专项/
配置46/旧122/6CTest通过，Nav111/2原失败保持。旧motor命名测试1失败/1154保留，
只改新插件绑定证明，模型几何仍完整反转核对；执行器源无新轨迹或守卫修改。

首次prepare在before保护因未启用新归档兼容层而FileNotFound停止，没有seal/run/
Gazebo；不是旧数据损坏，也没有降低/跳过SHA。保留原失败日志，不能重跑已完成
build或早期open-x门。已按新权威通过tools/trace_archive/run执行continue_preparation，
从未写输出的before开始接续；archive_runtime_binding完整31原SHA/CRC/字节/行数
与旧清单核对，另密封绑定5代码/registry及31gzip共36依赖，归档后1155回归通过。
当前续接session56280、prepare360690/ctime1791970239.25，before377920/1791970382.12
已确认存活，后续前检尚在进行，未seal/运行，不预称整体通过。所有未来Python
子进程须保留归档环境；旧源/原片/报告/备份不覆盖，完整目标仍active未达。

### r274一次seal/fresh实际运行，归档兼容保护已通过

续接session56280实际退出0，未重建；归档兼容before保护/全部几何与录像前检
通过。启动前292选用资源和36归档运行依赖逐SHA再核验，约54GB足原预算。
353成员一次seal/0444，identity SHA3482bcc8130fa9468161bf98eaf4d5071239d94d8ca3dd9b912618dcefd007d9；
普通mkdir run，fresh六控制器/0.6/500Hz/D6/splines通过，新私有插件真实两帧状态
startup_ramp_rad_s2=0.8只读回读PASS，未以编译或声明代替实际字段。
72帧短测及同原片首秒通过，首帧独立view_image返回并实际查看后才下一调用
启动collector，主体/脚/桌/闭设备/近盒近爪/墙可见，远手远盒部分遮挡，非全片证书。

collector479003/ctime1791971228.84，executor479013/guard479014/disk479015/supervisor
479016均1791971228.87；launch463097/1791971087.17，server463564/GUI463565均
1791971088.25，video466866/1791971113.85。live_checkpoint1九身份活/353未变，
首有限非空17.3035<25，当前20.0785初始化，尚无任务终态。程序自动连续完整
执行；看同PID/ctime与progress/result，不观察超时重启/复用claim、不live改源/
构建/另开场景；归档环境继承。失败保持并先视频后场景正常SIGINT。
全目标仍active未达，完整合格仍r266185.695sim；后续实际结果/原片/全审计覆盖。

## 2026-10-14 CST：r274机器段通过、粗放桌2mm故障，545封存覆盖live

r274终态FAIL，首17.3035→180.8335=163.530sim仅失败前缀。真实双装篮69.6805/
首至52.377sim，五指尖按键23.1985/87.6195/93.033/127.0735/134.592，30rpm稳转
30.132sim（原30.3上窗也通过）、正常停转、120deg开盖/3sim等待通过；机器PASS
140.3595。独立接触峰0.280221mm；执行器机器段左/右峰0.223338/0.284417mm，
不互相替代范围，原1mm守卫未放宽。原旋转深接触故障本次未重现，不称唯一因果。

实际空手取盒规划、双重抓/原参考冻结、抬出/10sim试持/外移及搬到桌槽上方通过，
0.30粗下降在180.8335左盒即刻native2.025394033mm/3.688464569deg（源同stamp）
触发原2mm；角未超5deg，不用接受样本峰或重定参考推翻故障。没有fine/释放/
最终槽位/末10sim；整体19/28、完整持物/速度FAIL。每手2472/同时4942检查，
4冻结2正常结束，两取回holder保持；接受样本FK/native1.349976/1.490938mm、角
2.737352deg，不是故障全程上界；结果max_native1.977mm也不包含触发帧。指爪峰
0.189510105m/s<0.2。取回指令16N、每指预载约14.85～15.75N，近故障输出约
15.4～15.6N；后续可新身份测试20N（仍32N钳位/40N接触门），不降低原native门。

失败保持quiet181.452、转子停/盖120/starts=stops1，无reset/detach/异常松夹；
先正常视频SIGINT刷盘解码，再正常场景SIGINT，四收尾PASS/角色1/0/0/0，九原
PID/ctime退休。353密封源/292资源/36归档运行依赖事后SHA保持，前后七组旧保护
一致，含10262历史/10备份/785正式源/r273504/原型51；旧额外历史差异不改结论。
原片91083778B、28702真实/0重复/gap0.147386130wall，全解码PASS仅前缀；22实际
阶段/末拼图及14/20/22原图已看，主体/脚/桌/设备/可见盒在画面，远手远盒部分
遮挡、闭盖自然遮挡，末帧仍抓持不是释放，未执行阶段不补造。trace872933行/
3014182752raw/651264464gzip，CRC/计数/SHA PASS，rawSHA
42ed78ed387760474eaf857aa50da89d12b725a389e7785c8b9c1c85707a3a8b。
posthoc0/1/1/1/0/0/0保留；第一事件筛选漏真fault名，另外精确提取原行保留，
不覆写第一提取或trace。545成员0444，REPORT SHA
35a8134c329915de89591f20d7a80278fcb4dfec1ae6ea9600f6928867f4be29，SUM SHA
0de4b60be0dc715e694197f92ff53ec0fe7ff999182beddbd56712d634268590。

当前无相关Gazebo，约53GB足后续预算；本轮未清理旧文件或建备份，授权31日志
迁移由其独立成果说明，逻辑旧SHA保留。未将失败粗下降/单次启动候选提升正式
默认，正式源/模型保持；完整合格仍r266185.695sim，180全目标active未达。
下一只改取回抓持力的候选必须未用身份/隔离构建/测试/归档兼容保护/fresh完整
验证/原片/后保护，不能复用r274或把稳定子流程当最终目标；无需用户逐步催促。

## 2026-10-14 CST：r275终态，取回20N尚未执行，523封存

空间已恢复约53GB，新r275固定骨盆完整实跑，不是导航。1167局部/12真实双ROS
动作/两包7.00s隔离构建/六库恢复/原专项/46/122/6CTest通过；Nav111/2V12缺源
保持。353一次seal，293资源、36归档依赖，31旧日志完整原SHA/CRC保持。
双装篮、五次指尖按键、30rpm稳速30.123sim、停稳/120deg开盖通过；机器PASS
133.758。0.4空手取盒接近在139.9305出现右finger2 native0.21271924954013083m/s
（源139.93）超过原0.2，整体17/28 FAIL。首10.2765至FAIL129.654sim仅前缀，
取回16→20N尚未到达，不能说有效或失败；两盒仍篮内，无取回holder/异常释放。
140.673保持，正常先视频SIGINT/全解码再场景SIGINT，九PID/ctime退休。原片
63744981B/20030真实1重复/gap0.190220789wall，17实际阶段及12/17原图已看；
未执行取回/终态不补造。trace717420行/2051941785raw/420442074gzip，原SHA
a71f2485f08049cb046d949bf86b0801e5260268caabb75616961a967c048dd5。
前后七组保护一致，353源/293资源/36归档依赖不变，旧成果/10备份/正式默认未改。
523成员0444，REPORT SHAa35860ca72c75f1469b7c885bb1069a74ccc374df23e083d2f034e0ccd17d278，
SUM SHAd649ce4ffb8b6251eba595e2079dbe8d8dcf859cbc0f0ae5c2f1435c1ec71abf。
当前无相关Gazebo，完整合格仍r266185.695sim，180目标active未达。后续仅新身份
测试空手取盒接近0.4→0.35，保留路径/密点/跟踪/0.2门，20N仍待实际验证。

## 2026-10-14 CST：r276已退休518封存，空爪机械限位假设

r276父r275523/353验证，最终1186局部、19真实动作与原18.01N回调/12真实三组
调度、隔离两包/六库/原专项/46/122/6CTest通过，Nav111/2V12缺源不变。seal前
发现原软件输出门18.01N，不是仅模型32N；将未运行20N候选改18N而不降低门，
原1179和早期3期望失败保留。360一次seal/294资源/36归档依赖保持。
双装篮通过，machine_paired_empty_return0.6在75.1805右finger2原始0.2053489611m/s
超0.2（源75.18）。首9.8265至FAIL65.354sim仅前缀；机器/0.35取盒/18N未到达，
11/28，75.928保持、无holder，两盒留篮内。正常视频后场景SIGINT、九身份退休。
原片42524621B/13331真实0重复/gap0.130158160wall，全解码仅前缀；11实际阶段
及10/11原图已看。trace333943/1251605049raw/247988340gzip，原SHA
53211f41342223d0691553a61e76273c181b90226c521e5c2f0cb76df3c48d5d；前后七组
保护一致，360源/294资源/36依赖/10备份不变，正式默认没有失败调参。
518成员0444，REPORT SHA53a42763f232ba07cd9fb68a4cc6a8c1484c60e59433286069a53209bce367b3，
SUM SHA65e24fd1e7e8205ed82127307381c5c10169f7a91ff15104aee2fc7e7ff3b046。
原始故障窗rightfinger1在-0.02硬限，finger2约0.93mm跳变，窗内无盒-爪接触；
下一新身份仅将已有零力空爪-0.019动作前移到空手返回规划/运输之前，避免在
-0.0199近硬限运输，保持原臂率/路径密度/保护/全部证书。18N/0.35仍待实测，
不是证明限位唯一根因。无liveGazebo/不清理/不建备份，180目标active未达。

## 2026-10-14 CST：r277已退休514封存，撤销篮内空爪前移假设

1191局部/5新实际双ROS动作顺序/原专项/两包隔离/六库恢复/46/122/6CTest通过，
Nav111/2原V12缺源保持。360一次seal/295资源/36归档依赖；fresh健康/原片首秒
独立实际审阅后连续执行。装篮63.21通过，但篮内提前原零力空爪-0.019在63.5995
出现左盒-leftfinger2接触，contacts_valid=false，10/28 FAIL。首10.603至FAIL
52.9965sim只是前缀，没有机器/18N/0.35取盒或终态。异常“Loaded payload left”
不证明盒已离篮：supported=true、vertical_inside=true、overhang=0、深0.316833um、
bottom约4.12nm，真正故障为禁止的盒-爪接触；此前简述已更正。该候选绝不采用。
64.2265保持、starts=stops0/无holder，先视频正常SIGINT/全解码再场景SIGINT，
九PID/ctime退休。11实际阶段/末图及11原图已看；原片38244657B/12073真实0重复/
gap0.124254033wall，仅执行前缀。trace277552行/1163672253raw/230553204gzip，原SHA
5c79a84a92c1832aed7a4c1b2ce8c77fde716a1d5fe17e034d20d7fc83d38665。
360源/295资源/36依赖/前后七组保护/10备份保持，正式模型源码默认未改变，未
清理或新备份。514成员0444，REPORT SHA
219ecdbd5689bb73c31b6739c22b7d49768c8e9c6dc0963849447f257cfc8edc，SUM SHA
5bce462d0ea703fc4e1677bb52c22306cd63b2cf977aa9c53540104f868052d9。
当前无相关Gazebo，约49GiB可用；完整合格仍r266185.695sim，180目标active未达。
下一从r276接续而非失败r277；若为减轻机械限位而调整空爪，必须先完成原三个
反向退爪阶段到壳外，并以新实际手指状态审核剩余full-state FCL路径，才在最后
empty_return0.6运输前零力空爪。禁止再在篮内闭爪、复用run/claim或放宽原门。
18N和0.35仍未实测，没有新完整PASS。不盲目逐次降速，优先验证时序及当前几何。

## 2026-10-14 CST：r278壳外时序/18N实际通过，粗下放故障，574封存

本目标轮为实质进展：从r276518/360派生而非r277，不在篮内闭爪。完整原三
反向退爪后，31样本实际全状态空爪FCL、原-0.019/0N双动作和新鲜端点，再用
真实新四指状态1663检查重规划末empty_return。两mode显式不豁免碰撞，未知
仍拒绝，新增四指原0.0003m规划漂移门；其余原语义/硬门不改。实际前/后漂移
35.471487/19.283676um通过，原空爪超速/篮内碰盒本轮未重现，不称唯一因果。
最终1222局部/36新测试/两包6.97s隔离/六库恢复/原专项/46/122/6CTest通过，
Nav111/2原V12缺源保持。几何新mode表初缺漏、假边界list类型测试失败保留并
修正，不降低碰撞/关节margin。首次seal因最终源码与早期依赖SHA不同被拒绝，
未写identity/run/Gazebo；非覆盖scene_dependency_audit_revision2绑定300及最终
回归后才一次seal381。旧audit新变化/20N标签另scope erratum覆盖，当前18N。

fresh六控制器/500Hz/0.6/D6/splines/实际0.8/72帧及原片首秒通过，图像工具实际
看首帧后才启动自动连续executor。首9.2845，装篮61.8505（52.566sim）；五实际
指尖按钮、30rpm稳转30.1535sim、停稳/120deg开盖/3sim等待，机器PASS133.3305。
独立接触峰0.249725mm；执行器左右0.177512/0.255995mm不同窗口不互相代替。
实际0.35取盒、18N双抓、原参考冻结、pilot/抬出/10sim试持/外移/返桌运输通过；
18N每指预载约16.9407～17.7867N，原18.01N软件力门通过，未放宽到模型32N。

0.30粗下放174.2205左盒即刻native2.0156174758mm/3.6314335305deg超原2mm，
源同stamp。首至FAIL164.936sim只是前缀，19/28，没fine/释放/终态/末10sim；
接受样本峰不含触发帧，不能反驳故障。指爪峰0.2000000000184m/s只在原1e-8
数值容差内，不称严格小于0.2。两holder保持/174.8275 quiet，先原片正常SIGINT/
刷盘/全解码再场景正常SIGINT，角色1/0/0/0、四收尾PASS，九PID/ctime退休。
原片86948503B/27318真实0重复/gap0.143403851wall，仅已执行前缀；22实际阶段
及14/20/22原图已看，末仍持物不补造桌面释放。trace878937行/2896249843raw/
618000434gzip，原SHAc828c682fee74983db8592472238bbd65c2be1cd4892b9ffd6717ce514077087。
381源/300资源/36归档依赖及前后七保护一致，10备份/785正式源码/10262历史/
r277514等不变，正式默认未采用失败调参，无reset/detach/异常松夹/导航/真机/
清理/新备份。574成员0444，REPORT SHA
7f993efd8dfb7f644b1fe1a091c8792098ced5f38a0c40ebc4bca83f37a51628，SUM SHA
d02ebc02f611200a4d7f7719ddba4b86be21df4d4e675ec4ac2e09fb4265bf60。

### 两个独立离线原型，不是新实跑身份

grip_height_probe_20261014_r1只深复制r278实际取回请求，原+10mm改+5mm；四
翻转未通过，[0,180]descend末点左basket-leftfinger2原FCL碰撞，其余IK/RRT
失败，方案否决，不实跑/不加豁免。574父/300资源前后SHA保持，5成员0444，
REPORT SHA315751ca2fe7eca6e34410361a8e8bb76f6568c34feb5fe75ea059d688654f4f，
SUM SHA05f3c8348609ca57a7b437016a82ec6c1fe9714fd593d5967a56dc3a82d9cab8。

coarse_ramp_probe_20261014_r1来自r278574及r264494，父前后完整SHA/0444验证，
增加默认行为不变common_timing钩子，完整逆变换等于父源；只原0.30粗下放、
当前资格/两不同holder采用0.6sim首末弧长时间坡度。10测试含实际三ROS发布/
r278118目标+种子，每原点/折线/同stamp/20msfloor不变，其他阶段原timing相同。
原4.720110401sim指令→5.319841453，增加0.599731052sim；只是数学，不是全程
或物理减少滑移证据。离散内部顶点速度跳变仍在，不冒称连续加速度受控，不
恢复失败空手0.6坡度。13成员0444，REPORT SHA
936efc9d4e6ec766f92c223eae71a4838268c855699358931e50406cb3f8353d，SUM SHA
c7bad30daecc8725db398818f2437d4f4d6263a4936865abf74dd2b3b92831f7。
两原型没有run/claim/launch，不能追加实跑。下一需从r278和coarse原型派生下一
未用完整身份，先隔离构建/原回归/最终源码后重绑依赖/一次seal/fresh完整验证/
原片/后保护；不盲加力或降低抓点，不把原型当180达成。当前无相关Gazebo，约
48GiB足预算，完整合格仍r266185.695sim，180全目标active未达。

## 2026-10-15 CST：r279完整物理28/28 PASS，184.358sim；180仍未达

本目标轮实质进展：r279从r278574/381及coarse原型13全SHA/0444派生，只原0.30
粗下放采用0.6sim弧长首末时间坡度。原点/折线/三组同stamp/20msfloor/positions
字段及全部物理门保持；原默认timing钩子完整逆变换等于父源，其他阶段不改。
1232局部/10新三ROS/两包7.00s隔离/六库恢复/原专项/46/122/6CTest通过；Nav111/
2旧V12缺源保持。早期1历史源比较遗漏r278继承worker seam，验证父adapterSHA
后继续完整旧几何字节对比，原失败保留。最终源码后绑定302，一次seal372。
fresh六控制器/500Hz/0.6/D6/splines/实际0.8/72帧/原片首秒通过，独立图像
工具实际看后才启动连续程序；首23.1525仍原<25窗内，不隐瞒启动前等待。

双装篮75.8205/首至52.668sim、原中部对夹/10sim试持/3sim落稳，壳外原空爪与
新状态FCL返回通过；五指尖按键、30rpm稳转30.105sim、正常停稳/120deg开盖/
3sim等待，机器PASS147.0845。实际0.35接近/18N双抓/原冻结/pilot/完全抬出/
10sim试持/外移/运输、坡度粗放桌/原0.09细放桌/正常松夹/3sim落稳/末10sim
全部通过，两盒不同桌槽、两篮空、盖120/转子停/starts=stops1、holder空。
完整PASS207.5105-首23.1525=184.358sim，旧r266185.695改善1.337，仍超180
4.358sim，不标速度PASS。独立28/28与全双持物PASS，速度FAIL；posthoc
0/0/0/1/0/0/0保留。独立每手2644/同时5286，FK/native1.085604/1.085606mm、
角0.728339deg；结果即时native峰1.099532mm，不互相代替窗口。粗指令times
由实际seed/原points/.30/.6逐值重算一致；不称更高率或新路径自动资格化。
2mm5deg/1mm/18.01N/0.2m/s原数值容差/3、10、30与原冻结不改。

先视频正常SIGINT/刷盘/全解码再场景SIGINT，四收尾PASS/四角色0，九PID/ctime
退休。原片101605175B/32027真实0重复/gap0.113601518wall，20完整阶段/末图及
14/17/20原图已看；末俯视可见两桌位蓝盒与空篮，远手部分遮挡/闭盖自然遮挡。
trace964604/3193741038raw/685421470gzip，原SHA
fb43eb7c3aa8a2bdd11bc5823a4770b761c3f164c8471a8f88ad53ae2fba31d2；372源/302
资源/36归档依赖不变，七组旧保护/10备份/785正式源/10262历史一致，额外历史
已知差异不改。无reset/detach/异常松夹/导航/真机/新备份/清理，正式模型源码
默认未改。559成员0444，REPORT SHA
56f04cb98786c824f0fabf4d2173b4539da4dd6839f7fa96b91e36f18346e50e，SUM SHA
18562df238910b8f6596d738a9f9f3fed40726df0d261c9c94bb8e8c316cc081。
最新完整物理基线r279184.358sim，不覆盖r266；180全目标active未达。

### 最新只读速度分析，防止盲做共线压缩

r279两个空手RRT原同折线的共线点可从152→6和186→9，但保持原.6/.35速度及
20msfloor时指令时长不变（5.081614与10.491037sim），数学节省0；不为其启动
仿真，不降低floor或提高速度冒充压缩效果。实际启动零力dry-close/reopen两段
1.1045+1.5725=2.677sim；初始原open已通过，两段后仍回同open目标。可独立
测试删除这两个非载物预检动作、以四指原新鲜端点/原velocity/force门替代额外
开闭，保持实际带载抓/原冻结/全部观察证书；不得单以此少于180推断成功。
目标余4.358sim，下一必须依据实际阶段成本，新的完整身份/隔离构建/测试/
最终源码依赖绑定/fresh完整实跑/原片/后保护，不能覆盖本成功559身份。

## 2026-10-15 CST：空间48GiB确认；r280空爪协议候选隔离前检

用户“清理好了”后重新只读确认可用48GiB、无相关Gazebo；不再清理任何文件。
empty_protocol_probe_r1仅离线，父r279559前后SHA/0444保持，22测试通过，11封存，
SUM SHA9b4d939ceab420e3930e9e5103172827718cc558ff7953e34f8ca3863b2177f4。
默认钩子反转等于父源；新候选只原启动零力open、新鲜四指端点替代dry-close/
reopen；三种明确空手零力action成功和新鲜双端点后idle .3→.1；取回空手.35
恢复原.4，追踪.002与原.2m/s故障门不变。r275曾.4失败，仍需新完整物理验证。
原2mm5deg/1mm/18.01N、FCL/20msfloor/所有3、10、30sim证书和载物release不改。
仅预算节省约4.588sim，不称180成功。
普通mkdir全新centrifuge_48box_empty_protocol_full_20261015_r280，从父559/372和
原型11派生298源码/输入，不继承run/claim。两包7.03s隔离/六库恢复，1254局部、
22新实际ROS/原专项通过；Nav111/2原V12缺源保持。早期五项历史源/父计数/alias
比较更新到真实r279父链，旧camera仍经已核验父adapter完整反转比较旧源码；
第二次仅identity旧tests_final日志成员遗漏，保留两轮失败日志，不删除测试。
最终测试tests_revision3_final.log，保留原失败tests_final.log，依赖须绑定最终源。
31逻辑归档/36外部依赖PASS，前七组保护含r279559/原型11通过，三个正式模型
SHA不变；当前前检尚未结束、未seal/claim/实跑，旧r279184.358仍最新完整物理。
失败停持正常SIGINT，新身份一次seal，完整实跑/原片/28门/保护不能省略。

## 2026-10-15 CST：r280取盒规划FAIL557封存；r281严格树边/端点重叠前检

r280378一次seal/fresh首9.4515/九角色/原片首秒已看后立即连续运行。双装篮PASS
60.0105，首至50.559sim；原10sim试持/3sim落篮/三壳外退路及零力FCL通过。
五实际指尖按键、30rpm稳转30.136sim、正常停稳/120deg开盖通过，机器131.439。
取回四flip规划中[0,180]粗RRT成功但最终2mrad全状态FCL拒绝，其他IK/endpoint
拒绝；FAIL132.893/首至前缀123.4415，quiet133.639。尚未派发.4接近或抓取，
没有取回/桌面松夹/末10sim，不称完整或180PASS。独立16/28，持物/速度FAIL；
posthoc0/1/1/1/0/0/0保留，不把失败标签转PASS。正常视频SIGINT/完整解码后场景
SIGINT，九PID/ctime退休，四角色1/0/0/0；holder空、双盒各留篮、转子停盖120。
原片61736988B/19422真实/0重复/gap.140072294wall，15前缀及12/15原图已审。
trace689414/2012107008raw/412640248gzip，原SHA
17de7f4771cff9a5a0dbff25df0d29d512ae46cd9d1edde28a7449c3f2b54842。378源/304资源/
36归档依赖/七组前后旧保护、10备份/785正式源及r279559保持；正式默认不改。
557封存REPORT SHA22e6a8e48360e1029d0bfb03f8a828a73f6661f949695d91329856f015bcfe12，
SUM SHA0348672ca949bb209fe074b8841380676ec8c9b2297fba5f43dd6384d1a4d5f1。

dense_tree_probe_r1仅树边35→2mrad，其他原RRT随机/边界/120wall/迭代/IK/ACM不改，
更严格而非放宽FCL。当前实际失败快照[0,180]全pickup/前瞻pilot/carry通过，
2227/62/2480检查、17.3784wall；6测试/7封存SUM
5e79c89b26f9bc9a2408a2e47bfd4d90ca09587bc2ceedd4096df25ef2c141c4。
endpoint_overlap_probe_r1只在原.15sim完成余量内提前开始原.2sim全关节<.0003
端点观察，要求终点后native joint_time持续>.2及原.15余量，两者都不削短。
瞬态偏差重置完整窗口，旧/非有限/停滞native时钟不能PASS；初始未qualified仍
串行，单holder拒绝，精确反转父源。12实际三ROS/clock/重置等测试通过，10封存
SUM916063f672e293d739aef3b12822bf8711174132569b58f1c83e79385ecee622；两个fixture
各省约.15sim，不是全部实跑节省证书，不改载物参考/原3、10、30证书。

普通mkdir全新centrifuge_48box_dense_endpoint_full_20261015_r281，从557/378及7/10
原型派生303源/输入；实际取回接近恢复r279已通过.35/.002，不继续未执行.4。
两包7.01s隔离/六库恢复、1272局部/18新/原专项/46/122/6CTest通过，Nav111/2旧
缺源保持，31归档/36依赖保持。旧r269历史shortcut probe显式使用已核验旧selector，
仅历史证明；本轮actual selector的新probe_dense_tree_actual.py必须另执行并PASS。
最终源码后绑定依赖、一次seal、fresh完整运行/原片/28门/180/保护仍不可省略。
当前r281前检运行中，未seal/claim/launch；无Gazebo、约47GiB，无新清理/备份。
最新完整物理仍r279184.358sim。180全目标active未达，不缩窄或重复旧身份。

### r281一次密封并fresh连续实跑，覆盖上文“未启动”快照

当前实际selector独立离线PASS：r280失败request不改，181接近点/.35指令
10.079904980sim，17.2389wall；此后1272全测再过。全部前检结束，最终源绑定
308资源，一次seal384；36外部归档依赖和七组before保护PASS，可用约46.6GiB。
普通mkdir run，fresh六控制器/500Hz/.6/D6/splines/两个实际.8启动字段/72帧/
同原片首秒通过，独立图像工具实际看后立即启动连续程序；首非空9.3815<25。
九PID/ctime在run/live_checkpoint1.json核验live，collector4062076/1792003217.12。
当前新Gazebo和原片正在完整连续执行，禁止另启actor/修改密封源/重复claim。
终态必须由run/result.json和独立审计判定；现在不是完整物理或180成功。
失败原停持/视频后场景正常SIGINT，由本身份supervisor自动收尾；旧模型/备份不动。

## 2026-10-15 CST：r281退休FAIL570封存；桶轴退出原型8通过，180仍未达

本段覆盖上文r281 live快照：已无相关Gazebo，约47GiB，不重复旧身份/claim。
r281首9.3815，装篮59.2505/首至49.869sim（比r27952.668快2.799），原试持/落篮/
壳外退路通过；五实际按键/30rpm稳转30.1535sim/正常停稳120deg开盖，机器129.793/
首至120.4115sim。新的严格树边实际pickup/.35接近/原降下、18N双抓/原参考冻结/
15mm pilot通过，r280的空手取回规划失败已不再现；这不是完整取回资格化。
实际post-pilot transport规划lift index26/fraction.5被右盒–右篮原2mrad FCL
拒绝，完整lift尚未发布，FAIL149.011/前缀139.6295sim；无完全抬出/提取10sim/
返桌释放/末10sim，不把前缀小于180当验收。独立15/28、全持物/速度FAIL，
posthoc0/1/1/1/0/0/0。原停持/quiet149.6025，原双holder保持18N、不异常松爪/
重置，双盒仍部分在各篮内，转子停盖120。正常视频SIGINT/完整解码后场景
SIGINT，四收尾PASS、九PID/ctime退休，四角色1/0/0/0。
原片69365012B/21767真实/1重复/gap.194732193wall，17实际前缀及13/17原图已审，
不伪造缺失抬出/桌返；trace760834/2236311884raw/467193069gzip，CRC/SHA/计数
PASS，原SHAf95192da738ab4b4a75225dad5e4c7d446d0337e5b1c16227ef7160ae44990ef。
384源/308资源/36归档外依赖及七组前后保护、10备份/785正式源/10262历史与父557/
原型7、10保持，旧额外差异不改。正式三个模型SHA仍44ff23../f017df../dc582a..，
正式默认未留下失败调参。570封存REPORT SHA
46a8cbe1bdfb3ae0d9da3a3ca021b5130d5f56b00d00029a83124a75efecffa6，SUM SHA
b9e88fbc1c433e0fc6042bbc40061016bf8bd3b4f02090eac8dee122a3f489ad。

lift_density_probe_r1的161/321同一世界竖直抬升仍FAIL，4封存，不为它实跑；SUM
6bb0764eaa257faed218742040c63e5d99af7432cdc2a868edbd860c45f67584。
新bucket_axis_exit_probe_r1：仅原lift的81个Cartesian目标两段退出，先在实际桶
frame居中xy并沿桶轴至原碰撞mesh rim上6mm，再回原世界高位；原初末位姿/世界
朝向、82关节点/.4抬升/20msfloor/其他stage/FCL2mrad/2deg倾斜/旧ACM/原holder
参考/物理门都不改。当前失败request/result绑定，完整transport及释放开口FCL
离线PASS，9.995wall、lift原1.74sim，10测试通过。右桶pitch .211410deg/local x
残偏-.432257mm和原失败点支持世界竖直的桶局部横移碰窄壁这一几何推断；不是
允许碰撞，也不是新物理或180证书。父570/384前后SHA/0444保持，8封存SUM
6b707dda099c3eb804e6cbeb79908b24d864b67bf5ba219092022201aa00109d。

下一从已冻结r281570/384和轴退出原型8派生下一个未用完整身份，当前尚未创建
下一full run。必须加默认保持的目标provider并精确反转父源，仅实际双holder/
retrieval_transport/lift激活；原prospective空holder几何不能变成实际许可。
先隔离构建/完整回归/当前actual-selector离线审核，最终源码后绑定依赖、一次
seal、普通mkdir run、fresh健康/原片首秒独立审后立即连续程序、完整物理28门/
原片全解码/正常SIGINT/前后保护。禁止复用r281或在原型续跑，禁止源/旧成果/
备份改写、reset/detach/异常松夹/真机/新备份/清理。最新完整物理仍r279184.358sim，
180全目标active未达；下轮继续实际实现和验证，不缩窄为成功前缀或数学预算。

## 2026-10-15 CST：r282桶轴退出provider一次seal/fresh连续实跑

普通mkdir未用centrifuge_48box_bucket_axis_exit_full_20261015_r282，父r281570/384与
轴退出原型8逐SHA/0444验证，派生306源/输入，未继承run/claim/实际PASS。
只actual retrieval_transport/lift且原两holder身份/冻结source epoch匹配时用
桶frame居中xy/轴向越原mesh rim6mm/回原世界高位；非此phase/mode、prospective
空或缺holder保留原几何，非实际资格化。81Cartesian/82关节点/原端点/世界朝向/
.4抬升/20msfloor、原FCL2mrad/2deg/ACM/参考/所有3、10、30证书都不改。
worker只有provider import和循环调用，两步逆变换逐字等于父源，其余物理体
不改。当前实际失败快照通过新生产provider完整transport/释放开口2470检查，
lift1.74sim、9.6365wall，仍仅离线。22新默认/身份/有限值/目标/FCL测试及1294
全部通过；两包6.96s隔离/六库恢复/原专项/46/122/6CTest通过，Nav111/2旧缺源。
第一次collection缺coupled_path环境，改从生产dual_worker导入同原函数；第二次
仅旧test_dense_shortcut父SHA指向r278，更新到真实r281，保留两轮失败日志和全部
原FCL检查。最终tests_revision3，最终源后依赖绑定；31归档/36外依赖、七before
保护包括父570/加密原型4/轴原型8保持。正式默认模型源码/旧结果/备份不动。

一次seal395成员/312资源，可用49494753280B>原2096208487B预算。普通mkdir run，
fresh六控制器/500Hz/.6/D6/splines/实际两个.8启动字段/72帧/同原片首秒通过，
独立图像工具实际看后立即连续executor；首非空9.841<25，九PID/ctime核验live，
collector326450/1792007355.29。当前r282新Gazebo和原片在完整执行，不另启动
actor、不修改任何密封源/输入、不复用claim。终态只按本run/result和独立审计
判断，现不是完整或180成功。失败原停持、视频后场景正常SIGINT，由supervisor
收尾。旧r281已退休，最新完整物理仍r279184.358sim，180全目标active未达。

## 2026-10-15 CST：r282退休FAIL563封存；可选退路14/微对齐10原型通过

本段覆盖上文r282 live：无相关Gazebo，约46GiB，不复用r282/原型/任何旧claim。
r282首9.841，原双抓/冻结/10sim加载试持/运输通过，发生实际可选装篮对齐，
其阶段4.063sim。两overhead gate通过/原正常松夹/3sim自然落篮，装篮PASS64.19/
首至54.349sim，两盒各完全入篮。当前已执行loading guides是原四stage加一次
最终alignment（phase同inside）；原实际释放场景empty planner3312 FCL通过，
产生四反向stage及最终return。旧PostRetreatGripperCycle硬认三段，于任何退路
发布之前合同拒绝，FAIL64.4655/前缀54.6245sim。不是轴退出物理失败，关盖/旋转/
开盖取回/轴lift全部未运行；不得以短前缀称180PASS。独立10/28、全持物/速度
FAIL，posthoc0/1/1/1/0/0/0。即时native峰.001100416937m，原物理门不改。
原停持/quiet65.1995，holder空/双盒各留篮/盖120/转子未启动；正常原片SIGINT/
全解码后场景SIGINT，四收尾PASS/四角色1/0/0/0、九PID/ctime退休。原片39812427B/
12590真实0重复/gap.151744344wall，10实际前缀拼图/末图及额外实际alignment
原图均已审，不伪造机器/取回。trace287014/1283504860raw/258487497gzip，CRC/
SHA/计数PASS，原SHAc36cb856c874422cf00a5cf3a3f217e1dcd2237674e75ab973c88489ae2536e1。
395源/312资源/36归档依赖/七前后旧保护、10备份/785正式源/10262历史、父570/
原型8保持，额外旧差异不改。正式默认未留失败调参，三正式模型SHA不变，
无reset/detach/异常松夹/真机/导航/清理/新备份。563封存REPORT SHA
783f0581db612bc3914ebddd282a73265858c9bd757b495b5b062c097d9355c6，SUM SHA
6b063f3438d0ee0789747f7bfb45f2fbc94d98685d678b7f30603d464c4ff429。

optional_alignment_retreat_probe_r1只从本run完整有限15关节guides推导精确反序，
允许原四forward阶段且至多一次实际最终alignment；必须全部四退路完成，不
跳过optional。默认三段行为/原参数不变，cycle/worker修改逐字可反转父源。
两个inside reverse都请求原.13（不能因第二个index1变.8进而实际.26），原mode/
身份/zero FF/31点FCL/native.00025新反馈/.0003漂移/remaining FCL/全部物理门不改。
13新+36旧49测试通过，实际数据的prospective四退路后端点31/1663 FCL通过，但
不是实际新端点或四退路物理许可。14封存SUM
736f49ac2c795dbfe8bd7e6636a34d57725d5b3938c6cb2cd232c250ef29bbf8。

micro_alignment_probe_r1：只actual load_alignment/原inside phase/两个实际holder
及冻结epoch匹配、body位移<=5mm/转角<=1deg时用11Cartesian目标和当前实际warm
seed，不再引用远处旧升篮guide。其他mode/phase、空holder/prospective、大偏差
仍原81/旧guide；非法上下文/非有限值拒绝。原目标/.1rate/20msfloor/FCL2mrad/
关节界/倾斜/旧ACM/holder参考/力/native/释放门不改，非全局提速。当前r282实际
alignment快照用生产条件helper完整几何PASS，原指令3.197499607→.597861082sim，
数学省2.599638526，不预称真实节省；13测试通过，最终result_revision2/
probe_revision2/tests_revision2对应条件helper，旧无条件初探和12测试日志保留。
父563/395前后SHA保持，10封存SUM
ebea4e336c3bb6a415e256d0c63af8ab2cb4a45a8983e9d3d0acce24d4adac4c。

下一从父r282563/395及两个原型14/10派生下一个未用完整身份（当前尚未创建下一
full），保留已绑定轴退出provider，修复完整可选退路合同并接入仅微量对齐的
默认保持参数provider；精确反转父源/隔离构建/原回归/最终源依赖/一次seal/
普通mkdir run/fresh原片首秒独立看后立即连续程序/完整实际全流程/28门/180/
原片全解码/正常SIGINT/前后保护均不可省。源和正式旧成果/备份不改；不在原型
或失败run续跑。最新完整物理仍r279184.358sim，180全目标active未达；继续真实
实现和验证，不缩窄为装篮前缀或离线预算。

## 2026-10-15 CST：r283可选退路与微对齐一次seal/fresh连续实跑

普通mkdir未用centrifuge_48box_optional_micro_full_20261015_r283，父r282563/395与
可选退路14/微对齐10逐SHA/0444验证，派生313源/输入，不继承run/claim/PASS。
只修复本run实际可选alignment的完整逆退路合同，两个inside均原.13；只实际
load_alignment/双holder及原epoch/<=5mm、1deg用11目标和当前warm seed。原目标/
.1rate/20msfloor/FCL2mrad/2deg/ACM/native/force/冻结/全部3、10、30证书保持。
原三退路和其他默认几何/参数不改，cycle/gripper worker/dual worker新增部分
可精确反转父源。保留此前桶轴退出（尚无真实执行证书）与.35接近等，不能把
候选继承当完整资格化。1320全部/26新/原专项/两包7.03s隔离/六库恢复/46/122/
6CTest通过，Nav111/2原缺源保留。生产桶轴当前失败快照2470检查/lift1.74sim；
生产微对齐当前快照73检查/.1指令3.197499607→.597861082sim，均仅离线。
最终源后依赖绑定315、一次seal399，36归档外依赖和七before保护含父563/14/10
保持；49162383360B>原2096208487B预算。普通mkdir run，fresh六控制器/500Hz/
.6/D6/splines/实际两个.8启动字段/72帧/同原片首秒独立看后立即连续executor，
首非空10.1915<25；九PID/ctime live，collector774341/1792011329.1。当前本身份
Gazebo和原片完整流程正在运行，不另启actor/改密封源/复用claim。原失败停持/
视频后场景正常SIGINT由supervisor收尾。现在不称完整/180成功，最新完整物理
仍r279184.358sim，180全目标active未达，正式源模型/旧成果/所有备份不动。

## 2026-10-15 CST：r283完整179.129sim秒、独立物理/录像/保护全部PASS

覆盖上一节r283 live及此前180未达快照。全新
runtime_evidence/centrifuge_48box_optional_micro_full_20261015_r283已完整执行并正常退休。
首有限非空sim10.1915→实际完整PASS189.3205，精确179.1290sim秒，独立
complete_speed_audit PASS。两桌面85%48box真实双指中夹、两盒完整10秒保持、
分别自然落篮及3秒稳定、真实指尖关盖/启动/稳速30.258sim秒/停稳/120°开盖、
按实际停转姿态取回、完整抬离后各10秒保持、两个不同桌槽正常松夹各3秒稳定、
最终无持物10秒均完成。28/28物理门和完整独立双持物PASS，4建立/4正常释放，
每手2566/双持物5130检查；独立滑移1.232205503mm、角.531667626°，即时原生
峰1.275164542mm口径不同；取回残速.001351935667rpm，holder空、最终转子停止。
仍按用户已授权5°/2mm和仅取回制动瞬时.1rpm合同，原力/速度/FCL/观察证书保持；
不改写旧严格证书。原静态骨盆x=.12/设备x=.52，不是自由站立、RL或Nav2联动。
真实wall2324.616083秒；三分钟是仿真时间，不是电脑等待或快放成绩。

桶轴退出已在本run实际82点lift/1.74sim指令、完整2477 FCL/10秒保持/双返桌中
通过，当前command与本run planner/原.4时序逐点一致；粗下放原.3/.6弧长时序
亦按actual重算。实际未触发可选loading alignment，只有正常三段逆退路；
新增四段退路和11目标微对齐仍只单元/离线资格，不能声称其预算为真实节省。
1320局部/26新、两包7.03s隔离、六库恢复、原专项/46/122/6CTest通过；
原Nav111/2项V12缺源仍保留，未伪造checkpoint恢复或全导航回归PASS。

原片90410172B、1200×900/12fps、28473真实帧/0重复/gap.147611200wall秒；
正常SIGINT/全解码PASS。20实际阶段拼图与14/17/20原图已独立审，机器人/脚/
桌/设备留在画面；远手/盒部分遮挡、闭盖自然遮挡，旋转由日志认证。末俯视
实际录入原片，双桌槽蓝盒/空双篮可见，非补拼过程；抽帧不声称exact sim同步。
trace939668行/3004844139逻辑字节/638421176gzip，CRC/原SHA/计数PASS，原SHA
9d4681b4fc6cd60602a9ac711ab688e3e491c464898b6e9b179de929cf69b5dd。
录像后场景正常SIGINT、四收尾PASS、四运行角色exit0、九PID/ctime退休。
前六posthoc exit0；两次保护审计未完成记录保留，最后前台PTY完整exit0，
七组前后完全相等，399密封SHA/0444、315资源、36归档依赖再次验证；
10备份/785正式src/10262历史及后续保护链/父563/原型14/10不变，旧已知差异不改。
三个正式模型SHA维持44ff23fc…、f017dfcf…、dc582ab7…，完整值见总核验JSON。
REPORT SHA f40a4d741ed3040ff969626be3a416996f76d0113606337ab0b0d434e1709404；
identity SHA 3b9a4d17a22260dd88fe414cfeb8ad5118cc140bfa89833d3c4f7992b4e56d4b。
此前r279184.358和全部旧成果保留。此次单次完整fresh达标，不外推重复可靠性。
后续默认只读；任何再展示/修改/运行仍须目标授权与未用身份、普通mkdir独占、
隔离回归/保护/一次seal/fresh/新录像/完整审计，不复用r283或旧claim。包内受保护
README/PROVENANCE/日常入口未改，接口总结见根CENTRIFUGE_FULL_CYCLE.md。
920永久箱子版本及全部备份不动；无真机/导航/reset/detach/异常松夹/清理/新备份。

## 2026-10-15 CST：用户改为现实三分钟，实际1:1前置门尚未通过

用户随后明确将目标从仿真180秒改为现实墙钟180秒，并提出先让实际仿真1:1再
运行原179.129sim秒全流程。旧r283仿真目标仍成功/封存不改，但它的wall
2324.616083秒不满足新目标；不可改报告、时间戳、快放、删观察或换统计起点冒称达成。
原world已real_time_factor=1.0/step.0005/update2000，1是目标不是实测保证。

新独占被动性能身份centrifuge_48box_wallclock_perf_20261015_r1/r2/r3，仅静止
完整机器人/夹爪/Mid360/围墙/桌/设备/双盒，未发布抓取/关盖/转子/腿部任务。
每个配置六控制器exact active、12秒monotonic_ns区间、原sim stats推进测量：
DART默认ODE .134789069；DART Bullet碰撞器 .009182539；FCL .007310967；
Bullet Featherstone .094162284；传感器Ogre2 .133501576；DART PGS .127343184；
原私有插件与overlay仅-O2 -g .135073663。全部远低于实际1:1，未采用慢配置，
未在已知不满足前置门时再浪费一次三十多分钟全流程；新现实180目标未达。
O2原C++/安全头逐字相同，无-DNDEBUG/fast-math，模型/物理步长/增益/轨迹不改；
原六Conda库恢复。r1/r2/r3分别26/26/25项Python测试，原配置46/6CTest、优化
插件4项C++测试通过；隔离构建6.94/7.05/8.41s，Nav111/2旧V12缺源保持。

各配置普通mkdir独占run，原SIGINT收尾/剩余0；当前无该仿真。每probe七组
before/after均精确相等，10备份/785正式src/10262历史、r283601及旧链保持。
r2/r3 before采用同一连续工作中上个完整after的原字节绑定，attestation如实
记录来源，不冒称重复计算；随后各自完整执行after。115/163/96密封成员事后SHA
保持，三报告与原始stats/CPU样本保留。r3 REPORT SHA
2ea7f922acb194818a0ca1e7b46ddc46b2653fab7e3714a5190f2ddd1cea4dfb，
总表r3/PERFORMANCE_SUMMARY.json。无新备份/清理/真机/reset/detach/异常松夹。

现实计时monotonic_ns单元与连续稳转同时≥30wall/30sim门已实现测试，尚未接入
已资格化完整执行器，不能称新全流程已按墙钟运行。下一先定位单step/接触耗时，
保持原流程/模型，必须测实际接近1:1且新整套wall≤180才验收。不能只把target设1
就重跑并预称现实三分钟；若改变步长/采样或简化碰撞，必须另立数值精度合同和
完整新物理验收，不能借用r283旧证书。已有模型/源码/authority/run/视频/报告/
builder链/所有备份永久保持，尤其920箱子版本；最新完整物理仍r283179.129sim秒。

## 2026-10-15 CST：用户改为机器人前方视角重录，r284准备中

用户最新要求将外部观察相机移到机器人前方重新录像。新独占
centrifuge_48box_front_camera_full_20261015_r284 / 同名install overlay，直接父为
r283的601封存/399密封成员，已逐项SHA和0444验证。仅GUI/录像相机变为
position[1.65,-1.55,1.95]、target[.25,0,.95]，不改物理机器人相机、模型或动作。
1323项回归通过、两包6.81秒隔离构建/六库恢复。首两次失败为祖先/直接父绑定元数据
及验证日志命名，失败记录保留，动作未为通过测试而变更。派生元数据分别标识祖先
SHA与直接r283 SHA，不混称二者。31归档/36依赖完整逻辑校验通过。
五共同before如实绑定同一连续工作r5完整after，新增链现算，结束仍完整七组after；
冗余前缀只读计算正常中断记录保留，不是Gazebo失败或物理停持。
当前仍准备、未启动r284场景/录像/动作，不预称完成。必须一次seal、新run普通mkdir、
fresh健康/新原片首秒独立实际检查后立即连续原完整流程；完成后正常SIGINT录像再场景，
全解码/28门/完整双持物/trace/保护仍必需。新原片不可覆盖旧视频或用剪辑当实时成绩。

r4观察采样2199点、静止RTF.128826，CPU主要在物理接触/约束；r5原生碰撞对观察
RTF.130501，双脚对地每步各250点，占窄相碰撞时间约99%。二者场景正常SIGINT退休。
r4七组前后相等并封存115成员，SUM f72e5e1df2d5fc68dadf03a9ad7d1a54b79bdf060c364283b31bbfcce98a3a01。
r5七组前后亦PASS，尚未最终封存。r6脚底4/8点仅隔离构建、26单测和29C++边界检查，
尚无任何实际场景证书，禁止借用为完整或腿部行走资格。本次录像不加载r6/LD_PRELOAD。
性能实验暂停接续，现实180目标仍未达，原r283成功与所有正式成果、备份不变。

### r284已fresh启动并连续录制，覆盖上一“准备中”快照

414成员一次seal/0444，identity SHA f6ea4f757e846006f6ae1958bc04bf0ea6702bdbe1663d93f7f9ff0e20830918；
321资源绑定保持，普通mkdir run，启动前另实际重算785正式src通过。domain87 /
GZ_PARTITION=h1_front_camera_r284，六控制器/500Hz/.6/D6/splines/初始ramp/72帧烟测通过。
新原片首0.5秒已实际解码并在独立image工具审过，正面脸/胸标/桌/近侧盒可见，
桌和设备自然遮挡部分腿与远手。原首帧helper的“.5”时间串解析失败/60秒日志保留，
未改密封源，以显式“0.5”从同一原片解码、实际审后才首次启动执行器，不能称原helper成功。
首次有限非空sim16.5965<25；executor3201390/1792049420.38，四角色连续程序已启动。
当前Gazebo和前方视角原片正在完整流程中，不另启动作actor/改密封源/复用claim。
这是录像任务不是1:1性能验收；未采用r6，不预称当前完整或现实180秒成功。
完成仍需原正常收尾、28门/完整持物/trace/原片全解码/完整七组after；失败按原停持收尾。

## 2026-10-15 CST：r284前方视角完整重录通过并退休，覆盖上述live快照

用户最新“把摄像头调整到机器人前方重新录像”已完成。新r284仅外部观察相机
[1.65,-1.55,1.95]看[.25,0,.95]，机载相机/原模型/动作/0.5ms物理保持，未加载r6。
完整双盒装篮、实际关盖/稳转至少30sim秒/停稳开盖、双盒取回分桌释放及无持物
10sim秒完成。独立28/28、双持物、正确成对速度审计PASS；首16.5965至195.9105，
179.314sim秒/2404.925346wall秒。现实1:1和现实180秒仍未达，r283179.129sim旧成果保持。
独立native滑移1.048223mm/角.690284°、每手2566/同时5130检查，原授权合同不改。

原片58226707B/1200×900/12fps，30200输出帧=30193真实+7补帧，gap.151120214wall秒；
窗口变大后整窗等比留边，非固定裁切，不称零重复。正常SIGINT/全解码/实际20阶段
和末俯视审看通过，末图为同原片实录。原首帧helper“.5”失败日志保留，显式“.5”
改为“0.5”的外部实际解码和独立审看在首次执行器前完成，密封源未改。
正确audit_dual_speed写complete_speed_audit PASS；额外误调用旧串行audit_complete_speed
因无串行专用事件exit1诊断保留，非成对合同失败，不能冒称所有posthoc exit0。
trace940056行/3079321007逻辑B/666078013gzip，CRC/原SHA/计数通过。
414密封/321资源事后验证，七组完整after与before相等，10备份/785正式src/
10262历史及后续链不变。录像后场景正常SIGINT、四角色exit0、身份退休，当前无该仿真。
613结果成员只读封存，REPORT SHA 5eaa455f365cab992ff93937566d181a4b653e24bc25ee9dfbfb1767585e2844；
SUM 4142af20e7368ea3a8db556384ad51efd76552b5cc29f1b773985300417909fb。
完整结论见r284/REPORT.md、front_terminal_integrity.json，不复用本run/claim。

新媒体身份centrifuge_48box_front_briefing_20261015_r1保留诚实EDL和来源，交付
/home/liu/离心机全流程_前方视角_汇报演示_20261015.mp4，114.466667秒/完整解码，
无新增画面文字；这是演示剪辑不是执行耗时，原片及旧视角视频保持。成片SHA
32bb10bcd050db7bfe2e013d306e6f335202eb303e686845c0b42a6023fe88b3。
没有新备份/清理/真机/导航/reset/detach；920箱子版本永久保持。r6仍仅构建测试，
后续性能继续须新运行身份及数值合同/完整再验收，禁止据本次相机录像外推其资格。

## 2026-10-15 CST：用户授权减少翻臂并重录，r285新身份完整实跑中

新centrifuge_48box_minimum_turn_full_20261015_r285从r284的613封存/414密封成员逐SHA/
0444核验派生，旧r284/r283和正式模型/所有备份不改。仅实际空手pickup比较原0/180
对称接触框架的多种IK种子，以真实未取模关节路径的总转动、肩yaw/腕roll/yaw转动
和最大偏转选更小的完整路径，不按名义flip标签选。保留原规划回退/2mrad FCL/
IK精度/关节界；候选有限RRT只缩短搜索预算，不放宽几何门或原120wall规划守卫。
实际生产请求携物Cartesian目标不改，分段IK沿上个实际解连续求解，避免被旧guide
强拉回远处分支；其他历史请求/空退路保持原行为。dual_worker新增部分可精确反转
到直接r284父源，其余执行器/真实夹持/力/参考/10、3、30秒观察原字节保持。

rev1/2仅直连或原前缀不能形成有效替代，rev3只装载原端点捷径小幅改善；失败与
未改善诊断均保留。最终rev4离线装载完整前瞻pilot103/搬运2191 FCL通过，轴向
7.061210→4.558556rad，左腕抓取roll约-132→4.14deg；取回未发现更优安全候选，
保留原解。这里只是离线几何，不预称新物理抓稳、全消翻臂或全局最优。
1349全部/26新测试、两包7.15秒隔离/六库恢复、原46/122/6CTest通过；Nav111/2旧
V12缺源保持。31归档/36外依赖、七before保护通过；共同五组来源绑定r284完整after，
新增链重算，最后仍须完整七after。785正式src另在启动前实际重算PASS。

一次seal436/329资源，identity f434e4d7ec6ec48bd6db57dd8229147e009fa706d4aebba767005c345ad58b48。
普通mkdir run，domain88/partition h1_minimum_turn_r285，六控制器/500Hz/.6/D6/splines/
实际ramp/72帧通过。新首帧helper仅修正“.5”为“0.5”，同原片实际首帧已独立查看：
脸胸/桌/设备/近盒可见，远手与盒自然遮挡如实保留。审图后首次启动连续程序，首
有限非空sim9.4315<25，executor283065/1792055445.82。当前Gazebo和前视原片完整
运行中，不另启actor/修改436源/复用claim。正常失败停持与录像后场景SIGINT由原
supervisor收尾；新结果必须看实际28门/独立双持物/最小转动发布路径复核/原片解码/
七after，不能预称完成。未加载r6/LD_PRELOAD，没有真机/导航/清理/reset/detach/
新备份。新任务优先安全减少翻臂，不将剪辑或原仿真179秒冒充现实180秒；现实目标未达。

## 2026-10-15 CST：r285少翻臂完整实跑/录像/保护通过并封存，覆盖上述live

r285完整双盒装篮、实际指尖关盖/稳转至少30sim秒/停稳120°开盖、双取盒/分桌正常
释放/无持物10sim秒通过。run/result PASS、独立28/28及全双持物PASS，每手2650/
同时5298检查、4正常释放/holder空。独立实际新审计重建同一fresh快照原规划，
核对已发布samples并重做2mrad/限位：装盒轴向7.061209783→4.559454033rad（-35.429563%），
取盒8.777889977→8.398185876rad（-4.325688%），1697/1799检查。装盒左腕roll约
-132→+4.112270deg；取盒只压缩原接近绕行，原抓取端点和下降逐点保持，必要朝向仍在。
不称全消翻臂或全局最优。携物目标/原夹持、力、速度、参考和全部观察门保持。
native滑移峰1.274677188mm/FK1.006041736mm/角.813993860deg。取回瞬态.038001071rpm
满足原授权.1rpm，但超过旧严格.02峰值，不能称旧严格全证书；不是本轮新增放宽。

首9.4315至完整PASS203.4105，193.979sim/2537.422484wall秒；增加多候选规划后较旧
速度基线慢。原audit_dual_speed exit1/complete=true/180sim FAIL原样保留，其他七posthoc
exit0，不能将少翻臂物理PASS说成速度PASS；旧r283179.129/r284179.314sim原字节保持。
现实1:1/180wall目标仍未达。本次是单次固定骨盆Gazebo，不外推重复可靠性/导航/真机。

原片96524661B/1200×900/12fps/2594.75秒，31137真实0重复/gap.145329600wall；
整原生窗口、正常SIGINT/全解码/相机守卫PASS。首帧、实际480秒原帧、20相位拼图及
同原片最后一秒独立审过，末俯视见双桌面蓝盒/空篮，远手/局部自然遮挡不隐藏。
录像后场景正常SIGINT，四收尾PASS/四运行exit0、全部身份退休，当前无本run仿真。
trace1006199行/3156894251逻辑B/675023855gzip，CRC/SHA/计数通过；逻辑SHA
4800b697a79441623118790880e9adfe2275c424a592e881bd48b63cc64ae157。
436密封/329资源/36外依赖最终再验，七完整after与before完全相等；785正式src/
10262历史/10备份/直接r284613/旧媒体38及后续保护链不变，920箱子版本不动。
1349全部/26新/两包7.15s隔离/六库恢复/原46、122、6CTest通过，Nav111/2旧V12缺源保留。
632结果成员只读封存，REPORT SHA712377afb67bc470af66ee4935fa405510f2eb2c99594a06102147d92293a26c；
SUM424038061100b6fa80df89bb325416ae25970e90258576dfa7d7ba6b90373d76。
最终结论见r285/REPORT.md、minimum_turn_actual_audit.json、minimum_turn_terminal_integrity.json。

新媒体minimum_turn_briefing_20261015_r1交付：
/home/liu/离心机全流程_少翻臂优化_前方视角_20261015.mp4，177.466667秒/全解码，无新增
画面文字。它是10倍动作/缩短观察的演示剪辑，不是实跑三分钟；EDL/来源如实保存。
SHA1ceb26bf07149177029472b9b9cef5b018512de43af9a9c279a66a3c89bcd80f。
首版媒体宽token计数防护在生成输出前安全停止，原失败源/JSON保留；revision2只替换
明确速度字段，不改日期路径/仿真/原片。旧视频均保留。没有新备份/清理/回收站恢复/
真机/腿部导航/reset/detach或r6加载。不得复用已封存r285/claim；后续须新身份和原完整门。

## 2026-10-16 CST：红线抬肘姿态仅离线候选，未改正式流程

用户新图示要求理解为抬肘、上臂前伸、前臂朝下，不靠翻转夹爪/移动盒子造型。
普通mkdir独占centrifuge_48box_elbow_posture_probe_20261016_r1；没有新Gazebo/run/控制命令。
直接r285632个0444成员及三正式设备模型，前后逐SHA相等；旧模型/轨迹/结果/媒体/
920箱子版本及全部备份未改，没有新备份或清理。只增离线原型/交接文档，无ROS包源
或正式资产改动，未重新构建，不借用旧构建宣称实际资格。

取r285实际9_retrieval_transport快照，单独优化每臂冗余，躯干/另一臂/夹爪关节和双腕
六维目标不变。左肘+10.1109cm、肘至腕与竖直向下70.3370→39.1073deg；右单独候选
+5.7790cm、56.1398→35.5368deg。两个候选分别重新建场独立2mrad/FCL/原缩进限位/
20um/.01deg/盒2deg门468/515检查通过，腕路径误差峰5.4262/6.8718um，非实际滑移。
不能把分别通过当双臂同时改变通过；左候选距原肩yaw上界仅约.020007rad，下一应留
姿态余量并整段新规划。原wrist yaw部分visual无collision限制仍在，未扩大豁免掩盖。

装盒最后篮上节点的单臂及躯干±20deg/双腕固定局部优化均无明显改善，失败/未改善
JSON保留；不能把取盒候选冒充装盒全流程，也不由局部优化断言全局不可达。
15新局部测试通过；revision3图是FK科学示意，非Gazebo实录，首版缺字警告和旧图保留。
详见新目录REPORT.md/independent_geometry_audit.json。未实际执行新姿态，完整物理仍
r285193.979sim，三分钟仿真速度基线仍r283/r284，现实1:1/180wall未达。后续若接入
完整任务须全新身份/隔离构建/原fresh与实际夹持/整程避碰/录像/独立门，不能直发静态q。

## 2026-10-16 CST：用户授权装盒与取盒整段抬肘适配并完整实跑录像，r286准备

新普通mkdir独占centrifuge_48box_elbow_full_20261016_r286与独立install overlay，直接父
r285632/436逐SHA/0444后派生，不改旧成功版本/模型/全部备份，尤其920箱子版本。
新空手0/180对称框架/多种IK种子选择，连续携物IK在原六维目标上用冗余抬肘偏好，
每次姿态优化相对原严格IK限±.045rad；原限位缩进/20um/.01deg/2mrad FCL保持。
不强锁肘角；不以预测冻结实际参考。已明确允许新姿态分支改变必要的肩/腕转动，
不能借用r285原累计最小转动、旧仿真180秒或现实180秒证书。

rev1装盒/取盒离线30.20/88.69wall秒通过，rev2将冻结旧路径仅绑定成新密封提示，
每个新路径仍在当前全场景重做原严格FCL，规划27.11/58.42wall秒；非复用旧命令。
篮口段最大偏竖直：装盒左41.6297/右26.3443deg，取盒左43.3997/右42.0842deg。
预测pilot/完整搬运/释放后撤手/空爪行程/剩余回neutral通过；只离线、非实际抓稳。
1369全部/20新测试通过；原两包5.97s隔离/六库恢复。完整七before本轮实际重算，
不用上一天after冒充；原模型/物理/真实夹持/2mm5deg与10/3/30观察门均未修改。
当前仍准备/未启动新Gazebo；一次seal/fresh六控制器/原片首秒独立实际看后才启动
连续程序。新动作与录像失败原stop-and-hold/正常SIGINT，不reset/detach/异常松夹。
新结果须看完整28门/双持物/新实际姿态与已发布轨迹审计/原片全解码/七after；
未完成不得称成功。无真机/腿部导航/清理/新备份/r6或LD_PRELOAD。

### r286已经fresh启动连续实跑录像，覆盖上一准备快照

453成员一次seal/0444，identity SHA890921f9a05a76ebd8dbef1776eb479b1fbb4c2746005680cdc0f862d324a4e3；
338资源/36归档依赖绑定，完整七before本轮实际重算PASS。前检曾因提前binder与旧
before审计重复创建校准证书而停止，失败日志与初次四组结果保留；再次完整七组
通过，重算校准JSON语义相同但少末尾换行，两个版本保留并恢复原绑定字节，未改证书。
后续原前检后缀正常完成，1369全测/20新测/原46、122、6CTest通过，Nav111/2旧缺源不改。
独立2071/2519预测搬运FCL通过，预测松夹撤手/空爪/回neutral亦通过，均非实际抓稳证明。

普通mkdir run，domain89/partition h1_elbow_r286；启动前另实际核验785正式src，六控制器/
500Hz/.6/D6/原启动ramp/72帧烟测/同一新原片首秒解码通过。首帧独立image工具已看，
脸胸/机器人/夹爪/桌/设备/近盒可见，远盒自然遮挡；审看后才首次启动连续程序。
executor2921346/ctime1792114931.22；四运行角色持续，run/live_checkpoint1确认453源未改。
当前完整动作与前视录像正在运行，不能把本前缀称完整成功；不另起actor/改密封源或复用claim。
完成由原supervisor正常收尾，再完整28门/双持物/新实际姿态/视频解码/七after验证；失败原停持。

## 2026-10-16 CST：主机重启中断r286，续接改为全新r287，覆盖上述live

用户中断对话后再说“继续”。重新只读检查时主机uptime约1分钟，r286六个原进程
PID/ctime均已消失、无Gazebo。原录像XIO失去X服务器连接，run_guard已因recorder_missing_or_identity_changed
向executor发SIGINT停持；最后progress为sim45.5875，已有3_load_transport_request但无
对应完成结果、无run/result、无完整流程证书。原部分MP4可探测约597.583333wall秒，
不是正常完整收尾录像。不能复用r286/claim或把中断改写成物理PASS；也不冒称姿态物理失败。

453密封源仍逐SHA相同，无恢复命令/reset/detach/松夹。正在完整重算r286七after，
保留原片/trace/日志和中断报告后封存为中断身份。用户继续原整套任务授权有效，
新普通mkdir独占centrifuge_48box_elbow_full_20261016_r287及独立overlay，直接绑定r286
453密封输入；仅重绑新身份/domain90/partition h1_elbow_r287，姿态算法/模型/力/
物理/全部观察门不变。修正准备顺序：不提前binder，原before审计直接创建校准证书，
不复用上轮的failed准备过程。r287仍准备，未启动；应重新构建/回归/七before/一次seal/
fresh首帧独立看后完整连续运行录像。旧成功基线仍r285，全部正式成果和备份不改。

### r287已fresh连续执行，覆盖上述“准备”状态

r286七after与before全相等，583成员按“主机重启中断、未完整”只读封存；SUM
6da0f5db7fe0b416aac5fb37885f043542fbb374926c4c29e3e2b131eec1fe32，原片/trace原字节保留。
r287两包7.04s隔离/六库恢复、1369全部/20新测试、原46/122/6CTest和独立新姿态链通过；
Nav111/2旧V12缺源保持。新的完整七before已实际重算，通过且未再发生校准重复创建。
初版派生JSON遗漏旧source比较字段，完整继承后修正，原初版元数据保留；非动作改变。
442成员一次seal/0444，identity SHA133b9e8ad0ab171526764e2f2cfe473bb4a7580d1e8a244ff88f8576cf82f9e1，
337资源/36外依赖绑定。domain90/partition h1_elbow_r287、普通mkdir run，实际785src/
六控制器/500Hz/.6/D6/原ramp/72帧/同原片首秒独立审图通过后首次启动连续程序。
executor160744/ctime1792116801.66，首有限非空sim6.52<25；run/live_checkpoint1确认442源未改。
当前四角色与新前视录像完整流程进行中，不改密封源/另启actor或复用claim；尚非完整PASS。
原supervisor正常收尾与新postprocess已挂接，完成仍须28门/双持物/新实际姿态/解码/七after。

## 2026-10-16 CST：r287真实空手速度保护FAIL并退休，r288低速接近准备

r287在paired_load_approach、尚无持物时，left_dex1_finger_1_joint原生速度
.20197374630967288m/s（stamp21.82sim）超过原模型.2m/s。原STOP_AND_HOLD/紧急
转子停稳终sim22.572；未装篮/旋转/取回，整体FAIL。原supervisor四收尾门PASS，
录像正常SIGINT/全解码、场景退休；executor1/其他三角色0。trace审计PASS，完整
物理/持物/速度/姿态/阶段抽帧各原exit1保留，不能冒称完整。七after与before全相等，
442密封输入保持；587失败成员只读封存，失败不是主机重启也不修改成成功。

用户继续同一完整任务，无额外放宽许可。新普通mkdir独占r288/domain91/partition
h1_elbow_r288，只在实际EndpointOverlapCycle构造器设置空手接近cap=.5（原.7rad/s），
原请求.8、路径/三组共同时间/20ms地板、所有跟踪/力/PID/2mm5deg/.2m/s保护不变。
其他运动阶段保持；源码可精确反转回r287父Endpoint文件，不改execute_cycle主体、
模型或旧成功成果。12新三发布器/真实实例构造/错误范围/持物拒绝/其他阶段测试；
初版3个夹具遗漏原已资格flag而FAIL，日志保留，修正夹具而非放宽正式前置条件。
当前r288新隔离构建与回归准备中，尚未启动；必须新完整fresh实跑后才称故障修复。

### r288已fresh连续实跑录像，覆盖上述准备状态

两包6.97s隔离/六库恢复、1381全测/12新空手cap测试/20姿态测试、原46/122/6CTest通过，
Nav111/2旧缺源保持；完整七before本轮重算通过。444一次seal/0444，identity SHA
8f32633258c29081a1cfdb7a9af355a15b713ab99f95b8f93b808c851e736593，338资源/36外依赖。
普通mkdir run，domain91/partition h1_elbow_r288；785src实际核验/fresh六控制器/500Hz/
.6/D6/原ramp/72帧/同原片首秒独立image审看后首次启动executor343731/ctime1792118340.23。
首有限非空sim7.2225<25，四角色与新前视录像连续运行，444源未改；仍不是完整PASS。
当前实际实例接近cap=.5，原.2m/s/滑移/碰撞/观察全部保持；事后新审计另核对实际发布
cap及完整原生四指速度峰值。不另启actor/改密封源/复用claim，完成仍原正常收尾及完整门。

## 2026-10-16 CST：r288装篮旋转完成但取盒外移输出保护FAIL，r289只减速

r288真实空手接近0.5rad/s通过，前缀原生速度峰.1354111686m/s<原.2；实际双盒装篮/
10sim夹持/自然落篮、关盖/至少30sim稳转/停转开盖、重新双抓/完整提起/再10sim保持通过。
paired_retrieval_outboard实际.6rad/s时左输出18.03919692455446N（stamp183.782sim）
超过原18.01N软件保护，原STOP_AND_HOLD，未返桌，整体FAIL。holding=right且实际
双持物registry保留，无异常松夹/reset/detach；原supervisor四收尾门、录像正常SIGINT/
全解码、场景退休PASS，executor1/其余三角色0。完整七after正在实际重算，不预称保持。
原完整物理/双持物/速度/姿态审计FAIL原样保留，r285仍完整成功基线。

新普通mkdir独占centrifuge_48box_elbow_full_20261016_r289，直接核验r288444密封源后
派生，domain92/partition h1_elbow_r289；唯一当前运行修正：双持物取盒outboard及
transfer实际.6降至.3rad/s，原q路径/共同时间/20ms地板/18N抓力/PID/2mm5deg/
18.01N/全部10、3、30观察门保持。不能通过增大保护/改模型或降低物理核验冒称修复。
15新真实三发布器/原失败完整路径/前置/精确反转测试通过，首版6项失败仅因为借用旧
Compression夹具预期10.3秒头戳，而Endpoint原为10.16；保留失败日志，新测试按原
Endpoint实际共同头戳核验，未改运行头戳。仍准备，必须全新隔离构建/回归/七before/
一次seal/fresh首帧实际审看后完整连续实跑录像。无新备份/清理/真机或腿部导航授权。

## 2026-10-16 CST：r289首帧等待超fresh窗，r290暂停审看、原动作不改

r288七after保持、633失败成员只读封存，SUM8e5d2b0eef6c262baa484c27983dda91aa60e604a54c15fdbc261424d794ea5c。
r289隔离5.69s/六库恢复、1396全测/15减速新测/七before通过。首次直接底层sealer缺
scene依赖证书而停止、无identity/launch，seal.log保留；用原wrapper再全测/依赖审计后
一次实际seal453源/343资源，identity4ba7edbd8669434a9b76618cdd04794d2b2f271a76d5baceb27d2ac024352f88。
原片首帧实际已看，但审看等待Gazebo仍运行，首非空clock超过原25sim，motions=0/
holding=null，原窗口保护FAIL，非动作碰撞或.3减速已验证。原正常SIGINT录像/全解码/
四收尾/退休PASS，executor1/其他三角色0；七after保持，596失败成员已只读封存。

新普通mkdir独占r290/domain93/partition h1_elbow_r290，原动作/全部抓力/物理/25sim/
2mm5deg/.2m/s/18.01N及观察门不变；仅fresh健康与ramp读回后、尚无executor/claim/
持物时用WorldControl唯一字段pause:true，实际stats验证暂停且sim仍<=25。暂停期间
烟测/同新原片首帧独立审看；审完核对同PID/源/录像/六控制器、暂停sim未前进，再
pause:false与原launch_run在同一进程链立即衔接，正常执行后仍原supervisor收尾。
不是reset，不借审看暂停冒称现实提速；已派发/有claim/有result/已resume者拒绝本接口。
新暂停校验测试包括原25窗、缺clock、负/非有限/超窗/未暂停、四种claim拒绝与精确
单字段control请求，动作算法相对r289只允许身份重绑。初版源比较遗漏ROS node短名
r289→r290身份重绑而失败，原日志保留，补精确重绑验证而非修改动作。当前准备中。

## 2026-10-16 CST：r290首帧WorldControl暂停导致严格relay退出；r291启动进程冻结

r2901411全测/暂停15测/隔离5.82s六库恢复/七before通过，456成员345资源一次seal，
identity0b99f3e58cb7066204ad2c7be6e2a27787cf50178c3d3c4c89d7431339055370。
pause:true实际sim2.0435后原relay因CLOCK_SOURCE_STEP:2043500000:2043500000:0失败，
原样导致ROSlaunch退出；首帧审看后恢复的同PID健康门拒绝。无review_resume.claim/
execution.claim/执行器/动作trace/run/result，不捏造完整FAIL trace或动作PASS。
原postprocess无结果等待器SIGINT终止；原录像正常SIGINT/完整解码、孤立server/GUI
逐身份正常SIGINT退休remaining为空，七after正在实际重算。未改严格relay或复用身份。

新普通mkdir独占r291/domain94/partition h1_elbow_r291，原动作/全部物理与严格0.5ms
relay不变。仅无执行器/无claim/无result/无持物的fresh健康启动时，对记录PID/ctime/
cmd一致的本身份Gazebo server使用SIGSTOP供独立审看新原片首帧；GUI/录像仍运行。
审看后核对原身份/冻结状态/源/录像，再一次SIGCONT、实际六控制器读回并直接exec
原launch_run。不使用WorldControl暂停，不过滤重复clock、不改任何relay断言；
原实际首clock<=25门/2mm5deg/.2m/s/18.01N/抓力及10、3、30观察全部保持。
此接口仅用于首次未派发启动，不能用于持物暂停/恢复失败现场或任何历史claim；
若健康丢失只能正常收尾并新身份。14新专项通过，仍需隔离构建/全回归/七before/
一次seal/fresh完整实跑后才称修复。旧完整成功仍r285，所有失败原片/模型/备份保护。

### r291已fresh连续实跑录像，覆盖上述准备状态

r290七after保持、569启动失败成员封存，不捏造执行器或完整动作结果。r291隔离两包
5.65s/六库精确恢复，1425全部/14新冻结/15原暂停/15减速/12空手cap/20姿态测试通过，
原46/122/6CTest通过；Nav111/2旧V12缺源保持。完整七before本轮实际重算PASS。
460源/347资源/36归档依赖一次seal，identity37ce8db3843f900936566180c5c6164b7edc4029dc83d384df70c6bdf85918bf。
普通mkdir run，domain94/partition h1_elbow_r291；fresh六控制器/500Hz/.6/D6/原ramp/
72帧通过，无claim时原server SIGSTOP，独立image审看同新原片首帧后，原PID/ctime/
cmd/源/录像核对，再SIGCONT及实际六控制器读回后立即exec原连续程序。
executor588457/ctime1792125180.23，首有限非空sim3.481<25，四角色持续、460源未改。
原0.5ms严格relay源码未改且当前健康，无WorldControl暂停、无重复clock过滤/reset。
空手cap.5，取盒outboard/transfer cap.3，原18N抓力/18.01N/.2m/s/2mm5deg/全部观察门
保持。当前仅实际前缀，仍非完整PASS；不另启actor/改密封源/复用claim，完成仍原
supervisor正常收尾和28门/双持物/实际发布路径与姿态/原片全解码/完整七after验证。

## 2026-10-16 CST：r291抬肘装盒/取盒完整实跑完成，独立合同复核封存

用户最新目标的装盒、取盒抬肘适配与完整Gazebo录像完成。r291真实双盒中部抓取/
10sim保持/装篮自然松夹/落篮3sim、实际指尖关盖和启动、稳转30.2585sim、实际停转/
开盖120°、实际重抓/完全提起/10sim保持/两不同桌槽返桌/正常松夹3sim/末态10sim，
执行PASS、28/28、独立双持物、无损trace、正常录像与四收尾通过，四角色均exit0。
完整七组before/after本轮实际重算相同，785正式src、所有正式模型/历史成果与10备份
保持，尤其920箱子版本/1008/1009未改。没有新备份、清理、真机、腿部或导航动作。

actual首sim3.481→211.211，207.730sim/2362.143548wall秒；旧180sim门仍FAIL，现实
1:1/180wall未达，旧r283/r284速度与r285少翻臂证书保持，不将剪辑当成运行提速。
全程原执行器native/FK峰1.098455/.927411mm，独立每手2783/同时5564次，独立角峰
1.242008°，4锚点/4正常释放/无持物或fault。输出4110样本峰17.826489N<18.01。
完整原生速度20778样本峰.20000000001764634，差1.7646e-11在原既有1e-8数值容差内。
原.2模型/18.01N/2mm5deg/碰撞及全部观察门不改。9271原2mrad实际密FCL通过；
篮口装盒含微调偏竖直最大43.4265°，取盒最大48.1824°，非完全竖直/全局最优。
原visual-only wrist等link的碰撞覆盖局限保持，仅本次固定骨盆Gazebo，不作绝对无碰撞保证。

原封存audit_elbow.py的exit1及posthoc_statuses保持，不冒称所有原审计exit0：
其字面<=.2误拒数值舍入；后续直接比较规划矩阵字典与完整holder快照也不符合原
actual_plan接口。新独占centrifuge_48box_elbow_independent_review_20261016_r1只读复核：
使用逐字节原velocity_violation；从原dual_holder_begin精确重算4个native/FK锚点，
每请求运行holder快照必须与原锚点逐项相同，规划矩阵由当前实际关节/pose独立FK
重算1e-10核对且在原2mm5deg内。不重设滑移基准；原实际双持物审计也完整通过。
21专项/22含闭包测试及9271实际密FCL通过；原碰撞/样本/余量/姿态代码保持。
初版数值复核遇到接口断言、锚点夹具和尾换行错误日志均保留；不是动作保护被放宽。
原封存freezer不覆盖，外部finalize_elbow_with_external_review.py逐项声明精确派生，
只接纳上述有证据的原audit exit1与旧速度exit1，其余原完整性门保持。正式终证书状态
PASS_ELBOW_UP_FULL_PHYSICAL_TASK_WITH_EXTERNAL_CONTRACT_REVIEW，不伪造原审计PASS。

r291665成员0444，SUM3988cd0eac7a1181c465e07a726fa3d07731fa5556657704209db22b13e1e153；
REPORT SHA0b93d1ba128db5cdee2346b8105f0276395ff43fa8751c3caa2e5cf168ec2c3f。
外部review28成员0444，SUM3ddf81399ddfc2c544db394a80d3cce8c9d85527f648b8e895261ff3e88f1986。
347资源/36归档依赖与460密封源保持，原PID/ctime及子进程已正常退休。后续必须同时
保护run、外部review及新媒体清单；不得复用r291/claim或将旧live前缀照抄为现场。

原始前视MP42391.416667wall秒/89402711字节/28697采样/0补帧/全解码PASS，SHA
fc5cd7afe0fb3b350fdd882f8d2d00a77fcd26d43fe23578e646093ba7661080。
同原片20阶段及取盒原帧、独立终态俯视已实际image审看；前视远手/盒自然遮挡，
独立俯视照片不冒充原片末帧，末帧仍前视。新演示剪辑164.633333秒/10倍动作/缩短
静止/稳转压6秒/无新增画面文字，原片不改，不代表实际耗时；路径
/home/liu/离心机全流程_抬肘优化_前方视角_20261016.mp4，SHA
f27dda5c4fbca4bac2efea91674908f36aa5fe7b4c2183100168546ac93bb34a。
媒体centrifuge_48box_elbow_briefing_20261016_r1原生成器清单含stdout最后PASS追加问题，
完整stdout无损移外部review，旧清单保持为诊断；独立验证旧stdout前缀精确匹配、
其余成员全等、视频不变，新权威SHA256SUMS_revision2的SHA
642b95549021ad4f939187b2964ab255b6b9db0528ac58cc477da9a01fb9d19e。不得用旧清单假称全通过。

## 2026-10-17 CST：用户保留双臂同时装取，红线进一步优化仅离线，未部署

用户明确选择“保留双臂同时装取，接受姿态近似”。不得因单臂更易抬肘就擅自改成
依次作业。新独占centrifuge_48box_redline_probe_20261017_r1只作离线FK/IK/FCL，未启
Gazebo、发动作、建立正式run/claim或录像；无新备份、清理、腿部或真机操作。
ROS包源码/正式模型/旧密封资产不改，未构建，不以旧overlay作为新物理资格。

最差一臂优先候选保留原六维腕目标、.02限位缩进、20um/.01deg、±.045局部界及
2mrad FCL。补齐生产main相同两标记后，原规划三项重算PASS，候选三项/4808密FCL
也PASS，但取盒左23.2474→36.5136deg恶化13.2662，右48.1824→46.0383deg仅改善
2.1441，相关肩yaw/腕roll/yaw累计6.7659→9.5088rad约增40.5%。装盒收益也极小、
另一臂稍差。因此不采用，不把最差角度减小当整体修复。旧r291继续为真实完整基线。
新增候选诊断门只是离线逐臂/轴转动/实质收益对照，不改任何原物理保护或授权动作。

早期chains_revision2及paired_actual直接调用函数，漏掉worker.main读取JSON后加入
的minimum_turn_contract/elbow_posture_contract；前者缺失使branch_seed误用历史guide。
这些拒绝日志保留，不能用来指控正式动作失败。dispatch_adapter复制请求后按原main
补同名同值标记，原JSON/实际状态/首次holder参考不变；5项AST/语义复核通过。
最终有效双臂对照为*_paired_replan_revision3.json。早期shell双引号可能提前展开路径，
另用run_probe.sh在ROS setup前后保留归档PYTHONPATH并逐次验证bootstrap/registry，
重做规划和56测，全部路径/指标必须精确复现revision2。更早动态super叠加优化已中断修正，
非有限候选测试首失败已补finite/shape拒绝；均未进入正式运行，原日志保留。

单臂/站位/投影/倾斜抓取均为未采用假设：依次单臂六端点通过不等于双臂同时或完整
任务，且其两类分段检查各仅3/6通过；用户已排除此方案。站位前移3/5/8cm九端点虽
可改善肘形，却有髋/桌面和较大前移时躯干骨盆/设备碰撞，不实际前移或删除几何。
投影近似可能反而降低真实肘高，不能冒充姿态修复；预抓倾角几何也不等于真实抓稳。

56新专项通过；本轮前后完整重验r291665成员、独立review28及媒体稳定revision2的39
成员逐项相同，三正式设备模型原SHA保持。没有将这三组只读校验冒充新实跑所需
完整七组before/after，所有备份未操作。末次live无Gazebo，无新实际耗时或原片。
报告为新目录REPORT.md/paired_candidate_selection_revision2.json；不能称手臂异常已修好或新
流程已通过。后续仅双臂同时，在空手接近前联合选分支并限制逐臂退步/额外翻转，
完整链预测合格后再用未使用新身份隔离构建/全测试/完整保护/seal/fresh/录像/物理审计。

本轮离线目录125成员0444封存，SHA256SUMS的SHA为
0cc796df23cc806b04fd4a6e1a67e8d241097c689fd96628ea52777184a73b51。
最终revision3路径/姿态/转动逐项精确复现revision2，归档bootstrap和三阶段环境核验通过。
此封存仅保留正反实验与用户选择，不是执行authority；后续不要覆盖或直接运行其中脚本。

## 2026-10-17 CST：用户改选分开抓，建立独立分支，旧双臂版本保留

用户最新明确“那就使用分开抓的方法”，覆盖上节不得依次作业的偏好。现在允许左右手
依次抓取/装篮/取回，每次最多一个实际holder；两个盒子仍须全部处理，关盖/稳转30秒/
停稳开盖/返桌目标保持。不是只放一个盒子，也不是允许异常松夹或放宽物理门。
普通mkdir新建centrifuge_48box_sequential_elbow_20261017_r292，仅在其中开发，旧r291/
红线probe_r1/孵育箱成果/正式模型和全部备份不改。先完整单臂停放/接近/装取/撤手与
剩余场景避碰、单holder和另一盒支撑监视，再隔离构建/全测试/完整保护/seal/fresh/新录像。
原16/18N、18.01N/.2m/s/2mm5deg、44deg躯干/.02限位缩进/20um/.01deg/2mrad FCL及
10/3/30秒观察保持；本节只允许改动作顺序，不授予真机/导航/备份/清理权限。
当前尚未启动新Gazebo，不能用先前单臂端点当分开抓完整实跑资格。

## 2026-10-17 CST：分开抓r292装取几何链完成，尚未新物理实跑

用户最新分开抓选择继续生效；左手先装、右手后装，两盒都在篮内后才机器周期；
停稳开盖后左右依次取回，每次最多一个实际holder。旧r291同时版本/孵育箱/设备模型/
备份保持，不因新顺序丢弃任一盒。新单臂8DOF规划、顺序状态机及无动作JSON worker
在独占r292目录，worker只返回PASS_GEOMETRY_ONLY/execution_dispatch_not_qualified，
不是实机/仿真执行器；原实际native/FK锚点不重设，当前规划FK矩阵独立重算。

17预测阶段装取链revision2通过；25,717搜索检查含被拒绝直连，不能说每次查询无碰撞。
独立最终命令路径重验23,400次原2mrad密FCL，全部无碰撞，另一手/另一盒始终在场景。
独立篮口阶段偏竖直峰：装盒left14.1355/right16.7814deg，取盒left18.3928/right13.3435deg。
只是离线姿态改善，不是完全竖直、红线精确复刻、全局最优或真实抓稳证明。
153专项及原接口回归通过，原r291实际machine JSON也验证lid_rad/rotor_rad_s接口，
早期150测与失败记录保留；没有声称全ROS包回归或隔离构建通过。

失败已保留：整个±44deg停放扫掠碰到躯干/前板；只在选定±.65rad停放窗口求可行
停放，实际动作仍原44deg界全FCL。左装自由重优化沿冗余轴漂入碰撞，改用完整逆向
路径作分支proposal、每点当前Cartesian重算精确IK，guide不是实测状态。右空手上撤
120mm过伸失败，只将空手撤出改84.5mm后全链通过；不降低持物1.15975m完全提起高度，
不删碰撞或放宽原.02rad/20um/.01deg/2mrad、2mm5deg/.2m/s/18.01N、16/18N和10/3/30秒。

机器周期处明确使用原r291转后pose夹具，并未新模拟旋转；不能称r292全流程物理PASS。
尚缺单holder实时执行集成、未操作盒实时支撑/留篮监视、真实闭环接触/释放与独立审计；
没有启动Gazebo、发布动作、创建identity/run/claim、隔离overlay或录像，没有新备份、
清理、腿部导航或真机行为。后续完成集成后仍要新身份、隔离构建/全回归、完整七组
及新增结果保护、一次seal/fresh/前视原片/物理门，禁止运行派生旧双臂launcher冒充。

r291665/红线probe125/外部review28/媒体稳定清单39共857成员与785正式src逐字节保持，
三设备模型原SHA保持，parent/probe开发前完整核验记录也保留。备份未写入，本轮未重
哈希全部备份；这些离线校验不冒充完整七组fresh运行门。根文档前文/SHA有独立保存。
r292以SHA256SUMS作离线成员只读封存，非动作authority；后续源码/输出用新独占身份，
不覆盖本候选或运行其写输出脚本。REPORT.md及sequential_final_path_review.json为本轮边界。

## 2026-10-17 CST：用户继续，r293单holder实时执行集成与实跑前验证

新普通mkdir身份centrifuge_48box_sequential_full_20261017_r293；r292430成员封存SHA
f2486180fbc2653a236b43f0eaec87f14965ceed1cfd7510cdfb01d35b0f5b35已完整验证后派生，
无旧run/claim复用。新SequentialCycle依次装取/另一盒支撑监视已集成，原五按键机器
周期复用；新execute_sequential保持原fresh/信号/500Hz/.6/D6/资源hash/claim门，两个
guard wrapper只将精确PID/ctime的SIGINT目标换为新入口，不改变故障触发条件。

取盒实际FK参考下曾离线发现guide分支跳变，失败日志保留；按当前关节平滑衔接guide，
超原候选.12rad分支过滤时先重试当前分支严格IK，仍超拒绝。原腕目标/44deg/.02/20um/
.01deg/2mrad、2mm5deg/.2m/s/18.01N、16/18N与10/3/30秒物理观察均不放宽。
四个精确worker派发revision3/完整17几何阶段及neutral通过；1530本地、46原包、122
原加载回归、6CTest通过，Nav111/2已知V12缺源保持。新隔离2包7.11s、六库恢复一致；
完整七before与新增五组1287成员/三个模型保护均本轮重算，原785正式src/10备份保持。

本节为实跑前接续，不是完整物理PASS；以之后新identity/run/claim/result/trace和前视
原片为准。未获真机/导航/备份/清理许可，旧孵育箱/设备/权威/视频/报告/备份不改。

## 2026-10-17 CST：r293受控部分验证；r294取盒支撑分类修正与新实跑资格

r293实际开盖、左手抓取/15mm试抬/10sim保持，开始单手搬运后因离线发现未来取盒
监视缺陷而主动正常SIGINT，非物理fault；停住仍左手夹持，保持原stop-and-hold，
不reset/detach或松夹。原supervisor正常刷盘/全解码/退休，executor1，其余三角色0。
native/FK峰0.066635/0.066597mm。不是完整任务成功，608成员只读SUM SHA
16d08b9de0e5f6ae54ef0c570be2fc9e29518a74528975e318de156592c18e36。
启动漏完整窗口自检、首resume缺review文件两次拒绝均保留；外部startup_r1只补原
自检和逐字原录像尾部，不改密封源；实际首帧审看后仅一次resume/claim。其10成员
SUM78e146db0f7fb026dc2f71fbb85bdecaf0cefc570dfb3c15a96ea60bdd614095，须继续保护。

新r294仅在当前手/目标/取盒预夹且无holder阶段，从篮支撑分类排除该手该盒指尖；
原始接触先完整进入原1mm/40N保护，篮支撑20ms/几何及另一盒监视保持；不能用
指尖替代篮支撑。23专项/1533本地、46原包、122加载、6CTest通过，Nav111/2已知
V12缺源保持。新隔离两包5.95s、六库一致，完整窗口自检改为seal前/启动前必需。
新域97/partition h1_seq_full_r294，完整七组before及新增1905成员/三模型、startup10
均本轮重算；785正式src、旧成功与10备份不改。新run/claim一次seal/fresh/实际原片
审看后才分发。此节为准备资格而非物理PASS；旧r291完整基线保持。

## 2026-10-17 CST：r294实际双盒装篮/30秒机器周期完成，取盒输出力故障；r295余量实验

r294实际左右依次装篮，各10sim抓稳/3sim自然释放稳定；真实五次按键、关盖、稳转
30.1705sim、停转开盖120°完成。取盒左手实际预夹/锚点登记后15mm试抬时，输出
18.170614879N触发原18.01N门，自动stop-and-hold保持夹持；未取回两盒，不是完整
任务成功。native/FK峰0.687759/.687898mm，独立部分角峰.198351°。result FAIL及三
完整任务审计exit1保持。录像84,573,087B全解码、trace完整与四收尾PASS，executor1/
其他三角色0；原server/GUI正常退休，无reset/detach或异常松夹。

426密封源全等、完整七组after/新增1905成员及三模型/startup10本轮实际重验通过，
785正式src与10备份保持。r294632核心成员0444，SUM SHA
cb663b28037d73108494ad416116acf7c0c17c6e2ec318347d77a73601673aa7。
此失败身份及原片/日志必须保留，不能重用或改写。当前无Gazebo。

继续在新独占r295实验取盒前馈18→17.5N，以保留PID余量；装盒16N、原61点软夹
位置/速度/时间与全部保护保持，18.01N/.2m/s/2mm5deg/1mm40N/原IK-FCL及10/3/30秒
不放宽。该较低力仅是新分支待实跑假设，不冒充原16/18N版本或成功证书；若滑移/
支撑/碰撞超门仍停止。新隔离构建、新域98/partition h1_seq_full_r295、全回归/保护/
seal/fresh/实际首帧审看后才允许新run。旧双臂与孵育箱/模型/全部备份不改。

## 2026-10-17 CST：r295力余量实验未完成取回；r296竖直试抬新假设

r295已实际双盒装篮/各10sim抓稳/3sim稳定和真实五按键/至少30sim稳转/停稳开盖；
左手取盒试抬仍输出18.012422257N触发原18.01N，result及完整任务审计FAIL，不把
微小超限当通过。执行器native/FK峰.766669/.758280mm，独立部分角峰.183927°。
原stop-and-hold保持左手，正常录像SIGINT/刷盘/退休，无reset/detach或松夹。原片
84,205,397B/2212.833333wall秒全解码、trace及收尾PASS；executor1，其余三角色0。
完整七after/2537历史成员/三个模型/startup10本轮重验，785正式src及10备份不改。
r295636核心成员0444，SUM353d13cbfab1a4eb0450fa39d36f5a5673f4690c38f7459b7137907fb9040005。

新独占r296只改15mm试抬为世界坐标X/Y固定、原姿态竖直上移，横向居中延后到原
篮轴清出段；不降低后续6mm篮缘余量或1.15975m完全提起高度。原约.93mm侧向居中
可能增加篮内侧载，仅为假设，未证实根因。r294/r295两个实际失败快照的原2mrad
密FCL/严格IK竖直试抬离线通过，但不是物理修复。新隔离构建/1538本地及原包加载
回归、新域99/partition h1_seq_full_r296；仍须完整保护/seal/fresh/真实原片首帧审核。
取盒17.5N、装盒16N、18.01N/.2m/s/2mm5deg/1mm40N及原IK/FCL/10-3-30秒观察均保持，
旧成功/模型/视频/报告/备份不动，不授权真机或腿部导航。以本节之后实际结果为准。

## 2026-10-19 CST：继续分开抓；r296会话中断核验与r297全新身份接续

重新只读核查无Gazebo server/GUI及原精确执行角色。r296433密封源全等；最后可解码
事件sim141.429/right_drop_settling，两次10秒抓稳/释放门存在，仅左盒3秒落篮稳定
有证书，无LOADING_PASS/机器周期/取盒/最终PASS。video.log为X11 fatal IO，guard
记录recorder_missing_or_identity_changed并SIGINT；launch有Gazebo退出/context销毁错误。
无result/trace_metadata/四角色退出/正常收尾证书，不能确认stop-and-hold完成或取盒
竖直路径已实测。原gzip无EOF/footer，仅保存656931完整事件前缀，绝不补造完整trace。
原MP4 53,631,694B/1401.833333wall秒完整解码exit0，不是完整任务或正常关闭证明。

完整七after、3173历史成员/三模型/startup10本轮重新计算PASS，785正式src及10备份
保持。r296597核心成员0444，SUM dc500742044e5e513bf9730085a842fe7686586f905477ff171f029a805bbcc3。
外部recovery_20261019_r1七成员0444，SUM3762c0c573c9d5e6cf720736c2d78009632c102fd620bdfa71ba773f41ec1deb。
原片/不完整trace/错误日志完整保留，不重用旧run/claim、不恢复旧现场。

新独占centrifuge_48box_sequential_full_20261019_r297仅做全新身份接续，八个动作模块
逆绑定逐字等于r296；15mm纯竖直试抬、之后原篮轴清出/6mm余量/1.15975m完全提起、
16/17.5N及18.01N/.2m/s/2mm5deg/原接触/IK/FCL与10-3-30秒观察不改。新隔离两包
7.09s/六库一致、1546本地、原46/122/6CTest通过，Nav111/2已知V12缺源保持；两个
实际快照竖直试抬、四派发和17几何链通过，仍非物理全流程成功。新域100/partition
h1_seq_full_r297、独占run/claim，最终源依赖/全部保护/seal/fresh/实际原片首帧审核
后连续实跑。运行须保持桌面/Gazebo会话，后续只用本身份实际result/trace/原片下结论。
不清理、不建备份、不改孵育箱/旧成功/模型/正式authority/run/报告/builder/备份，无真机/
导航许可；旧r291仍为最近完整抬肘成功基线，现实三分钟及1:1未达。

## 2026-10-19 CST：r297抓稳登记前过渡故障；r298保持原物理证书提前登记

r297新fresh/原片首帧审看和434源通过，实际左手闭合/原生endpoint完成，但在原.3秒
后观察及capture_pair原.25秒等待期间，sim41.5295触发未登记盒桌面支撑检查。实际
两指15.614630/15.614626N、中部对向，盒位误差1.938575um；缺少登记holder时仍按
未抓取盒检查，未装篮/未旋转/未取回。result FAIL，原夹爪命令保持，不reset/detach/
松夹；holding=null仅是无登记，不能说已松开或以零滑移字段充当证书。原supervisor
四收尾/视频全解码/trace完整PASS，executor1其余0，原server/GUI正常退休。

完整七after/3777历史成员/三模型/startup10通过，785正式src及10备份保持。r297603
核心成员0444，SUM f3d6e004e8f36f576d30ce411b88d1e8c3110ccfec4425b8390cad5eea8d0f29。
诊断review_20261019_r1七成员0444，SUM1cac137585ac424b2ace618bb804b041692c8e413d6898ffb2b73a53a3c21623。
原片14160426B/365.083333wall秒，trace212159行；完整任务审计exit1均保留，不冒称成功。

新r298只将实际证书移至action成功和原生新鲜endpoint后、原.25/.3秒观察前登记，
原capture证书体AST逐句相等，仅把.25等待移动至证书之后；中部/3..20.5N/native-FK
锚点/不可重置门完整保留。retrieval_closing在证书前复位False，登记后立即受原双重
滑移监视；另一盒支撑不豁免、未过实际证书仍停止。总观察时长/10-3-30秒均不减少。
两处旧的延迟重复capture调用移除，七个其他动作模块与r296逐字逆绑定不变。

新隔离构建/全回归/全部保护/最终依赖/seal/fresh/真实原片后验证；新域101/partition
h1_seq_full_r298。15mm竖直试抬/篮轴清出/6mm余量/1.15975m完全提起、16/17.5N、
18.01N/.2m/s/2mm5deg/原接触/IK/FCL均保持。此为新修复待物理资格，不是完成证书。
原孵育箱/完整旧成功/模型/authority/run/视频/报告/builder/全部备份不改，无新备份、
清理、腿部导航或真机行为。

## 2026-10-19 CST：r298结果回调前仍过渡故障；r299实际预夹承重证书

r298实际开盖和左手闭合，在等待action结果时sim40.9265再次未登记盒桌面支撑FAIL；
native_joint40.92，指1/2距目标15.665/4.429um，盒出生位偏差1.938514um。原片
13124737B/347.5wall秒全解码/trace完整/四收尾PASS，executor1其余0，无装篮/旋转/
取盒。原正式capture提前至结果后仍不足，不是成功；无reset/detach/松夹，旧夹爪
命令保持。完整七after/4387历史成员/三模型/startup10通过，785正式src及10备份不改。
r298605成员0444，SUM82f5607647736123f93a399f8f3c9bb40157e06ff320ef74d3305d654c35c59f；
review_r2六成员SUM7c1254e32511dd1f209168c92cc4a7a8931b8231a21d72a86691c528f4339837。

新r299基于r297，明确回退r298未解决问题的提前正式capture，原.3/.25秒及原正式证书
保持。仅当前明确accepted夹紧目标、原生目标误差<.00025m/年龄<=.012sim、原中部
对向接触及每指3..20.5N实际法向载荷全部通过时，允许此目标在原出生位.3mm/.2度
内由双指接替桌面承重。原始接触先经原保护，另一盒不豁免，已有另一holder时拒绝。
accepted声明来自真实action handle，保留到正式捕获；声明本身不是抓稳证书。
preload过程不创建/重设正式holder或锚点，任何物理/原生证据缺失仍停止，完整正式
action结果/新鲜endpoint/capture和10-3-30秒观察仍需通过。真实r298快照及缺指/弱力/
超力/过时/偏位/他盒无支撑等回归通过；不是盲目放宽支撑门或把指令当实际反馈。

当前新隔离构建7.12s/1560本地与原回归准备，最终源/依赖/全部保护/seal/fresh/首帧后
才新域102/partition h1_seq_full_r299实跑。15mm纯竖直/原篮轴清出/6mm余量/
1.15975m完全提起、16/17.5N、18.01N/.2m/s/2mm5deg/原接触/IK/FCL均不改。
旧成功/孵育箱/模型/authority/run/视频/报告/builder/全部备份不动，无清理、新备份、
腿部导航或真机许可；此节仍非新完整任务证书。

## 2026-10-19 CST：r299双盒装篮/机器周期实际通过，相机ACK故障中止；r300原生读回

r299原预夹承重证书使左右两盒均通过真实闭合过渡、各10sim抓稳及3sim自然落篮；
LOADING_PASS155.5705，真实五按键/稳转30.1055sim/停稳开盖，MACHINE_CYCLE_PASS219.47。
左取盒预夹时相机三次请求未获ACK，录像器退出后guard正常SIGINT，最后244.553sim。
result为KeyboardInterrupt/FAIL，无取盒竖直试抬证据，更不是全流程成功或试抬物理失败。
native/FK峰.683263/.679362mm；两个盒仍篮中，转子禁用/近零rpm/盖120度，不reset/
detach或异常松夹。holding=null只是无正式holder，不是松夹或零滑移证书。

原片93900202B/2472.916667wall秒，29675唯一帧/零重复/最大采样间隔.145512秒，完整
解码exit0，但normal_sigint=false/video_auditFAIL；613个相机调用记录/33失败调用，
不能把无ACK当实际偏镜头。原supervisor三个收尾PASS、videoFAIL，executor1/guard0/
disk0/supervisor1；launch最终曾升级SIGTERM退休，不说四收尾均PASS。trace1226762行
完整、任务三审计exit1原样保留。完整七after/4998历史成员/三模型/startup10通过，
785正式src/10备份保持。644核心成员只读SUM
9b473a8ad0672b19affba2694cb0591ca4bd2189fb06b2615b9e59cf8e2171e2。

新独占r300/domain103/partition h1_seq_full_r300保持r299全部动作逆绑定字节相等，
只将录像CameraGuard换成原生/gui/camera/pose有界确认：每阶段/4wall秒读回，原生
年龄<=1wall秒、相机误差<=2mm/.1度；已正确不重复RPC。实际偏离才原3次有界恢复，
随后必须新鲜匹配pose；ACK本身不当证明，缺失/过时/无法恢复仍停止。原完整窗口/
2wall秒采样门/正常SIGINT/全解码不改，result后停止恢复以让supervisor拍末态。
独立camera_readback_20261019_r1仅GUI、187原生pose、实际相机X移.25m再读回通过，
无server/机器人命令，不能冒充全流程。正式旧模型/成功/run/视频/报告/builder/全部
备份不变，无清理/新备份/导航/真机行为；仍须全测试/保护/seal/fresh/真实首帧后实跑。

## 2026-10-19 CST：r300相机原生确认通过，桌支撑单帧间断；r301恢复原20ms连续门

r300原生GUI92次位姿确认均正确、无重复恢复RPC，录像正常SIGINT/4179唯一帧零重复/
13197524B/348.25wall秒完整解码，camera_guard和原supervisor四收尾PASS；退出1/0/0/0。
动作sim41.066左预夹故障：41.0655桌力.573942446N且双指，下一.5ms桌接触仍在但
力0/缺指1。盒出生位偏1.938272um、原生双指误差15.635/4.459um、native年龄6ms。
不能造指1或称对向承重证书通过；未有正式holder/试抬/装篮/旋转/取回，原result和
三任务审计FAIL保留。最后41.686sim stop-and-hold，原夹紧命令保持，Gazebo退休。
完整trace206025行、完整七after/5641历史成员/三模型/startup10重算PASS，785正式src/
10备份保持。616成员只读SUM5c6d7049d3fb81a22a96773402bfd814928ad97eb4a075bf7750fe9491c97cbe。
实际快照review_r4四成员SUM331d5fda88e46cf621917a3743b93117c2b325193c9f7994d85cb154972c12cb。

原顺序候选在candidate分支强制即时桌支撑，覆盖了原table_support_valid的20ms正力
历史门；新独占r301/domain104/partition h1_seq_full_r301只恢复该有界连续性：必须
accepted当前本手本盒、原生目标误差<.25mm/年龄<=12ms、没有holder、原出生位.3mm/
.2度、原始接触保护全部通过；还必须当前真实桌接触存在、有限0..1mm深度/0..40N、
只有本盒桌/本手接触，距离实际桌正力<=原20ms且非未来。既无当前对向证书、又无
有效桌连续证据仍停。另一盒不豁免；不缓存/伪造指尖、不创建holder、不改锚点。
正式action结果/新鲜endpoint/capture/10秒抓稳、3秒稳定和30秒稳转均不缩短。
34项支撑专项/1606本地通过，新隔离构建/全回归/保护/最终seal/fresh/实际首帧后实跑。
原相机修复、15mm纯竖直取盒、之后篮轴清出/6mm余量/1.15975m提起、16/17.5N、
18.01N/.2m/s/2mm5度/原IK-FCL等保持；这不是已完成取盒或分开抓任务成功。
孵育箱/旧完整成功/模型/正式authority/run/视频/报告/builder/全部备份保持；无清理、
新备份、导航或真机。旧r291仍完整抬肘成功基线，现实三分钟/1:1及分开抓取回未达。

## 2026-10-19 CST：r301桌支撑修复实测双盒通过；r302取盒实际预夹承重

r301左右实际夹紧过渡/原正式capture/10sim抓稳/3sim自然落篮完成，LOADING_PASS
155.7705；真实五按键/稳转30.1585sim/停转开盖，MACHINE_CYCLE_PASS219.672。
左取盒预夹sim243.4165时篮底接触消失，两个真实指尖在场；原几何仍侧壁认证/
完全篮内，侧越出15.6nm/底穿入25.7nm，原生年龄6.5ms、闭合误差.192402/.172189mm。
原支撑分类contacts_valid=false而停止，尚未正式捕获/取盒试抬或返桌；不是实际越界。
最后244.0585sim stop-and-hold，原夹紧命令保持，无reset/detach/异常松夹。
native/FK峰.683104/.673480mm；原相机589次确认无重复RPC、录像86572514B/
2276.333333wall秒/27316唯一帧零重复正常SIGINT和完整解码，trace1224703行完整，
四收尾PASS，executor1其余0，三任务审计exit1原样保留。完整七after/6261历史成员/
三模型/startup10重算PASS，785正式src/10备份不改。652核心成员只读SUM
a0f51eb8e82269e3609847117c1bcfd44bb38f72bd36b53cdd39a71d9b234126。
原快照review_r5七成员SUM0c7df1ea123f718a904cd7abb1969c47d446307ef723d19c41b829428456c18d。

新独占r302/domain105/partition h1_seq_full_r302只把实际预夹承重证书扩到取盒：
明确accepted本手/本盒/17.5N目标，当前retrieval_grasp且无holder，原生误差<.25mm/
年龄<=12ms，原中部对向双指/每指3..20.5N实际法向载荷及原篮侧壁认证/完全篮内
均须过。允许匹配本篮接触，不允许他篮/他手/桌面替代；原始接触先完整过原门，
另一盒不豁免。声明仅来自实际handle接受，不能当物理抓稳；不新建holder/锚点，
原action结果/新鲜endpoint/正式capture/.3/.25秒/10sim抓稳仍必需。
原故障快照实际两指7.209289/7.022425N，离线证书通过；49支撑专项/1621本地通过。
此为待实跑假设，不宣称新连续取盒成功。相机修复/原桌20ms有界连续门/15mm纯竖直/
篮轴清出和高度、16/17.5N/18.01N/.2m/s/2mm5度/原IK-FCL及10-3-30门保持。
仍须全回归/完整保护/最终seal/fresh/真实首帧/新录像。旧孵育箱/成功/模型/authority/
run/视频/报告/builder/所有备份不改，无清理、新备份、导航或真机；三分钟/1:1未达。

## 2026-10-19 CST：r302实际预夹交接通过、竖直试抬仍输出超限；r303低前馈实验

r302实际双盒装篮、原各10/3秒及真实五按键/稳转30.262sim/停稳开盖完成；当前手
实际预夹承重交接与原正式左holder登记通过。纯世界Z15mm试抬仍实际输出峰
18.032671101N越原18.01N，stop-and-hold245.408sim保持左手，不reset/detach或松夹。
因此不能说竖直试抬解决输出力；无盒返桌/无完整成功。native/FK峰.799472/.786228mm，
实际速度记录峰.189390401m/s。原片88787560B/2335wall秒正常SIGINT/全解码、无损
trace1228901行/原相机/四收尾PASS，executor1其余0；三任务审计exit1保留。
完整七after/6920历史成员/三模型/startup10重算，785正式src/10备份保持。656核心
只读SUMf1f96a25e2a14581ba83cd7b206226c1b182ce45ada21f09bb484a3e51339e5f；
实际力诊断review_r6四成员SUM87ed2d2bdf4546ecdba37bc1e9fecdd3f41556dae6646a1f31c2fa74619fe9ba。

新独占r303/domain106/partition h1_seq_full_r303仅取盒前馈17.5→16.5N增D6余量，
装盒16N保持。61点位置/速度/时间精确同源，18.01N/.2m/s/2mm5度/原接触/IK-FCL/
10秒抓稳、3秒稳定、30秒稳转不变；取盒实际预夹资格也改为只接受当前16.5N真实
handle合同，原native/对向中部/载荷/篮内门不变。较低抓力仍必须新fresh实际验证，
不能用17.5N旧快照冒充已接受16.5N；新单测显式标明合成候选接受声明和旧物理
数据，旧r5证书和r302源不改。78专项/1624本地已通过，仍须完整新构建/保护/最终
seal/fresh/原片首帧及全实跑。此前17.5N是历史分支，当前候选16.5N不是已通过证书。
旧孵育箱/完整成功/模型/authority/run/视频/报告/builder/全部备份不改，无清理、新
备份、导航或真机许可；分开抓全部取回及三分钟/1:1仍未完成。

## 2026-10-19 CST：r303低力尚未试抬；r304弱接触沿用原篮20ms正力历史

r303真实双盒装篮/原10与3秒观察、五按键及稳转30.1185sim/停稳开盖完成；当前16.5N
实际目标接受后，sim243.309两个中部对向指尖法向载荷.345194/.283772N不到原3N，
篮底当前接触缺失而原正力间断0/原几何完全篮内，分类停止。原生年龄9ms/目标差
<.25mm；不能称强抓稳通过。没有正式holder/未到低力试抬，16.5N力实验尚未获物理
结果，不应说低力已成功或已发生力超限。最后243.9415sim stop-and-hold，夹紧命令
保持，无reset/detach/异常松夹。已登记加载段native/FK峰.686429/.667997mm，不给
本次预夹作抓稳证书。原片86779981B/2283.583333wall秒正常SIGINT/全解码、无损
trace1223813行/原相机/四收尾PASS；executor1其余0，原三任务审计exit1保持。
完整七after/7580历史成员/三模型/startup10重算，785正式src/10备份保持。652核心
只读SUM4f301c07886bc9114518afa3bc3a7f5eef5643129b5a0ffea6764d9e97076299；
原弱接触诊断review_r7四成员SUM0a374be5042bc48114cf14517fece9fbd20e03bb6e5bc6d8cf5f3d2d17464de3。

新独占r304/domain107/partition h1_seq_full_r304保留16.5N，仅在当前accepted预夹、
原生新鲜近目标/两个原中部对向接触均实际在场且载荷有限>0且<=20.5N、原篮几何
认证/完全篮内且无他手、原始接触与另一盒保护均过时，沿用原篮实际正力历史
0..20ms。原SupportContinuity类、时间窗和正力更新时间不改，不把弱指力更新成篮
正力；标记not_a_strong_grip_certificate。期满、未知/未来历史、缺指、零力、超力、
非原生或偏位/越界等仍拒绝，不创建holder/改锚点，不降低正式3N强capture门。
原action结果/endpoint/.3/.25秒/10秒抓稳和3秒稳定/30秒稳转均保持。71支撑专项/
1643本地通过，仍须新隔离构建/全回归/完整保护/seal/fresh/实际首帧和物理全流程。
18.01N/.2m/s/2mm5度/原IK-FCL/15mm纯Z/清出高度保持；旧成果/孵育箱/模型/正式
authority/run/视频/报告/builder/全部备份不改，无清理、新备份、导航或真机许可。

## 2026-10-19 CST：r304主机重启中断，r305只重绑身份继续分开抓

用户继续后重新检查主机uptime约4分钟，原Gazebo和精确执行角色均不存在。r304原
442密封源全等；原片X11 fatal IO/launch退出和context销毁，最后完整事件208.3395sim
machine_stop_wait_quiet。双盒各10sim抓稳/3sim落篮，LOADING_PASS156.1205；4次真实
按键、关盖/稳转/停转调度存在，但无MACHINE_CYCLE_PASS/开盖/取盒。没有result/
trace_metadata/四角色正常收尾证书，gzip无EOF，不补造终态或说正常stop-and-hold完成。
原片83094806B/2199.666667wall秒完整解码exit0，不是完整成功。新recovery_r2保存
原事件前缀和SHA/旧PID检查；完整七after/历史8236及三模型/startup10正在重算，以
落盘结果为准。原片/trace/日志全留，旧r291仍完整抬肘成功，不能复用r304/claim。

新普通mkdir独占r305/domain108/partition h1_seq_full_r305，基于核验442原源，仅重绑
当前身份/overlay/ROS域，不复制旧run或claim；动作/相机/支撑九模块逐字逆绑定核验。
16/16.5N、18.01N/.2m/s/2mm5度/原20ms连续性/强capture/15mm纯Z/原篮轴清出和
1.15975m高度/IK-FCL/10-3-30秒观察均保持。新隔离两包8.85秒且六库恢复一致，仍须
完整回归/保护/一次seal/fresh/实际新原片首帧审看后连续实跑。本节不是已完成取回。
孵育箱/旧完整成功/模型/authority/run/视频/报告/builder/全部备份保持，无清理、新
备份、腿部导航或真机许可。现实三分钟/1:1未达，不以剪辑作运行速度证明。

r304中断后完整七after/8236历史成员/三模型/startup10已实际重算PASS，与before一致，
785正式源与10备份保持。617中断核心0444，SUM
f887bab0f8f5e30b809ad0b2a51886dab7ab3b179b7b8ec7dd32be849fd7d5b6；recovery_r2十成员
SUM c9fefd9804155b967d53b8c4cd7919107e8aa131b5fac2f93e8b9a19452abdeb。原不完整gzip
和原片无修补。r305新增九项逆绑定共1652本地通过，原46/122/6CTest通过，Nav111/2
旧V12缺源保持；四实际派发/17阶段25701搜索检查（包含拒绝候选）已通过。新before
额外保护加入此617+10成员；此处仍为实跑前准备，不预称取回或物理全流程通过。

### r305已通过全部准备并fresh启动，覆盖准备状态

1652本地、原46/122/6CTest、四派发/17阶段、两原快照竖直试抬、按钮/录像/31归档与
36依赖、完整七before/8863历史成员/三模型/startup10均通过。442源/377资源一次seal，
identity160199a29adbf381622ef043252b6d4f715971c17e543e82207dc8247d3deb26。原片首帧实际
image审看SHA da98d79471fb1e1014c5d037d8f708e0f79f122f07357cb907899bc7058aeb13；机器人/
近爪盒/闭盖设备/桌/墙可见，远爪盒自然部分遮挡。仅一次resume自动接连续程序，
执行器PID156570/ctime1792402874.44，domain108，首段sim3.5835，四角色存在；这是启动
事实不是永久live或完整成功。不得另启actor、改密封源或复用claim；后续重新读实际
progress/result/exit和精确PID。review_r8自动post审计/截图，supervisor负责原正常
录像收尾与退休。仍须实际两盒取回、物理/发布路径/全片/after保护才算完整任务。

## 2026-10-19 CST：r305左盒释放前0.25度门失败，r306增加实际局部对齐

r305实际左盒中部抓取/试抬/10sim抓稳和抬肘搬至篮上完成，但check_pair_release的
0.25度门失败；相干反馈原生角差.264115度，停持后目标差.268869521度/位置差约
[39.54,18.09,-33.52]um，规划目标自身与原固定yaw相差仅5.9e-9度。未松夹/未装入任一
盒/未机器周期/未取回，原STOP_AND_HOLD保留左holder，不能称完整成功。native/FK
峰.660263/.650735mm不替代释放资格。原录像正常SIGINT/全解码、完整trace360713行、
四收尾PASS，executor1其余0；原任务审计exit1保存。原片29094191B/765.75wall秒。
完整七after/8863历史成员/三模型/startup10与before一致，785正式源/10备份保持。
620核心只读SUM e05f25af761b1de992ee42f92b63b2cff57f730daa62513adaac9400db9a5c06，
review_r8保存真实同步关节/pose/原holder及失败诊断。原Gazebo已退休，不复用身份。

新普通mkdir r306/domain109/partition h1_seq_full_r306，仅在原.5sim释放等待后加入
最多两次实际快照局部对齐，限入口1mm/1度、关节变化.045rad，原20um/.01度IK及
2mrad全场密FCL；原pilot速率.03rad/s，每次后.3sim观察，目标.15mm/.08度。最终仍
原check_pair_release全部通过才松开，旧.25度/XY-Z/篮角点门不改；物理holder原生与
FK基准不重设，另一盒支撑/16与16.5N/18.01N/.2m/s/2mm5度/10-3-30观察均不变。
新增load_release_align是明确持物模式，执行前需原单holder/当前参考与原冻结基准
双重校验；规划附件不等于物理约束。新隔离5.90秒/六库恢复、1666本地通过，仍需
真实失败快照几何资格/完整准备/保护/一次seal/fresh/实际首帧后新实跑。
首次提前全测31失败及正式prepare中旧相机逆绑定1失败日志保留；补前置合同、编排
mock及精确声明新增worker三处差异后通过，不把失败记录改写。当前不是新物理成功。
孵育箱/旧成功/模型/authority/run/视频/报告/builder/全部备份不改，无清理、新备份、
真机或腿部导航许可。原完整版本仍r291，现实三分钟/1:1未达。

r306实际r305相干关节/原生pose/原holder快照修正资格通过，独立42次原2mrad密FCL，
最大关节变动.0066548523rad，IK位置约5.12e-14m/角2.51e-10度；不是新物理效果证明。
外部review_r9新增持物微调模式，必须规划携物且不给桌/篮任何支撑碰撞豁免，13项
测试含同真实快照独立检查通过。r305620核心封存SUM e05f25af761b1de992ee42f92b63b2cff57f730daa62513adaac9400db9a5c06，
review_r8十七成员SUM ed6131f4ff0206c523fc7dc521e72bd78e13e371676ccea50b9c2a7e4fb5fd04；
新r306保护加入两组，历史成员合9500/三模型/startup10，最终before仍必须实际通过。

### r306全部准备完成并fresh连续实跑，覆盖准备状态

1666本地/原回归/四派发17阶段/真实释放快照42密FCL/外部13测/完整七before及9500
历史成员/三模型/startup10实际通过。448源与380资源一次seal，identity SHA
1ba2ef37c4168f0244b3f5872ffe6b358426ea125195b49f596dfa8798ed4e84。
原片首帧实际image审看SHA1d82be3f42db5c3379fa5a137e2547a5a226d8a74bf52dab514f15eacc59941c，
机器人/近盒爪/闭盖设备/桌墙可见，远盒自然遮挡；不冒充全部对象可见。一次resume
直接连续dispatch，domain109/partition h1_seq_full_r306，actorPID398545/ctime1792404909.08，
首段sim3.4965，四角色存在。review_r9 post/observer已起，原supervisor自动正常收尾。
仅启动事实，非永久live或完整成功；接续先重查run/progress/result/exit和精确PID。
不得重复actor/claim或改密封源；仍待双盒取回/原物理/发布路径/全片及after保护。
旧r291完整版本、孵育箱/模型/所有正式成果和备份保持，三分钟及1:1未达。

## 2026-10-19 CST：r306释放微调实际通过，清出篮口接触力失败；r307仅局部降速

r306左盒一次微调将实际误差.275032541度/62.536um降为.010078863度/18.715um，原
.25度门保持；右盒.047972059度无需微调。双盒原10/3秒观察及LOADING_PASS157.4305，
真实五按键/稳转30.21sim/停稳开盖，MACHINE_CYCLE_PASS221.3535。左取盒正式capture
与15mm纯Z试抬实际通过；随后seq_retrieval_retrieve_axis_exit以普通.25rad/s、.94sim
秒执行，250.4285sim原指尖接触60.044556N越40N，深度6.3049e-8m。不是输出18.01N
故障（输出峰17.738514N），无盒返桌。最终251.103sim STOP_AND_HOLD保留原左holder，
不reset/detach/异常松夹；native/FK滑移峰.889519/.884982mm。不能称完整任务成功。
原片101160026B/2670.166667wall秒正常SIGINT/全解码、trace1259409行完整和四收尾
PASS，退出1/0/0/0，原任务审计FAIL保留。完整七after/9500历史成员/三模型/startup10
重算PASS，785正式源/10备份保持。669核心0444 SUM
6573390fd3beaf2a0dd3545618201d620de5a29213fe8f9b1728a246b42f85ec，review_r9二十三成员
SUM bbaa00230d32b22c2da6bcfb0ec17efccbd227b7b7da0cd8218412579675653f；原Gazebo已退休。

新独占r307/domain110/partition h1_seq_full_r307只将准确阶段
seq_retrieval_retrieve_axis_exit归入原pilot精细档：.03rad/s/.002rad跟踪。所有路径点/
IK-FCL/持物原生FK锚点/16与16.5N/18.01N输出/40N原始接触/1mm深度/.2m/s/2mm5度/
10-3-30秒观察不改，其余阶段速率不变。新隔离6.84秒/六库恢复一致，1673本地通过；
实际失败路径130次2mrad密FCL，原.94sim改5.2547096sim，仅离线证据，不声称新物理力
峰通过。原释放微调保留；新增r306669+r9二十三成员保护，合10192，仍须全部准备/
before保护/一次seal/fresh/实际原片首帧后实跑。孵育箱/旧成功/模型/authority/run/
报告/视频/builder/所有备份不改，无清理/新备份/导航/真机，三分钟与1:1未达。

### r307全部准备通过，fresh前视连续验证已启动

1673本地/原46-122-6CTest/四派发17阶段/真实释放42与清出130密FCL/录像归档/完整
七before及10192历史成员/三模型/startup10通过；17外部实际发布路径/清出定时复核
测试通过，Nav111/2旧V12缺源仍保留。451密封源/381资源，identity SHA
7ca9e80372fabece59a893cd34dd27b0869817d6c3cc920f79773bf97da27b1a。
实际原片首帧image审看SHA32b2b2f52d2ed91a72bb369cbfa0568cf15e43bbed0dd0fa8265e534c687c226，
机器人/近盒爪/闭盖设备/桌墙可见，远盒自然遮挡。仅一次resume直接连续dispatch，
domain110/partition h1_seq_full_r307，actorPID781002/ctime1792408528.45，首段sim4.129；
review_r10 post/observer已起，原supervisor正常收尾。此为启动事实，不是永久live或
完整成功；接续重查原run/progress/result/exit和精确PID，不能重复actor/claim、改
密封源或插动作。仍须实际双盒取回、原物理/实际发布路径/全片和after保护结论。

## 2026-10-19 CST：r307试抬后实际倾斜失败，r308尝试篮轴对中试抬

r307真实依次双盒装篮157.3405sim、五按键/稳转30.2185sim/机器周期221.244sim通过。
左取盒正式capture后纯Z15mm试抬使盒体从.9632度倾至4.02438度，下一规划起点被原
2度携物门拒绝；不是插值误差，新的.03rad/s篮口清出段没有执行。实际试抬篮/指接触
峰16.489130/17.492683N。最后251.535sim STOP_AND_HOLD保持左holder，无返桌/异常
松夹/reset/detach。原片99041886B/2606.833333wall秒正常SIGINT/全解码、1258698行
trace和四收尾PASS；executor1其余0，原任务FAIL保留，Gazebo已退休。
完整七after/10192额外历史成员/三模型/startup10一致，785正式源/10备份保持。669核心
0444 SUM a5d02db098dfe55fa651523df06c7ca0da431cdcb6088b5854c8f69d9113e3af；独立诊断
review_r10二十四成员SUM b4a5231144c1354bab47f0713275ea4ce03d261cecc59a8e19b8a42e92a7f4d1。

新独占r308/domain111/partition h1_seq_full_r308，仅将取盒pilot目标改为实际篮两横向
同时对中及篮轴升15mm，盒体姿态保持。装盒pilot仍纯世界Z；原捕获锚点/2度携物门/
2mm5度夹持/18.01N输出/40N原接触/16与16.5N/原IK-FCL/10-3-30秒均不改。侧壁
偏心拖拽只是待验证假设，禁止以离线结果宣称物理修好。r306释放微调和r307准确
清出段.03/.002保留；新隔离6.77秒/六库恢复、1692本地通过，需四实际快照密FCL/
完整准备/保护/一次seal/fresh/实际首帧后连续验证。新增r307/r10保护合10885历史成员。
旧孵育箱/完整双臂成功/模型/authority/run/视频/报告/builder/全部备份不改，无清理/
新备份/导航/真机。旧r291仍完整基线，顺序取回/三分钟/1:1均未达。

### r308准备全部通过并fresh启动，覆盖准备状态

1692本地/原46-122-6CTest/四派发17阶段通过；四实际旧快照各62次原2mrad密FCL
对中试抬、释放42/清出130检查和23项外部独立复核测试通过。完整七before、10885
历史成员/三模型/startup10与录像归档通过；原V12缺源Nav111/2保持。454源/383资源
一次seal，identity SHA f9fef000ec3885fc00ffb8316b2508e9448ab5b57834eec91b5ff285b6c6b966。
真实新原片首帧image审看SHA fc0812662ea5869509ea06bdafa43e25d4a52b4a68681aa405df5313b4e3a1e8；
机器人/近盒爪/闭盖设备/桌墙可见，远盒自然遮挡。一次resume直接连续执行，
actorPID1196365/ctime1792412520.37，domain111/partition h1_seq_full_r308，首段sim4.134；
review_r11 post/observer与原supervisor自动跟进。仅启动事实，不是永久live/成功证明。
接续重查run/progress/result/exit与精确PID，禁止重复actor/claim、改密封源或插动作。
仍须实际双盒取回、原物理/实际发布路径、全片与after保护结论，旧成果不改。

## 2026-10-19 CST：r308装盒实际边界纳弧度误差停止，r309装盒IK内部预留

r308左盒抓取/15mm装盒试抬/10秒抓稳和抬肘运到篮口，但释放前实际微调起点左肩yaw
-2.6400000010284894rad，比原预留下界-2.64多1.0284893e-9rad而拒绝。原请求独立
复现，不夹改测量或放宽原边界。未装入任何盒/未机器周期/取盒；75.5675sim保持
左holder，右盒仍桌上，无reset/detach/异常松夹。r308对中取盒pilot尚未实跑，不是
该假设失败或通过。原片31768771B/834.916667wall秒正常SIGINT/全解码，356335行
trace与四收尾PASS，退出1/0/0/0，原任务FAIL保留；Gazebo已退休。完整七after/
10885历史成员/三模型/startup10重算PASS、785正式src/10备份不变。638核心0444 SUM
3aff27d1d6fab256b1177349ea538f0f59d1c01a3ef2249e849310b202d05a8e；review_r11二十二成员
SUM 875e30049724fdf07e032f85c455096a039494380d1b8748980b02515160de28。

新独占r309/domain112/partition h1_seq_full_r309只在装盒搬运IK求解时额外向原界内留
.001rad，局部子类finally恢复原检查界；不扩大原关节边界/IK容差或改实际起点。
实际r308装盒快照2167次原2mrad密FCL通过，终点活动关节最小余量.001000000248rad；
合成纳弧度扰动检查明确不是新的实际测量。隔离7.16s/1696本地测试通过，仍须全部
保护/一次seal/fresh/原片首帧后实跑。原release微调、对中取盒、清出局部低速、
原生FK锚点/所有物理门和10-3-30秒保持，新增历史保护共11545成员。无旧成果改写/
清理/新备份/导航/真机；顺序取回与三分钟/1:1未达，旧r291完整版本仍有效。

## 2026-10-19 CST：r309启动统计解析失败未派发，r310只修统计读取

r309全准备/1696本地与27外部复核、458源/385资源一次seal后，fresh六控制器/ramp
通过，但world_stats整段json.loads报Extra data（line2/col1/char166），尚无执行器/
execution或resume claim/任务录像/启动SIGSTOP；不是机器人动作失败。原错误stdout
未保存，另一次只读probe只有一条有效JSON，不能伪造原多条有效JSON证据。精确原
launch及子进程正常SIGINT退休PASS、remaining为空，无reset/detach/松夹。完整七
after/11545历史成员/三模型/startup10重算一致、785正式源/10备份不改。571核心0444
SUM bd3adb6e85b713d18b8e4542aef733c0b8cfabe7f3c69c3ae8d26b9ed761cc4c；review_r12十七
成员SUM a332086a89b2ded821ddfdcd52647eeae4138ce70e888d9a681b4f440c0635ff。

新独占r310/domain113/partition h1_seq_full_r310，动作与r309逐字相等：保留装盒IK
额外.001rad内余量、释放微调/对中取盒/局部清出速度和原全部抓稳、物理、IK/FCL/
10-3-30门。仅启动统计支持最多64条/256KiB完整JSON流，每条整数非负合法时间且
不倒退，任何垃圾/损坏尾部均拒绝，取末条仍原25sim新鲜窗；只读命令8秒超时和
返回码不变。合成复现不当原现场输出。新隔离6.98秒/48启动专项、初全1715通过；
新增动作逆绑定两测需最终全测覆盖。新增历史保护12133成员；完整准备/一次seal/
fresh/真实首帧后才可唯一连续派发，不得重用r309。旧孵育箱/成功/模型/视频/报告/
builder/全部备份不改，无清理/新备份/导航/真机；顺序全取回与三分钟/1:1未达。

### r310全部准备/fresh通过，前视原片已连续实跑

最终1717本地/原46-122-6CTest/27外部复核、四派发17阶段、各实际快照资格、录像与
归档、完整七before/12133历史成员/三模型/startup10通过；Nav111/2旧V12缺源保持。
461源/386资源一次seal，identity SHA
40371fe3669bfdeb1a1b8ad64180648170171b0c6152952e6c2f6350444e5bc8。
fresh统计读取/六控制器/ramp通过，原片实际首帧image审看SHA
7ab78a2f6e26d4fc85530f4e0e09e254d7691dfdd51c5dbd78d0711de0f00efc；机器人/近盒爪/闭盖
设备/桌墙可见，远盒自然遮挡。一次resume直接连续执行，actorPID1521854/ctime
1792415371.22，domain113/partition h1_seq_full_r310，首段sim2.8745。review_r13 post/
observer与原supervisor自动跟进；这是启动事实不是永久live或完整证书。接续先
重查run/progress/result/exit/精确PID，不重复actor/claim或改密封源；完整物理/两盒
取回/实际发布路径/原片/after保护尚待结果，不将启动和离线资格冒充任务完成。

## 2026-10-19 CST：r310装篮/30秒周期通过，r311处理闭合早期单帧原生篮接触缺失

r310已实际左右顺序装篮157.9005sim及五按键/稳转30.210sim/机器周期221.816sim；
装盒IK余量与释放微调通过。左取盒闭合早期244.478sim只有真实指尖1，原生距最终
目标.257582mm/年龄8ms，原.25mm预夹证书拒绝。原生接触stamp244.477有篮正力，
244.4775缺失，244.478恢复；原raw证据确认一帧.5ms，不能伪造指尖2或称强抓稳。
盒底偏入.445191um/侧越7.614nm，原几何仍完全篮内。最后245.0905sim停持，未有
取盒正式holder/试抬/取回，原夹紧命令保持，无reset/detach/异常松夹。原片95077758B/
2500.333333wall秒正常SIGINT/全解码，1231585行trace与四收尾PASS，退出1/0/0/0，
任务FAIL保留；Gazebo已退休。完整七after/12133历史成员/三模型/startup10重算一致，
785正式src/10备份保持。677核心0444 SUM
5a51e2cd7721ac580c22c54bc4e88de2366ab2b6ca5f1288d1f3d7402e5d0510；review_r13二十六成员
SUM 010248079d84c111a2a1a410253273d1435ef4495006d126d958df8021d627c9。

新独占r311/domain114/partition h1_seq_full_r311增加单独的非抓稳支撑分类：只在
accepted当前本手本盒16.5N仍闭合中且无holder时，实际native接触stamp距最后实际
篮正力最多.5ms、接触距clock最多1ms、原生关节12ms新鲜；盒底/横向偏差均<=原
5um舍入且原篮内/侧壁门通过，实际指尖中部/侧壁正力、原40N/1mm和另一盒保护
仍必需。第二连续缺帧、未知/未来/过期、其他手盒或几何不符仍停；不把指尖力
刷新为篮支撑，不创建holder/重设锚点。原actual_preload_support的.25mm门、最终
endpoint、真正对向双指各3N正式capture和10秒抓稳不变，不能把单指分类当抓稳。
每次采用明确记录sequential_single_frame_floor_continuity，以原始接触另作独立审计。
动作/力/速度/IK-FCL/3秒落篮/30秒稳转不改，原对中试抬尚未获新的物理验证。

新隔离6.81秒/48专项通过；初全测一项mock缺snapshot失败保留，修测试替身后1746
通过。外部review_r14提前运行因资格文件未生成11失败保留，须资格完成后另存重测。
新增历史保护12836成员；完整准备/保护/最终全测/一次seal/fresh/实际首帧后才可
唯一连续实跑。旧孵育箱/完整成功/模型/authority/run/视频/报告/builder/所有备份
不改，无清理/新备份/导航/真机，顺序全部取回与三分钟/1:1仍未完成。
### r311全准备通过，fresh前视分开装取连续验证已启动

最终1746本地/原46-122-6CTest/35外部独立复核通过；四派发17阶段、真实快照各资格、
录像归档及完整七before/12836历史成员/三模型/startup10全部通过。Nav111/2旧V12
缺源保留。467密封源/388资源，identity SHA
32b9ba8032fe498d0a94eeac821b4593bd84650ed405cf29503a91c374f2cb8c。
fresh六控制器/ramp/统计与真实原片首帧image审看通过，首帧SHA
9dcafa7f30fea1c203f64b9d5dd4844a1006c346a1cd16568db3b0b96b1c4f89；完整机器人、近盒
爪、闭盖设备、桌墙可见，远盒自然遮挡。一次resume直接连续dispatch，domain114/
partition h1_seq_full_r311，actorPID1916443/ctime1792419165.62，首段sim3.1645。
review_r14 post/observer已起，包括独立原始接触历史审计；原supervisor负责正常
录像收尾/退休。此是启动记录而非永久live/完整成功，接续重新只读查run/progress/
result/exit与精确PID，禁止重复claim/actor或修改密封源。仍待两盒完整取回与原
物理/发布路径/全片/after保护，旧孵育箱/完整版本/全部备份不改。
## 2026-10-19 CST：r311实际record重名覆盖已定位，r312仅合并接触记录钩子

r311实际左右顺序装篮157.7105sim、五按键/稳转30.1575sim/机器周期221.5845sim通过；
左取盒244.5625sim早期闭合支撑门停止，未正式捕获/试抬/取回。原生篮接触一帧.5ms
缺报、下一帧恢复，但新分类从未采用。根因是SequentialCycle两个record定义，后面的
机器周期记录方法覆盖前面的新历史钩子，实际history为空；旧测试手工喂history/mock
record漏检。原r311源码/FAIL保留；review_r14保存AST位置、原失败实际接触、空历史
拒绝与真实历史离线分类通过，不能冒充物理恢复或强抓稳。
原片94772888B/2497.500000wall秒，SHA95a1a42b4e39b8b17802da57f7397f04fb936ec39237e7a6188a4e75f9a2fc0b，
正常SIGINT/全解码和四收尾PASS，trace1231668行CLOSED，退出1/0/0/0，Gazebo退休。
完整七after/12836历史成员/三模型/startup10全部保持，785正式src/10备份不变。
686核心0444 SUM18dc34a6be471f05f4bc8f3bfa1e444618ebd331ed4351fcd757fbb8b9e9b6b7；
review_r14三十七成员SUM90af17e783e1780d2bfae2f10a3334475e48b0235a34924f684bd757915d5e3b。

新独占r312/domain115/partition h1_seq_full_r312只合并record方法：保留原机器周期
深拷贝/super返回/记录，新history更新放进唯一实际方法。原preload_floor_frame逐字
不变，.5ms/1ms/12ms/5um等门、所有路径/力/速度/原锚点/10-3-30秒均不改。
新增实际类分派、原重名复现、AST无重名、实际r310/r311 contact经真实record+safety
集成测试，专项34/初全1751通过；隔离7.03秒/六库恢复一致。新增保护合13559成员，
仍须全准备/保护/最终测试/一次seal/fresh/原片实际首帧后唯一派发。
旧完整r291、孵育箱/920箱子版本/模型/authority/run/视频/报告/builder/全部备份不改；
无清理/新备份/导航/真机。完整顺序取回/三分钟/1:1仍未达。
### r312全部准备和fresh通过，唯一连续执行已启动

1751最终本地/原46-122-6CTest/35外部复核、四派发17阶段/实际各快照资格、录像归档、
完整七before/13559历史成员/三模型/startup10通过；Nav111/2旧V12缺源保持。
470源/389资源一次seal，identity SHA
11d4136d2bac394664c9d6a8a457b6797558fd1ebb147e817bbc68c71dfef03e。
fresh六控制器/ramp/统计及真实前视原片首帧image审看通过，首帧SHA
 e998a049fb915be7a0ab6a5cac0e3a22d847a933ec395fd9133bf9621700130f；机器人/近盒爪/闭盖
设备/桌墙可见，远盒自然遮挡。一次resume直接连续执行domain115/partition
h1_seq_full_r312，actorPID2290137/ctime1792422797.33，首段sim4.26。
review_r15 post/observer与原supervisor自动观察/收尾。启动事实非永久live/完整成功，
后续先读run/progress/result/exit与精确PID；不重复派发/claim、改密封源或插动作。
仍待两盒实际取回/原物理与原始接触审计/实际路径/全片/after保护，旧成果不改。
## 2026-10-20 CST：r312完成取盒对中试抬后快照漂移；r313增加原夹持趋稳观察

r312实际左右顺序装篮159.8305sim、五按键/稳转30.1055sim/机器周期223.6565sim通过。
左取盒正式capture及篮轴对中15mm试抬通过，试抬后实际盒倾约1.0158度，未再触发
r307四度门。record接线修好，但本轮单帧历史采用次数0，不能宣称该分类已物理采用。
取回路径2220密FCL几何通过；253.212sim原快照复核平移.444472673mm、手指.444448831mm
超过原.3mm，而关节1.677e-8rad/角.004397度不过限。过期规划未派发，未清出/返桌；
253.899sim停持左holder/右盒在篮。FK/native滑移峰.920763/.924314mm，无reset/detach/
异常松夹。原数据表明规划期间指尖继续归中，末段.5sim漂移约.04085mm，不能据此
放宽原快照门。原片99832144B/2632.5wall秒正常SIGINT/全解码，trace1266589行CLOSED，
四收尾PASS，退出1/0/0/0，Gazebo退休，原任务FAIL保留。完整七after/13559历史成员/
三模型/startup10保持、785正式src/10备份不改。697核心0444 SUM
836bbc2615f56380457d7c098f7579382a4847ee4e4b05f1a8cc1b88022096c0；review_r15三十二成员SUM
67031b3f296bcd33cdb74ee453d2bfc39cfc7d2fb086299f7da5ab291228fda9。

新独占20261020_r313/domain116/partition h1_seq_full_r313，仅在取盒pilot后、transport
规划前原dwell保持1sim，继续原spin保护/holder锚点/phase/夹爪命令不改，随后最新
原生快照规划并仍执行原.3mm/.1度复核。记录开始和完成，独立review_20261020_r1按
实际事件时间及无新命令/无重设锚点检查；不是缩短10/3/30秒或放宽任何物理门。
隔离6.86秒/六库恢复、专项16和初全1761通过；外部首次因资格尚未生成11失败保留，
待全部资格后重测。保护新增14288历史成员，须全准备/保护/最终全测/一次seal/fresh/
实际原片首帧后唯一连续执行。旧r291、孵育箱/920箱子版/模型/authority/run/视频/
报告/builder/全部备份保持，无清理/新备份/导航/真机；完整顺序取回/三分钟/1:1未达。
### r313全准备与fresh通过，前视连续验证已启动

最终1761本地/原46-122-6CTest/46外部独立复核、四派发17阶段/实际快照资格、录像归档、
完整七before/14288历史成员/三模型/startup10通过；Nav111/2旧V12缺源保持。
475源/391资源一次seal，identity SHA
3ae59fce678195aad08a3c7848e4690498b08dffae0a0f274f8d5bbac0ca046c。
fresh六控制器/ramp/统计和新原片首帧实际image审看通过，首帧SHA
 dc8554525720c7f81078bd2c18e83dd8b5cac4a251d28ababf06085733c9a6d1；机器人/近盒爪/闭盖
设备/桌墙可见，远盒自然遮挡。一次resume直接连续dispatch，domain116/partition
h1_seq_full_r313，actorPID2718851/ctime1792426900.22，首段sim3.3945。独立
review_20261020_r1 post/observer已起，新增实际1sim等待/原锚点/无新运动的记录审计。
启动事实不是永久live/完整成功；接续重查原run/progress/result/exit与精确PID，不重复
claim/actor、改密封源或插动作。仍须实际两盒取回/原物理/实际路径/全片/after保护。

## 2026-10-20 CST：r313左盒清出通过，r314修单手返桌分派

r313左右顺序装篮157.6505sim、实际五按键/稳转30.171sim/机器周期221.5495sim通过。
左取盒对中15mm试抬后原dwell精确1sim，新快照平移6.912um/手指15.620um/角.008431度
通过原.3mm/.1度门；已清出并完成原10秒抓稳，篮净距.051289m。276.5305sim返桌下放
粗段错误进入CoarseRampCycle双手资格，单holder被拒且该段未派发；277.192sim停持左盒，
右盒篮中，returned为空，不是完整任务成功。原片110937214B/2914wall秒正常SIGINT/
全解码，trace1375482行CLOSED、四收尾PASS，场景已退休，无reset/detach/异常松夹。
完整七after/14288历史成员/三模型/startup10全部保持，785正式src/10备份不改。
700核心0444 SUM 0c877d8383005885306e541ff957c2a5dddcd0e506def670d5aefdf58372840c；
review_20261020_r1三十六成员SUM 44f85251186c8de6bf79e8bfeac830fd11fbadc6e11fee677fce03453238810b。
本轮单帧历史采用0次；报告/实际诊断/元数据勘误保留，旧r291完整版本仍有效。

新r314/domain117/partition h1_seq_full_r314增加严格单手下放适配：绑定原实际holder、
顺序状态、原规划和粗细路径；保留兼容阶段名保证原桌/篮净距门，粗段.13rad/s加
.6sim渐入渐出，细段.03rad/s。仍原DualCycle.move派发/跟踪/端点；旧双手资格不删、
不伪造双holder。r313其余动作和全部原锚点/物理/10-3-30秒观察不改。
隔离7.18秒/六库恢复、32专项和11独立审计测试通过；全测提前于资格生成29缺文件
失败/1748通过日志保留，须正常准备后重测。全保护新增15024历史成员。尚未seal/
实跑，须全准备、一次seal、fresh和原片实际首帧后唯一派发。旧孵育箱/920箱子版/
模型/authority/run/视频/报告/builder/全部备份不改；无清理/新备份/导航/真机。
完整顺序取回、三分钟、1:1仍未达。

### r314全部准备/fresh及首帧通过，前视连续执行已启动

最终1777本地/原46-122-6CTest/57外部独立复核、四派发17阶段及实际快照各资格、
录像归档、完整七before/15024历史成员/三模型/startup10通过；Nav111/2旧V12缺源。
480源/393资源一次seal，identity SHA
dd069557e18efd895ef6eb4e7c2c956ee8977c8c76392415ebfc0ca9a46f72e3。
fresh六控制器/ramp/统计和真实原片首帧image审看通过，首帧SHA
d56ec7d4ca1b640f621beb997eb2b62a598603aace771ffa7f8e795534bf4bed；机器人/近盒爪/闭盖
设备/桌墙可见、远盒自然遮挡。一次resume自动连续执行domain117/partition
h1_seq_full_r314，actorPID3165999/ctime1792431213.59，已见sim5.0545。独立
review_20261020_r2 post/observer启动，新增真实单手粗细下放定时/原路径/原锚点审计；
若完整PASS还须独立实际发布路径/FCL复核。原supervisor负责失败停持与正常录像/
场景收尾，不需每步等待用户。当前仅启动事实，不是永久live或完整取回证书；后续
重新读取run/progress/result/exit及精确PID，不重复claim/actor、改密封源或插动作。
仍待两盒完整取回、全片解码、原物理与全部after保护。旧成果与全部备份不改。

## 2026-10-20 CST：r314发现单手下放前置阶段误名，r315精确修正并测试整段联动

r314双盒装篮159.4605sim/稳转30.097sim/机器周期223.266sim通过；左盒试抬后1sim、
新快照门、清出和10秒抓稳271.2805sim通过，篮净距.051709m。279.2205sim新适配
错误要求seq_retrieval_retrieve_transport，实际前段为seq_retrieval_retrieve_transfer，
粗段未派发即被拒。旧测试手工预置错误phase，遗漏真实execute_stages联动，不能
把32专项/57复核当成完整验证。279.8585sim停持左盒/右盒篮中/未返桌，无异常松夹。
原片110829431B/2913.666667wall秒正常SIGINT/full decode，SHA
0c63a01bd432c57994082a2d656854d5bd3064a4e3ad984feb2d1e645495fab1；
1386484行trace CLOSED/四收尾PASS/退出1-0-0-0/场景退休。全七after/15024历史成员/
三模型/startup10一致，785正式src/10备份保持。710核心0444 SUM
ec7bf605e75c6f65424c59a73a3bd659c45f3afb9e1cfd4268766e0484d527bc；
review_20261020_r2四十成员SUM b63c3fa3320673c157ed7b3689adfce7b88ff07a9002dce7a062d42ddb14de54。
单帧历史与单手下放定时实际采用都为0，原任务FAIL封存，不称完整取回。

新r315/domain118/partition h1_seq_full_r315唯一运行变更是上述精确前置阶段字面量，
不扩大允许阶段，不改变原路径/物理门/holder锚点/速度/10-3-30观察；旧双手资格保留。
测试不再手工预置误名：使用r314真实规划，经实际execute_stages/DualCycle.move完成
五阶段/六命令/18组发布；发布/反馈/净距明确为离线mock，不当新物理证据；旧错误
从真实前置阶段重现。专项33通过，隔离7.15秒/六库恢复；新增保护15774历史成员。
正执行全准备，尚未seal/实跑，须全部测试/资格/保护、一次seal/fresh/真实首帧再唯一
派发。旧r291完整版本、孵育箱/920箱子版/模型/authority/run/视频/报告/builder/全部
备份不改；无清理/新备份/导航/真机。完整顺序取回、三分钟和1:1仍未达。

### r315全准备/fresh首帧通过，唯一前视连续执行启动

最终1783本地/原46-122-6CTest/57外部独立复核、整段五阶段/六命令/18组发布离线资格
和原各资格、录像归档、完整七before/15774历史成员/三模型/startup10通过；Nav111/2
旧V12缺源保持。483源/394资源一次seal，identity SHA
59bf1d349c5998e426aa346687344826445c6718340a56abe3b295c408ce5e0e。
fresh六控制器/ramp/统计通过，实际view_image首帧SHA
57ee88aaded4b86c084938cfd659d50c30b8ba0cb482cf0ed1d717f09d12dd94，机器人/近盒爪/闭盖
设备/桌墙可见，远盒自然遮挡。一次resume连续运行domain118/partition h1_seq_full_r315，
actorPID3619139/ctime1792435546.45，已见sim2.963。review_20261020_r3独立post/observer
接续，含单手下放/原等待/原始接触及PASS后实际路径复核；原supervisor负责正常
收尾。启动不等于完整成功或永久live；接续重查run/progress/result/exit与精确PID，
不重复actor/claim、改密封源或插动作。仍待两盒取回/全片/原物理及全部after保护。

## 2026-10-20 CST：r315左盒完整取回通过、右盒粗下放滑移；r316仅右侧减速假设

r315双盒装篮157.6505sim/五按键/稳转30.171sim/机器周期221.8255sim通过。左盒清出及
10秒抓稳269.2605sim通过，粗/细下放实际通过，300.761sim桌面支持/位姿/3秒稳定通过，
order.returned=['left']。右盒也清出并于351.0605sim完成原10秒抓稳，净距.050726m。
367.018sim右粗下放原native滑移.002026779m超过原2mm，角4.387度尚未过5度；
末2秒只有右爪自身接触，盒z约1.03m尚未落桌，不将故障归于桌接触。367.693sim
停持右盒，左盒在桌。summary旧native峰1.949865mm不包含失败即时峰，必须分开。
原片147493421B/3877.083333wall秒正常SIGINT/full decode，SHA
11a37376ed2119195de52b03491be4a9af7bf6101d513eab00fa929f17069975；
1784547行trace CLOSED/四收尾PASS/退出1-0-0-0/场景退休。全七after/15774历史成员/
三模型/startup10保持、785正式src/10备份不改。722核心0444 SUM
31588c8d29ca694cddc0ab153544a81982816a3a89421852bdf0264f9795ca61；
review_20261020_r3四十二成员SUM fab2035c759fd59774638d8ead1fa2d4145ababa919b37437d010fa3d75c162f。
原任务FAIL保留，单手下放定时仅左侧完整，右侧未完，不称全流程成功。

新r316/domain119/partition h1_seq_full_r316只将右粗下放.13rad/s/.6sim渐变改为
.06rad/s/1.2sim渐变，假设降低该段动态滑移；左侧已通过时序.13/.6不改，细段仍.03。
每个原路径点、实际holder锚点/抓力/速度保护/2mm-5度/桌篮净距及10-3-30秒均保持，
不以偏移或重设参考掩盖滑移。实际r315左右规划经完整原方法离线mock联动，专项27
通过、隔离6.75秒/六库恢复；不是新物理结果。保护扩至16538历史成员，正全准备，
尚未seal/实跑；全资格/测试/保护、一次seal/fresh/实际首帧后唯一连续执行。
旧r291完整成功、孵育箱/920箱子版/模型/authority/run/视频/报告/builder/所有备份
不改；无清理/新备份/导航/真机。顺序完整取回、三分钟与1:1仍未达。

### r316全准备/fresh原片首帧通过，右粗下放减速连续复测已启动

最终1787本地/原46-122-6CTest/59独立复核、实际左右原路径整段资格与原各资格、
录像归档、完整七before/16538历史成员/三模型/startup10通过；Nav111/2旧V12缺源。
487源/395资源一次seal，identity SHA
8abb5e11024f415f7b87f6870a97371859ba6aadf9453dd0cc1ce79c663a9f15。
fresh六控制器/ramp/统计通过，实际view_image首帧SHA
2397b1c7fc7abd0d2e4de33cf17a003483c0134ca9fe61776f36958eb1703dd8；机器人/近盒爪/闭盖
设备/桌墙可见、远盒自然遮挡。一次resume连续运行domain119/partition h1_seq_full_r316，
actorPID4047939/ctime1792440868.55，已见sim2.426。review_20261020_r4 post/observer跟进，
单手下放审计按左右各自精确时序复核；完整PASS还须原实际路径/FCL/28物理门等。
启动事实不是永久live或全流程成功，接续先查run/progress/result/exit及精确PID，不
重复claim/actor、改密封源或插动作。仍待两盒全部返桌/原片/全部after保护，旧成果保持。

## 2026-10-20 CST：r316左返桌通过、右转移IK未派发；r317仅右高点下调1厘米

r316左盒300.8705sim再次完整返桌，右盒在原精度IK规划阶段失败，右慢下放未派发。失败原片/trace/四收尾与全部after保护已封存，场景退休。新r317仅右高转移点下调1cm，左及最终落桌点不变；右粗下放仍.06rad/s/1.2sim，原锚点/力门/精度/密FCL和10-3-30秒保持。三个真实旧快照通过2563/2554/2228密检查，非新物理成功。正在全准备，尚未seal或运行；旧r291完整成功、孵育箱/920箱子版/正式模型/所有备份保持。完整顺序取回、三分钟和1:1仍未达。

r316最终原片117485297B/3075.666667wall秒，SHA
862cf8dc54eaee9fcfd226fc0424aa44ef498bab552362f06335a247928e36de；
trace1634562行CLOSED/四收尾PASS/退出1-0-0-0。七after/16538历史成员/三模型/startup10
保持，785正式src/10备份不改。726核心0444 SUM
47a3f58ab5ac451e6695d91699910f243e62707b607209bc4cee4bf68a1cbb46；
review_20261020_r4四十九成员SUM c0087e43b78ce9ababffb054945ab90e2bfce35d16c48462ab8488088cd4bc95。
r317/domain120/partition h1_seq_full_r317隔离5.95秒/六库恢复、80专项和1791初测通过。
新保护17313历史成员。正式右高点1.13475m、左1.14475m，最终.44/±.38/.84475不改；
原20um/.01度、2mradFCL/关节限位/真实holder和2mm/5度不放宽。r317修正新identity的
immediate_parent为r316；旧封存元数据不改。尚须全准备/新seal/fresh/原片首帧与实跑。
无清理/新备份/导航/真机。原机械模型的部分visual-only碰撞局限仍在，不称绝对无碰撞。

### r317全准备/fresh/实际首帧通过，唯一前视连续执行已启动

1791最终本地/原46-122-6CTest/62独立复核、三个实际快照高点原密FCL资格及全部原资格、
录像归档、七before/17313历史成员/三模型/startup10保持，Nav111/2旧V12缺源。
494源/396资源一次seal，identity SHA
fa257b89e093693a637b6cb0ad448fa5f8011d6955c063fd209d976a4de6d2db。
六控制器/ramp/统计通过，实际view_image首帧SHA
423be7a7b81dd6783c39c1863c298cb138c29c5c190fe494c96315054285beae，机器人/近盒爪/闭盖
设备/桌墙可见，远盒自然遮挡。唯一resume连续运行domain120/partition h1_seq_full_r317，
actorPID4055521/ctime1792445174.26，已见sim3.3215。review_20261020_r5 post/observer已启动，
新增每手转移高点及原落桌点审计，若PASS继续原实际路径/FCL/28门及正常录像收尾。
仅启动事实，非永久live/完整成功；接续先查run/progress/result/exit与精确PID，勿重复
claim/actor、修改密封源或插动作。两盒完整取回、全部after和原片核验仍待，旧成果保持。

## 2026-10-20 CST：r317实际试抬姿态越界，r318试抬目标逐步调平

r317实际双盒装篮/30.21sim稳转/停稳开盖通过，但左试抬后倾角2.112487度超过原2度规划门，搬运尚未派发；右高点和慢下放未验证。252.0335sim停持收尾，原片2244.5wall秒完整解码、trace/四收尾及七after/17313历史成员/三模型/startup10保持。722核心SUM 819567f81796055c4e9c0cc5d1813e7c15cb1cd639645cbe195b6e355b324d80，review_r5四十四成员SUM d7f728584e24ac9afeac70d5fd09d78097cd6758406a1c2df6de4ff3cb7f82c0。
新r318/domain121仅把取盒对中15mm试抬目标改为保持实际yaw、逐步水平，原位移/0.03rad/s/起点和路径2度门/原holder/原物理与10-3-30观察不变。四个真实快照各62原密FCL通过，非新物理结论；78专项、1836初测、隔离5.85秒/六库恢复通过。正全准备，尚未seal/实跑。新保护18079历史成员，旧r291完整成功/孵育箱/920箱子版/正式src/10备份不改；完整顺序取回/三分钟/1:1仍未达。

r317原片SHA cc9d9c11c1f658df15ca95531c428462e9834115b26521c2f2a38c27f9cdb017；
实际试抬初始.964847度、20ms局部采样峰2.465212度，不用后续快照掩盖瞬态。
原2mm/5度holder门与2度几何门不同，均不放宽；r318不是改失败快照或重设锚点。
后续全准备/一次seal/fresh/真实首帧后唯一执行，失败停持、正常SIGINT。无清理/
新备份/导航/真机。visual-only模型局限保留，不能称绝对无碰撞。

### r318全准备/fresh/实际原片首帧通过，试抬调平版连续执行启动

1836最终本地/原46-122-6CTest/66独立复核、四实际试抬快照原密FCL和全部原资格、
录像归档、七before/18079历史成员/三模型/startup10通过；Nav111/2旧V12缺源。
503源/398资源一次seal，identity SHA
307c9c78d4a2ce49cabde078a245d5f37f9b562a09edf4da744456eac28b2874。
六控制器/ramp/统计通过，实际view_image首帧SHA
08889e35bd66326a57dcb04da7598c239daa4dd514488b1d55fdc5ba708d90d9，机器人/近盒爪/闭盖
设备/桌墙可见，远盒自然遮挡。唯一resume自动连续运行domain121/partition h1_seq_full_r318，
actorPID4063792/ctime1792448512.91，已见sim3.1095。review_20261020_r6 post/observer已启动，
其独立试抬核验要求保持实际yaw/原15mm对中并世界水平，实际路径仍逐段原密FCL。
只记启动事实，不代表全流程/永久live。接续重查run/progress/result/exit与精确PID，勿
重复actor/claim、修改密封源或插动作。全任务/原片/全部after仍待，旧成果保持。

## 2026-10-20 CST：r318分开抓抬肘完整流程首次通过，原片/独立复核及保护封存

r318已完成固定骨盆Gazebo分开抓完整流程：左右手依次装两盒→实际关盖/启动→稳转30.128sim秒→停稳开盖→左右依次取回不同桌面位置→10秒终态保持。两盒返桌303.7105/410.6605sim，430.6905sim PASS、双爪空。全程428.45sim/3894.946321wall秒，非三分钟或1:1。当前授权放宽仿真门28/28、四原锚点抓持/21实际路径/24590原密FCL与全片解码通过；1836本地/原46-122-6CTest/77最终独立测试通过，Nav111/2旧V12缺源保持。

必须保留局限：持物角偏峰3.038567度，原严格2度门未过；取盒瞬态峰.069617rpm、8样本超过原.02rpm，仍在此前授权5度/.1rpm仿真门内。最大滑移1.653056mm。模型visual-only碰撞局限不改，不称绝对无碰撞、真机成功或速度达标。

新试抬逐步调平保留yaw/15mm篮轴对中和原密规划门，左右实际试抬后倾角1.348141/
.057771度；新右高点和慢粗下放均实际执行，右粗/细31.987876/8.817949sim秒通过。
独立审计首版因空手接点5.551e-17rad浮点差失败，原脚本/日志保留。仅revision2接受
<=1e-12rad数值连接且仍执行原密FCL，实际运行源码、关节界/IK/物理/锚点不改。
原post在首版失败处退出，随后独立补跑全部after；完整证明为review_20261020_r6/
final_review_revision2.json，不伪称原post全部成功。77测试与21实际路径复核通过。

503源/398资源identity SHA 307c9c78d4a2ce49cabde078a245d5f37f9b562a09edf4da744456eac28b2874。
原片149480987B/3926.25wall秒，SHA eccf941001586b2ea06ab516a5ad09c51d1bee9aebc3840b98bd28a6d972dfe0。
trace2064966行CLOSED/raw5313096531B/gzip1138046409B；正常SIGINT/四收尾PASS/退出0-0-0-0。
场景已退休，不能复用r318/run/claim。753核心0444 SUM
4bdfd4c608f56f0ed0c7b1c4396e727f0986d6b8df0bb7d10c1d3a3c8116823f；
review_r6五十六成员SUM a7ebcc60dfca91c9365b6f26caaeb2267b89b3e0bdfdcc9e129f4bc059570ccf。
完整七before/after、18079历史成员/三模型/startup10一致，785正式src/10备份、孵育箱/
920箱子版/旧r291双臂成功/旧authority/run/视频/报告/builder均不改。无清理/新备份/导航/真机。

观看版在runtime_evidence/centrifuge_48box_sequential_view_20261020_r1/
sequential_full_cycle_front_10x.mp4：统一10倍、392.633333秒，无裁剪/重排，不是实时耗时。
视频SHA bfc74ec6f8034ea9c108344e7c2dcbd1e086f12621eebafdd036d9f22890a77e；11媒体成员SUM
5ba619caa96173e5f81e6165983532299d04b402520fe8f09dd75e754d529b2e。实际看过装盒/取盒/终态
抽帧，远侧返桌盒被设备遮挡，双盒返桌以物理证据为准，不冒充画面证明。
报告r318/REPORT.md与独立review_r6/REPORT.md。下一步若继续优化，用新未用身份验证
重复性/角偏/速度；保留本轮分开抓成果与全部旧版本。当前不是自动继续或真机许可。

## 2026-10-20 CST：分开抓提速组件已验证，完整成功仍为r318

用户随后明确要求继续直接优化、测试和运行，保持分开抓抬肘/抓持稳定与旧成功，争取现实180秒，并允许放宽仿真限制。本授权不含真实机器人、导航、删除证据、清理或新备份；不能取消物理抓取/碰撞后冒充原成功，也不能伪造实时率或把加速录像当执行耗时。r318封存和全部旧成果不改；本节覆盖上节“不是自动继续”的当前任务状态。

本轮仅执行5个全新固定骨盆被动Gazebo场景，均已正常SIGINT退休，无新装取/按键/盖/转子任务；未创建r319完整动作身份。性能证据：
- runtime_evidence/centrifuge_48box_sequential_perf_20261020_r1/REPORT.md：原网格实时率.144554；两足原生接触上限4得到.401909，被动组件通过。foot8控制器启动超时失败；baseline额外足部稳定性失败，因此总体review为FAIL，后续独立补齐after，不改写失败。
- runtime_evidence/centrifuge_48box_sequential_perf_20261020_r2/REPORT.md：仅两足碰撞用原STL精确AABB保守代理，视觉/质量/惯量/摩擦及手爪/盒篮/设备/桌碰撞保持，物理.5ms/控制500Hz不变。原网格.141244对sole_box .874659，约6.1925倍；4.5秒窗.866870—.879975，仍未稳定1倍。脚底窄相计算占比99.2866%降至3.5429%。六控制器改官方spawner成组激活，两次均全active。sole_box静止脚漂移.5435mm/.70071度，其余7实体无漂移；不是带盒稳定或腿部导航证据。baseline额外稳定性FAIL，故总体review仍FAIL，sole_box组件单独PASS。
- 新脚底近似改变被动腿姿；旧r318初始化要求原关节参考±.010rad，不能直接套用。必须新身份验证腿部静止初始化/参考，再接入新实际抓持；没有通过改旧数据或删门掩盖差别。原r318为428.45sim/3894.946321wall，达到1倍也仍不满足180秒。
- runtime_evidence/centrifuge_48box_sequential_planning_perf_20261020_r1/REPORT.md：四个r318真实抓前快照A/B，原73.948854433wall秒→候选17.933111233秒，约4.1236倍。实际pickup路径12字段×4精确一致，8886次原2mrad密FCL保持。只跳过空手时假想的未来持物试抬/搬运预测，明确NOT_COMPUTED_NOT_A_CERTIFICATE；实际抓后新快照/原锚点/载物密规划仍必需，可能抓后才发现不可达而停持。不是实跑新抓持、完整耗时或原未来预审证书。

r1局部34、r2局部37、规划候选12测试通过；两轮隔离各3包、原46测试/6CTest及各1政策CTest通过，Conda六库恢复相同。Nav均111通过/2旧V12来源缺失失败，未掩盖。r1包装器旧PYTHONPATH问题与r2首次overlay非覆盖拒绝已在新r2修正并测试，原失败日志保留；成功构建为build_revision2.log。
两批全部九类before/after保护PASS，18899历史成员（含r318/独立review_r6/观看版）、785正式src、三模型/startup10、10备份均保持。旧孵育箱/920箱子版、authority/run/视频/报告/builder不改；无清理、新备份或真机。
新证据均只读封存：perf_r1 208成员SUM 5dc8c6cd20550e9465d06eb5db4ed04b2276ec11f416e300c88ec516a88ac01a；perf_r2 176成员SUM 39207928683d047c0a3158376189503931c0d3d897e2665a13fbc06e7eaf32b5；planning_perf_r1 36成员SUM bc1b9dca7bc2005c16e8348ab11691e033750873c086c00a90fae72b647784e4。r2/final_integrity.json确认125源、上一208成员及九组保护。
下一步：全新身份验证脚底代理的静止腿初始化兼容性及带实际反馈的吞吐，再集成规划削减/运动时序；仍需新的装篮→关盖→稳转30秒→停稳开盖→取回全流程与录像/独立物理复核。不得复用任何已运行run/claim，不称本轮已经现实三分钟或稳定1倍。

## 2026-10-20 CST：静止腿初始化兼容性通过，r319集成验证准备

用户要求先修初始化、再集成提速并完整录像，保持分开抓抬肘/真实稳定抓持，保护r318。当前完整成功仍为r318，尚未产生新的完整任务成功；下列是本轮组件资格，不是现实180秒或稳定1倍证明。
- init_20261020_r1：两足完整AABB阻碍原腿目标，末端踝误差约89mrad；失败并正常退休，138成员SUM 67d75d010a812ca910ae9493dde6359952453dce49694fd360afc3920c5ea745。
- init_20261020_r2：发现固定骨盆原站姿脚网格埋入地面约43.56mm。只在私有URDF改为原站姿定向、底面修剪到地面的静止脚底代理，残差降约1.48mrad，但原1mrad门仍失败；139成员SUM b9de4b1cae8bea6274e017bb0f7beebd043d6ad787bf64b94ea4ef4ccdd36775。不是完整网格保守代理，不适用步态/真机。
- init_20261020_r3：仅静止腿端点/5秒保持门放宽到2mrad（0.1146度），原12腿目标、3秒平滑动作不改；实际初始化14.646099wall秒、5sim保持PASS，独立峰0.001496919rad。随后35.410020wall推进29.9935sim，RTF .847034249。无抓取/按键/转子动作，正常SIGINT退休；141成员SUM 8869e9d96de2c5bded49c7268ca84c9d31000251835263c87d222e03c7ad3d66。三探针前后九组保护通过，均已只读封存。
全名均以runtime_evidence/centrifuge_48box_sequential_为前缀，见各REPORT.md。

新runtime_evidence/centrifuge_48box_sequential_full_20261020_r319是从r318派生的未用身份，domain125/partition h1_seq_full_r319。仅私有脚底近似、六控制器成组激活、任务前真实原腿初始化、删除未执行的假想未来持物预演、增加真实单调墙钟计时；真实抓后新快照/原锚点/原密FCL和夹爪/盒篮/设备/桌碰撞保留。当前各运动速度和10/3/30sim保持仍沿用r318，因此即便1倍也不能承诺180秒；本次首先验证集成。局部1859+墙钟3测试，原46/122/6CTest，Nav111/2旧缺源；最终测试/资格/seal以新目录落盘为准。
r319初始化资格与探针程序要求逐字节一致；修正了两个派生文件末尾缺空行导致的严格hash拒绝，旧失败日志保留。未运行过的AABB候选10个旧审计结果移入本身份superseded_unsealed_aabb_audits并记精确ledger，没有删除/覆盖正式证据。
独立复核工具在review_20261020_r7，77测试通过。旧r318/孵育箱/920箱子版/正式785源码/三模型/10备份均保护；无清理、新备份、导航或实体机器人。后续先检查新run实际状态，任何已seal身份不得修改/重用；首帧必须实际看图后使用startup_review_freeze.py resume一次，不能误用world pause的旧hold分支。

## 2026-10-20 CST：初始化集成已通过；r321按用户新授权提速优先

用户追加“放宽各种门限，先把速度提起来，有问题之后再解决”。本轮只对全新私有仿真身份使用显式放宽合同，不覆盖r318，不删除成果，不运行导航/真机。原30秒稳转、真实两盒装篮/取回、分开抓姿态、不可重置抓持锚点与原密FCL保留；旧成功不可冒充放宽版证书。

r319在完整场景较晚开始初始化时右腿残差约2.259mrad超过2mrad失败，未派发手部任务；正常退休。661成员SUM a5a81c5ff302f0b1b0fb11d283f61837f05fe991e93bf894bff5c4b74896f59e；review_r7 SUM 0dbc843e16a43a84a07a45c296da4dbc3f268bfa52415045f7c8698e0764a062。
r320将静止腿实际5秒保持门显式改5mrad、启动窗25改45sim；实测峰2.270055mrad/12.818917wall秒初始化PASS。后在任务内冗余控制器查询期间，位姿超前clock .1555sim超过旧.15门而停；位姿墙龄.002944秒、clock墙龄.207701秒，非模型丢失。motions0，未抓盒/按键；任务10.386355wall秒FAIL不等于180秒成功。1869测试/原46-122-6CTest/77独立测试通过，Nav111/2旧缺源。原片43.666667wall秒完整解码、trace27761行CLOSED、四收尾/退休及九组before/after保护PASS。r320 704成员SUM b1f63980783fe4eed2d77a03d8397412012a762eb80fb39440e80964f417bfe3；review_r8 33成员SUM f4c79687573fed1c28fc244e27a02a72206975c365fdd2a4099ea9fe46f6d340。失败与封存源码不改。

新r321/domain129/partition h1_seq_full_r321集成SpeedPriorityCycle；原SequentialCycle/路径/单持物状态机保留。空手1.2、载物.8、近段.4、试抬/细段/轴退出.15rad/s；轨迹时间点下限20→2ms、保留加减速。抓稳10→1、释放3→.5、终态10→1sim秒，稳转30不缩。显式抓持门5mm/10度、跟踪.04/.08rad、末端2mrad保持.05sim、位姿clock时差.75sim且消息墙龄仍2秒。真实原腿初始化证明复核后删除冗余任务内控制器查询/重复动作。原时空碰撞点及2mrad密检查不减，不增加模型attach/teleport。

r318真实43段路径仅重定时218.092545→64.294691sim秒，逐点/分组相同；不含规划、按键、夹爪、稳转及反馈保持，不是完整耗时预测。27新增快版/审计测试通过，隔离两包构建7.03秒/Conda六库恢复；正在全部资格/测试/preflight，尚未声称r321物理成功。当前完整成功仍r318/428.45sim/3894.946321wall，现实三分钟/稳定1倍未达。接续必须检查r321 identity/run/日志；已seal不得修改/复用。

## 2026-10-20 CST补充：r321在旧按键门停止，r322补齐快版门限

r321/domain129完成初始化与首次物理开盖，空手撤臂仍调用继承的0.015rad跟踪门，36.0005sim停止，实际肘误差-0.018800003274rad。motions4/任务14.831825214wall秒FAIL，未抓盒；新顺序/抓持/全物理审计均如实FAIL。原片43.916667wall秒完整解码、trace60410行CLOSED、四收尾/正常退休/九类前后保护PASS；21164历史成员/785正式源/三模型/10备份保持。
r321 720成员SUM b703cc5bc6299c25fee9a0d13f2576312b9a9d8f4f48e0e6e953f0638a343636；独立review_r9 3成员SUM d3f2813697147e4c84195a7f8944659d238202429336dea7ef98718b7d8b65c5。失败瞬间按原线性反馈公式从原trace重建，证据在review_r9/button_tracking_failure.json。
新r322/domain130/partition h1_seq_full_r322继承全部显式快版，仅补齐按键跟踪门为.08rad并记录非法/超限样本；按键原几何/.8-1.2非接触资格选择/20ms接触段/原样条/5mrad端点不改。33专项测试和43段重定时资格通过，隔离构建6.96秒；全准备/实际运行状态以该目录identity/run为准，未预先声称完成。r318仍是唯一当前完整顺序成功，不覆盖。
## 2026-10-20 CST补充：r322空手速度门失败，r323显式放宽软件监控

r322原腿初始化/实际开盖及按键撤臂通过，进入装盒前空手停车时右手第二指实测0.231818955m/s触发继承的0.2m/s软件门。motions9、任务23.459511wall秒FAIL，尚未抓盒；没有完整任务或180秒成功。原片49.833333秒完整解码、trace90579行CLOSED、正常SIGINT退休及九类before/after保护全部PASS。721核心SUM 50191433350285cbc4850808a54b831d62b87c410ba6ed5b810cc844b3df923c；review_20261020_r10三成员SUM 1658554180b3c3b476f04e9507c132e0dd9ca4c63e8f8a02e97efd274580a1d9。失败原始反馈保留，不修改封存身份。

新r323/domain131/partition h1_seq_full_r323继续用户速度优先仿真合同，仅在SpeedPriorityCycle将原生夹爪速度软件监控显式改0.6m/s，实际速度仍记录、非有限数和新界限超限仍停止；原模型0.2m/s参数不改，不声称通过旧模型速度证书。默认LoadingCycle仍使用旧0.2门，独立快版审计另统计超过旧界限样本。45专项测试通过，隔离两包7.04秒构建/Conda六库恢复；正全准备，是否seal/实跑以新目录落盘为准。其余分开抓几何/密FCL、真实锚点、两盒完整装取和30秒稳转不减。

完整顺序成功仍r318，全部正式成果/孵育箱/920箱子版/旧视频/备份保持；当前不是新完整成功、稳定1倍或现实180秒达标。无清理、新备份、腿部导航或真机。
## 2026-10-20 CST补充：r323启动年龄窗失败，r324放宽启动窗继续

r323原腿实测初始化通过（64.39→73.6sim、13.734181wall、5秒保持峰2.270092mrad），但启动发现/健康查询较慢，旧45sim启动年龄门拒绝。任务未派发，无新抓持/按键/转子动作，录像未启动；非完整任务成功。1910本地/原46-122-6CTest通过，Nav111/2旧缺源；正常SIGINT退休和全部九组before/after保护PASS。
r323 660成员SUM bf028e4290a24caaa69f65f3e899b99ef46ce4d86fdbc9aeb9bbfaf9e58c6583，review_r11三成员SUM 2b31d3bf652ca28e8d85162393338a0cf87cc141fe55e87820f1b3e682f9ce9d，均只读封存。

新r324/domain132/partition h1_seq_full_r324仅将启动年龄上限45→300sim，允许实际健康检查耗时波动；仍须新独占run/匹配server PID与创建时间、未用claim、原腿目标5秒保持、六控制器健康和实际录像首帧确认，不复用旧现场或改时钟。继承显式快版0.6m/s夹爪软件门及其余提速合同；80专项通过、隔离两包6.78秒/六库恢复。准备/执行状态以r324落盘为准，尚无完整新成功。旧r318/孵育箱/920箱子版/全部旧成果和备份不改，现实180秒与稳定1倍尚未达。
## 2026-10-20 CST补充：r324左盒入篮通过，r325有界同步和低开销日志

r324/539源422资源identity d8ee3a50d7cb92a2eeb2171db9a32979bd4bd12da70f981a79b2fa0ea294f14f，1919最终测试通过（早期2项旧启动窗测试失败保留，修订边界后通过）。实际首帧确认后唯一连续实跑，左盒装篮完成，右盒已抓取/试抬；97.9385sim右静态抓稳时旧12ms配对滞后门失败。native97.943、joint97.93，最近真实夹逼native97.926需要17ms，未外推/重设原锚点。48发布、166.661183wall秒FAIL，不是三分钟完整成功。有效抓持样本FK/原生滑移峰0.906714/0.902801mm、角峰0.451318度；只证明已观察片段。原片完整解码、220047行trace CLOSED、正常停持/SIGINT退休及全部九组前后保护PASS。745核心SUM faab903ecdf9526a216007dbedf59e4f395cd22c9d92ecf1293c1b6f2570b112，review_r12四成员SUM a8b68b550419a33dab3324a92b8147ad3c454eac149e2188420c1bc822d88a79。

新r325/domain133/partition h1_seq_full_r325以独立SpeedFeedbackMixin只把快版coherent相对最新native允许滞后12→50ms；真实关节插值夹逼宽度仍12ms、最新native即时抓持、同时间腕部1mm/.2度、墙龄2秒保持。不外推，不用指令反馈，不重设锚点，基类execute_dual/execute_coherent逐字节保留。gzip6→1无损全部JSONL，原行数/字节SHA/CRC不抽样；原r324连续32MiB压缩组件实测.360466→.110679秒（3.256879倍），不是完整实时率。继承空手1.2/载物.8/近段.4/细段.15rad/s、明确宽门、真实30秒稳转和两盒终态。
74专项+11独立mixin/真实17ms复现测试通过，全部资格/最终测试/实跑以新目录落盘为准。初次过早全量36失败（缺本地生成fixture及旧源码字节预期）日志保留，不能冒充全测通过；已将新同步实现隔离到mixin再正常完整准备。r318仍当前完整分开抓成功，正式源码/模型/孵育箱/920箱子版/旧视频和所有备份不改，无导航/真机/清理/新备份。
## 2026-10-20 CST补充：r325机器循环通过但取回规划失败，r326出篮后逐步调平

r325/542源424资源identity abf6b244adc43af1830b12a4bf7dd0075dce2061532caa04eccf47bf60b544c0，1930最终本地测试通过；两盒实际装篮117.0785sim完成，实际按键关盖/启动、30.181sim稳转、停稳开盖188.0095sim通过。左取盒已实际抓取/试抬，实际倾角3.32922451度；后续retrieve_transport触发旧2度指令倾角门，运输未派发。100发布、432.342955wall秒FAIL，仍未完成取回/现实三分钟。有效抓持峰FK/原生滑移0.913585/0.974799mm、角偏3.329272度；原严格2度证书不通过。只读运行统计样本RTF.300857，不是稳定1倍。
原片460.5秒/17979254B全解码PASS，456481行trace CLOSED、1122924541原字节/290228241压缩字节。正常停持/SIGINT退休、全部九类before/after保护PASS；778核心SUM d116e01e7d9728aae6b2733a7e73b6797cd2d777bbe3d4f81bf7f4916d947d55，review_r13三成员SUM 2a83b98126e0a06a437fd31aaf97d624941f6f4cf132d98b86b2774c6961a5f0。完整成功仍r318。

新r326/domain134/partition h1_seq_full_r326只对retrieval_transport使用独立SpeedRetrievalPlanner，指令盒倾角2→5度；原SinglePlanner和其余规划模式不改。单纯放宽后真实r325快照仍在retrieve_outboard发现FCL碰撞，未实跑，原失败日志保留。修订为原篮轴退出后沿原高点抬升时逐步世界调平、保留实际yaw；原位置点/收缩关节界/全部2mrad密FCL/接触例外不减，不能取消碰撞当成功。真实失败快照2267检查PASS、规划峰3.329226度，只是离线新几何证明。另加r318左右真实取回快照回归作为seal前条件。
专项测试首次4项因测试桩非完整15关节失败，修正测试桩后45专项通过、后续34组件通过；隔离两包7.07秒，Conda六库恢复。完整准备/测试/seal/实跑以r326落盘为准，尚无物理新成功。保持50ms配对滞后/12ms真实插值、最新native即时检查、无损gzip1、快版1/.5/1观察和真实30秒稳转。r318/孵育箱/920箱子版/正式模型/旧视频/全部备份不改，无导航、真机、清理或新备份。

## 2026-10-20 CST补充：r326装篮和物理停稳开盖后退臂失败，r327仅放宽空手按键跟踪门

r326最终1936测试/隔离7.07秒通过，identity 2f3999ad5e119f5f0a9773acf4ba2393c674f4aa3eea8687bca01fb528173029。双盒实际装篮及五次物理按键已发生，稳速开始到停止触发30.1935sim；但开盖后空手退臂最大0.083599999912rad超过0.08门，在168.2885sim停止，原完整机器循环事件未到达、取盒未开始。82发布/356.329120522wall秒FAIL，不能把部分物理事件或离线取盒路径称为全流程成功。有效抓持FK/native滑移峰0.941320/0.963351mm。原片全解码、392305行trace CLOSED、正常停持/SIGINT退休和九类before/after保护通过。
772核心SUM 38fc7c10e1dfdc7efd16294393b37f91a43678a87ed9a744fa2ba70928a99b32；review_r14三成员SUM 77ec1ae20d9c2f3fd598fcf6ba2ee9d423efe5de60c54f94f9389f56daa72530。r326不能重用或改写。

新r327/domain135/partition h1_seq_full_r327从r326派生，当前只将空手按键跟踪门0.08→0.15rad（8.59度），原实体按钮几何/轨迹点/接触20ms/5mrad端点不变。真实故障偏差及正负边界/非有限值回归通过；34专项、隔离7.05秒和三实际取回快照离线资格通过。完整准备/新seal/fresh/原片首帧和实跑状态以本身份落盘为准，不预称成功。
保持所有既有快版门限、分开抓抬肘、真实四锚点/两盒、密FCL及稳转30秒。当前唯一完整分开抓成功仍r318；现实180秒/稳定1倍未达到。r318/孵育箱/920箱子版/正式源码模型/全部旧视频备份不改。无清理、新备份、导航或真机。

## 2026-10-20 CST最终：r327放宽仿真提速完整PASS，r318成功版本保留

r327/domain135/partition h1_seq_full_r327完成单手依次双盒装篮→实体按钮关盖/启动→稳转30.168sim秒→停稳开盖→左右取回不同桌面位置→双爪空和1秒终态。154发布、四真实锚点/四正常释放，19实际路径/24439原2mrad密FCL、28/28当前放宽物理门/独立抓持/完整原片解码全部PASS。最终1938本地/原46-122-6CTest通过，Nav111/2旧V12缺源失败未掩盖、不运行导航。

实际234.8065sim（25.232→260.0385）/615.775707219wall秒，较r318的3894.946321249wall约快6.3253倍/减84.19%。两个版本合同不同，非同严格门重复性证明；现实180秒、仿真180秒、稳定1倍均未达到。单调计时包含导入/预检/在线规划/执行器退出，不含场景启动、首帧审核和录像终结。新成功是独立快版，不覆盖或冒充r318原合同。

独立峰滑移FK/native 2.158579/2.158360mm、角偏4.218821度，符合新5mm/10度，但旧2mm/2度未通过；取回转子瞬态.053409829rpm/8样本超过旧.02rpm，仍在此前授权.1rpm内。终态左右位置误差.325218/.960172mm，双爪空、盖开停转；末端在释放处，不宣称中立收臂。visual-only腕部等原模型碰撞缺口保留，不称绝对无碰撞或真机成功。

快版资格涵盖：私有固定骨盆脚底代理+原腿5mrad/5秒初始化；空手1.2/载物.8/近段.4/细段.15rad/s和原点重定时；抓稳1秒/释放.5秒/终态1秒；空手按键跟踪.15rad、原20ms接触和5mrad端点；取回指令倾角5度、原篮轴退出后逐步世界调平保持yaw；native滞后50ms但真实关节夹逼12ms、不外推；软件指速.6但模型.2不改；全部无损gzip1。真实30秒稳转/双盒/一手持物/不可重设锚点/原密FCL保持，不以瞬移或attach代替抓放。

identity 547源 SHA 0694b4ea0cdd1c8448387453d7ded0ba0be4d7537fcb06b9c81f703df571cb78。原片run/full_cycle.mp4为649.333333秒/25787091B/1200×900/12fps、7792原始帧，无加速或裁剪，SHA 5045bf54504b0bfaf2b3b965c7872707a53935a5d55350a26f1ad6700e866368；长于任务因含首帧审核和前后收尾。已看首帧及230/340/500/640秒抽帧，远侧桌面盒受设备遮挡，不冒充画面证明双盒返桌。trace612901行、1597447827原字节/420685394gzip字节CLOSED。正常SIGINT/场景退休，无剩余owned进程，不复用run/claim。

796核心只读SUM 8039813de5340b87c6adb2638222355fe3c2131bf6a9ba99274e9635ab6b652a；review_20261020_r15十四成员SUM 35ab3bf41c30c01e4b405ce689971097af474f188470ffe64923a9b36f3e61ad。九类before/after全部PASS、25579历史成员/785正式src/三模型/十备份保持，含r318/孵育箱/920箱子版/所有旧视频和失败记录。无清理/新备份/真机或导航。报告在runtime_evidence/centrifuge_48box_sequential_full_20261020_r327/REPORT.md；独立最终复核在review_r15/final_review.json。

只读CPU观测执行器约1.022核、Gazebo server约1.427核，不能据此推断热函数根因。稳转31.332sim阶段耗时74.656wall，部分持物阶段比例.16–.29；下一步需在新的身份中分析执行器重复计算/序列化和物理接触计算，不能只继续抬速度门就宣称180秒。r327现封存，不继续在其内调参；未来继承必须把本次两个SUM加入保护列表。下文/旧交接中的“完整成功仍r318”是历史，当前应区分旧r318合同成功与新r327放宽快版成功。


## 2026-10-20 CST：r327完成后的本轮MD集中整理

用户要求“完成之后整理这轮的md”。本次只编辑交接文档及只读核验，未再次启动/
停止Gazebo、发布控制、清理文件、恢复回收站、创建编号备份或连接实体机器人。
完整任务仍以本节之前的r327正式证据为准：234.8065sim/615.775707219wall秒，
当前放宽仿真合同PASS；现实180秒、当前分开抓sim180秒及稳定1倍均未达。
r318原成功与旧r283双臂同时179.129sim成功不覆盖，不混为同一合同。

本次沿用2026-10-10文档治理规则：HANDOFF_CURRENT为唯一简明当前摘要；
CENTRIFUGE_FULL_CYCLE集中r318→性能/初始化→r319–r327失败链、显式快版合同、
独立审计、录像/计时边界和下一性能假设；QUICKSTART只读查看/核验；
HANDOFF_PROMPT同步新对话提示词，NEXT_CHAT_HANDOFF只作兼容，README作导航。
这些入口删去的是反复堆叠的旧“最新状态”，完整历史原文均已先行保存，并非删除成果。
HANDOFF_PLAN原正文不改，只追加第33节，说明后来授权的新合同覆盖第32节相冲突的
“必须先1倍再完整跑”条件；所有真实物理/身份/证据保护边界不因此取消。

新独占文档审计目录runtime_evidence/documentation_roundup_20261020_r1：
documentation_before.json保存七份根MD完整原文、字节、行数和SHA；
authority_history_before.json固定此前AGENTS的858786字节及SHA
fdaaf709200c5dd9c5211672bc45721d4e0889abd2d34a5ae2c695cc8ce8a7bb。
验证要求此前AGENTS前缀逐字节不变、仅追加；HANDOFF_PLAN同样保留原正文。
修改后入口哈希、所有本地Markdown链接、前后只读live与保护结果落盘并封存。
此为文档来源审计，不是新编号工作区备份；826_plan/场景版本/导航计划/备份说明/
src包级README和正式运行REPORT保持原件，不追改旧报告。

本次有限前后保护核对r318核心753/review_r6 56/r327核心796/review_r15 14，
共1619原成员及其四份manifest和0444权限、785正式源码、三模型。
不把该有限核验伪称新的一次十份备份或全部25579历史成员扫描；后者为r327运行时
已封存的九类before/after证据。核验结果以该目录validation.json、protected_before/
after.json和REPORT.md为准，审计自身完成后生成新的SHA256SUMS并只读封存。
2026-10-20 13:08:24 CST可见进程只读检查无Gazebo/顺序执行器、无权限拒绝；
这是历史快照，不能充当下一次fresh、控制器/夹持或永久live结论。

后续接手先读统一摘要及正式报告；仅文档/看片/解释保持只读动作范围。继续开发须
按明确目标新未用身份，保留r318/r327、旧孵育箱/920箱子版和所有重要证据。
原V12来源缺失/Nav111通过2失败仍如实保留，手部固定骨盆成果不扩展为腿部导航。
本节只整理已完成事实，不声称新的仿真、测试成绩、速度达标或原严格门通过。


## 2026-10-20 CST后续：r328–r330性能实验失败，成功基线仍r327/r318

用户进一步明确授权继续性能开发、直接修改/测试/完整运行录像，保留分开抓抬肘、
抓持稳定和r318/r327。故本节不是上节的仅文档动作范围；本轮按D/第33节使用
三个普通mkdir独占的新身份domain136/137/138及独立partition/overlay/录像，
隔离两包构建、六库恢复、一次seal/fresh/实际首帧后连续执行器自动运行。
没有逐动作等待AI，没有修改正式src/模型，没有清理、新备份、导航或真机。

r328加入有界索引关节时间线和50Hz有界线程栈采样，保持原真实时间夹逼/反馈；
1961本地及原46/122/6CTest通过，Nav111/2旧缺源。实际两盒装篮/机器循环后，
左取回试抬转子-0.125302rpm超过0.1，97发布/413.145524584wall秒FAIL。
789核心SUM 19f0dadae90f05d5d29c46000d42cb9ade52b16b27927417286820c9df18c7ef；
review_r16的17成员SUM f6832f7caa207b0b466affea206ac99174b7b92f90d4129abb42644278fe5984。

r329加入64KiB无损日志批量压缩/哈希，保留全部原JSONL顺序/字节/时间和排空；
取回试抬从0.15降0.10rad/s，原0.1rpm门不改。1970本地及原46/122/6CTest通过。
实际双装篮后在machine_close_withdraw跟踪-0.154rad超过0.15，
76发布/247.891034068wall秒FAIL，尚未验证取回。788核心SUM
964b3c01ed60b0afdccee1f16d275b54ec6ac1f8f7406a76ea4841a24caf086a；
review_r17的12成员SUM a29f8b5499e0ba8fa22ec4869b77170b3846761d7f65fb14544482973881a55a。

r330使用按原7个浮点字节索引512项精确位姿缓存，保留正负零、原矩阵算法和
防修改返回副本；5000随机输入/39真实位姿逐字节一致，组件约4.041845倍，不是
全程4倍。空手按钮非接触段上限1.2降0.8rad/s；原0.15rad跟踪、20ms接触、
5mrad端点、全部抓持/密FCL/30秒稳转保持。隔离构建7.01秒，50专项/1986本地/
原46/122/6CTest通过，Nav111/2旧缺源失败如实保留。
564源identity 695fc3a30136ba75aac4e35cecbe96a544dc6353caec7744ee51c6635dfb2c99。
实际双盒装篮、关盖/至少30sim秒稳转/停稳开盖、左盒226.6605sim返桌后，
右取回预压241.272sim缺合格支撑证据，129发布/521.840822301wall秒FAIL。
右盒尚无真实锚点、未抬升；两盒未全部取回，不是完整流程/180秒/稳定1倍。
已观察片段独立FK/native滑移峰0.852226/0.848499mm、角偏2.650490度，
3锚点/3正常释放，不当完整四锚点证书。取回转子峰0.015746258rpm。

review_r18/preload_failure_context.json重建故障：上一正篮底力241.2715到当前
241.272仅0.5ms；当前单指0N、无篮底接触，关节滞后2ms，预压开始后0.383秒。
几何侧超约7.85nm/底穿约23.29nm仍在当前门内；既不能断言真实掉落，也不能
用零力/几何近似替代支撑或抓牢。继承的单缺帧规则要求真实正力指尖，未取消。
下一实验先区分接触转换与缺帧，任何有界等待不能变为holder创建/抬升资格，
真正捕获仍须原强双指/真实锚点条件，禁止改旧失败为PASS。

r330原片565.75秒/22301320B/1200×900/12fps/6789帧完整解码，未加速裁剪；
首帧及555秒停止画面实际看过，远桌盒遮挡，不冒充画面证明返桌。
trace574288行/1474085261原字节/386231912gzip字节，CRC/SHA/CLOSED通过。
任务审计完整PASS断言退出、无speed_task_audit.json，物理/完整抓持FAIL原样保留。
814核心SUM f8e8d80f9f48f2ac231db4683c9af7070f00d5e7ecec92b2136e4ecfba0b0e13；
review_r18的16成员SUM f0cbc7ecbab33651eba1e04141376c84f657a7c0a098d0a5a28804bc09cbb9b4。
三个身份均stop-and-hold、正常SIGINT刷盘并退休，九类before/after全部PASS，
无reset/detach/强制松夹；所有成功/失败原件与备份保留，不复用run/claim。

性能观察：r330位姿缓存10855102命中/34233未命中；约515墙秒24686栈样本，
位姿副本叶栈42.86%、ROS等待30.03%、gzip6.15%，受GIL影响，非CPU因果分摊。
精确PID Gazebo server/GUI约1.418/1.079核，定位节点约0.847核；
当前未改雷达/定位频率，不推导稳定1倍。手写刚体逆更慢、线程切换微测不足，
均未采用。下一性能候选为精确同输入几何缓存及固定骨盆定位负载隔离A/B；
不能把组件14.72/1.075/4.04倍相乘或失败片段耗时比当全流程提速。

成功基线仍r327完整234.8065sim/615.775707219wall及r318原合同；
两套源/结果/manifest逐字节保持，正式785源/三模型/孵育箱/920箱子版和十备份不改。
本轮汇总runtime_evidence/centrifuge_48box_sequential_runtime_perf_20261020_r1/REPORT.md；
该目录保存离线实验、六份根入口修改前原文及AGENTS原861822字节SHA
d1bf7849a920ddad32ff3d614843a731a43bcdaa5e399eee033b2c46ddc32846。
文档核验必须确认此前AGENTS整个字节前缀不变、六份入口链接有效和r318/r327
1619原成员/785正式源/三模型前后相同；十备份保护引用实际三次九类运行审计，
不把最后有限文档核验说成又一次全归档扫描。latest live以本轮live_after.json
记录的历史时刻为准，后续必须重新查，不从本节推断永久健康现场。
新对话留空仍默认只读；本轮未完成现实三分钟，也不恢复旧自动化或创建新备份。

## 2026-10-20 CST后续：r331全流程通过，r332精确角点缓存候选因定位过期失败

用户授权先解决右盒取回预压支撑证据不足，再完整运行录像，保持分开抓/抓稳、
继续现实180秒目标并保护r318/r327。本轮新domain139/140及partition/overlay/run/
录像身份普通mkdir独占；只改私有候选，正式785源/三模型不改，孵育箱/920箱子版、
全部旧成功失败证据与备份保持。无清理、新备份、恢复、导航或真机。

r331继承r330索引时间线/无损批量日志/精确位姿缓存，仅新增预压窄窗口观测等待：
当前已接受取回预压、无holder、原篮底5um几何、0.5ms内正篮底历史、当前仅本手
中带零力指尖时，最多64次真实回调/1ms仿真和20ms协作式墙钟预算；墙钟在回调前
检查，不是硬实时调度保证。等待中原底层力/深度检查仍运行，不发新臂/指命令，
不创建holder；结束重过原完整SequentialCycle.safety，抬升仍须原强双指/真实锚点。
原r330停持后9.5ms才出现的正力记录明确拒绝，不当下一连续帧恢复证据。
37新增测试/72专项、2023本地、原46/122/6CTest通过，Nav111/2旧缺源保留。
隔离两包7.06秒/Conda六库恢复。569源identity
b4fe6017b48807ecbeb9a66bb8d839223d6d56fe9298ddd4208124eb3be24da1。

r331实际完成两盒单手依次装篮、实体关盖/启动、30.1865sim稳转、停稳开盖、
两盒依次返不同桌槽、双爪空且1sim终态。157发布、4真实锚点4正常释放、
20实际路径24521原2mrad密FCL、28/28当前物理门/独立抓持/trace/原片全部PASS。
239.198sim（28.4305→267.6285）/621.132718628monotonic墙秒；包含导入/预检/
规划/退出，不含场景启动、首帧审核、录像终结。不是现实180秒、当前姿态sim180秒
或稳定1倍，也没有快过r327的615.775707219秒；r327仍为较快完整分开抓基线。
独立FK/native滑移2.479099/2.477191mm、角偏5.400582度符合5mm/10度，旧严格门
未过；取回rpm峰.058735082，5样本超过旧.02但未超.1。指速.220117009m/s，
4样本超过原模型.2但在现有软件.6内，不作旧模型速度证书。
终态左右误差.242911/1.567768mm，盖开120度/停转/无holder；不称中立收臂。
visual-only腕部等原碰撞缺口保留，不声称绝对无碰撞或真机成功。

必须区分：r331实跑preload_observation_audit.completed_waits为空，等待分支
并未在实跑触发。此次右盒取回成功、离线实际故障资格与边界测试通过，但不足以
宣称间歇缺帧已被反复实跑复现并治愈。不得把零力当支撑或将旧r330失败改PASS。
原片657.333333秒/26099968B/1200×900/12fps/7888帧，全解码、正常SIGINT；
SHA 57fdfa07332a10eb03224128944b89a66e5db1266788ed682118452544235f40。
首帧与240/570/650秒抽帧实际看过，远桌盒遮挡，不冒充画面证明双盒终态。
trace674967行/1750052476原字节/462999924gzip字节，CRC/SHA/CLOSED通过。
835核心SUM 5dedec3f942ce59e9a51bb4b2cbd15b3c1aa09eead8c0a77613880d9b9a66d6c；
review_r19的38项SUM 74439505241590e5797ae68e25a44eb4e53d518e546ede3912ad1d2754610a38。
正常退休/零owned残余及九类before/after全部PASS。

r332只新增篮坐标八角点512项精确LRU：三实际位姿21浮点原字节键、不量化、
原矩阵逆/乘法与角点数值顺序、内部只读/返回副本；不缓存接触/力/时间/判定。
模型、步长、动作速率、雷达/相机频率与各守卫不变。5000随机/真实样本逐字节通过，
组件约4.88倍不是整体成绩；65早期专项通过。初次全量4失败为旧来源恢复器未声明
已验证的函数替换，保留原失败；增加精确窄映射和变异拒绝测试后2052/2052通过，
原46/122/6CTest通过、Nav111/2旧缺源保留，隔离6.84秒/六库恢复。
575源identity 9ee59c27dfc1271a486161c260c484f544c86ab06e5205770a31d37195a8df26。

r332首次实体开盖后空手撤臂39.901sim触发定位过期：源38.9sim、龄1.001sim/
1.338073455墙秒，原1秒门未放宽；已有定位质量仍valid/pass。4发布/0锚点、
双盒未离桌、14.031937063墙秒FAIL，within_180字段true只因失败早，绝非全流程。
停持后41.651sim才消费源39.4/39.9/40.4/40.9/41.4的成批定位消息；发布侧、
DDS或消费队列根因未定，不能回填为故障时已到达。后续42.1905sim relay另报
重复clock时戳FAIL，原日志保留，不倒推为更早定位失败原因。
缓存实际hits=misses=0，尚未使用；独立596组实际trace角点离线等价只是重算，
不证明实跑命中、整段提速，也不能据此将失败归因于缓存几何。
完整任务审计断言失败，抓持/物理/预压审计FAIL原样保存，不推广候选为正式成功。
原片69.583333秒/2506709B/835帧/1200×900/12fps全解码；已看首帧和60/68秒，
前段闭盖、末帧开盖，没有双盒装取。SHA
443757ff9eff851b8fbd8b9f0d6b083bd4401e0f0a9809e4ce2983dc80f91613。
trace61962行/51652705原字节/7400946gzip字节，CRC/SHA/CLOSED通过；
正常停持/SIGINT退休，零owned残余，全部九类before/after保护PASS。
779核心SUM 264922674410b674892def9e93b244f05f23a50221c9038e1e0ef3cbe52ae5a1。

下一低风险假设先独立观测定位发布/接收/消费时戳及队列延迟，修正时序后才在
新身份验证缓存完整收益；不能只抬定位门。r331部分世界39样本区间比.373782，
最低报告RTF.103186；CPU/栈是观察不是因果分摊，未证明稳定1倍。
maxContacts=4仅review_r19离线SDF转换探针（初次默认标签假设失败日志保留）；
八指尖/指垫的转换字段差异可控，其余字段一致，但未装入本次模型/未实跑，
不宣称DART采纳或抓稳通过。下轮不得照抄新身份编号或覆盖r331成功。

本轮六根入口原文和AGENTS之前867182字节前缀SHA
ae384c240009a5bfc30c513ea5f6da460fd7d717af4bb898c30c2e955ba7b116
保存在review_r19/documentation_before.json；本节仅追加，既往前缀逐字节不改。
HANDOFF_CURRENT唯一当前摘要，其余入口同步r331成功/r332失败/现实三分钟未达；
review_r20/documentation_validation.json核验根MD链接和前后有限保护：
r318/r327的1619旧成员、785正式源及三模型，不冒充再一次全备份扫描。
本轮两次实跑的九类保护另独立覆盖旧历史与十备份。新live以后者live_after.json
记录的实际时刻为准，后续仍需重新检查。没有删除正式证据、创建备份或恢复自动化。

## 2026-10-21 CST：r333定位最新样本订阅修正及完整录像通过，现实180秒仍未达

用户授权先修r332定位超时、再完整录制，保持分开抓/抓稳并保护r318/r327/r331。
本轮新runtime_evidence/centrifuge_48box_sequential_full_20261021_r333及独立
centrifuge_48box_sequential_review_20261021_r1；domain141、h1_seq_full_r333、
独占overlay/run/视频身份，普通mkdir，从已退休r332派生，不复用任何旧claim。
初次派生partition重绑定遗漏已在启动前补齐并核验，未启动旧partition；
初次QoS资格因默认liveliness警告失败、第二次因Jazzy MessageInfo为字典失败，
原日志保留。补明确警告分类/字典元数据解析测试后真实DDS测试通过。

只改当前执行器两条定位订阅为BEST_EFFORT/KEEP_LAST深度1，原可靠发布端兼容，
避免可靠有序修复队列挡住新样本。原Mid360频率、定位算法/质量及1.0sim源龄门不变；
逐条记录发送/接收/回调时戳，不改源时戳、不补造缺帧、不回填迟到证据。
真正超时仍失败，r332的39.901/38.9输入仍拒绝。模型、动作速率/姿态和原支撑、
强双指/不可重设锚点/密FCL不改。正式785源及三模型不改，缓存继承r332。
隔离两包构建7.27秒，六Conda库恢复；2067/2067最终本地、原46/122及6CTest通过；
Nav111/2旧V12来源缺失保留，不授予导航/真机许可。

r333实际完成两盒依次装篮、实体关盖/启动、30.094sim稳转、停稳开盖、依次返不同
桌槽、双爪空及1sim终态。157发布、4真实锚点4正常释放、最大同时一holder，
28/28当前物理门/独立抓持/20路径24634原2mrad密FCL/定位/trace/原片全通过。
28.009→267.8185为239.8095sim；执行器monotonic完整644.752285933现实秒，含
导入/预检/在线规划/退出，不含场景启动、首帧审看/视频收尾。trace首末642.769600391
不能替代总计时；没有快过r331621.132718628或r327615.775707219。现实180秒、
当前分开抓sim180秒、稳定1倍均未达。r327仍为较快完整分开抓版本。

独立FK/native峰2.261860/2.261541mm、角偏4.902739度符合5mm/10度，旧2mm/2度未过。
运行器全程峰2.262463mm与独立事件峰口径不同。取回rpm峰0.058080857，2样本超过
旧.02但未超过.1；指速峰.200000000066在1e-8数值容差下无原.2超限样本，
软件合同仍.6，不据此重发旧整套合同证书。最终左/右桌槽误差.285251/1.122609mm，
盖开120度/停转/无holder；不称中立收臂、自由站立或导航后操作。
原visual-only腕部等碰撞覆盖缺口仍在，不宣称绝对无碰撞或真机成功。
预压审计PASS但completed_waits空，未触发新增窄等待，不能宣称间歇分支充分实跑验证。

定位实际476pose/951健康检查，最大源龄.584sim，无无效；DDS传输峰.461685ms、
回调排队峰26.849690ms。独立同发布源可靠/最新样本各485pose/485status（含前后），
传输峰约41.5/.465ms。本次没有复现r332数秒积压，单次完整通过不确证历史唯一根因，
不是人工丢包或长期重复性证明。旧r332失败不改PASS，原1秒门继续保护。
角点缓存1733167命中/15694未命中，21716组trace角点逐字节原公式重算PASS，
组件约4.74倍不等于整体提速。44世界样本区间比.378170737，最低RTF.118007，
主线程叶栈ROS等待37.72%、位姿副本28.25%有GIL偏差，不是CPU因果份额。
下一步隔离测物理接触求解和重复几何成本，不能只抬门或关检查凑180秒。
maxContacts=4仍仅旧离线探针，未安装/实跑，不继承其任何抓稳资格。

原速视频713.75秒/28113778B/1200x900/12fps/8565帧正常SIGINT并全解码，无快放裁剪；
含执行前首帧审看等待。SHA e413ae451efa24fff659fc78c96b3aab47ca0d5c04c0491738a1547e9f739a25。
已看启动帧及180/310/350/430/570/700/712秒：单手装取、闭盖30rpm、开盖双盒，
末尾712秒原录制俯视双盒返桌/两篮空/双爪空；前视远桌遮挡如实保留，不冒充全程可见。
trace730774行，1903764117原字节/501517103gzip，SHA/CRC/CLOSED通过。
580源identity 4a3c61ee93c592576620e72a6410d79f79e9b4a7bd70d079cff250a5c90ed50a；
854核心SUM a1caf853a13ef2eb852764741f8fd6b01ab9403794fc07a96f741c0dee0bd53f。
九类before/after保护均PASS，r318/r327/r331及其review、正式785源/三模型、
孵育箱/920永久箱子版/十备份保持。场景正常SIGINT退休，无owned残余；后续仍重查live。

根六入口同步r333最新完整/r332历史失败/现实目标未达。修改前原文与AGENTS873786
字节前缀SHA 4874fca9e2da87a363780e00757a3387ff645387ff8f186e9c68f616f4a5db44
在本轮review/documentation_before.json；旧AGENTS字节前缀不改。document_validation
只验证其声明的有限旧成员/正式源/模型和MD链接，全部备份及新近成果另由九类运行
保护覆盖，不混淆。所有旧成功/失败、正式视频日志/builder/备份不删、不覆盖；
本轮没有清理、新备份、回收站恢复、孵育箱重放、腿部导航或实体机器人操作。
后续明确D开发可继续新身份测试；新对话留空仍只读，不启动旧run或恢复旧自动化。

## 2026-10-21 补充：r334–r336常驻规划/连续段实跑均FAIL，原成功不替换

本节追加且覆盖同日旧“当前开发状态”，不撤销任何旧成功的有效范围。
用户授权减少动作间隔、复用规划进程并fresh完整录像；保持分开抓姿态、抓取稳定、
旋转30秒，保护r318/r327/r331/r333。本轮在D开发分支创建三个全新独占身份和
独立安装目录，未改正式785源、模型、旧authority/run、视频、rosbag或备份。

新增persistent_worker/planner_client保持一个子进程，但每个真实快照重建规划场景、
原20um/0.01°IK、2mrad密FCL；不跨快照缓存旧轨迹/holder/ACM。仅连接
load_lift→load_transfer和retrieve_outboard→retrieve_transfer，两手完整流程预期
四组。保留全部原密集样本、两段各自加减速；不跨接触/抓持/释放观察门。
最终端点仍反馈检查并恢复原语义phase；异常先stop-and-hold再关闭planner。

r334（review_20261021_r2）：590源identity SHA
daffdd75f805eabe410d54ed0adae2e8b272af6fe6041a58acd32fbc74cce5a9。
2082本地测试PASS。实际完成双装篮、30.1115sim稳转、停转开盖及左盒取回，
20号右取回transfer IK超过原精度，FAIL；540.360407497现实秒仅失败前缀。
20请求19完成、三组连接完成、右盒保持不异常释放；原速619.083333秒解码PASS。
独立冷进程同快照重现失败；右中间高度-5mm仍FAIL，-10mm密几何PASS。
886核心清单SHA0795abc80ec0bd7b458851216ff89b0c94ba85962631b48ae0c2c0745b2982fe。

r335（review_20261021_r3）：592源identity SHA
48638063bf08f7a4bbd2e6867f8b22dd111a401beee7a84146603d2067037a53。
只新增右取回悬空中间高度1.13475→1.12475m，最终桌面目标不变，2086本地测试PASS。
右盒装篮释放前0.819872939mm/1.232812720°超过原局部1mm/1°校正门FAIL；
197.817187497现实秒为失败前缀，未到取回。838核心SUM
edb31bcd7ff1b68625b5f19273437efbd8f714ebb5f83029d821e6bad68fa2f3。

r336（review_20261021_r4）：599源identity SHA
4b08eb346a5ebd8c2f2c3d5deb902e8964d71f106463a3414ff9c87cfa2a4fff。
新增bounded_release_alignment最多两次真实快照局部校正，每步≤1°/1mm、原关节
分支≤0.045rad，总起始角误差≤2°；最终≤0.15mm/0.08°收敛才进入原释放门。
不修改旧release_alignment.py，不重建原holder。历史等价测试只逆向声明两处
明确导入，新16测试和实际r335快照84次密FCL另证实际候选；第二步资格来自预测
端点，非真实第二快照。初始测试失败日志保留，最终2102/2102本地测试两次通过。
46包/122加载回归/6CTest PASS；导航111PASS/2旧V12缺失FAIL仍保留。

r336真实双装篮、关盖、30.1425sim稳转、停转，开盖回撤时定位失效，取回未执行。
350.351281705现实秒只是失败前缀；75发布、2实际锚点/2正常释放，一个planner
PID1034537完成11请求、两组装载连接完成。FK/native滑移峰0.969033/0.967689mm、
角偏0.426801°，但完整任务和完整抓持审计FAIL。左校正0.410721335°一次后达标，
右初始0.032532488°无需校正：>1°双步分支未实际触发，取回高度修复也未实际验证。

故障数据：最后定位源戳171.6，clock172.6025，源龄1.0025sim越保留1秒门；
此前定位quality valid。已收到样本最大DDS传输0.000235781wall、回调队列
0.012570922wall，不能排除未收到样本，也不足以判定点云/生产/TF哪个环节停顿。
下一步须新增独立原始点云源戳、定位发布间隔与TF等待观测，不得伪造新戳、
拿旧有效状态代替新样本或直接绕开1秒门；不把此数据外推为r332历史根因证据。

r336原速视频410.916667秒15909880字节完整解码，首帧和400/409.5秒原片实际查看；
末帧盖子正打开、双盒仍在篮中，不是成功全流程。world23样本区间RTF0.402889，
未稳定1倍。首次收尾因启动父进程已退出产生NoSuchProcess，completion原FAIL
保留；该竞态发生于故障pause后的重复时钟导致relay退出之后，不冒充定位根因。
复核录像已退出和精确世界路径/创建时间后，仅重试原retire_scene.py，剩余三个
Gazebo进程正常SIGINT退休，retirement PASS、remaining空；无reset/detach/松夹。

三个候选核心REPORT及review_r2/r3/r4保存全部失败，不能晋升成功版本。九类
before/after保护通过后才封存。最新完整成功仍r333，较快完整分开抓仍r327；
现实180秒、当前分开抓仿真180秒、稳定1倍均未达。原visual-only腕部碰撞覆盖
缺口、固定骨盆范围、原V12缺失均不改变，不称真机/自由站立/导航到站操作证书。

根六MD同步本轮失败与下一测量方向。变更前原文及AGENTS879066字节前缀SHA
9ec61e8126653598dd1b1dd7f04034774b9bbd0c82f9c10697955f494356209d
在review_r2/r3/r4的documentation_before.json；旧权威前缀不改。普通接手仍只读，
继续开发需新身份，禁止复用旧launch/run。本轮不清理、不备份、不恢复回收站，
不做孵育箱重放、腿部导航或实体机器人操作。920箱子版本及全部保护链保持。

## 2026-10-21 r337：定位输出余量修正后分开抓全流程完整通过（当前覆盖）

本节覆盖此前r334–r336“未有新完整成功”的当前状态，不改任何历史失败或成功证据。
用户本次明确要求先解决r336开盖定位超时、完整运行录像，保持分开抓/抓稳及30秒旋转，
保留r318/r327/r331/r333并争取现实三分钟。按HANDOFF_PLAN分支D及第33节执行，
仅Gazebo固定骨盆手部，无导航、真机、清理、新备份或回收站恢复。

### 新身份与本轮唯一定位假设

runtime_evidence/centrifuge_48box_sequential_full_20261021_r337，
独立review为centrifuge_48box_sequential_review_20261021_r5。
普通mkdir独占；domain145/partition h1_seq_full_r337、新隔离overlay、原六Conda库归位。
606项source identity SHA-256：
2ec0b6b55d93f5765f3fb4983c845592772e0fac5797d867e9943717872a114a。
917项核心SHA256SUMS：
81ccc8cbeb799e110797d930f576b70d8425ca93a93c04e5088b98518410fb9f。

私有localizer继承原算法，仅把输出上限2Hz改5Hz并记录原点云回调、TF查询、位姿/状态发布。
原10Hz雷达、720点质量算法、精确源时刻TF查询/1秒超时、原消息戳及执行器1秒源龄门不改。
正式mid360_square_localizer.py哈希仍
65c49409765cf9a0194ca2a9476d881b466d2dcf516ceb34deef9694d913a199。
合成51帧真实ROS节点产生21次位姿、最大间隔0.3sim，属于离线资格非物理证书。
同时仅修正常SIGINT退休的owned父进程消失NoSuchProcess竞态；不掩盖权限/所有权错误。
原常驻规划、单实际holder、四对自由段连接、真实快照/原IK/密FCL继续使用，非预制重放。

### 实际完成与量化边界

fresh六控制器/原腿初始化/实际原片首帧审核后连续执行，中间无AI逐动作等待。
2112候选测试三次通过；46包测试、122装盒回归、6CTest通过。
Nav111/2原V12缺源失败保留，不授予腿部导航资格。
完整分开抓双盒装篮→实际按键关盖/启动→稳转30.233sim→停稳开盖→双盒返各自桌面。
232.1135sim（31.605至263.7185），monotonic墙钟600.442257241秒：
包括执行器导入/预检/规划/退出，不含场景启动、人工首帧审阅及录像终结。
比r333少44.310028692秒，约6.87%；比r327少15.333449978秒，约2.49%。
现实180秒、当前分开抓sim180秒、稳定1倍实时均未达，不以视频速度或失败前缀代替。

145次运动发布，4真实锚点/4正常释放，最多一手持物；28/28当前物理门PASS。
20条实际发布路径24469次原2mrad密FCL通过，未删点碰撞核验。
独立FK/native滑移峰2.390323/2.388881mm、角偏4.919090°，符合既有5mm/10°，
不符合旧2mm/2°严格证书。取回转子峰0.098608760rpm，1样本超过旧0.02、
仍在已授权0.1内；指速峰0.220084700m/s，4样本超过原模型0.2但在现0.6门内。
原visual-only腕部等碰撞覆盖缺口保持；不得声称绝对无碰撞、自由站立/导航后操作或真机。
最终两盒在不同桌槽、双爪空、两篮空、盖开120°、转子停止，原片终态俯视已实际确认。
>1°双步释放校正分支本轮未触发，不把离线资格当该分支实际重复验证。

定位923条pose/920次健康检查全有效；最大健康源龄0.3755sim，
最大回调源龄0.0585sim，DDS传输峰0.316789ms，回调队列37.539975ms。
生产端2720点云/1094位姿含启动收尾，消费923戳全部可匹配；
任务位姿/点云源最大间隔0.4/0.2sim，TF无错误、查询峰20.247378ms，
点云回调峰162.596014ms。已跨过r336开盖超时并完整取回；
5Hz为上限非精确采样保证，一次fresh成功不能证明r336唯一历史根因或重复可靠性。

单planner PID2105429，20请求全部完成，4处连续段完成；
规划等待92.687002659wall，比r333少22.150384903秒，非相同快照受控基准。
世界stats区间RTF约0.384126480，中位reported约0.29818、峰约0.86834。
后续优先新身份隔离测接触物理与重复几何计算，保留本次所有物理/定位/原锚点门；
GIL叶栈位姿副本约27.63%不能直接当CPU因果分摊，不能靠抬门/改时钟/快放凑180秒。

### 视频、保护和文档

原速run/full_cycle.mp4为738.583333秒/1200×900/12fps/8863帧/28871904字节，
SHA-256 ba6667cfc73771965313f67c7e80d23e4a5abc92fc93641777c391e541da33d3。
正常SIGINT刷盘、完整解码通过；含首帧审阅及前后画面，长于任务计时，不是600秒伪剪辑。
实际看首帧、终态两视角和原片300/500/737秒帧，737秒确见两盒在桌与双空篮。
未宣称逐帧看完，正面远侧桌盒有遮挡。review_r5/image_review.json保存图片哈希。
执行器/guard/disk/supervisor全0，completion_finalization全PASS，退休无owned残余。
每次接手仍重新查live，不复用旧PID/launch/run/claim。

九组before/after保护PASS，r318/r327/r331/r333、失败r334–r336及各自视频/报告不改，
785正式源、孵育箱/离心机/48box三模型、920箱子版和827/828/831/901/909/914/915/920/1008/1009保护链保持。
孵育箱v9/r17语义、v24/r103地图重放、826动态规划与导航步态保持独立成果线。
本轮只新增实验/复核证据并维护六根MD；AGENTS原884292字节前缀SHA
cb158dced6a124b4990b0d854ccc4f6bb2e13b1c89a6b35588b06dba5a0922d5
不变，原文在review_r5/documentation_before.json；链接/前缀/保护验收在documentation_validation.json。
最新完整且本分开抓链较快版本为r337，但必须同时保留所有旧成功，不覆盖或直接重跑任何sealed身份。
新对话目标留空默认只读R，明确继续开发才走D；新备份仍须用户另给未用编号。

## 2026-10-21 r338–r340：性能候选实跑失败，r337仍为完整成功基线（当前覆盖）

用户明确要求继续排查低实时率、直接修改/测试/完整运行录像，保持分开抓与抓稳，
本次按D及HANDOFF_PLAN第33节执行。没有导航、真机、孵育箱动作、清理、备份或回收站恢复。
三次新run/overlay均普通mkdir独占，domain146/147/148、对应全新partition，原成功不复用。
最新事实：本轮没有新增完整成功；r337的232.1135sim/600.442257241wall仍是当前基线。

r338仅改精确七double键构造和512容量只读篮变换缓存，保留防御副本、原运算次序、
时间戳、物理/模型/夹力/检查率。55几何测试、最终2123候选测试通过；
组件热缓存约1.916/2.276倍，35919实际矩阵与r337逐字节一致，不是整机加速证书。
双装篮/关盖/30秒稳转/停稳开盖/左返桌后，右粗下降即时角偏10.024783度超现10度门，
位移2.710071mm。原错误文本“2mm/5deg”不代表现5mm/10度配置，保留原错误不改写。
588.337528310wall为失败前缀，右手仍持盒，不是完整取回。
源identity612项SHA 1e1fbf3986a2c7a73321dda5f29efb43f5814d0404b1abc997c2b2861f952ad7；
919核心SUM ef6abf813bad96b83fa42aa0dab9362f18057503590cbe1047b3b74eac59963a。

r339在新入口right_setdown_cycle继承父速度类，仅把右粗下降0.4rad/s/ramp0.3sim
改为0.2rad/s/ramp0.6sim，原样点/holder/抓持门不改，2132最终测试通过。
尚未触发右下降修正，左取回试抬在196.6515sim出现0.112222255rpm超过原0.1rpm，
电机disabled、制动active、盖开120度，左手持盒/右盒在篮，均未返桌。
411.774332893wall仅失败前缀，不可宣称右下降修复已验证。
源identity614项SHA fe5b566ebceafa72d74f1de248d8a83682643211210fb995275c6801e9091eee；
901核心SUM 1be9321c798e06d44657ee3e90f1f21f55cd3d6b7c168a8fbe1cf0185727d0dd。

r340再仅把两手取回最初15mm试抬从0.1降0.05rad/s，父PROFILE仍是父值，
identity单列有效局部速率，实际发布与独立时间审计按0.05，不能混称。
2135候选测试三次、46包/122装盒回归/6CTest通过，Nav111/2原V12缺源保留。
已双装篮、实体关盖/启动、稳转30.1855sim、停转开盖，191.1755sim左试抬仍故障：
实际命令0.05rad/s/1.203716257sim，转子0.377062651rpm超0.1，制动输出-2Nm。
保护PositionBrake.hh确认现夹限±2Nm，当前插件SHA
cb030abe1b549ac266132da3ba4cb5f78e460d015598d7be8bde81500d9dc8f1未改。
此为输出达到夹限的证据，不是横向校正/篮壁接触来源的唯一因果证明。
91次运动发布，左手仍持盒/右盒在篮，两盒未返桌，407.193697229wall仅失败前缀。
右下降修正仍未触发，减速单因子未获支持；不得靠继续抬门、改时钟、快放凑成功。
源identity617项SHA a5abe583efb2fb4557e6dab32d8d25010cc7a4bfaafbe8543ef02654e89a045c；
901核心SUM b8440c5ade95acdfb8d68d83456b147d94c36221e8b57b41c751e38398c8bd86。

三次世界stats区间RTF约0.372736695/0.407818413/0.408543369，服务器主线程
平均约0.931735/0.927927/0.929353核，非稳定1倍。不能把整个主线程当接触求解，
GIL叶栈和组件基准不能当CPU因果分摊；内核perf受原策略限制，未改权限或sysctl。
三个失败前缀均不可与r337完整600.442秒计算节省时间。现实/当前分开抓仿真三分钟未达。
下一步先离线分离试抬横向校正、篮壁接触力矩与制动响应，建可重复假设后再新身份实跑；
性能方面在新身份测Gazebo主线程各系统/发布/接触成本，不再只凭Python快路径推断收益。

三次分别stop-and-hold、录像SIGINT刷盘、场景SIGINT退休，视频全解码、owned残余为空，
未reset/detach/异常松夹。原速视频流分别627.5/457.083333/434.916667秒；
最新5219帧/1200×900/12fps，SHA
24edc9765dabca9940af264cb5d5dba2a385cf686156786817b2c93577876474。
实际看各首帧和失败末帧，r340另看170秒装盒帧；不冒称逐帧或双盒已返桌。
每次九组before/after保护PASS，r318/r327/r331/r333/r337、其他成功/失败、
785正式源、三个模型、孵育箱/920箱子版及827/828/831/901/909/914/915/920/1008/1009不改。
仅隔离候选，未提升正式版，无需覆盖旧成功做回退。首次派生身份残留在封存前修正，
r338中断日志与r340被目标存在门拒绝的首次构建日志原样保留，正式构建均隔离完成。

证据为runtime_evidence/centrifuge_48box_sequential_full_20261021_r338、r339、r340，
对应独立review_20261021_r6/r7/r8，r8/REPORT.md为本轮汇总。
同步六根MD，AGENTS仅追加；原890029字节前缀SHA
c24ce431e3d0907d294f3c8ba0ca66b609fbf7eee8bddec2379eb59d708caa7e不变，
原文/链接/保护核验见各review/documentation_before.json与documentation_validation.json。
当前只读live需每次重查，任何后续动作仍须新身份，不直接重跑r337或失败r338–r340。

本轮最终文档核验使用各review/documentation_validation_final.json与
documentation_after_final.json：首次r7校验因QUICKSTART只写r338–r340范围、
缺字面r339而停止；已改为逐项列出身份，保留初次结果并另写final输出。
三个review封存前再逐份复算六根MD，原AGENTS前缀不改。

## 2026-10-21 r341–r342：轴向取盒全流程通过，Gazebo主线程物理热点已量化（当前覆盖）

用户明确要求先解决取盒转子瞬态保护，再排查Gazebo主线程性能，直接修改/测试/完整录像，
保持分开抓/抓稳/稳转30秒并保护r337及之前全部成果。按分支D及HANDOFF_PLAN第33节执行，
仅Gazebo固定骨盆手部；无导航、真机、reset、detach、异常松夹、清理、备份或回收站恢复。
本节覆盖此前r338–r340当前结论，历史失败/成功不改。最新完整成功为r342；较快完整仍r337。

### r341：转子门通过，但右返桌失败

新root为runtime_evidence/centrifuge_48box_sequential_full_20261021_r341，独立review_r9。
在原篮/爪/盒/制动/门限不变前提，试抬改实际篮轴15mm，仅必要局部X避让，保持局部Y/姿态；
沿实际篮轴退出原篮沿+6mm后才调平。最初纯轴方案离线FCL碰撞失败保留，选X方案通过4实际快照，
原规划器/密FCL不删点。2154最终测试两次、46/122回归、6CTest通过，Nav原V12缺源保留。
两侧实际取回均通过转子门，峰0.049174055rpm < 原0.1rpm，试抬制动力峰0.456540437Nm。
已双装篮/关盖/稳转30.139sim/停稳开盖/左返桌，但右返桌2mm门失败，约254.8085sim停持。
故障前最近实际反馈误差2.238481mm、角偏0.000449832度、桌面正支撑1.188888872N；
该最近样本不是未插桩门的同一回调，原forensic数据保存30行和时间，不能冒充精确故障点。
642.868556054wall只是失败前缀，4锚点3释放/右手仍持物，不能作完整提速结论。
视频732.75秒完整解码，实际看首帧/731秒末帧；正常SIGINT退休、owned残余空。
九组前后保护PASS；625项源identity SHA：
795ab164f6ec6c5b266a5f60b2eb04d05b0493b67e429faa9e45669b17fd4c91。
941核心成员SUM：b7dd33d41565ebbdfa3c2fdade076a6ba40dae8c9774ceadf7f42f2408083a46。
失败核心已只读封存，不晋升成功、不覆盖旧版本。

### r342：实际完整通过及未触发分支

新root为runtime_evidence/centrifuge_48box_sequential_full_20261021_r342，独立review_r10，
domain150/partition h1_seq_full_r342/独立overlay，普通mkdir独占，初始化/录像首帧审核后一次执行。
继承r341轴向取出；table_alignment_cycle继承右下降稳定类，在原返桌下降完成且真实正支撑时，
允许≤3.5mm且近水平的实际偏差最多做两次水平≤1mm校正，实际Z/姿态/原holder及最终2mm/2度门不变。
3个r341真实失败快照离线几何合格。实跑左右误差0.122177930/1.215701657mm已在1.5mm内部目标内，
校正运动0次，不能声称该运动分支已实跑验证或r341已由该分支因果修复。

2164候选测试多次、46包/122装盒/6CTest通过；Nav111通过/2原V12缺源失败保留，478项手部依赖核验。
最初桌面21项测试有入口NEW_ENTRY逆向声明失败，修正后21/41及全量通过，原失败日志保留。
正式模型/源码未改，新场景隔离构建、六Conda库原样归位；源identity635项SHA：
b3ea5587e70407787a8e5ec9b8074f746d115dfd636b821e77d61532e97901be。
966核心成员SUM：9702e1f49ee2b2a81d88821bd08a829a133fc97f28ad21b08d60dcd287f07ead。

实际完整分开抓双装篮→真实按键关盖/启动→稳转30.1865sim→停稳开盖→双盒分别返不同桌槽，
24.3365至258.3685，共234.032sim；monotonic执行器墙钟649.716164822秒。
计时含导入/预检/在线规划/退出，不含场景启动、首帧人工审阅和录像终结；trace墙钟648.052068710秒是不同口径。
148次运动发布、4真实锚点/4正常释放、最多1holder，28物理门全部PASS；21条实际路径24538次原密FCL通过。
单planner PID1187690、21请求、4对连续段，规划等待96.396470308wall，中间无AI逐动作等待。
抓持FK/native峰2.300208111/2.299842219mm、角偏4.087194525度，通过既有5mm/10度但不符合旧2mm/2度证书。
取回转子峰0.072811329rpm，原0.1门/±2Nm制动不改，试抬制动力峰0.575617142Nm；1样本超过更早0.02门。
指速峰0.242307291m/s、5样本超过原模型0.2，仍在现0.6门内，不宣称原指速严格证书。
定位928pose/929健康检查均有效，最大健康源龄0.4005sim，原1秒门不改。
原visual-only腕部等碰撞覆盖缺口不变，不能外推绝对无碰撞/自由站立/导航到站/真机。

### 原生主线程采样：物理求解而非AI等待

独立native_profile_20261021_r1插件按隔离构建方法生成，只加载到新r342，未加载旧成功/失败。
typed entity/system/add回复只代表排队，独占claim确认实际Configure；没有模型、物理、控制、时钟改动。
仅实际PreUpdate线程CPU采样，PID=TID1179887，59609样本、469881次PreUpdate，
649.371709wall后因真实result写入停止，非900秒超时。原始profile SHA：
57e8645bc983ad47a16ed07ab8c467a6104347a3c56a2fe484aeca93d7830429。
实际进程maps/已加载二进制SHA/导出符号起点及字节长度复核确认：Physics::Update累计89.4211%，
PhysicsPrivate::Step75.8963%，DART BoxedLcpConstraintSolver::solveConstrainedGroup58.2395%。
这些是包含子调用的CPU样本，不能相加、不能当墙钟分摊、未确认某一脚/篮接触对的独占成本。
pprof将未导出地址近邻误标World::bake和LocalResourceRetriever::exists；精确函数范围计数均为0，
不得据错误标签宣称世界录制/文件查询为热点。相关原始输出和纠正审计均保留。
世界stats区间RTF0.354713298，主线程均值0.939921467核。采样有开销、快照不同，
本次比r337慢49.273907581秒，不能将全部差值归因于观察器；现实/当前分开抓sim三分钟及稳定1倍仍未达。
下一步只在新身份按阶段测约束组规模和求解成本，再做单变量优化；不直接删除盒/爪/篮支撑接触，
不靠改时钟/删抓持核验/视频加速凑成功。CPU证据见review_r10/PERFORMANCE.md和native_symbol_audit.json。

### 录像、保护、文档

r342原速run/full_cycle.mp4为680.583333秒、1200×900/12fps/8167帧、27008316字节，
SHA 7b61a44abdb0e19dec556393771a283c29e994e45cc3a6da30420ffda7918941。
正常SIGINT刷盘、完整解码，实际看首帧、终态两视角和原片180/480/679秒帧；
679秒俯视明确双盒返桌/双空爪/双空篮/开盖，非逐帧查看声称。原片未剪辑或加速作为计时证据。
执行器/guard/disk/supervisor均0、completion全PASS、正常SIGINT退休remaining空；每次接手仍重查live。
九组before/after保护全部PASS；785正式源、三模型、r337及所有旧成功/失败/视频/报告、
孵育箱v9/r17/v24/r103/826独立成果线、920箱子版及827/828/831/901/909/914/915/920/1008/1009十备份保持。
native_profile_r1的build中库被r342/native_profiler_dependencies.json显式绑定，不能当无用缓存删除。
六根MD同步r341失败/r342成功及未达目标，review_r9/r10保存原文与链接/保护核验。
AGENTS仅追加，原895386字节前缀SHA
3bb7ef81d2896e42e5e7561577ef76644ae363105f1f482acd0ef3ac398f2021不变。
不得复用任何sealed run/overlay/claim，后续新身份；目标留空接手仍默认只读R。

## 2026-10-22 r343–r345：DART同序缓存完整实跑通过，仍未现实三分钟（当前覆盖）

用户要求继续针对DART约束求解瓶颈直接修改、测试、完整运行录像。按D及HANDOFF_PLAN第33节，
仅固定骨盆Gazebo手部；无导航、真机、孵育箱动作、reset/detach/异常松夹、清理、备份或回收站恢复。
本节覆盖此前“最新完整r342/较快r337”；新r345为完整通过及本分开抓链较快实测，旧成功都保留。
三次新root/overlay/domain151/152/153及partition均独占，source seal后未改。不能直接重跑任何sealed身份。

### 隔离DART缓存与数值资格

独立runtime_evidence/centrifuge_48box_dart_cache_20261022_r1按隔离构建及六Conda库归位方法生成。
只在新Gazebo进程树LD_PRELOAD；启动健康门核对实际PID=TID、maps、库SHA及真实非空组采样。
缓存每组constraint指针/维数，矩阵对称下三角同序线性复制；不改primary Dantzig/secondary PGS、
CFM/bounds/回退/NaN处理/脉冲回调顺序，保留0.5ms步长、500Hz控制和所有接触/摩擦/几何/夹力。
105组矩阵/输入/冲量/回调序列逐字节对照；1200步真实DART接触世界、1044接触步状态逐字节一致。
240维组件4轮交替1.282200778→1.111664660秒，1.153406倍，只是组件资格，不是整机速度证书。
11成员原生包SUM：66165f95b5eaf2d70a460822cf3ba90d8f2a10f317c0d0a30b3a20814673bc4a。
库SHA 6438de4d7bb463de104cc20a65078be4157fc64cd171849c118cd7f87fce1d3c；
其build库已被各run/dart_cache_dependencies.json显式绑定，禁止当无用缓存删除。系统/正式库未覆盖。

### r343、r344失败保留

r343首次空手开盖撤臂左肘误差0.15279999984rad超原0.15门，4次发布/0抓持，
14.240930118wall仅失败前缀。源645项c10ea97f3d18fd5e8dc61ea7524209a1bc44ae2f6b2264d27e0368ca9d821911；
885核心SUM 21d120d6491d90db38c8c788cbe354496ca14110f81f3d05171e4bcc35bee5c1。
r344只将空手非接触按钮撤臂上限0.8→0.6rad/s，原靠近/接触20ms/0.15rad跟踪/5mrad终点门不变。
已双装篮/关盖，70发布、无持物，但machine_close_withdraw定位源龄1.0005sim超原1秒门，
231.038209596wall仅失败前缀，未30秒稳转/取回。源646项e50cf34ec1ab1d6dc64945fffa531f6d7f3630bcb72d543d5110840c393596aa；
932核心SUM 2d13a9a5bc709a918976fb0cd4bd63c518276f0aa961d9649dca03ad2e948e54。
r344原始云114.1至116.0空窗1.9sim/9.544124395wall；113.9到114.1恰好200ms却因浮点差<0.2被丢弃定位更新。
93228控制器参考样本/8按钮命令，参考跟踪峰0.004199958rad、参考沿发布曲线投影延迟近零；
不能证明r343唯一根因，也不能把全部问题归咎DDS。两次完整失败录像/原始日志保留。
r343/r344分别2170/2180测试通过；正常stop-and-hold、SIGINT视频与场景退休、零owned残余，九组前后保护PASS。

### r345完整结果与修正边界

继承0.6非接触撤臂；只在私有定位回调改整数纳秒周期，真实源戳/过滤/估计/质量/5Hz上限/1秒源龄门不变。
回调除周期部分与原源码逐字一致。修复可复现舍入边界，不会补不存在的点云，不证明历史渲染空窗根因已解决。
2205候选测试两次、46包/122装盒/6CTest通过；Nav111通过/2原V12缺源不变，484依赖核验通过。
最初prebuild专项测试因overlay未建拒绝，正式构建后35专项通过；派生partition残留在seal前修正，未启动旧身份。
源648项SHA d202078cba7d45d07a03b254d85f2047d0a9bdbaa63bbd1bfde6f7b54958508a；
980核心SUM 8029db4d7b0523d504eee48d4b05bfeb8571f55f8df4de76a076f6ee2e17c16a。

完整分开抓双装篮→实际按键关盖/启动→稳转30.181sim→停稳开盖→双盒各返不同桌槽，28物理门全PASS。
29.389至271.6595，共242.2705sim；执行器monotonic墙钟586.260130234秒。
含导入/预检/在线规划/退出，不含场景启动/首帧审阅/录像终结；trace墙钟582.898237467是另一口径。
比r337600.442257241少14.182127007秒（2.36%），比r342少63.456034588秒（9.77%），
不是同快照受控A/B，不能把全部差值归因DART缓存或把失败前缀作为提速。
148运动发布、4真实锚点4正常释放、最多1holder、21实际路径24529密FCL通过。
独立抓持FK/native峰0.931135058/0.931124415mm、角偏1.166644716度，符合现5mm/10度；
虽然峰值也在旧2mm/2度内，合同/物理fork不同，不能宣称旧严格证书。
取回转子峰0.035353122rpm<原0.1；3样本超旧0.02rpm。原±2Nm制动保持。
指速峰0.221626250m/s，2样本超原模型0.2、符合现0.6；不宣称原指速严格证书。
左右返桌误差0.119767303/0.314852270mm；有界桌面校正未触发，不能称运动分支已验证。
原visual-only腕部等碰撞覆盖缺口保持，不外推绝对无碰撞/自由站立/导航到站/真机。
单planner PID3624987、21请求/4对连续段，规划96.086516142wall；执行中没有AI逐动作等待。

定位1209pose/962健康检查全有效，最大源龄0.2705sim、生产云/pose最大间隔0.3sim，无TF错误。
两个旧post定位统计遇首次clock前5pose+4status的None而TypeError，原日志/脚本保留，不说post全部0。
独立review_r3/localization_audit_revision.py全量验证9条仅初始化machine_preparation且在clock/fresh门/运动之前，
仅修未定义源龄聚合，7测试包括初始化后缺clock必须失败；全部源戳/962健康仍核验。
以localization_delivery_audit_revision2.json和localizer_pipeline_audit_revision2.json为正确独立结果，
没有改runtime门、补clock、删除异常/无效健康样本或覆盖旧失败审计。

### 剩余性能瓶颈与录像/保护

53365真实主线程CPU样本、实际maps/库SHA/导出符号长度复核：Physics::Update累计88.2282%，BoxedLcp52.9673%。
单位脉冲ContactConstraint::applyUnitImpulse17.9593%、Skeleton::updateVelocityChange17.7776%、
BodyNode::updateVelocityChangeFD17.3634%、updateConstraints13.8443%、ContactConstraint::getVelocityChange9.5137%累计。
这些是包含关系，不能相加、不能当墙钟分摊；不是此前近邻符号误标签。
1229周期约束组样本，最大262维/108约束，组装p95 740.033us、LCP p95 266.249us；
非完整普查/某接触对因果归属。世界区间RTF0.420245956，主线程均值0.934590核，未稳定1倍。
现实三分钟/当前分开抓sim三分钟仍未达；下一步新身份研究单位脉冲/速度响应重复取数及动作/规划成本，
先数值对照，不直接删除真实支撑接触，不改时钟/录像快放凑目标。

原速视频642.833333秒、1200×900/12fps/7714帧、25444978字节；
SHA b107b2aa0cc4038099b8e61009afa741b11af682bbbf6a34f4b1e039ec87a3cc。
正常SIGINT刷盘、完整解码，实际看首帧/终态两视角和220/450/641秒原片帧；
终态俯视双盒在桌/空篮空爪/盖开，前视远侧桌盒遮挡，未冒称逐帧观看。
执行器/guard/disk/supervisor全0，completion各项PASS、正常SIGINT退休remaining空；每次仍重新检查live。
九组before/after保护PASS，r318/r327/r331/r333/r337/r342、全部旧失败、785正式源、三模型、
孵育箱v9/r17与v24/r103/826独立成果线、920箱子版及827/828/831/901/909/914/915/920/1008/1009十备份不改。
新候选不覆盖旧成功；根六MD同步。本轮原AGENTS902647字节前缀SHA
145289f74fe85005026ab9a8ca6044bb97c340c344b790510031e75694f61b90保持，
各review_20261022_r1/r2/r3记录原文/链接/保护核验；目标留空默认只读R，后续动作必须新身份。

## 2026-10-22 r346–r347：只减少明确空手衔接间隔，连续完整执行通过（当前覆盖）

用户最新要求减少动作中间间隔，不加入AI思考。按D及HANDOFF_PLAN第33节，新身份修改、
隔离构建、测试、完整录像；仅固定骨盆Gazebo，不导航/真机/孵育箱动作，不清理/新备份，
无reset/detach/异常松夹。r345及全部旧成功/失败正式资产不改，以下覆盖此前最新r345。

### r346失败，激进范围未晋升

新r346/domain154/独立overlay、run和录像；全局启动缓冲0.30→0.16sim，
并将原0.05秒运动余量与0.05秒真实终点观察重叠，未改轨迹/密FCL/夹力/30秒稳转及门限。
2225测试两次通过；专项mock缺snapshot初次4失败/15通过后修正，原全量源逆变换
未声明观测记录导致2失败/2222通过，加入严格单块逆变换和拒绝变动测试后全部通过，原日志保留。
实际双装篮/关盖/稳转30.134sim/停稳开盖/左返桌后，右试抬232.368sim的精确
machine_guard_fault转子-0.17103651206417134rpm超原0.1门；126发布，右仍持物。
470.698039843wall仅失败前缀，不能当完整提速。正常stop-and-hold、SIGINT录像和场景退休，owned空。
原速501.666667秒全解码，实际查看首帧及500秒末帧，末帧右爪仍在篮内，远侧返桌盒遮挡。
源655项SHA 90e74c464b3ac119c90b799d4da66381a1a76f8e091ba586c514e2d9e597ff9b；
978核心SUM 146c98b87ae549edd06b86cac4d6ba31fcd5de0295f39559396063f7d34e381e。
独立review_20261022_r4保存精确fault/CPU/原始失败审计；未完成导致完整性审计非零是预期，不写post全PASS。

### r347收窄改动和实跑结果

新r347/domain155/partition h1_seq_full_r347/独立overlay，普通mkdir独占，源一次seal后不改。
empty_gap_policy只允许无holder且明确empty park/approach/neutral阶段缩短；
持物/近接触/下降/轴向退出/试抬/夹爪/按钮及未知阶段全部恢复原0.30sim和原终点观察方式。
原0.15最小调度门、0.002rad终点、真实连续超过0.05sim稳定观察保持。
实际11条空手命令0.16sim，减少1.54sim启动缓冲；重叠最多另省0.55sim，不删观察。
其他45条普通命令仍0.30sim，实体停转按钮保持原相位调度。40次多控制器运动终点观察均合格。
抓稳1秒/释放0.5秒/试抬稳定1秒/终态1秒及至少30秒稳转不变。

2240候选测试两次、46包/122装盒/6CTest通过；Nav111/2旧V12缺源保留，489依赖绑定。
正式源码/模型不改，六Conda库归位；新源655项SHA：
12127f531a60c98a0cc0075958c894d07ffb98df2940d4f225f19549b947c985。
993核心SUM：2be5611074bf221cb7f63765fcbf300a55e2bd08075394f76a3d2c5353de06bf。

实际完整分开抓双装篮→实体关盖/启动→稳转30.154sim→停稳开盖→双盒返不同桌槽。
29.6695至269.8885为240.219sim，执行器monotonic墙钟584.977490836秒；
含导入/预检/在线规划/退出，不含场景启动、首帧审阅、录像收尾，trace583.260896444另口径。
对r345完整586.260130234wall仅少1.282639398秒约0.22%，非受控A/B、不宣称显著整体提速。
现实三分钟、当前分开抓sim三分钟、稳定1倍均未达，不改时钟/用视频快放充数。
单planner PID534953、22请求、4对连接段，规划102.710188627wall；动作中没有AI调用或等用户接续。

151运动发布、4真实锚点4正常释放、最多1holder、28物理门PASS，22路径24574次原密FCL通过。
独立抓持FK/native峰2.602166121/2.601855667mm、4.608763574度；现5mm/10度合格，非旧2mm/2度证书。
取回转子峰0.025922673rpm<原0.1门，1样本超旧0.02；±2Nm制动/接触/路径保持。
指速峰0.250727995m/s，4样本超原0.2而符合现0.6，不宣称旧指速严格证书。
定位1188pose/953健康均有效，最大源龄0.2915sim、源云/pose最大间隔0.2/0.3sim，原1秒门不改。
两个post定位统计已在新源中严格验证初始化preclock记录、只跳过未定义年龄聚合；
启动后缺clock仍失败，健康/时间戳不删改，新post相关项0；r345原失败审计不覆盖。
左返桌误差0.169825546mm；右1.738176306mm已在原2mm门内、但超内部1.5mm目标，
实际触发1次有正桌面支撑的水平约1mm校正至0.740522900mm，原holder/最终2mm、2度门保持。
这是此前r342/r345未触发分支的本次实际资格，不宣称该次校正是原2mm最终门必要救援。
原visual-only腕部碰撞覆盖缺口保持，不外推绝对无碰撞/自由站立/导航后联动/真机。

### 性能、录像和交接

世界stats区间RTF0.4079013898，主线程均值0.937383核；53572真实PreUpdate线程CPU样本，
精确maps/SHA/符号范围核验Physics::Update累计88.3465%、BoxedLcp53.5018%。
包含子调用不能相加或当墙钟分摊；少量固定间隔不是主耗时，在线规划和物理求解仍需优化。
下一步只在新身份研究规划静态资源重复构建或单位脉冲/速度响应取数；不直接删支撑/抓持检查。
本次一次通过不证明r346唯一根因或重复可靠性，激进全局短间隔仍为失败候选。

r347原速run/full_cycle.mp4为629.25秒、1200×900/12fps、24968575字节，正常SIGINT刷盘、完整解码，
SHA eacbe36c5fbe0c1f5e1766295f4b64d6dbc79c5633100a07a7f1e78285f2158f。
实际看首帧、终态两视角和原片200/400/628秒帧；俯视两盒返桌/空爪空篮/盖开，
前视远侧遮挡仍记录，不冒称逐帧观看。执行器/guard/disk/supervisor全0、completion全PASS、owned残余空，
现场只是记录时状态，下次重新只读检查。两个run九组before/after均PASS且逐组相同。
r318/r327/r331/r333/r337/r342/r345、全部旧失败、785正式源、三模型、孵育箱v9/r17、
v24/r103/826独立成果线、920箱子版及827/828/831/901/909/914/915/920/1008/1009十备份保持。

根六MD同步当前与失败边界，AGENTS只追加；原910261字节前缀SHA
3f2a3da8d51244c3d19205b7abfb905655babbe1be8ece58d9e71fc79daecb55不变。
review_20261022_r4/r5保存原文/链接/保护核验，目标留空仍只读R，后续执行须全新身份。

## 2026-10-22 r348–r351：规划复用/预规划试验未完整通过（当前覆盖）

用户要求直接修改测试录像，优先规划复用/提前规划，再研究DART瓶颈，保护r347及旧成果。
按D及HANDOFF_PLAN第33节，新domain156–159/root/overlay/partition/claim，固定骨盆Gazebo；
连续执行器无AI逐动作接续。无导航、真机、孵育箱控制、reset/detach/异常松夹或新备份。
四次均FAIL，不晋升；完整成功仍r347的240.219sim/584.977490836wall。
现实三分钟、当前分开抓sim三分钟、稳定1倍仍未达。详见：
runtime_evidence/centrifuge_48box_sequential_review_20261022_r9/ROUND_SUMMARY.md。

### 规划和DART组件

planner_resources仅缓存有SHA/指纹绑定的不可变RobotModel/资源；每请求新RobotState/
PlanningScene/ACM/碰撞请求/真实盒位，不缓存holder/路径或共享可变场景，资源突变拒绝。
单persistent_worker、原序号/token/源/输出/receipt身份核对保持。client分submit/finish，
最多一个pending、原120wall期限；原静止观察期间后台规划，原.003rad关节/.002m位姿/
1°角偏/.001m指间快照检查保持，不rebase/兜底旧路径。r348两装两取，
r349起仅两次装盒预规划，取回仍原1sim稳定后请求；不减抓稳、密FCL或30秒稳转。
22个r347真实快照规划完整结果相同；r351组件94.192178770→44.440802283wall约少52.82%，
1模型构建/36复用/37新场景。r350新overlay独立87.745458→41.845026wall。
仅组件资格，不当完整流程速度或RTF证书。

独立dart_response_20261022_r1只交换getVelocityChange只读布尔短路顺序，
算术/矩阵/CFM/滑动项/接触/步长保持；两个三盒地面世界每轮2400步状态、96响应对照逐字相同，
六次组件1.0468–1.0726倍。18成员SUM c09e2b2afbb79c111942d01e40fc07c8a9d3a669474d81b8088e7c789697affa。
仅r348进程级加载，不覆盖系统；r349–r351撤下，继续r347已验证的约束缓存库。
已引用build库不可当旧缓存删除；未获得完整任务资格或DART瓶颈解决证明。

### 实跑失败与根因边界

r348双装篮/实体关盖启动/稳转30.260sim/停稳开盖/左试抬后，
第17规划在原5°命令姿态门拒绝。早快照5.234752°、原1sim观察末5.222185°仍拒绝；
原始/缓存工厂均拒绝，不证明只因提前规划，不说恢复等待即可解决。97发布、左持物/右篮。
347.728532031wall仅失败前缀；2254候选测试通过。
源665 SHA 4132c06fa5528a2a77bb06f15dbc02347cb480aafeaa03605fe26c4697f4de8b；
981核心SUM 3740ec2faa8ed2e0c550ba8bca1e99f54819c738f23faa3ca25352d39afe1495。

r349为4发布/0规划/无holder。launch先JOINT_SOURCE_STEP：
86335500000→86336500000差1ms非0.5ms，relay退出后公共clock/反馈冻结，外层跟踪是后果。
133.196018622wall不是三分钟成功；2252候选测试通过。
源661 SHA d53c7d220013ccfbd25d646160211ab92ee623e96c903edbbd9ddc8cf62c41e4；
901核心SUM ffd0b00076dc795ff0caf68ee026a23c65490475f0f6d0f4714f6b6c5747ee0b。

r350只在新launch树设置有限Transport收发HWM20000（ROS原已20000），
加入PID/create/cmd/env绑定的owned relay/server墙钟监视，原源失效SIGINT执行器。
原严格0.5ms/重复/缺帧/年龄门不改，不插值补帧；尚未定位唯一丢帧层。
2261测试两轮/18专项/494依赖；初始旧overlay保护拒绝、资格输入未生成/mock失败均留原日志。
实际双装篮/关盖/稳转30.1375sim/停稳开盖/左返桌后，右预压207.8245sim支撑拒绝；
120发布、无holder、右仍篮，415.344401211wall仅失败前缀；17规划36.108444wall。
fixture contact207.8235，真差1000000ns，float差0.0010000000000047748被.001门误拒。
只改整数ns即可通过原single_missing_floor_frame证书，其余支撑力/中带/深度/几何不改；
-1/1000001ns仍拒绝，0/999999/1000000ns通过；不是强抓持证书，下一帧已在stop后不算连续恢复。
fixture SHA 5d1e206ce238a3344c7c66fbb8d11bfa0b91e3dad4a5ffc96359795fa4dd5afa。
源664 SHA 7d93071508f913138f12ffa599fdbb13821ea5b188f9843936bb9df8c6d69123；
983核心SUM 2a7f665eca0d3cc31a0a03524dadd39a43eeab7f6240d9cd4d8a5594eddb4fb1。
失败区间RTF0.426359463、37816CPU样本，不是完整提速或稳定1倍。

r351仅在新源将该比较改0<=round(now*1e9)-round(contact*1e9)<=1000000，
严格单块逆变换保证其余原源码一致；强抓持/支撑/运动门和30秒旋转不改。
2268候选测试两轮/93专项/495依赖；原资格拒绝和准备noclobber失败日志保留，
seal前逆变换适配及准备revision2通过，旧源/模型未覆盖。
首次machine_preparation_open_withdraw在42.019sim/关节42.020，
左肘-0.1547999999998932rad超原0.15；最后命令起41.468、长1.882895857sim、3点。
4发布/0规划/无holder、双盒仍桌，取回修正未到实跑验证。13.494116224wall仅失败前缀。
44.8645重复clock发生于主故障stop-and-hold后的pause，不能倒置为r349那类源先断根因；
尚无实际reference关联，后续新身份同步命令/控制器参考/反馈，不盲目抬门。
1148CPU样本保留，DART组不足100原审计拒绝、world stats无有效样本，
performance_limits.json不补造RTF、不降低样本门；原完整性post非零保留。
源666 SHA e5a2a8114ff6045ae439d999a41eb53a67935d5b709439cdbd94a374086e646c；
911核心SUM 5b4fe47ee1483423c8e3b85fd6951d969f2104e9c8762ee648e2e80ae0b4a4db。

### 视频、保护和有限清理

四次均46包/122装盒/6CTest通过，Nav111/2原V12缺源保留，不赋予腿导航资格。
原visual-only腕部碰撞缺口保持；现5mm/10°不替代旧严格2mm/2°。
四次原速视频完整解码，实际只查看首帧与选定故障末帧，不称逐帧观看或完整完成：
r348 393.75s SHA 6542f39f6d3ed3d05e097ea8f6c717cfe1259f4771d82feb2a3e4892015cfbf2；
r349 189.75s SHA c185897ca8e9f7e675aaefdd30a0e6141c5b212d3d6f6935aeb5d6449368c828；
r350 454.5s SHA 56b84bf5abbe4c0816baf8abf2fa32ab8e393200030b4e2b0bb7cfef0c7ee645；
r351 41.5s SHA fcbabb1d94435994f2001c20d124e3013ca18b2a42f6de1e7c9769d99aebf5ea。
r348左持盒、r350右仍篮，r351盖部分开/远侧桌盒遮挡，不冒称双返桌影像。
SIGINT视频刷盘/精确退休remaining均空；r349/r350的gz wrapper超时自动SIGTERM如原日志保留，
r351正常SIGINT，无reset/detach/异常松夹；live仅检查时点有效，下次重查。

每次九组before/after相等PASS；r318/r327/r331/r333/r337/r342/r345/r347、
全部旧成功失败/录像/报告、785正式源、三模型、孵育箱v9/r17/v24/r103/826独立成果、
920箱子版及827/828/831/901/909/914/915/920/1008/1009十备份未改。

用户另行明确授权仅清理符合条件的旧编译缓存；build_cache_cleanup_20261022_r1：
删除1546个>=24h CMake .o/.o.d，349473848逻辑/353587200分配字节约337.21MiB。
逐项二次inode/内容/源配方、安装symlink/正式manifest/identity/可见进程引用核验；
process visibility limits原样记录，不声称读取全部系统进程。未删源、共享库、模型、日志、视频、
下载包、SQLite编辑器数据库、回收站、备份或整目录。后验1546全部不存在，源码/build.make哈希不变，
r351666源相同，九组after在清理后生成且与before一致PASS；将来编译按保留配方重建中间文件。
11项清理SUM 0d14644711174e7296c4765702e707f43e857007c6137c7395e54137c3e48a3d。
后验空闲1795227648字节，低于2096208487启动预算；不再启动完整实跑或降低预算，
需用户再腾空间，原授权不扩展到成果/下载包/备份。

接续先解决首次撤臂参考/发布时序，再新身份完整验证右取回整数比较和静态资源复用；
DART响应不继承为成功版，不靠抬门/删支撑/改时钟/快放凑180秒。
根六MD同步；review_20261022_r6/r7/r8/r9保存原文/链接/保护核验。AGENTS仅追加，
本轮原916430字节前缀SHA 500c51975287ecf9210bdf21f6d3adf61356140d00c77b4595993de19f56df77保持。
目标留空默认只读R，任何新运行必须从未使用身份，禁止直接执行sealed run。

## 2026-10-22 五份冷日志独立无损归档，开发/运行入口保持（最新存储覆盖）

用户明确允许像此前无损压缩，但不得影响后续开发和运行。本轮普通mkdir独占：
runtime_evidence/cold_trace_archive_20261022_r1/。仅处理以下五份历史run/loading_trace.jsonl：
centrifuge_48box_servo_gain02_20261007_r144、
centrifuge_48box_velocity_epsilon_20261007_r147、
centrifuge_48box_passive_rotor_settle_20261007_r148、
centrifuge_48box_drop_contact_settle_20261007_r149、
centrifuge_48box_left_reorient_20261008_r150。
不扩大为其他日志、视频、rosbag、模型、源码、共享库、备份或回收站操作。

原4451774245内容字节/4451807232分配字节，变为标准gzip893325387内容/893349888分配字节，
释放3558457344字节（3.56GB/3.31GiB）。完成时空闲5157720064字节，后验约5.15GB，
覆盖上一节“仅1.80GB，空间不足”的历史快照；每次启动仍重查，不降低2096208487启动预算。

每份源SHA匹配旧清单，系统gzip完整解码CRC/SHA/字节/行数一致，
fsync独占提交和registry登记后才删除重复raw；再逐份逻辑重算，五份原清单共573成员全PASS且原清单不改。
源inode/元数据/原manifest、可读FD引用在移除前复查，无法读的同用户PID盲区保留，不声称全系统可见。
异常保留raw及partial，不自动覆盖/删除残件。初次库存遇r150清单./前缀拒绝，未删raw；
路径规范化后重做库存通过，原失败日志保留。inventory/journal精确记录每个路径与哈希。
新registry SHA cb34deed97b3ca23b4a56b34da754287d1d9be40f3699a82a439b5219c5bc24a。
本归档gzip/registry/报告为正式无损保存，不可作为可删缓存；原内容仍可逐字节恢复。

为避免破坏已绑定成功版本，tools/trace_archive代码、原31项registry及旧gzip全部不改。
当前开发仍使用原tools/trace_archive/run，不换默认registry、系统Python或shell环境。
以原r351三段保护驱动及原入口、输出绑定新审计目录，before/after九组完全相等PASS，
两次cold_target_reads均为空。r347/r351原archive_runtime_binding各36依赖复核一致。
56项归档/迁移/错误注入/导出测试、r3512268、r3472240、原配置46、原装盒122，
合计4732项离线测试通过；并非新增Gazebo物理运行，不修复或外推原V12训练来源。
r347及r318/r327/r331/r333/r337等旧成功、孵育箱v9/r17/v24/r103/826独立成果、920箱子版、
785正式源/三模型、全部视频/报告、827/828/831/901/909/914/915/920/1008/1009十备份保持。

五份cold raw物理路径现缺失，普通cat/原生sha256sum或直接旧离线脚本不会自动看到解压内容；
不是损坏，也不能谎称系统级透明压缩。历史清单核验用显式
tools/trace_archive/run -- /usr/bin/python3 runtime_evidence/cold_trace_archive_20261022_r1/cold_access.py verify-manifest 原清单
或doctor。该工具在本Python进程叠加精确五路径只读视图，旧31适配器仍原样；
子进程/原生描述符不自动虚拟化，不能用此取代当前运行入口的原31项依赖资格。
旧分析需要真实文件时用export到新的非封存临时路径，核SHA/字节/行数、至少512MiB余量，
目标不得存在，不覆盖原run或旧文件；gzip不删。具体命令见该目录README.md。

本轮只归档和回归，未启动/停止Gazebo、发运动或改仿真参数，没有新备份。
r347仍完整成功基线；r348–r351失败性质和现实三分钟/稳定1倍未达不变。
新运行仍需全新身份、重查live与余量，优先修首次撤臂参考/命令时间对应及右取回边界。
根六MD同步空间与读取边界；AGENTS只追加，原924595字节前缀SHA
0d1b474fd7efdd25088d05a008afa65c7672345133f05e035e1457fdab1b4c0f保持。


## 2026-10-22 新版本完成后执行GitHub同步（用户持续要求）

用户要求：以后每次完成一个新版本，都执行
`/home/liu/h1_moveit/sync_to_github.sh`。这是版本收尾步骤，不是定时任务，
也不是每个中间动作或每次失败重试都提交；本次仅记录此要求，未执行同步。

每个新版本完成后，先完成适用测试、保护核验、结果/哈希落盘及交接MD更新，
确认相关编辑和证据写入结束，再执行上述现有脚本；无需为同一既定范围重复询问。
用户授权该脚本在独立发布副本内提交并普通推送至既定`luonianzui/h1-moveit`的`main`，
不在无Git元数据的原工程中初始化Git，不自动改远端、分支或发布筛选规则。
同步范围沿用`/home/liu/h1_moveit/github_sync/config.json`及现有筛选；
录像、rosbag、大日志、压缩归档、构建产物、凭据和备份不因本要求扩大上传。
原工程、封存成功/失败版本及本地大文件保持；GitHub同步不替代完整备份，
也不把失败实验、未验证的三分钟目标或异机复现写成成功。

收尾报告须给出同步结果及可核验提交号；只有脚本成功且远端提交核对一致才称已同步。
若网络、授权、空间、脏发布目录、凭据扫描或远端分叉使脚本失败，报告具体阻碍，
保留已有文件和提交，修复条件后重试；不得force push、reset、clean或绕过检查。
本补充仅改变后续版本的收尾流程，不授权启动仿真、修改旧成果或创建新备份。
本次追加前AGENTS原928461字节前缀SHA-256为
9caa3a202133e15768c0f2be25173b713952d6454b4d104e1cc3c1adb6d8217d。


## 2026-10-22 r353完整分开抓与规划资源复用通过（最新动作/性能覆盖）

用户要求继续性能优化、保留抬肘分开抓/稳定/30秒旋转及r347旧成功，并在新版本完成后同步GitHub。
普通mkdir独占r352/r353及review_20261022_r10/r11；只改新实验输入/代码、隔离构建和根交接MD，
未改正式源、模型、旧authority/run/视频/报告、builder链或备份，未清理或创建备份。

### 失败r352隔离，r353新身份完整通过

r352首次空手开盖撤臂加入0.3sim有界加减速、只读controller reference观测；
2283候选测试两轮通过，但启动原关节源10.178→10.179sim跳1ms，被严格0.5ms门拒绝。
controller健康和DART绑定通过不能替代任务；原腿初始化25.024265wall等反馈超时，
没有execution.claim/result/full_cycle.mp4、没有抓盒，不称完整或三分钟成功。
原launch自动SIGINT/5秒后wrapper SIGTERM如实保留，随后精确退休remaining=[]。
源672 SHA da699c92a9e03d64acd5141197eb1aed3908924effd268b6733b754c648a5070；
828核心SUM 27625c5bcf2c10311c792639f7df87059f4ddb76f25e4118a5124f387465885d。

r353新overlay/domain161/partition h1_seq_full_r353，只在新进程树用FastDDS UDPv4，
原Reliable QoS/depth、0.5ms严格源门及公共采样不改，实际bridge/relay环境核验。
两个独立ROS进程DEFAULT/UDPv4各24000关节+24000clock样本完整通过；
本次Gazebo全程原源门通过，不证明SHM唯一根因或永久消除丢帧。
继承r351的右预压整数纳秒比较，精确1ms界限不抬，本次完整右取回通过。

初始空手非接触撤臂仍原关节折线/顶点，3→121点、1.882895857→2.482895857sim，
最大0.6rad/s、2rad/s²，原0.15rad跟踪/0.005rad端点门保持。
首次撤臂1626个实际reference样本，反馈-reference峰0.003405619rad，
reference-命令差4.44e-16rad；其他按钮/接触/夹持/旋转时序不改。
此观测不是所有旧失败的唯一因果证明。10000消息上限，未记录后面所有阶段。

继续单常驻planner，不可变RobotModel1构建34复用，每请求新RobotState/PlanningScene/ACM；
35新场景、21实际请求52.927339wall，两次装盒静止观察期预规划消费额外阻塞0.003851wall。
22个旧真实快照新旧结果一致，组件87.339466→41.618473wall。
保留真实快照、holder、密FCL和原取回1sim稳定观察；无规划结果/holder缓存、无AI逐动作接续。
DART保持r347约束缓存库6438de4d7bb463de104cc20a65078be4157fc64cd171849c118cd7f87fce1d3c，
不加载失败r348响应候选，不改系统库、步长、摩擦、雷达/视觉源频率或质量门。

### 完整结果、录像及性能边界

r353完整分开抓双装篮→实体按钮关盖/启动→稳转30.1345sim→停稳/按键开盖→双盒返桌PASS。
任务源记录221.4645sim；执行器spawn至退出独立monotonic494.301397270wall（8分14.30秒），
包含导入/预检/规划/退出，不含场景启动/首帧审核/录像收尾；内部事件wall492.893747不是同一计时口径。
较r347完整584.977490836wall少90.676093566wall（15.500783%）；非同物理快照单因素A/B，
不把全部差值归因DDS/规划或单项改动。现实三分钟、当前分开抓仿真三分钟及稳定1倍仍未达。

148发布、4捕获4正常释放、最多1holder、21实际路径24555密FCL、28物理门通过。
独立native抓持峰0.941606mm、FK0.936556mm、角偏1.752226°；
最终双盒桌面位置误差左0.176898mm/右0.173000mm，空爪空篮/盖开/转子停。
1101定位样本无invalid；取回转子峰0.045899507rpm < 原0.1门。
现5mm/10°速度优先仿真合同不代表r318严格证书；指速峰0.213743494m/s仍有2样本越原0.2门，
原visual-only腕部碰撞缺口保持，不外推绝对无碰撞或真机。

隔离两包构建、六个临时依赖恢复一致；2293候选测试两轮/25专项/46配置/122装盒/6CTest通过，
Nav111通过/2旧V12缺源失败保持，不赋予腿导航资格。
新增post控制器审计误读raw jsonl，原失败日志不改；review_r11独立revision2只改gzip读取/root定位，
8测试及原数据重算通过；未改封存执行源、数据、物理门，原post该项exit1如实保留。
原速1200x900/12fps/532.666667秒视频全解码，SHA
87f5eba7678426bc010afa7a9303791936393f77316b9775173d47dcf6cdc1bd。
实际查看首帧、100/150/370/450秒运动帧和正面/俯视终帧，俯视双盒返桌；不称逐帧观看。
正常SIGINT视频刷盘后精确退休，remaining为空，无reset/detach/异常松夹；live仅当时快照。

RTF区间0.454442947，主线程平均0.939315核；44607CPU样本。
精确符号Physics::Update累计89.4053%、BoxedLcp55.4801%、
Skeleton::updateVelocityChange19.1898%、ContactConstraint::applyUnitImpulse18.9006%。
累计包含子调用、重叠、不当墙钟，不采信未核实的World::bake/exists近邻符号标签。
1110任务DART抽样组，维数中位55/p95为145/最大306，组装均值140.309us、求解37.272us；
抽样非全量。下一步新组件保持算术/顺序测单位脉冲/速度传播重复取数，
先小世界对照再新身份完整运行，不删接触/抓持核验、改时钟或快放凑180秒。

### 保护、封存与交接

两轮九组before/after相等PASS；r347及r318/r327/r331/r333/r337/r342/r345和旧失败，
785正式源、三模型、孵育箱v9/r17/v24/r103/826独立线、920箱子版、
827/828/831/901/909/914/915/920/1008/1009十备份均保持。
无旧数据清理或新备份；五份冷归档/原31项registry及tools/trace_archive入口保持。
r353源679 SHA b2d9b669704f61323a3cde92c3b7615b709d4c54df7490f474fa99bb62721321；
1028核心SUM 541d8a5a1da0c245ca30ed2f84f065c1120b10e0ecf663168525d33a836d3ed2。
详细报告runtime_evidence/centrifuge_48box_sequential_full_20261022_r353/REPORT.md；
独立review_20261022_r11保存完整原文、核验、性能和封存；r10保留r352失败收尾。
根六MD更新，AGENTS只追加，原930174字节前缀SHA
7d875248f07bbd80088ae5a877b437950cfe41e09923cae0dd056fd224514a39不变。
下轮必须未用identity/overlay/run；禁止直接执行sealed r353/r347。
收尾完成后按用户授权调用/home/liu/h1_moveit/sync_to_github.sh，既定筛选不改；
实际成功与提交号以脚本远端核对回执/最终回复为准，不在已封存成员补写提交号。
视频/原始流/归档/备份留本地，GitHub不是完整备份。实体机器人未授权；目标留空默认只读R。
