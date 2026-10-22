# H1-2 Mid-360 自主移动与导航实施计划

> 整理说明（2026-10-10）：本文件是独立定位导航成果线的历史计划，当前状态与证据统一见[HANDOFF_CURRENT.md](HANDOFF_CURRENT.md)。严格30秒/8cm驻足验收仍未通过；r214固定骨盆手部任务不能替代导航或自由站立证据，本文不授权启动或移动。

更新时间：2026-09-14 CST  
适用范围：Gazebo 仿真  
当前状态：Mid360 方墙定位、Nav2 和官方 Unitree H1-2 步态已接入完整孵育箱场景；r17
可接受基线已到站，但严格 30 秒 `<=0.08 m` 动态驻足仍未通过；v21/r31 已验证雷达地图
与固定孵育箱双循环兼容

## 1. 目标

在不破坏现有孵育箱双臂动作成果的前提下，为 H1-2 建立独立的 Gazebo 导航成果线，使其能够：

1. 使用顶部 Livox Mid-360 感知周围墙面与障碍物；
2. 建立或加载二维地图；
3. 接收导航目标并自主规划无碰路径；
4. 根据路径生成底座速度命令；
5. 在仿真环境中移动至目标点并安全停止；
6. 最终将导航速度接口替换为真实双足步态控制接口。

所有结论严格限于 Gazebo。实体机器人没有连接或执行授权。

## 2. 当前基准

### 2.1 已完成

- Mid-360 已安装在原模型预留的 `lidar_link`；
- 传感器 frame 为 `livox_mid360_frame`；
- Gazebo 原生话题为 `/livox/mid360` 和 `/livox/mid360/points`；
- ROS 三维点云话题为 `/livox/mid360/points`，类型为 `sensor_msgs/msg/PointCloud2`；
- 点云配置为 10 Hz、水平 360 度、垂直 -7 至 +52 度、`1024 x 20`；
- 环境已有内部净尺寸 `5.0 x 5.0 m` 的正方形静态围墙；
- 围墙高 `2.5 m`、厚 `0.1 m`，四面墙均具有对应 collision 和 visual；
- 当前机器人仍通过 `world_joint` 固定在世界中；
- 已新增独立 `h1_2_navigation` 包；
- 已发布 `/scan`、`/map`、`/mid360_localization/pose` 和定位质量状态；
- 增高墙和重新标定二维裁剪上限后，`/scan` 四个 90 度象限均为 `180/180` 有效值，
  总计 `720/720`；
- 已通过工作空间本地只读前缀部署 `nav2_amcl` 与 `nav2_lifecycle_manager` 1.3.12；
- 已建立固定测试链 `map -> odom -> base_footprint`，其中 AMCL 独占
  `map -> odom`；
- 已从 `(0.20 m, -0.15 m, 3 deg)` 偏置初值收敛至 `0.078 m / 0.76 deg`；
- 已部署 planner server、NavFn 和三层全局 costmap；
- 已成功计算 `(0, 0) -> (1.0, 0.8)` 的 50 点、`1.283195 m` 路径；
- 墙外 `(3.0, 0.0)` 目标以 error code 208 正确拒绝；
- 当前方墙扫描匹配器仍可独立运行，不依赖 Nav2；
- 当前没有底座速度控制器、运动里程计或 Nav2 controller 配置。

### 2.2 后续仍缺失的组件

2026-09-08 系统 `/opt/ros/jazzy` 未安装完整 Nav2；当前工作空间本地前缀只部署了
AMCL 与 planner-only 所需的 16 包闭包。后续移动和建图仍缺少：

- `nav2_bringup`；
- `nav2_controller`；
- `slam_toolbox`；
- `robot_localization`；
- 完整 Nav2 行为树与移动执行配置。

## 3. 成果线隔离

导航功能必须建立独立成果线，不得覆盖或冒用：

- v9/r17 双循环动作语义权威；
- v18/r28 固定轨迹视角重放；
- 826 AprilTag 动态规划证据；
- 备份 909 或其他受保护备份。

建议新增独立包：

```text
src/h1_2_navigation/
```

建议新增独立启动入口：

```text
h1_2_navigation/launch/navigation_sim.launch.py
h1_2_navigation/launch/slam_sim.launch.py
h1_2_navigation/launch/localization_sim.launch.py
```

导航 launch 不得直接改造成孵育箱正式动作 replay 入口，也不得自动启动任何机械臂动作。

## 4. 移动实现路线

H1-2 是双足机器人，Nav2 输出的 `/cmd_vel` 不能直接驱动腿部关节。必须在 Nav2 与机器人之间增加移动执行层。

### 4.1 路线 A：平面底座导航验证

推荐先实施，用于验证雷达、地图、定位、规划器和 Nav2 状态机。

- 为导航专用模型提供可选的自由平面底座；
- 仅在 `navigation_sim.launch.py` 中移除或禁用 `world_joint`；
- 新增 Gazebo 平面移动控制器，订阅 `/cmd_vel`；
- 发布 `/odom` 和 `odom -> base_footprint`；
- 限制线速度、角速度和加速度；
- 机械臂、夹爪和躯干保持中立，不发布相关轨迹。

该路线会让机器人模型在地面上平移或旋转，不代表真实双足行走。其结果只能称为“导航栈和平面移动验证”。

建议初始限制：

```text
maximum linear velocity       0.20 m/s
maximum angular velocity      0.30 rad/s
maximum linear acceleration   0.20 m/s^2
maximum angular acceleration  0.30 rad/s^2
command timeout               0.30 s
```

### 4.2 路线 B：真实双足步态导航

在路线 A 全部通过后实施，用于让 H1-2 在 Gazebo 中真实迈步。

- 接入 Unitree H1-2 可用的步态控制器或自行实现双足控制器；
- 将 `/cmd_vel` 转换为步态速度、落脚点和全身关节命令；
- 使用足底接触、IMU、关节状态和质心状态闭环；
- 生成可信 `/odom`；
- 增加站立、起步、停止、转向和跌倒检测；
- 对机械臂姿态与行走摆动进行协调；
- 失败时立即零速并保持安全站立或受控停止。

只有路线 B 通过足底接触、平衡和连续步态验收，才能称为“双足自主导航”。

## 5. 分阶段实施

### 阶段 0：基准冻结与离线审计

1. 记录导航开发前的源码、模型和世界文件 SHA-256；
2. 确认当前 Gazebo、控制器、支撑和夹持状态；
3. 创建新的导航包和独立 launch；
4. 不修改任何正式 authority/run；
5. 使用 `AGENTS.md` 规定的 Miniconda 六库隔离方式构建。

验收：新包能够空构建，旧动作资产和受保护备份哈希不变。

### 阶段 1：Mid-360 导航感知管线

状态：2026-09-09 已完成 `2.5 m` 高正方形围墙场景的完整 360 度 live 验收。

实测结果：

```text
raw point-cloud points             20480
raw finite returns                 8166
visible square walls               4
localizer point count              6374
final inlier fraction              100 percent
final inlier RMS                   0.018179 m
LaserScan finite bins              720 / 720
front / left / rear / right        180 / 180 / 180 / 180
cmd_vel / odom publishers          0 / 0
controller trajectory publishers   0 / 0 / 0 / 0 / 0
```

当前定位使用已知 `5 x 5 m` 方墙模型和附近初始 yaw；由于环境具有 90 度旋转对称性，
不能在无先验情况下唯一判定四个等价方向。它不会发布 `map -> odom`，该接口留给以后
RL 步态里程计接入。

1. 保留 `/livox/mid360/points` 作为原始三维点云；
2. 增加点云裁剪，去除机器人本体、地面和过高点；
3. 使用 `pointcloud_to_laserscan` 或等价节点生成 `/scan`；
4. 明确扫描高度区间、最小/最大距离和角分辨率；
5. 为 `/scan` 使用 sensor-data QoS；
6. 增加帧率、空帧、NaN 比例和时间戳诊断。

当前已验收二维切片：

```text
target frame        base_footprint
minimum height     -0.35 m
maximum height      1.25 m
range minimum       0.15 m
range maximum       15.00 m
scan topic          /scan
```

该高度范围已经根据 Mid-360 向上 `13.760 deg` 的安装俯仰、高墙回波和机器人自遮挡标定。
完整验收证据在 `runtime_evidence/mid360_tall_walls_360_scan_20260909/REPORT.md`。

验收：围墙四边在 `/scan` 中稳定出现，连续 60 秒无时间戳倒退、TF 错误或持续空帧。

### 阶段 2：TF 与里程计

状态：固定定位测试链已完成；真实运动里程计等待 RL 步态接口。

建立唯一 TF 链：

```text
map -> odom -> base_footprint -> pelvis -> torso_link -> lidar_link -> livox_mid360_frame
```

要求：

- `robot_state_publisher` 只发布机器人内部 TF；
- 移动执行层发布 `odom -> base_footprint`；
- SLAM 或 AMCL 发布 `map -> odom`；
- 禁止两个节点同时发布同一 TF；
- `/odom` 与仿真真值必须使用明确分离的话题；
- 里程计时间戳使用仿真时钟。

验收：TF 树无环、无重复发布者，静止时里程计不漂移，运动时无跳变。

### 阶段 3：平面移动执行层

1. 创建导航专用模型参数，默认保持固定，仅导航 launch 显式解锁；
2. 实现 `/cmd_vel` 到 Gazebo 平面速度或位姿执行接口；
3. 发布 `/odom`；
4. 添加速度、加速度、超时和边界限制；
5. 增加围墙碰撞停止；
6. 增加零速看门狗；
7. 禁止控制机械臂、夹爪和躯干。

验收：手动发送有限测试速度时可前进、后退、原地转向并在超时后自动停止；不得穿墙或跳变。

### 阶段 4：SLAM 建图

1. 集成 `slam_toolbox`；
2. 使用 `/scan` 和 `/odom` 建图；
3. 在 5 m 方形围墙内执行低速覆盖路线；
4. 检查回环闭合和地图尺度；
5. 非覆盖保存地图 YAML/PGM；
6. 记录地图与配置哈希。

验收：四面墙闭合，边长误差不超过 `0.10 m`，地图无明显重影或断裂。

### 阶段 5：AMCL 定位

状态：2026-09-08 已完成固定机器人模式的 Nav2 AMCL 集成和 live 验收。

当前结果：

```text
Nav2 version                       1.3.12
AMCL lifecycle                    active
LaserScan frame                   base_footprint
biased initial pose               (0.20 m, -0.15 m, 3 deg)
stationary radar updates          20
final position error              0.078 m
final heading error               0.76 deg
map -> odom publisher             nav2_amcl
odom -> base_footprint publisher  fixed test static TF
cmd_vel / odom publishers         0 / 0
controller trajectory publishers  0 / 0 / 0 / 0 / 0
```

当前无运动里程计，因此偏置静止收敛验证使用 AMCL 标准
`/request_nomotion_update` 服务。接入 RL 里程计后必须关闭固定 TF，并重新完成直行、
转向和持续运动定位验收。

1. 加载或发布已知围墙地图；
2. 配置 AMCL 使用 `/scan`；
3. 设置初始位姿；
4. 比较 AMCL 与 Gazebo 真值；
5. 测试静止、直行、转向和重新定位。

固定模式验收已通过：稳定定位误差不超过 `0.10 m`，航向误差不超过 `5 deg`。
运动模式验收仍待 RL 里程计接入。

### 阶段 6：Nav2 规划与控制

状态：2026-09-08 已完成 planner server、全局 costmap 和 NavFn 的零运动验收；
controller server、行为树导航和速度输出仍未部署。

当前 planner-only 结果：

```text
planner lifecycle                  active
inside goal                        (1.0, 0.8) / SUCCEEDED
path pose count / length           50 / 1.283195 m
outside goal                       (3.0, 0.0) / rejected / code 208
cmd_vel / odom publishers          0 / 0
controller / BT / smoother         not started
```

完整证据：

```text
runtime_evidence/mid360_nav2_planner_20260908/REPORT.md
```

配置以下组件：

- planner server；
- controller server；
- behavior-tree navigator；
- behavior server；
- waypoint follower；
- lifecycle manager；
- global costmap；
- local costmap；
- velocity smoother。

初始建议：

- 全局规划器使用 NavFn 或 Smac 2D；
- 局部控制器优先 Regulated Pure Pursuit；
- footprint 使用机器人最大水平包络，不使用过小圆形半径；
- inflation radius 至少覆盖定位误差和机器人摆动余量；
- 禁用未经验证的快速倒车、碰撞旋转和自动恢复动作；
- 所有速度继续受阶段 3 的执行层限制。

验收：在无障碍方形区域内完成至少 5 个目标点，路径不穿墙，终点误差满足定位门。

### 阶段 7：障碍物场景

在不覆盖当前方形围墙的前提下，增加独立导航测试世界：

1. 单个固定方柱；
2. 两个错位障碍物；
3. 宽通道；
4. 不可达目标；
5. 临时动态障碍物。

每种场景必须有独立名称、配置和结果目录。失败场景也要保留失败原因，不得修改碰撞体来伪造通过。

验收：可达目标成功，不可达目标正确失败；任何情况下不得碰撞或穿墙。

### 阶段 8：真实双足步态接入

状态：已完成 Isaac Lab V12 `model_17250.pt` 身份冻结、NumPy/TorchScript 非覆盖导出、
258 项 MOVE/STOP 输出对拍、78 维观测与完整 MOVE/BRAKE/HOLD/RECOVER 离线运行时移植；
默认禁用的六门速度安全门仍未启动。接口权威为：

```text
src/h1_2_navigation/RL_LOCOMOTION_INTERFACE.md
```

V12 为 50 Hz、78 维观测到 12 维腿动作的有状态策略；精确零命令会触发 STOP 状态机。
当前 Gazebo 固定根、无腿控制器、无训练等价 IMU/足接触/自由根速度观测，因此离线策略
和运行时尚未连接 Gazebo 或 Nav2 controller。

2026-09-08 零运动审计结果：12 个腿关节的运动学、位置限位和 effort 一致，总质量同为
`66.984 kg`；但 12 个 velocity limit 全部不同，28 个公共惯量只有 27 个逐值一致，
collision 为 `2 vs 22`，默认腿姿态不同，自由根、腿控制器、IMU 和足接触仍未就绪。
策略导出 gate 已通过，但 `deployment_ready=false`，所以尚未进入 Gazebo 策略加载或站立。

1. 选择并固定步态控制器版本（已完成候选身份与哈希冻结）；
2. 建立 `/cmd_vel` 适配层（已完成默认禁用的输入安全门和离线策略运行时）；
3. 配置腿部控制器和足底接触；
4. 验证站立与零速保持；
5. 验证单步、连续直行和原地转向；
6. 接入 Nav2 低速目标；
7. 增加失衡、滑移、足底失联和跌倒停止门。

验收至少包括：

- 双脚接触时序合理；
- 质心和姿态保持在安全范围；
- 无脚底穿透；
- 无持续滑移；
- 停止后稳定站立；
- 连续三次导航成功；
- 任一异常能够零速并安全停止。

## 6. 启动结构

建议统一入口：

```text
navigation_sim.launch.py
  -> Gazebo navigation world
  -> H1-2 navigation model
  -> Mid-360 PointCloud2 bridge
  -> point-cloud filter
  -> pointcloud_to_laserscan
  -> planar base or gait controller
  -> odometry / TF
  -> SLAM or AMCL
  -> Nav2
  -> RViz navigation view
```

启动参数建议：

```text
mobility_mode:=fixed|planar|biped
localization_mode:=none|slam|amcl
use_sim_time:=true
world:=square_enclosure
start_nav2:=false
```

默认值必须保持 `mobility_mode:=fixed` 和 `start_nav2:=false`，防止普通模型查看意外移动机器人。

## 7. 安全门

移动前必须同时满足：

1. Gazebo server/GUI 为本轮 fresh 实例；
2. 机器人没有夹持、支撑或附着对象；
3. Mid-360 点云和 `/scan` fresh；
4. TF 链完整且唯一；
5. `/odom` fresh；
6. 地图和 Nav2 参数哈希匹配；
7. 机械臂、夹爪和躯干 trajectory publisher 为 0；
8. 速度限制和零速看门狗已启用；
9. 导航目标位于围墙内部安全区域；
10. 用户已经明确授权本轮 Gazebo 移动。

任一门失败时不得发布非零 `/cmd_vel`。

## 8. 测试顺序

1. XML/xacro/SDF 离线解析；
2. TF 静态合同测试；
3. 点云到 LaserScan 单元测试；
4. 速度限制和超时单元测试；
5. launch 生成测试；
6. Gazebo 零运动健康检查；
7. 单轴短距离低速测试；
8. 方形路线测试；
9. SLAM 建图测试；
10. AMCL 定位测试；
11. Nav2 单目标测试；
12. 多目标和障碍物测试；
13. 三次 fresh 重复测试。

测试必须从最小范围开始。没有通过前一阶段，不进入后一阶段。

## 9. 最终验收指标

### 9.1 传感器

- Mid-360 点云帧率稳定在 8--12 Hz；
- 点云 frame 固定为 `livox_mid360_frame`；
- `/scan` 连续 60 秒无持续空帧；
- 围墙四边可被稳定识别；
- TF 无环、无重复发布者、无持续 extrapolation。

### 9.2 平面导航

- 最大线速度不超过 `0.20 m/s`；
- 最大角速度不超过 `0.30 rad/s`；
- 目标位置误差不超过 `0.15 m`；
- 目标航向误差不超过 `5 deg`；
- 不碰撞、不穿墙；
- stale command 后 0.30 秒内归零；
- 至少 5 个目标点连续成功；
- 三次 fresh 运行结果一致。

### 9.3 双足导航

- 满足平面导航全部指标；
- 无跌倒、脚底穿透或持续滑移；
- 足底接触与步态时序通过；
- 停止后能够稳定站立；
- 机器人实际通过腿部关节迈步，不能用平面滑移结果替代。

## 10. 交付物

最终至少应包含：

```text
src/h1_2_navigation/
src/h1_2_navigation/launch/
src/h1_2_navigation/config/nav2_params.yaml
src/h1_2_navigation/config/slam_toolbox.yaml
src/h1_2_navigation/config/ekf.yaml
src/h1_2_navigation/maps/
src/h1_2_navigation/test/
runtime_evidence/<new_navigation_identity>/
```

证据应包含：

- 源码和配置 SHA-256；
- 构建与测试日志；
- TF 树；
- 点云和 LaserScan 频率；
- 地图文件；
- Nav2 参数；
- 规划路径和目标误差；
- 碰撞与速度审计；
- 成功和失败场景报告；
- 必要时的 Gazebo 视频与 rosbag。

## 11. 推荐执行顺序

推荐按以下顺序继续：

1. 保持已通过的 AMCL 与 planner-only 链，不启动 controller；
2. 新建导航专用自由根模型并逐维对齐 V12 的 78 维观测；
3. 导出并校验 actor，移植完整 STOP 状态机和 12 腿位置控制器；
4. 先做精确零命令站立，不接 Nav2 controller；
5. 由 RL 状态估计发布 `/odom` 和唯一 `odom -> base_footprint`；
6. 使用 `publish_fixed_odom_tf:=false` 重验 AMCL 运动定位；
7. 获得明确移动授权后，再部署 controller、行为树并低速测试。

当前不得执行移动或导航目标；进入 RL 步态接入前仍需明确接口、速度门和本轮 Gazebo
移动授权。

## 12. 2026-09-09 初始定位行走完成状态覆盖

本节覆盖第 5、8、11 节中的历史“未接入/不得移动”状态。用户已对本轮 Gazebo 非零移动
作出明确授权，导航专用自由根模型、Mid360、IMU、双足接触、truth-aided 仿真 odom、
V12 在线控制器、六门速度监督和 Nav2 controller/BT 已完成接入。孵育箱双循环资产和
826 动态规划成果线均未修改。

最终解决的运行时问题：

1. `map -> odom` TF 未来覆盖增加到 `2.0 s`，重发率提高到 `50 Hz`；
2. 禁止不安全的 effort-to-position 终态切换，捕获姿态以 return rate `0.0` 冻结；
3. RPP `max_angular_accel=0.40 rad/s^2`，解除 `0.02 < 0.03 rad/s` 的起步死锁；
4. 下游 yaw 限制为 `0.05 rad/s`；
5. 捕获态使用 `900 N/(m/s)` 严格耗散阻尼，其他状态仍为 `300 N/(m/s)`；
6. Nav2 XY success tolerance 从 `0.08 m` 收紧为 `0.04 m`，消除动作过早成功。

最终 `r101/r102` 两个 fresh 运行连续通过 0.15 m 前向目标和各 30 秒冻结保持。主验收
Mid360 终点误差 `0.022361/0.038079 m`，定位/odom 位移差
`0.008842/0.011020 m`，最小 `up_z` `0.997345/0.995704`，最大平面速度
`0.190466/0.170551 m/s`；保持期腿目标漂移均为 0，Mid360 全部有效。

```text
runtime_evidence/nav2_mid360_goal_tolerance_repeat_r102_20260909/REPORT.md
SHA-256 3a62a1f88e8bf46e297f3583259d8cd0c3e7d678839a1afddc2b5b0cb7be9cdc
```

当前完成范围是 Gazebo 方墙内单个短距离直行目标的定位行走闭环。阶段 7 障碍物、多目标
连续路线、任意终态航向、SLAM 和实体机器人不在本次 PASS 范围，后续必须使用新身份和
新的明确移动授权继续验证。

## 13. 2026-09-10 固定开机位至孵育箱旁自动导航完成状态覆盖

本节新增完成的是不覆盖孵育箱成果的组合导航层。固定开机位为
`(-0.60, 0.00, 0.00 rad)`，孵育箱前安全等待位为 `(-0.45, 0.00, 0.00 rad)`；桌子、
绿色方块和孵育箱继续使用原位置。导航机器人补回左右夹爪的 16 个纯视觉部件，但不增加
质量、惯量、碰撞或夹爪控制关节，不能用作孵育箱操作模型。

自动导航默认 fail-closed：`movement_authorized`、`upright_assistance`、`auto_navigate`
均默认为 false。明确启用后，节点先等待 V12、Mid360 和全部 Nav2 lifecycle 健康，再
核对固定开机位、审计 `ComputePathToPose` 的路径长度和工位 keepout，最后才发送
`NavigateToPose`；运动中另行监督 Gazebo 物理根位移与走廊，终点要求物理/定位误差、
航向、直立、接触和连续 50 个稳定样本同时通过。

静态 `r5` 验收确认完整场景、固定开机位、四墙定位、`/scan.range_min=0.10 m`、工位
地图占据、门 locked、support 不存在和零命令。顶部 Mid360 在该姿态下看不到较低工位，
所以当前使用精确已知工位静态矩形和独立物理走廊作为约束，不把结果外推为未知低矮障碍
检测。

正式 `r9` 从调用到 PASS 为 `50.427610 s`，规划路径 `0.150000 m`，物理位移
`0.152063 m`，物理/定位终点误差分别为 `0.020185/0.020000 m`，最终 `up_z` 为
`0.9999863014`，双脚接触为正并进入 `CAPTURED_HOLD`。之后 `30.045255 s` 冻结保持
最大平面速度 `0.000026730 m/s`、腿目标漂移 `0.0 rad`、定位有效/无效 `30/0`。

```text
runtime_evidence/navigation_incubator_fixed_start_static_20260910_r5/REPORT.md
runtime_evidence/navigation_incubator_auto_staging_20260910_r9/REPORT.md
```

构建和最终 `84/84` 测试通过，孵育箱 v9/r17、v19/r29、源码模型和备份 909 的运行前后
哈希完全一致。本 PASS 仅限 Gazebo 固定已知工位的单个近距离目标；未知/动态低矮障碍、
多目标、SLAM、在线动态重规划、孵育箱操作衔接和实体机器人仍不在完成范围。

## 14. 2026-09-11 步态整改计划覆盖

用户已否决当前 V12 Gazebo 步态；fresh r10 同时触发物理路径包络失败并已安全停止。
因此第 12--13 节的导航 PASS 仅保留为旧适配器机器回归，不再作为合格步态或完整定位
行走的验收依据。当前无相关仿真进程，孵育箱受保护资产和 backup 909 均未改变。

后续按以下顺序推进，且每一阶段使用隔离身份：

1. 在 Unitree 官方独立 MuJoCo 场景只评估本机 H1_2 `motion.pt` 的连续步态观感；
2. 用户认可后新建 V13，不覆盖 V12，并忠实复制 47-D 观测、12-D 直接位置动作、默认角、
   关节顺序、PD 增益及 50 Hz 时序；
3. 禁用旧链的积分动作、步频缩放、15 周期突发和 50 周期重启驻留；
4. 先验收零命令站立，再验收 `0.05--0.10 m/s` 连续直行和连续停车保持；
5. 依次通过 Isaac 批量、MuJoCo sim2sim、Gazebo DART 的轨迹/接触/足滑/姿态门；
6. 只有上述门全部通过后，才把 Nav2 `cmd_vel` 接到新步态层并恢复固定开机位导航；
7. 若官方策略观感仍不合格，新训练 V13 motion-imitation/AMP 策略，并加入动力学随机化、
   足滑/步宽约束及跨仿真残差校正；MPC + whole-body control 只作为长期高成本路线。

下一项非只读动作应是独立 MuJoCo 可视化，不启动 Gazebo、不加载孵育箱，也不修改项目
源码。仍需用户明确授权后才能运行；之后任何 Gazebo 移动同样需要 fresh 场景、新证据
身份和单独授权。实体机器人不在授权范围。

## 15. 2026-09-11 官方 H1-2 步态完整场景接入完成状态覆盖

本节覆盖第 14 节的待办状态。用户已认可 Unitree 官方 H1-2 MuJoCo 步态，并授权继续
Gazebo 集成。隔离 `unitree_h12` profile 已按 47-D 观测、12-D 直接位置动作、LSTM、
官方默认角/PD 和 50 Hz 时序接入；旧 V12 的频率缩放、累计目标、动作速率限制、运动
突发和重启驻留均未复用。

完整组合 launch 同时包含方墙、Mid360、双夹爪、桌子、绿色方块和原孵育箱。固定开机位
到箱旁等待位的 Mid360 + Nav2 + 官方 RL 步态闭环已在两个 fresh 运行中到站并 PASS；
最终采用 r17 的单次 station-entry LSTM memory reset 与 `[0.10,0.08,0.05]` 到站策略
命令上限。r18 的横向 `0.10 m/s` 试验超时失败并已回退。

```text
runtime_evidence/unitree_h12_incubator_nav2_lstm_entry_reset_20260911_r17/REPORT.md
SHA-256 a097e27959c1926d32c8338ae188d5214e836a9cd1d0dabddf061290c1e673db
```

r17 规划路径 `0.149999963 m`，终端物理/定位误差 `0.060261/0.056569 m`，20 秒动态驻足
无 fault、最小 `up_z=0.999520`。到站动态摆动峰值仍为 `0.108891 m`，因此阶段 8 的
“正确步态接入、短距离单目标定位行走”已完成，但严格 8 cm 长期驻足、连续三次复验、
多目标、障碍物、任意航向、SLAM 和真机仍是后续项。

最终隔离构建通过，`112/112` 测试 PASS；全部孵育箱受保护哈希与 backup 909 不变。
当前 live 离线。再次运行必须使用 fresh Gazebo、新证据身份和明确移动授权。

## 16. 2026-09-11 r19 阻尼下降假设否决

首轮持续优化只测试驻足 `linear_velocity_gain: 0.8 -> 0.4`。fresh r19 虽通过 Nav2 的短
稳定门，但 PASS 后 30 秒内目标/走廊误差峰值达到 `0.615209 m`，最大平面速度
`0.699432 m/s`；该假设已否决并逐字节回退，当前仍是 r17 配置，`112/112` 测试 PASS。

后续顺序调整为：

1. 让 PASS 后的物理路径、姿态、定位监督继续 fail-closed，任何越界立即结束当轮；
2. 保持位置/速度增益和 `[0.10,0.08,0.05]` 上限不变；
3. 单独加入 station-entry reset 相位门，优先在 `phase=0.5` 附近执行一次 LSTM reset；
4. 每个 fresh 身份先要求 Nav2 PASS，再要求至少 30 秒连续 `0.08 m` 物理目标包络；
5. 只有连续三个身份同时通过，才更新正式完成状态。

r19 证据目录为
`runtime_evidence/unitree_h12_incubator_nav2_reduced_velocity_damping_20260911_r19/`，不得复用。

## 17. 2026-09-11 持续 30 秒 fail-closed 验收完成

计划第 16 节第 1 项已落地：短稳定门不再直接生成 PASS，而只进入
`VALIDATING_LONG_HOLD`。连续 30 秒必须逐样本通过定位、8 cm 物理/定位目标误差、航向、
`up_z>=0.98`、高度 `>=0.95 m`、动态驻足和支撑合同；通过后 PASS 仍继续监督。隔离构建
和 `113/113` 测试通过，controller/config 保持 r17，只有自动验收器和测试发生变化。

r20 是 Gazebo 启动前的 recorder Python 预检失败。fresh r21 已证明新的整体链不会把一次
Nav2 action success 当作 PASS：到站后 `0.520142 s` 物理走廊越界并 FAILED，峰值物理
目标/走廊误差 `0.152793/0.149483 m`，随后出现 fall-envelope latch。r21 不计入验收，且
因过早失败尚未 live 覆盖 `VALIDATING_LONG_HOLD`。

下一轮只验证一个步态变量：等待 gait phase 接近 `0.5` 后再执行一次 station-entry LSTM
reset，保持增益、策略命令限位、Nav2 和场景不变。若 phase 等待或 reset 造成任何故障、
走廊越界或长驻足失败，必须回退；只有全新身份连续完成 30 秒严格包络才计为 1 次 PASS，
累计 3 次前不得宣告完成。

## 18. 2026-09-12 r22 相位门试验结果

第 17 节的相位门单变量已执行并否决。实验在 `phase=0.5 +/- 0.026` 才允许一次 LSTM
reset，115 项测试通过；fresh r22 实际正好以 phase 0.5 进入驻足，Nav2 成功但 0.275 秒
后越过 8 cm，21.630 秒后走廊越界 FAILED。期间定位持续有效、姿态高度合格且无 fall，
但从未进入 30 秒长驻足验收，因此计数仍为 0。

实验改动已完整回退并以 `113/113` 测试确认；r21 长驻足监督继续作为当前验收器。下一轮
只把 Nav2 `goal_checker.xy_goal_tolerance` 从 `0.08` 收紧到 `0.04 m`，以在切换动态驻足
前增加至少约 4 cm 的目标裕量。不得同时改变 gait gain、策略限位、reset 或 station anchor。
若 Nav2 timeout、走廊/姿态门失败或 30 秒持续门失败，则回退该容差；只有 fresh run 完整
长门 PASS 后才开始同参数连续复验。

## 19. 2026-09-12 r23 位置容差试验结果与下一门

第 18 节的单变量已经执行并否决。`0.04 m` goal tolerance 让 fresh r23 在物理误差
`0.033634058 m` 时完成 Nav2，但没有限制交接速度：第一条 station-keeping 样本平面/
横向速度为 `0.253626616/-0.248743008 m/s`，反向横向策略命令立即饱和。到站后仅
`0.610307 s` 就越过 8 cm，`1.619540 s` 后走廊越界 FAILED；没有进入 30 秒长门，验收
计数仍为 0。参数已回退到 `0.08 m`，隔离重建与 `113/113` 测试通过。

runner v2 因后台 launch 父进程继承忽略 SIGINT 而不能正常收尾，延迟期间产生 joint-limit、
fall 和定位丢失；v2 永久退出后续运行。signal-reset runner v3 的无害父 PID SIGINT 探针
通过，但下一次 Gazebo fresh run 必须把“父进程正常退出、无孤儿、无 shutdown rcl error”
列为并行基础设施门。

下一轮只修改 Nav2 goal checker：

1. Unitree launch 的 `xy_goal_tolerance` 保持 `0.08 m`；
2. `goal_checker.plugin` 从 `nav2_controller::SimpleGoalChecker` 改为本地已确认存在的
   `nav2_controller::StoppedGoalChecker`；
3. 仅显式新增 `goal_checker.trans_stopped_velocity=0.15 m/s`，其他 Nav2 与 gait 参数不变；
4. 先静态测试参数解析、插件加载和 runner v3 信号门，再用普通 `mkdir` 创建 fresh 身份；
5. 必须同时通过 Nav2、连续 30 秒 8 cm 包络、姿态/高度/定位/支撑、无 fault/fall 和无
   orphan 收尾，才计 1 次严格 PASS；
6. 任一门失败即 stop-and-hold、正常收尾、回退插件配置，不得保留失败参数。

r23 证据目录不得复用：

```text
runtime_evidence/unitree_h12_incubator_nav2_xy_goal_tolerance_004_20260912_r23/
```

## 20. 2026-09-12 r24 StoppedGoalChecker 试验结果与下一门

第 19 节的单变量已执行并否决。fresh r24 确认
`nav2_controller::StoppedGoalChecker` 成功加载，配置仍为 `0.08 m` XY tolerance 和新增的
`0.15 m/s` translational stopped velocity。但低层 controller 直接以 gated command 精确
为零作为 station transition：它比 Nav2 action-success callback 早 `0.006020 s` 进入
station keeping。第一条 station 样本虽只有 `0.107827809 m/s`，anchor error 已达
`0.084899912 m`；callback 时物理误差也已为 `0.104127277 m`。之后 `1.431943 s` 走廊越界
并 FAILED，未进入 30 秒长门，验收计数仍为 0。

本轮结论是 action goal checker 不拥有 gait handoff，后续不得重复收紧 Nav2 位置容差或
stopped velocity。插件改动已回退，隔离重建与 `113/113` 测试通过。r24 证据目录不得
复用：

```text
runtime_evidence/unitree_h12_incubator_nav2_stopped_goal_015_20260912_r24/
```

runner v3 的 signal-reset 解决了 launch 父 PID 忽略 SIGINT 的问题，但 `ros_gz_sim` shell
wrapper 仍遗留 Ruby Gazebo 子进程，所以 v3 退出使用。runner v4 已静态/无害 session 探针
通过，下一 fresh run 必须首次验证其“真实 Gazebo 单 PID SIGINT、launch 父 PID SIGINT、
全 session 残留 0”门。

下一轮只实现一个低层 transition-state 假设：

1. `MOVE` 后外部命令首次精确归零时，不立即清除 motion reference，不立即 reset LSTM；
2. 保持原 recurrent state，以零前馈继续现有 pose-tracking，状态标记为 settling；
3. 只有平面速度 `<=0.08 m/s` 且 motion-reference tracking error `<=0.04 m` 连续 5 个
   50 Hz 样本，才捕获 station anchor、执行一次 reset 并进入 station keeping；
4. 不改 Nav2、policy weights、tracking/station gains、策略命令限位、场景或 30 秒长门；
5. 先补 transition、连续计数、恢复移动和 fail-closed 单测，按六库隔离构建；
6. 用普通 `mkdir` 创建全新身份，同时通过 strict 30 秒包络和 runner v4 无残留收尾才计
   1 次 PASS；任一失败立即回退，不得把试验代码留作正式状态。

## 21. 2026-09-12 r26 双条件交接结果与下一组合门

第 20 节已执行并否决。r25 因 Gazebo 前的 Python 解释器绑定预检失败而封存；使用正确
`/usr/bin/python3` 的 fresh r26 通过 focused `20/20`、隔离构建和运行前 `116/116` 测试，
完整场景、Mid360、Nav2 lifecycle、规划和 action 均成功。

r26 没有找到可执行的严格交接窗口：175 条 settling 样本中速度门满足 47 条、tracking
error 门满足 28 条、共同满足 0 条。入口 motion reference 离导航目标
`0.103319675 m`，比当时物理 root 的 `0.075754276 m` 更差，并被 `0.10 m` leash 随漂移
移动。Nav2 action 后 `7.861665 s` 走廊越界，最大物理目标误差 `0.177695692 m`；姿态、
高度、支撑和定位有效，但未进入长门、未 PASS，验收计数仍为 0。实验已全量回退，隔离
重建 `113/113` PASS。

下一轮采用一个由两份失败证据支持的组合假设，而不是放宽安全门：

1. Nav2 `xy_goal_tolerance` 从当前 `0.08 m` 设为 r23 使用过的 `0.04 m`；
2. 同时恢复 r26 的低层 settling，仍要求 speed `<=0.08 m/s`、tracking error
   `<=0.04 m` 连续 5 个 50 Hz 样本；
3. 不改变官方 policy weights、station/motion gains、命令限位、LSTM reset 语义、场景或
   30 秒 fail-closed supervisor；
4. 依据：r23 的 station anchor 离目标 `0.015969416 m` 且 tracking error
   `0.024892196 m`，但速度为 `0.253626616 m/s`；r26 则拥有动态门，却以离目标
   `0.103319675 m` 的 reference 开始；两项分别提供空间和动态裕量；
5. focused/完整测试与六库隔离构建必须先 PASS，再用普通 `mkdir` 独占创建新身份；
6. fresh run 必须同时通过 Nav2、连续 30 秒 8 cm 目标包络、走廊、姿态、高度、定位、
   支撑、无 fault/fall、无孤儿且 Gazebo clean exit，才计 1 次 PASS；
7. 任一失败立即回退两项实验改动，不得简单提高速度门、放宽 tracking error 或增加驻足
   速度阻尼。

r25/r26 均不得复用：

```text
runtime_evidence/unitree_h12_incubator_nav2_settling_handoff_20260912_r25/
runtime_evidence/unitree_h12_incubator_nav2_settling_handoff_20260912_r26/
```

## 22. 2026-09-12 r27 组合门结果与下一减速门

第 21 节组合假设已执行并否决。fresh r27 通过 focused `20/20`、隔离构建和运行前
`116/116` 测试，Mid360、Nav2 规划和 action 成功；物理 root 在 callback 时仅离目标
`0.012776665 m`。但 9 条低层 settling 样本没有一条满足速度或 tracking-error 门，首条
即为 `0.300232447 m/s / 0.064263944 m`，并从一个离目标 `0.075303879 m` 的 motion
reference 产生饱和纠偏。callback 后 `0.621643 s` 走廊越界，峰值物理误差/速度
`0.151577153 m/0.824428799 m/s`、最低高度 `0.925956763 m`，因此未进入 30 秒门、未 PASS。

组合改动已逐字节回退，隔离重建 `113/113` PASS；当前仍是 r17 gait、r21 持续监督、
`SimpleGoalChecker/0.08 m` 和完整孵育箱场景。runner v4 无孤儿但 Gazebo SIGINT 仍
segfault/`-2`，clean-exit 门未完成。证据目录不得复用：

```text
runtime_evidence/unitree_h12_incubator_nav2_xy004_settling_handoff_20260912_r27/
```

下一轮只测试正常授权零命令的减速连续性：

1. `VelocityGate.apply()` 在所有授权门通过且 fresh 输入进入 deadband 时，不再立即清零；
2. 使用现有线/角 acceleration limits 将上一条 authorized output 逐步收敛到 exact zero；
3. 任一权限门失败、watchdog stale、无效输入或 locomotion fault 仍立即 exact zero；
4. 保持 Nav2 `SimpleGoalChecker/0.08 m`、official policy、gains、limits、controller、scene
   和 30 秒 fail-closed supervisor 不变；
5. 先补正常 slew、授权撤销、watchdog 和非有限值的单元测试，再六库隔离构建；
6. 只用普通 `mkdir` 创建 fresh 身份，严格 30 秒门和无孤儿/clean Gazebo exit 全通过才计
   1 次；任一失败立即收尾、回退，不得与 r26/r27 settling 组合。

依据是 r17、r22、r23、r24、r26、r27 的安全速度 trace 都显示 nominal authorized 输出在
`0.078--0.198 ms` 内从非零跳到 exact zero，而对应物理交接速度约
`0.108--0.300 m/s`。这只是待验证的共同变量，不代表减速已经证明有效。

## 23. 2026-09-14 r28 完整场景展示结果

用户视角 fresh r28 未引入参数或源码改动，只展示当前 r17/r21 基线在完整孵育箱 GUI 场景
中的 Mid360 + Nav2 导航。Nav2 action 成功，终端物理/定位误差
`0.070414705/0.055000000 m`；定位全程有效且机器人保持直立。

严格验收仍失败：Nav2 callback 后物理误差峰值 `0.126195152 m`，监督器在
`25.085504351 s` 后以 `STABLE_HOLD_TIMEOUT` fail-closed，未进入 30 秒长门。r28 仅作为
可视化和基线复现证据永久封存，不计 PASS：

```text
runtime_evidence/unitree_h12_incubator_nav2_user_view_20260914_r28/
```

七项集成哈希和六项孵育箱/backup 909 保护哈希均未改变，最终 live 离线。后续计划不因
本次展示改变：仍只准入第 22 节的 authorized deadband zero-command slew 单变量试验，
先补 fail-closed 单测和隔离构建，再以全新身份验证；不得叠加其他调参。

## 24. 2026-09-14 Mid360 地图内孵育箱双循环兼容性验证

fresh v21/r31 已证明完成的四墙 Mid360 地图、雷达 ROS 观测/定位与既有孵育箱固定动作可
在同一 Gazebo world 中共存。组合场景包含 `5 x 5 m`、高 `2.5 m` 四墙、H1-2 Mid360、
双夹爪、桌子、方块和孵育箱；完整双循环调用至 PASS 为 `167.302430689 s`。终态定位
valid、720 点、4 墙、inlier RMS `0.017826493411 m`，动作终态和 60 条控制消息全部通过。

```text
runtime_assets/candidate3_roundtrip_831_authority_v21/
runtime_assets/candidate3_roundtrip_831_r31/RUN_SUMMARY.md
```

该运行中的 Mid360/localizer 只观测，机器人没有执行 Nav2 腿部导航；因此它不增加严格
30 秒导航驻足 PASS 计数，也不证明边走边操作。导航生产基线和下一假设保持不变：仍只
准入第 22 节的 authorized deadband zero-command slew 单变量试验。v21/r31 不修改导航
源码、policy、gain、limit、goal checker 或 supervisor；914 早于本运行，不包含该证据。

## 25. 2026-09-14 开箱操作站位与视觉精对准交接

用户要求把开箱动作所需站位作为最终导航目标，并用箱体 AprilTag 到把手的刚性偏移吸收
Mid360/Nav2 的小量到站误差。该组合工作拆成两个不能混淆的合同：

1. 导航合同：从规定开机位安全到达名义操作站位并完成严格驻足；固定动作名义 robot
   root 约为 `(0,0,0)`，当前 `(-0.45,0,0)` 只作为等待/接近点，较长导航尚未验证；
2. 操作合同：驻足后由 Tag 10/12 和 exact-stamp TF 求 `T_pelvis_cabinet`，结合门/把手
   运动学求实际把手 pose，再运行 826 动态 IK 与碰撞规划。不得把固定关节轨迹整体平移。

本轮只读诊断发现 826 dynamic scene 已拥有正确 pelvis 相对碰撞几何，但 worker 仍以
world 中物体偏移计算目标 delta，尚不能补偿导航后 pelvis 残差。下一步先离线修复并跑
基座残差 plan-only 矩阵；不得与导航 deadband slew 调参同时混做。Gazebo 联调、移动和
操作执行仍需新的明确授权与全新身份。

## 2026-10-12晚：固定骨盆离心机速度支线，不是导航驻足完成

最新完整固定骨盆双盒r227461.299sim秒/27门PASS，全窗口录像修复/17相位抽帧主体可见，
尚未达180sim秒；r228500Hz小幅空臂组件0.8率/0.0064rad跟踪通过，未操作设备或载荷。
两者均正常SIGINT退休，r229准备新的带载/按钮时序候选。详情以各REPORT与AGENTS末节为准。
用户本线程已授权连续修改测试实跑至完整180，不等待每步“继续”；硬门/3/10/30秒不变。
正式导航源码/策略/孵育箱操作成果和所有备份保护，原导航111/2V12缺源事实仍保留，
严格Nav2驻足PASS仍0，不能把固定world→pelvis组件/手部任务称为Nav2到站自动操作。

## 2026-10-15：手部离心机三分钟完成，不扩大导航资格

独立手部支线r283完整双盒装篮/关盖/30.258sim秒稳转/停稳开盖/双返桌通过，
首非空至完整PASS179.1290sim秒，28/28/双持物/原片/保护全PASS，已正常退休。
固定骨盆x=.12、设备x=.52，不证明RL步态、严格30秒驻足、Nav2到站视觉联动或真机。
正式导航源/policy与孵育箱成果保持；原111通过/2项V12原缺源仍保留，不伪造恢复。
导航后Tag刚性偏移动态规划仍按独立合同开展；本轮未发布腿部导航动作。
完整操作证据与边界见CENTRIFUGE_FULL_CYCLE.md及r283 REPORT.md，旧状态只作历史。
