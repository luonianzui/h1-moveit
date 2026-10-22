# H1-2 Nav2 与 V12 RL 步态接口契约

更新时间：2026-09-09 CST  
范围：Gazebo 仿真设计与离线运行时集成；当前不授权腿部运动

## 1. 选定策略身份

后续双足执行候选固定为 Isaac Lab V12：

```text
task        H12-Human-Omni-Recover-Stop-Gait-Flat-v12
checkpoint /home/liu/robot/gym_sim/IsaacLab-3.0-beta2/logs/rsl_rl/
           h1_2_human_omni_recover_stop_gait_v12/
           2026-08-10_18-09-30_omni_recover_stop_v12_v11m150_16h_8192_r1/
           model_17250.pt
size        3,799,642 bytes
sha256      92ae9d9e945ed72e5aca91370acf94673d57181113ca89ab4f1c6639e0e87405
task sha256 9395bc00d19cea9699c0d86589a14defe517da4a0c4b4ee7020580bd00b726ba
control     50 Hz (0.020 s)
interface   78-D observation -> 12-D leg action
```

正式 Isaac Lab 结果为运动存活 `252/256`、条件停稳 `249/252 = 98.81%`、端到端
`249/256 = 97.27%`；固定 13 方向满回合率为 `485/514 = 94.36%`。这些结果仅证明
冻结 Isaac Lab 资产中的策略表现，不证明 Gazebo 迁移已经完成。

## 2. Nav2 速度合同

未来 Nav2 controller 的输出必须先重映射到 `/cmd_vel_nav`，不得直接连接腿部控制器。
适配器将 `geometry_msgs/msg/Twist` 转换为 V12 的三维命令：

```text
[linear.x, linear.y, angular.z] -> [vx, vy, yaw_rate]
```

命令必须按机器人底座坐标解释，并在 50 Hz 控制周期内采样。训练范围为
`vx=[-1.5, 1.5] m/s`、`vy=[-0.5, 0.5] m/s`、`yaw=[-1.0, 1.0] rad/s`，但第一次
Gazebo 集成必须进一步限制为：

```text
|vx| <= 0.15 m/s
|vy| <= 0.10 m/s
|yaw_rate| <= 0.05 rad/s
linear acceleration <= 0.15 m/s^2
angular acceleration <= 0.20 rad/s^2
command timeout <= 0.30 s
```

V12 使用三个分量“精确全零”作为 STOP 状态切换条件。适配器在死区、超时、取消目标、
安全门失败时必须输出精确 `[0, 0, 0]`，不能仅渐近逼近零。

## 3. 观测合同

78 维观测必须保持训练时的次序、单位、坐标系和数值语义：

| 索引 | 维数 | 内容 |
|---|---:|---|
| `0:3` | 3 | torso IMU 坐标系角速度 |
| `3:6` | 3 | torso 局部 `+Z` 在世界坐标中的方向 |
| `6:18` | 12 | 腿关节相对位置 |
| `18:30` | 12 | 腿关节相对速度 |
| `30:42` | 12 | 上一控制周期策略动作 |
| `42:45` | 3 | `[vx, vy, yaw_rate]` 速度命令 |
| `45:57` | 12 | 移动时为关节速度副本；精确零命令时为 STOP 上下文 |
| `57:69` | 12 | 实际关节位置减累计目标位置 |
| `69:71` | 2 | 左、右脚二值接触 |
| `71:75` | 4 | 左右步态相位的 cos/sin 编码 |
| `75:76` | 1 | 步频 |
| `76:77` | 1 | gait index |
| `77:78` | 1 | 抬脚高度命令 |

STOP 上下文 12 维依次为当前平面速度、上一周期平面速度、最后一个非零三维速度命令、
切换相位 sin/cos、带符号的 capture/stop-age、左右脚接触。该状态由 V12 动作运行时
维护，不能只导出神经网络后丢弃。

腿关节顺序固定为：

```text
left_hip_yaw_joint
left_hip_pitch_joint
left_hip_roll_joint
left_knee_joint
left_ankle_pitch_joint
left_ankle_roll_joint
right_hip_yaw_joint
right_hip_pitch_joint
right_hip_roll_joint
right_knee_joint
right_ankle_pitch_joint
right_ankle_roll_joint
```

## 4. 动作合同

移动分支动作是受关节限位约束的累计位置目标：

```text
target[t] = clip(target[t-1] + 0.3 * action[t])
```

V12 的 STOP 分支另外包含残差缩放、逐关节目标变化率限制、双脚接触判断、稳定驻留、
捕获姿态 HOLD 和失衡恢复滞回。部署端必须移植并验证完整状态机，不能把 checkpoint
的 12 维输出直接当成绝对关节角。

## 5. TF 与里程计所有权

移动接入后的唯一链为：

```text
map -> odom -> base_footprint -> pelvis -> torso_link -> lidar_link -> livox_mid360_frame
```

- AMCL 独占 `map -> odom`；
- RL 状态估计独占 `/odom` 和 `odom -> base_footprint`；
- 启动 RL 里程计前必须设置 `publish_fixed_odom_tf:=false`；
- Gazebo 真值必须使用独立话题，不能伪装成估计里程计；
- `/odom`、关节状态、IMU、接触和策略周期都使用仿真时钟。

## 6. 已完成的零运动基础

以下机器可验证基础已落盘，但没有被任何 launch 自动启动：

```text
config/v12_locomotion.yaml                    冻结策略、资产、观测、关节和限速合同
h1_2_navigation/locomotion_contract.py        合同校验与 78 维观测装配
h1_2_navigation/velocity_gate.py              六门全通过才放行的纯速度安全门
h1_2_navigation/nav2_velocity_gate.py         默认禁用的 ROS Twist 适配节点
h1_2_navigation/v12_asset_audit.py            只读 Isaac/Gazebo 资产一致性审计
h1_2_navigation/v12_policy.py                 纯 NumPy 双分支确定性推理
h1_2_navigation/v12_policy_export.py          非覆盖导出与 PyTorch 对拍工具
h1_2_navigation/v12_runtime.py                完整 MOVE/BRAKE/HOLD/RECOVER 内核
policy/v12_model_17250_policy.npz              ROS Python 可加载的冻结权重
policy/v12_model_17250_policy.pt               等价 TorchScript 参考导出
policy/v12_model_17250_golden.npz              258 项 MOVE/STOP 对拍语料
```

`enabled` 和 `movement_authorized` 均默认为 `false`；任一门失败、命令进入死区或超过
`0.30 s` 都输出精确零。当前没有启动 `nav2_velocity_gate`，所以它没有创建速度发布者。
策略推理和状态机目前都是纯函数式离线核心，不订阅 ROS、不发布腿目标；状态估计、传感器
同步、腿控制器和输出侧监督仍未实现。

导出前 `--self-test` 对 checkpoint 身份、结构和 258 项 MOVE/STOP 样本完成只读检查，
`writes_performed=false`。一次性非覆盖导出的 NumPy 与 PyTorch 最大绝对误差为
`3.0517578125e-05`，TorchScript 与 PyTorch 参考输出逐值一致。

只读审计命令：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run h1_2_navigation v12_asset_audit
```

该命令只校验文件和模型，不启动 Gazebo、不发布 ROS 消息。当前结果为
`baseline_valid=true`、`deployment_ready=false`。

## 7. 当前资产差异与阻塞项

审计确认 12 个腿关节的名称、父子链接、原点、轴、位置限位和 effort 全部一致，总质量
均为 `66.984 kg`。策略导出项已经完成，但以下八项仍阻止 Gazebo 部署：

1. 12 个腿关节的 velocity limit 全部不一致：训练源是历史超大覆盖值，Gazebo 为实际
   `23/14/9 rad/s` 限值；
2. 28 个公共惯量中 27 个逐值一致；`right_shoulder_pitch_link` 的 `ixy/ixz` 符号不同；
3. 训练源只有 2 个脚底 box collision，Gazebo 源有 22 个 collision；
4. 训练默认腿姿态为髋 pitch `-0.16`、膝 `0.36`、踝 pitch `-0.20 rad`，Gazebo 当前
   初始腿姿态全零；
5. Gazebo 导航模型尚未自由根化；
6. 没有独立 12 腿控制器；
7. 没有 Gazebo IMU sensor；
8. 没有左右足接触 sensor。

checkpoint 本体只读加载审计通过：iteration `17250`，actor 的 22 个张量全部有限，
normalizer 为 `(1,78)`，移动分支为 `78 -> 128 x 4 -> 12`，STOP 分支为
`78 -> 64 x 2 -> 12`。这只确认冻结权重结构，不解除上述迁移阻塞。

## 8. 后续接入顺序

当前只允许定位、纯路径规划和零运动离线验证；不得启动在线策略节点、解锁根关节或发送
腿部目标。进入首个零速 Gazebo 策略加载前，必须逐项通过：

1. 新建导航专用自由根 Gazebo 模型，不修改正式孵育箱动作模型；
2. 对齐 Isaac Lab USD 与 Gazebo URDF 的关节轴、零位、限位、质量和执行器参数；
3. 增加 12 腿位置控制器，并证明它不接管手臂、躯干和夹爪；
4. 补齐 IMU、双脚接触和自由根速度，逐维比对 78 维观测；
5. 导出并逐样本校验 V12 actor，同时移植完整 STOP 状态机（离线已完成）；
6. 先做精确零命令站立，不接 Nav2 controller；
7. 用户再次明确授权 Gazebo 移动后，才做单轴、低速、短时测试。

任何实体机器人连接或执行仍未获得授权。

## 9. 2026-09-09 Gazebo 定位行走状态覆盖

本节覆盖第 6--8 节中“尚未实现/仍阻塞”的历史状态，但不删除其中的设计边界和安全要求。
导航专用自由根模型、12 腿控制器、IMU、双足接触、truth-aided 仿真 odom、在线 V12 节点
和 Nav2 输出侧六门监督已经接通；所有结果仍严格限于 Gazebo。

最终运行合同：

```text
control rate                    50 Hz
trained nonzero onset           0.03 m/s or rad/s
safe yaw clamp                  0.05 rad/s
Nav2 XY goal tolerance          0.04 m
captured effort return rate     0.0 rad/s
normal planar damping           300 N/(m/s)
CAPTURED_HOLD planar damping    900 N/(m/s)
maximum planar damping force    300 N
yaw assistance torque           0
```

增强平面阻尼只在 V12 status 为 `CAPTURED_HOLD` 时生效；`MOVE/BRAKE/HOLD` 仍使用
`300 N/(m/s)`。两档阻尼都严格与平面速度反向，不提供路径推进或动作权威。冻结 effort
目标不会回归默认站姿，因为历史 r53/r93 已证明 effort 后切回 position hold 会快速跌倒。

最终候选 `r101/r102` 在两个 fresh Gazebo 中连续通过 0.15 m Nav2 前向目标和 30 秒冻结
保持。主验收 Mid360 终点误差分别为 `0.022361/0.038079 m`，定位/odom 位移差为
`0.008842/0.011020 m`，最小 `up_z` 为 `0.997345/0.995704`；保持阶段最大平面速度为
`0.000275/0.000028 m/s`，腿目标漂移均为 `0.0 rad`，Mid360 无无效样本。

权威报告：

```text
runtime_evidence/nav2_mid360_goal_tolerance_repeat_r102_20260909/REPORT.md
SHA-256 3a62a1f88e8bf46e297f3583259d8cd0c3e7d678839a1afddc2b5b0cb7be9cdc
```

该结果只证明当前方墙内短距离直行定位导航；多目标、障碍物、任意终态航向、SLAM、动态
重规划和实体机器人仍未验收。`movement_authorized` 与 `upright_assistance` 默认保持关闭。

## 10. 2026-09-11 官方 Unitree H1-2 profile 状态覆盖

本节覆盖第 1--9 节中旧 V12 作为当前部署接口的结论，但保留其历史设计和失败边界。
用户已认可本机 Unitree 官方 H1-2 `motion.pt` 步态，新的隔离 profile 使用：

```text
observation             47-D
action                  12-D direct position target
recurrent memory        LSTM 64
control period          0.02 s
action scale            0.25
default leg pose        [0,-0.16,0,0.36,-0.20,0] x 2
policy source SHA-256   ea3003268d5d2e4b5004eb26d0969fbfdedced265ceeec7bf0b335309259cc80
policy export SHA-256   68a1f2dff967fdc464451956eb6b4c60288cf375e38629092edab23b2adb014c
```

入口为 `incubator_navigation_gazebo.launch.py locomotion_profile:=unitree_h12`。速度监督、
Mid360、AMCL/Nav2、truth-aided Gazebo odom、12 腿 effort PID 和完整孵育箱组合场景均已
接通；`movement_authorized` 与 `auto_navigate` 仍默认 false，正式运行不使用 upright
assistance。零命令使用连续 recurrent station-keeping，而不是 V12 冻结 effort capture。

最终 r17 在固定 `(-0.60,0,0)` 到 `(-0.45,0,0)` 目标上 Nav2 succeeded 并由自动门判定
PASS，20 秒后验期间无 fault 且保持直立。报告：

```text
runtime_evidence/unitree_h12_incubator_nav2_lstm_entry_reset_20260911_r17/REPORT.md
```

动态驻足仍有最高约 `0.109 m` 的物理终点周期偏移，因此当前接口只验收完整场景中的
短距离定位行走，不验收严格冻结保持、多目标、未知障碍、SLAM、动态重规划或真机。
