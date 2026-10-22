# H1_2 Isaac Lab 步态强化学习交接文档

更新时间：2026-08-11（Asia/Shanghai）

## 1. 当前结论

Isaac Lab 训练、评估和可视化链路已经部署完成，当前机器可稳定使用 `8192` 个并行环境训练，实测约 `70k steps/s`。项目现状需要区分三个目标：

1. **基础前进步态**：已完成 24 小时训练并保留全部正式产物。
2. **更像人的步态**：V1 已取得明显改善，当前最佳为 `model_49600.pt`，但 foot slip 和步宽仍需优化。
3. **全向速度控制**：V12 已同时通过持续全向运动与运动到精确零命令的正式批量评估；当前最佳为 `model_17250.pt`。

V2 `model_49700.pt` 仍是纯移动对照基线。当前推荐的统一全向停走策略是 V12 `model_17250.pt`：三种子 `64 × 1000` 快筛中，进入停车阶段的 `182/182` 个独立 attempt 全部成功；单回合 `256 × 2000` 长时 transition 正式评估取得运动存活 `252/256`、条件停稳 `249/252 = 98.81%`、端到端 `249/256 = 97.27%`。固定 13 方向 `256 × 2000` 评估总体满回合率 `485/514 = 94.36%`。

V12 已于 `2026-08-11 10:09:27 +08:00` 按 16 小时时长正常结束，状态为 `completed_duration`。`timeout` 的退出状态 `124` 是时长训练 launcher 的预期结果；日志中没有 CUDA、OOM、NaN、Inf、磁盘或 traceback 异常。

## 2. 环境与项目位置

- 项目根目录：`/home/liu/test/h1_2_isaaclab`
- Isaac Lab：`/home/liu/robot/gym_sim/IsaacLab-3.0-beta2`
- Isaac Lab 版本：`3.0.0-beta2`
- Isaac Sim 版本：`6.0.1-rc.7`
- GPU：NVIDIA GeForce RTX 5090 D，约 32 GB 显存
- 外部任务注册回调：`h1_2_isaaclab.register_tasks`
- 冻结机器人资产：`/home/liu/test/h1_2_isaaclab/assets/usd/h1_2_feetonly/h1_2_feetonly.usda`
- 观测/动作接口：`78-D observation / 12-D leg action`
- 物理步长：`0.004 s`
- decimation：`5`
- 策略控制频率：`50 Hz`
- 标准回合：`20 s = 1000` 个控制步

运行 Isaac Lab 命令时必须避免 Conda 环境污染：

```bash
env -u CONDA_PREFIX -u CONDA_DEFAULT_ENV \
  /home/liu/robot/gym_sim/IsaacLab-3.0-beta2/isaaclab.sh ...
```

## 3. 必须保留的 checkpoint

### 3.1 更像人的前向步态 V1

- 任务：`H12-Human-Gait-Flat-v1`
- checkpoint：

```text
/home/liu/robot/gym_sim/IsaacLab-3.0-beta2/logs/rsl_rl/h1_2_human_gait_v1/2026-08-10_10-23-57_human_v1_1h_8192_r1/model_49600.pt
```

- SHA-256：`9e9286dacd2b1db14d072569353627ca9ebc9e4abf598aac1b6f3bca17505843`
- 大小：`4,170,271 bytes`
- `256 env × 2000 steps`：满 1000 步率 `74.1%`，平均回合 `790.0` 步。
- 平面速度 RMSE：`0.2825 m/s`；yaw RMSE：`0.3263 rad/s`。
- 相比 warm-start，姿态、步频、双支撑和稳定性明显改善。
- 仍有问题：foot slip 偏高、实际步宽约 `26.60 cm`，高于 `18 cm` 目标。

完整数据见 `HUMAN_V1_EVAL.md`。

### 3.2 全向移动 V2

- 任务：`H12-Human-Omni-Gait-Flat-v2`
- checkpoint：

```text
/home/liu/robot/gym_sim/IsaacLab-3.0-beta2/logs/rsl_rl/h1_2_human_omni_gait_v2/2026-08-10_12-34-00_omni_v2_360it_8192_r1/model_49700.pt
```

- SHA-256：`1a573924a1cdc9622555a76691fdecd30f44b3f8504d3454c58451a6874f247e`
- 大小：`4,170,271 bytes`
- `256 env × 2000 steps` 固定全方向批量评估：总体满回合率约 `93.4%`。
- 各非零方向约 `87.5%–100%`，覆盖前后、左右横移、四个斜向、左右原地转向和前进加转向。
- 明确缺陷：精确零命令不能长期稳定站立。

这是后续所有停止策略都应保护的移动分支，不要直接在它上面无约束全量微调。

### 3.3 V8 停止策略基线

- 任务：`H12-Human-Omni-Context-Stop-Gait-Flat-v8`
- checkpoint：

```text
/home/liu/robot/gym_sim/IsaacLab-3.0-beta2/logs/rsl_rl/h1_2_human_omni_context_stop_gait_v8/2026-08-10_16-38-35_omni_context_stop_v8_latch_sync_900it_8192_r3/model_50.pt
```

- SHA-256：`15829a0ce11063fde8d851c14f6160b2270face8c73b8e0f81c317941fe46ef5`
- 大小：`3,799,339 bytes`
- 任务源码哈希：`9d192e471ff17475462ea164ccef0d5d975d814e06ac810060d3f866d1ef55ac`
- 严格 `64 × 1000`：运动存活 `62/64`，条件停稳 `5/62 = 8.06%`，端到端 `5/64`。

V8 是 V9–V11 的对照基线。它冻结 V2 移动 MLP，只训练精确零命令分支，并显式提供 STOP 上下文。

### 3.4 先前停止研究最佳 V11

- 任务：`H12-Human-Omni-Focused-Stop-Gait-Flat-v11`
- checkpoint：

```text
/home/liu/robot/gym_sim/IsaacLab-3.0-beta2/logs/rsl_rl/h1_2_human_omni_focused_stop_gait_v11/2026-08-10_17-37-19_omni_focused_stop_v11_v8r3m50_101it_8192_r2/model_150.pt
```

- SHA-256：`679e299670f7778b6a0e2800d75d37b1be63427d9d753e8bf7a16b6cb0aeb601`
- 大小：`3,799,440 bytes`
- 任务源码哈希：`7131afbfd5e45f8b674ab5889717a922126941b8aa8bd6a066e289be2b3b83d8`
- 父 V8 哈希：`9d192e471ff17475462ea164ccef0d5d975d814e06ac810060d3f866d1ef55ac`
- checkpoint `iter=150`，模型和优化器张量均可完整反序列化且为有限值。
- V2/V8 冻结移动路径的 `15/15` 组张量与 warm-start 按位一致；只有 STOP 分支及其探索参数改变。

V11 的训练分布：

- `8192` 并行环境；
- 从 V8 r3 `model_50.pt` warm-start；
- 五个 curriculum stage 全部固定为 paired-only；
- `speed_scale=0.15`；
- 运动到停止切换步为 `U[30, 60]`；
- 回合长度 `5 s = 250` 控制步；
- 追加训练 iteration `50–150`。

正式运行状态：

```text
/home/liu/test/h1_2_isaaclab/train_logs/2026-08-10_17-37-16_omni_focused_stop_v11_v8r3m50_101it_8192_r2.status
```

状态为 `completed_iterations`，无 CUDA、OOM、NaN、Inf、磁盘或 traceback 异常。

### 3.5 当前推荐 V12

- 任务：`H12-Human-Omni-Recover-Stop-Gait-Flat-v12`
- 源码哈希：`9395bc00d19cea9699c0d86589a14defe517da4a0c4b4ee7020580bd00b726ba`
- warm-start：V11 r2 `model_150.pt`
- 训练：`8192` 环境，16 小时时长，checkpoint 间隔 250 iterations。
- 完成状态：`completed_duration`，最终 iteration `19757`，约 `5,140,119,552` 个环境步。
- 推荐 checkpoint：`model_17250.pt`
- checkpoint SHA-256：`92ae9d9e945ed72e5aca91370acf94673d57181113ca89ab4f1c6639e0e87405`
- checkpoint 大小：`3,799,642 bytes`
- 正式 run：

```text
/home/liu/robot/gym_sim/IsaacLab-3.0-beta2/logs/rsl_rl/h1_2_human_omni_recover_stop_gait_v12/2026-08-10_18-09-30_omni_recover_stop_v12_v11m150_16h_8192_r1
```

- launcher 日志：

```text
/home/liu/test/h1_2_isaaclab/train_logs/2026-08-10_18-09-27_omni_recover_stop_v12_v11m150_16h_8192_r1.log
```

- 状态文件：

```text
/home/liu/test/h1_2_isaaclab/train_logs/2026-08-10_18-09-27_omni_recover_stop_v12_v11m150_16h_8192_r1.status
```

训练完成后共保留 `79` 个 checkpoint（`model_250.pt` 到 `model_19750.pt`）。最后 checkpoint `model_19750.pt` 的 95 个张量全部为有限值；候选 `10000/11500/17250/19250/19750` 均通过训练快照、源码哈希、模型结构和冻结移动路径 `15/15` 组张量按位一致校验。根据 TensorBoard 候选筛选和严格批量评估，最终选择 `model_17250.pt`，而不是简单选择最后一个 checkpoint。

## 4. 全向停止版本结果

所有表中 STOP 结果均来自严格独立 attempt 评估；失败后的自动重置轨迹永久排除，不会被误计为成功。

| 版本/checkpoint | 关键设计 | 运动存活 | 条件停稳 | 端到端 | 结论 |
| --- | --- | ---: | ---: | ---: | --- |
| V7 `model_150` | 规则斜率限制/接触对齐多种变体 | `61/64` | `0/61` | `0/64` | 规则 supervisor 无效 |
| V8 r3 `model_50` | 冻结移动头 + 上下文 STOP 残差 | `62/64` | `5/62` | `5/64` | 首个非零基线 |
| V8 r3 `model_200`，低速同协议 | 延长同分布训练 | `62/64` | `6/62` | `6/64` | 延长训练收益很小 |
| V9 `model_100` | HOLD 动作接口变体 | `62/64` | `3/62` | `3/64` | 比 V8 退化 |
| V10 `model_100` | smooth-stop 动作接口变体 | `62/64` | `1/62` | `1/64` | 明确退化 |
| V11 `model_100` | paired-only 短回合聚焦训练 | `60/64` | `8/60` | `8/64` | 有改善但未过门槛 |
| V11 `model_150` | 同上，继续 50 updates | `60/64` | `12/60 = 20.0%` | `12/64 = 18.75%` | 先前最佳，但未达到放行门槛 |
| **V12 `model_17250`，三种子快筛合计** | 捕获姿态 HOLD + 驻留 + 恢复滞回 | **`182/192`** | **`182/182 = 100%`** | **`182/192 = 94.79%`** | 快筛稳定通过 |
| **V12 `model_17250`，单回合长时正式评估** | `256 × 2000`，30 步运动 + 1970 步停车 | **`252/256`** | **`249/252 = 98.81%`** | **`249/256 = 97.27%`** | 当前正式最佳 |

V11 预先规定的筛选门槛是至少约 `21%`，即基准口径下至少 `13/62` 个停稳成功。`model_150` 只取得 12 个成功样本，因此没有事后降低门槛，也没有继续做 `256 × 2000` 正式扩展评估。

严格日志：

```text
/home/liu/test/h1_2_isaaclab/eval_logs/v11_r2_model100_focus_scale015_m30_transition_strict_64x1000.log
/home/liu/test/h1_2_isaaclab/eval_logs/v11_r2_model150_focus_scale015_m30_transition_strict_64x1000.log
```

V11 `model_150` 的关键诊断：

- `captured_attempts=56/64`；
- `captured_then_failed=44`；
- capture 到首次失败延迟为 `56/102.4/283` 步（min/mean/max），平均约 `2.05 s`；
- 首次失败原因以 `bad_orientation=45` 为主；
- 成功样本最后 100 步速度 RMS `[vx, vy, yaw]=[0.0069, 0.0068, 0.0407]`。

因此 V11 的主要瓶颈不是“刹不住”，而是“已经捕获低速双支撑状态后，无法长期维持平衡”。

V12 `model_17250` 的正式 transition 关键数据：

- 运动阶段 RMSE `[vx, vy, yaw] = [0.0703, 0.1974, 0.1734]`；
- 停止阶段实际 RMS `[vx, vy, yaw] = [0.0138, 0.0239, 0.0255]`；
- 完整回合最后 100 步 RMS `[vx, vy, yaw] = [0.0093, 0.0054, 0.0147]`；
- `captured_attempts=249/256`，`captured_then_failed=1`；该回合在 capture 后约 1080 步才失稳；
- 7 个失败中，4 个发生在运动阶段，3 个发生在长时间停车阶段；首次失败标签主要为 `bad_orientation`。

V12 `model_17250` 的固定全向 `256 × 2000`：

- 总回合 `514`，满 1000 步 `485`，总体满回合率 `94.36%`；
- 各场景大多为 `90%–100%`，最差为 `yaw_left=76.9%`，其次为 `yaw_right=89.5%`；
- 总体 tracking RMSE：planar `0.2095 m/s`，yaw `0.1901 rad/s`；
- torso RMS：roll `1.927°`，pitch `3.175°`；
- 非有限存活样本为 `0`。

V12 正式评估日志：

```text
/home/liu/test/h1_2_isaaclab/eval_logs/v12_model17250_scale015_m30_transition_strict_seed1_64x1000.log
/home/liu/test/h1_2_isaaclab/eval_logs/v12_model17250_scale015_m30_transition_strict_seed2_64x1000.log
/home/liu/test/h1_2_isaaclab/eval_logs/v12_model17250_scale015_m30_transition_strict_singlecycle_256x2000.log
/home/liu/test/h1_2_isaaclab/eval_logs/v12_model17250_fixed_omni_256x2000.log
```

seed 0 快筛日志沿用候选筛选命名：

```text
/home/liu/test/h1_2_isaaclab/eval_logs/v12_model17250_scale015_m30_transition_strict_64x1000.log
```

## 5. 关键代码现状

### 5.1 任务版本

主要任务代码均位于：

```text
/home/liu/test/h1_2_isaaclab/source/h1_2_isaaclab/h1_2_isaaclab/tasks/manager_based/
```

- `h1_2_human_velocity/`：V1 更像人的前向步态。
- `h1_2_omni_velocity/`：V2 全向移动基线。
- `h1_2_omni_stand_velocity/`：V3。
- `h1_2_omni_stable_velocity/`：V4。
- `h1_2_omni_transition_velocity/`：V5。
- `h1_2_omni_arrest_velocity/`：V6。
- `h1_2_omni_settle_velocity/`：V7。
- `h1_2_omni_context_stop_velocity/`：V8，停止架构基线。
- `h1_2_omni_hold_stop_velocity/`：V9。
- `h1_2_omni_smooth_stop_velocity/`：V10。
- `h1_2_omni_focused_stop_velocity/`：V11，聚焦过渡训练版本。
- `h1_2_omni_recover_stop_velocity/`：V12，当前推荐的可恢复捕获姿态 STOP 版本。

V8/V9/V10/V11/V12 源码哈希分别为：

```text
V8  9d192e471ff17475462ea164ccef0d5d975d814e06ac810060d3f866d1ef55ac
V9  47057bb23532f1c05cc72a6d7938c29cd1dc4abbb91af72b76e8412344bd991a
V10 4424108b5b30d2aba43448f44c5e998e5e19ec56574f4c1dde88dfaec546625e
V11 7131afbfd5e45f8b674ab5889717a922126941b8aa8bd6a066e289be2b3b83d8
V12 9395bc00d19cea9699c0d86589a14defe517da4a0c4b4ee7020580bd00b726ba
```

这些哈希同时保存在各正式 run 的 `params/env.yaml`；评估时应以快照校验为准。

### 5.2 控制入口

- 交互控制：`scripts/control_h12_omni_checkpoint.py`
- Isaac Sim viewer：`scripts/view_h12_checkpoint.py`
- 固定方向批量评估：`scripts/eval_h12_omni_checkpoint.py`
- 运动到停止批量评估：`scripts/eval_h12_omni_transitions.py`
- V11 训练：`scripts/train_h12_omni_focused_stop_v11.sh`
- V12 训练：`scripts/train_h12_omni_recover_stop_v12.sh`

两个全向评估器会先校验训练快照、任务源码哈希和资产配置。V11 的训练回合只有 250 步，严格评估需要 1000 步；评估器会在快照校验通过后，仅为评估延长 episode horizon，不会改变训练快照的真实性。

`eval_h12_omni_transitions.py` 还会输出：

- 首次 capture 的 STOP 步数；
- captured attempt 数；
- capture 后又失败的数量；
- capture 到首次失败延迟。

这些诊断是只读的，不改变成功、失败或 intact cohort 判定。

## 6. 必须遵守的评估规则

1. **策略稳定性只能用批量评估判断。**
2. MuJoCo viewer 和 Isaac Sim viewer 只用于定性观察，不能人工数几个回合后选择 checkpoint。
3. 固定方向正式评估使用 `256 env × 2000 steps`。
4. 停止策略在正式扩展前先使用严格 `64 env × 1000 steps` 快筛。
5. transition evaluator 必须使用同步独立 attempt；失败后的自动 reset 轨迹永久排除。
6. 不要用训练中的 mean reward 或 TensorBoard success 指标替代最终批量评估。
7. 不要跨任务静默加载 checkpoint；确需跨配置诊断时必须显式记录并使用 `--allow-config-mismatch`。
8. 每个新动作语义都应建立新任务版本和新源码哈希，不要原地改写已评估版本。
9. transition 的正式 `256 × 2000` 应使用一个 2000 步独立回合（例如 30 步运动 + 1970 步停止，`episode_length=2000`）。不要把它拆成同一 env 实例内两个 1000 步同步周期；周期末 time-out 已由 Isaac 自动 reset，再调用整批 `env.reset()` 会形成连续二次复位并污染下一周期。

V11 `model_150` 的复现实验命令：

```bash
cd /home/liu/robot/gym_sim/IsaacLab-3.0-beta2
env -u CONDA_PREFIX -u CONDA_DEFAULT_ENV \
  ./isaaclab.sh -p /home/liu/test/h1_2_isaaclab/scripts/eval_h12_omni_transitions.py \
  /home/liu/robot/gym_sim/IsaacLab-3.0-beta2/logs/rsl_rl/h1_2_human_omni_focused_stop_gait_v11/2026-08-10_17-37-19_omni_focused_stop_v11_v8r3m50_101it_8192_r2/model_150.pt \
  --task H12-Human-Omni-Focused-Stop-Gait-Flat-v11 \
  --num-envs 64 --num-steps 1000 --episode-length 1000 \
  --motion-steps 30 --stop-steps 970 --stop-tail-steps 100 \
  --motion-command-scale 0.15 --seed 0 --viz none
```

## 7. 已确认的关键教训

### 7.1 默认站姿不是根因

零策略动作能够让默认模型在 `256` 个环境中完整站立 1000 步。精确零命令失败来自策略/动作积分目标和运动到站立转换，而不是 USD 资产天然不稳定。

### 7.2 不能把简单 HOLD 当作动态平衡

固定关节目标、回中、命令斜率限制和接触对齐均不能可靠解决停止问题。V7 的四种 supervisor 变体全部为 `0/61` 条件停稳。

### 7.3 训练数据比例必须按实际占用计算

V8 早期包含大量简单整回合站立样本。由于 transition episode 很快摔倒，整回合站立在实际采样占用中会被进一步放大，真正的 STOP 过渡样本被稀释。V11 改为 paired-only 短回合后从 `5/62` 提升到 `12/60`，证明训练分布确实重要，但单靠重采样仍不够。

### 7.4 当前捕获锁存可能是结构性瓶颈

V8/V11 中 capture 一旦锁存，只会在重新进入非零运动或 reset 时清除。捕获后动作参考从实测关节姿态转向默认姿态。V11 有 `56` 次成功捕获，却有 `44` 次随后失稳，说明永久锁存和默认姿态回归值得作为下一版的首要单变量实验。

相关代码：

```text
source/h1_2_isaaclab/h1_2_isaaclab/tasks/manager_based/
  h1_2_omni_context_stop_velocity/mdp/actions.py
```

### 7.5 RSL-RL 续训 iteration 语义

从 `iter=50` 的 checkpoint 恢复时，`max_iterations=N` 表示再运行 `N` 次，而不是训练到全局 iteration N。V11 的最终 launcher 已修正为追加 `101` 次，得到 iteration `50–150`；完成状态通过解析日志中的 `current/total` 判断，不能简单比较 `max_iterations-1`。

### 7.6 transition evaluator 不能在 time-out 后再整批二次 reset

V12 诊断中，同一 checkpoint 的第一轮 `256 × 1000` 为运动存活 `252/256`、端到端 `250/256`；但把 `num_steps=2000` 配成两个 1000 步同步周期后，总结果降到运动存活 `264/512`。由第一轮结果可知第二轮只有约 `12/256` 个运动 attempt 存活。`64 × 2000` 也复现为总运动存活 `62/128`，排除了并行规模因素。

原因是每周期最后一步的纯 time-out 已触发 Isaac 自动 reset，下一周期 evaluator 又调用一次整批 `env.reset()`。这不是 V12 单回合停走能力的有效测量。原始缺陷日志保留为证据：

```text
/home/liu/test/h1_2_isaaclab/eval_logs/v12_model17250_scale015_m30_transition_strict_256x2000.log
/home/liu/test/h1_2_isaaclab/eval_logs/v12_model17250_scale015_m30_transition_diag_256x1000.log
/home/liu/test/h1_2_isaaclab/eval_logs/v12_model17250_scale015_m30_transition_diag_64x2000.log
```

## 8. V12 设计、结果与下一步

V12 已完成训练与正式评估。它保持 V2 移动分支冻结，加入稳定驻留、捕获姿态锚定和失衡恢复滞回。该设计把 V11 的条件停稳率从 `20.0%` 提高到单回合长时正式评估的 `98.81%`，同时固定全向满回合率保持在 `94.36%`。后续不要覆盖 V12 源码或 checkpoint 快照；任何动作语义变化应建立 V13。

### 8.1 先做无训练诊断

在 evaluator 中记录 capture 和 failure 时的：

- torso `up_z`；
- roll/pitch 角速度；
- root 垂直速度；
- 双脚接触；
- 捕获姿态与默认姿态的关节距离；
- 若可行，质心投影相对双脚支撑区域的位置。

不要改变原成功判定，只增加只读诊断。

### 8.2 三状态 STOP 控制器

推荐状态：

1. `MOVE`：完全复用冻结的 V2 动作路径。
2. `BRAKE/RECOVER`：精确零命令但尚未稳定捕获，围绕当前实测姿态做较大残差修正。
3. `HOLD`：连续稳定若干步后，围绕捕获瞬间的姿态做小残差平衡，不立即回到默认姿态。

若 HOLD 中倾角、角速度或接触恶化，应允许回到 `RECOVER`，不能永久锁死。

捕获条件建议加入：

- 连续满足 8–10 个控制步；
- 更严格的 `up_z`（例如 `0.98–0.985`）；
- torso 横滚/俯仰角速度；
- root 垂直速度；
- 双脚接触。

### 8.3 分离 BRAKE 与 HOLD 策略头

当前一个 `zero_head` 同时承担制动和长期保持，目标冲突。建议保留 V11 权重初始化 `brake_head`，新增较小动作范围、较低探索噪声的 `hold_head`。移动 MLP 必须继续冻结。

### 8.4 训练数据和奖励

- 约 50% 正常运动到停止 transition；
- 约 50% 从捕获姿态附近开始，并加入小倾角、角速度和接触扰动；
- 这不是以前的简单默认站立 episode，而是动态恢复样本；
- 动作架构修正后再把 episode 延长到 `8–10 s`，覆盖延迟失稳；
- 捕获前奖励降速和恢复步；
- 捕获后奖励存活、姿态、角速度、垂直速度和支撑区域稳定；
- 暂时关闭或降低“回到默认关节姿态”的惩罚。

### 8.5 放行标准

- 单变量版本先跑 `64 × 1000`，建议至少 seed `0/1/2`；
- 不要因一次恰好多一个成功样本就宣布完成；
- 快筛通过后再跑 `256 × 2000` 固定全方向和 transition 正式评估；
- 最终报告必须同时包含运动 RMSE、运动存活、条件停稳、端到端成功、各方向最差值和失败原因。

可靠停走完成后，再隔离优化 human likeness：参考动作模仿、骨盆运动、摆臂、步宽、foot clearance、能耗和 foot slip。不要在 STOP 架构未稳定前同时改变这些因素。

V12 已达到本节的 STOP 放行要求。下一步建议按优先级：

1. 先针对纯原地左/右转向做单变量改进，当前固定方向最差值是 `yaw_left=76.9%`；冻结已经通过的停车路径。
2. 再做 human-likeness 隔离实验，优先降低 foot slip、将约 `26.47 cm` 的实际步宽向目标收窄，并检查 duty 左右不对称（`58.66%/63.31%`）。
3. 为跨 episode 重复评估修正 evaluator 的复位时序并加回归测试；不要用有二次 reset 的两周期数字评价策略。
4. 若面向真实机器人部署，增加命令斜率、观测噪声、时延、摩擦和质量随机化，以及急停/恢复压力测试。

## 9. 清理记录

本次整理只清理以下可再生或已确认废弃的过程产物：

- 所有 Python `__pycache__` 和 `.pyc`；
- 名称包含 `smoke` 的训练日志、状态文件、评估日志和 Isaac/RSL-RL smoke run；
- `2026-08-10_10-23-21_human_v1_1h_8192` 重复启动失败残留及对应 `.nohup`；
- V11 因错误理解续训 `max_iterations` 而主动中止的 r1 run、日志和状态。

上述训练 run 和日志使用桌面回收站删除，可恢复；Python 缓存直接删除且可自动重新生成。

本次共将 `126,996,060 bytes`（约 `121.1 MiB`）过程产物移入回收站。只有清空桌面回收站后，这部分空间才会真正释放。

以下内容明确保留：

- 全部正式 checkpoint 和 `params/` 快照；
- V1、V2、V7–V11 正式训练日志；
- 所有正式严格批量评估日志；
- 24 小时 duty-ratio 训练的约 79 MB 原始日志；
- 各任务版本源码和启动脚本；
- 冻结 USD 和资产构建脚本。

## 10. 相关文档

- `README.md`：安装、资产、训练和基础评估说明。
- `HUMAN_V1_EVAL.md`：V1 更像人步态的完整候选评估。
- `OMNI_CONTROL_REPORT.md`：早期全向控制阶段报告；本文件是更新后的主交接文档。
- 原 MuJoCo/Brax 交接：`/home/liu/test/h1_2_train/HANDOFF_TO_CODEX.md`
- 原调参时间线：`/home/liu/test/h1_2_train/PROGRESS.md`
- 原模型版本表：`/home/liu/test/h1_2_train/MODEL_VERSIONS.md`
