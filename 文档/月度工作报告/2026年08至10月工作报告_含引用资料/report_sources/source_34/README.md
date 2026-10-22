# H1_2 Isaac Lab gait training

这是 H1_2 人形机器人步态任务从 MuJoCo Playground/Brax 到 Isaac Lab 的外部任务实现。当前已在以下组合上完成资产、环境、零动作诊断和 RSL-RL PPO 烟雾训练验证：

- Isaac Lab `3.0.0-beta2`：`/home/liu/robot/gym_sim/IsaacLab-3.0-beta2`
- Isaac Sim `6.0.1-rc.7`
- Gymnasium 任务 ID：`H12-Gait-Flat-v0`
- 默认并行环境数：`8192`
- 训练算法：RSL-RL PPO

任务由 `h1_2_isaaclab.register_tasks` 外部回调注册，不需要修改 Isaac Lab 源码。

## 安装

把外部任务以 editable 方式安装到 Isaac Sim 的 Python 环境：

```bash
env -u CONDA_PREFIX -u CONDA_DEFAULT_ENV \
  /home/liu/robot/gym_sim/IsaacLab-3.0-beta2/isaaclab.sh -p -m pip install -e \
  /home/liu/test/h1_2_isaaclab/source/h1_2_isaaclab
```

这里必须清除 `CONDA_PREFIX` 和 `CONDA_DEFAULT_ENV`，避免当前 Conda 环境污染 Isaac Sim 自带的 Python 依赖。

## 机器人资产

训练运行时直接加载已经转换并检查过的冻结 USD：

```text
/home/liu/test/h1_2_isaaclab/assets/usd/h1_2_feetonly/h1_2_feetonly.usda
```

它是自包含的约 40.8 MB USD，不依赖运行时 URDF 导入。其机器人结构为 28 个刚体、27 个转动关节、总质量 `66.984 kg`；与原训练 MJCF 一样，只保留左右脚的两个盒形碰撞体。

如源 URDF、网格、惯性或碰撞配置发生变化，按以下顺序重建。第一步生成项目使用的精确 feet-only URDF，第二步必须通过 Isaac Lab 的 Python 转换为冻结 USD：

```bash
python3 /home/liu/test/h1_2_isaaclab/scripts/build_h12_urdf.py

env -u CONDA_PREFIX -u CONDA_DEFAULT_ENV \
  /home/liu/robot/gym_sim/IsaacLab-3.0-beta2/isaaclab.sh -p \
  /home/liu/test/h1_2_isaaclab/scripts/convert_h12_urdf.py
```

`build_h12_urdf.py` 默认读取 `/home/liu/moveit_ws/src/h1_2_description/urdf/h1_2.urdf`。不要用通用转换命令替代 `convert_h12_urdf.py`；脚本中的 importer 配置是这个资产可重复构建的一部分。

## 环境诊断

正式训练前建议跑 1100 个零动作控制步，覆盖一次完整的 1000 步回合及第 501 步指令重采样：

```bash
cd /home/liu/robot/gym_sim/IsaacLab-3.0-beta2
env -u CONDA_PREFIX -u CONDA_DEFAULT_ENV \
  ./isaaclab.sh -p /home/liu/test/h1_2_isaaclab/scripts/diagnose_h12_env.py \
  --num-envs 8 --steps 1100 --viz none
```

诊断会检查 78 维观测、12 维动作、27 个关节、28 个刚体、两只脚的接触视图、初始 IMU 约定、指令变化步、根高度、足底点位置、接触力、终止/超时以及观测有限性。零动作模型应稳定站立至 1000 步超时，不应提前摔倒。

## 正式训练

当前机器上实测吞吐量最高的已验证配置是 8192 个并行环境：

```bash
cd /home/liu/robot/gym_sim/IsaacLab-3.0-beta2
env -u CONDA_PREFIX -u CONDA_DEFAULT_ENV \
  ./isaaclab.sh train --rl_library rsl_rl \
  --external_callback h1_2_isaaclab.register_tasks \
  --task H12-Gait-Flat-v0 --num_envs 8192 \
  physics=physx --viz none
```

默认配置训练 30000 个 iteration，每个 iteration 每环境采样 20 步；在 8192 环境下合计约 `4,915,200,000` 个环境步（约 49.15 亿步），每 100 个 iteration 保存一次 checkpoint。日志和 checkpoint 位于：

```text
/home/liu/robot/gym_sim/IsaacLab-3.0-beta2/logs/rsl_rl/h1_2_gait_duty06/<run>/
```

在 RTX 5090 D 上进行的短训练实测如下；数值是包含 PPO 更新的稳定训练吞吐量，不是纯物理仿真峰值，长跑会受机器负载和版本影响：

| 并行环境数 | 每 iteration 样本数 | 实测吞吐量 |
| ---: | ---: | ---: |
| 4096 | 81,920 | 约 47,000 steps/s |
| 8192 | 163,840 | 约 72,000 steps/s |

## 从 checkpoint 续训

`--load_run` 填日志根目录下的 run 文件夹名，`--checkpoint` 填该 run 内的文件名：

```bash
cd /home/liu/robot/gym_sim/IsaacLab-3.0-beta2
env -u CONDA_PREFIX -u CONDA_DEFAULT_ENV \
  ./isaaclab.sh train --rl_library rsl_rl \
  --external_callback h1_2_isaaclab.register_tasks \
  --task H12-Gait-Flat-v0 --num_envs 8192 \
  --resume --load_run <RUN_FOLDER> --checkpoint <model_N.pt> \
  physics=physx --viz none
```

例如 `<RUN_FOLDER>` 是 `2026-08-07_12-34-56`，checkpoint 参数只写 `model_1000.pt`，不要把两个参数合并成一个路径。

## 批量评估 checkpoint

策略稳定性必须用无 viewer 的批量评估判断。标准评估规模是 256 个并行环境乘 2000 步：

```bash
cd /home/liu/robot/gym_sim/IsaacLab-3.0-beta2
env -u CONDA_PREFIX -u CONDA_DEFAULT_ENV \
  ./isaaclab.sh -p /home/liu/test/h1_2_isaaclab/scripts/eval_h12_checkpoint.py \
  /home/liu/robot/gym_sim/IsaacLab-3.0-beta2/logs/rsl_rl/h1_2_gait_duty06/<RUN_FOLDER>/<model_N.pt> \
  --num-envs 256 --num-steps 2000 --episode-length 1000 --viz none
```

评估器使用 checkpoint 的确定性推理策略，只统计评估窗口内已经结束的回合（窗口末尾仍在运行的回合不计），并报告提前终止率、满 1000 步比例、平均步数和奖励。它默认还会核验同一 run 下 `params/env.yaml` 和 `params/agent.yaml`，如果当前 duty ratio、奖励、终止、观测、资产或策略配置与训练快照不一致就停止，避免把旧 checkpoint 静默放进新配置评估。新 run 还会在 `env.yaml` 自动记录任务源码 SHA-256、冻结 USD SHA-256 和 Isaac Lab/Sim/RSL-RL 版本，防止同路径内容被覆盖后漏检；旧 smoke run 没有这些字段时会明确警告只能做配置级校验。只有明确要做跨配置实验时才使用 `--allow-config-mismatch` 绕过。

MuJoCo viewer 或 Isaac Sim viewer 只适合定性检查动作、接触和视觉效果，不能据此判断策略稳定性或选择最优 checkpoint。

## V1 更像人的步态任务

`H12-Human-Gait-Flat-v1` 是在已验证的 `duty_ratio=0.6` 基线上做的保守微调任务。它保持策略接口不变（78 维观测、12 维腿部动作），因此可以从基线最佳 `model_48000.pt` warm-start，同时新增以下约束：

- 步频和抬脚高度不再独立随机采样，而是随速度连续变化；例如 `0.6 m/s` 对应约 `0.96 Hz` 和 `0.081 m`，静止时两者均为零。
- 左右肩 pitch 按步态相位反相摆动，最大偏移 `0.20 rad`，摆幅随平面速度缩放；策略动作仍只有 12 个腿关节。
- 腿部累计位置目标保留在物理关节范围中心的 90%，给硬限位留出缓冲。
- 在原合并 gait reward 之外，加入相位接触、躯干直立、躯干角速度、竖直速度、root 高度、动作幅值、关节限位余量和步宽目标。
- 速度指令片段延长到 `20.02 s`，10% 环境训练静止；pelvis 低于 `0.70 m` 视为 collapse。

训练入口会自动从基线 run 的 `model_48000.pt` 续训，并把新 checkpoint 隔离到 `h1_2_human_gait_v1`：

```bash
H12_TRAIN_DURATION=1h H12_NUM_ENVS=8192 H12_RUN_NAME=human_v1_1h_8192 \
  /home/liu/test/h1_2_isaaclab/scripts/train_h12_human_v1.sh
```

V1 checkpoint 必须用专用批量评估器。除提前终止率、满步率和回合奖励外，它还报告速度跟踪 RMSE、左右接触 duty、双/单支撑和腾空比例、cadence、支撑脚滑移、步宽及对称性、root 竖直起伏和 torso RMS：

```bash
cd /home/liu/robot/gym_sim/IsaacLab-3.0-beta2
env -u CONDA_PREFIX -u CONDA_DEFAULT_ENV \
  ./isaaclab.sh -p /home/liu/test/h1_2_isaaclab/scripts/eval_h12_human_checkpoint.py \
  <V1_MODEL_PT> --num-envs 256 --num-steps 2000 --episode-length 1000 \
  --contact-force-threshold 5 --viz none
```

定性查看时使用同一个 viewer，但显式指定 V1 任务。V1 的步频和抬脚高度由速度自动决定，viewer 的旧 `--gait-frequency`/`--foot-height` 参数不会覆盖它：

```bash
cd /home/liu/robot/gym_sim/IsaacLab-3.0-beta2
env -u CONDA_PREFIX -u CONDA_DEFAULT_ENV \
  ./isaaclab.sh -p /home/liu/test/h1_2_isaaclab/scripts/view_h12_checkpoint.py \
  <V1_MODEL_PT> --task H12-Human-Gait-Flat-v1 --lin-vel-x 0.6
```

## 与原 MuJoCo/Brax 任务的语义对齐

当前移植保留了原任务的关键训练语义：

- 12 个腿关节动作，按控制步累积为关节位置目标，尺度 `0.3`；关节目标最后按关节限位裁剪。
- 78 维策略观测，包含 torso IMU site 的角速度、世界系 up vector、腿关节状态、上一动作、速度指令、历史项、双脚接触和步态参数；训练时保留原观测噪声。
- 仿真步长 `0.004 s`、decimation `5`，即策略频率 `50 Hz`；回合长度 `20 s = 1000` 个控制步。
- 平地、静/动摩擦系数 `1.0`、固定 home reset；无质量、质心、外力、推搡和地形随机化。
- 速度指令保持 501 个控制步（`10.02 s`）；步态 `duty_ratio=0.6`。
- 奖励仍为合并后裁剪的 gait reward，包含相位足力、线/角速度跟踪、姿态、foot slip 和 action rate；foot slip 使用足底碰撞点速度，接触使用当前帧足底净力。
- 摔倒判据使用 torso IMU up-vector 的 z 分量 `< 0.85`，并保留腿关节越界终止。
- PPO 的网络宽度、Swish、观测归一化、rollout 长度、clip、学习率、折扣、GAE 和更新参数已按原 Brax 配置对齐。

这里的“语义对齐”不表示两个物理引擎逐步数值相同。PhysX 与 MuJoCo 的接触、约束求解、积分和浮点执行存在差异，因此奖励曲线和最优 checkpoint 需要在 Isaac 环境中重新批量评估。

原 MuJoCo Playground checkpoint 是 Brax/JAX 参数树，新训练使用 PyTorch RSL-RL `model_*.pt`，二者不能直接互相加载。策略分布也仍有一个明确差异：原策略是 `tanh-Normal`，当前 RSL-RL 是 Gaussian，随后由环境 wrapper 把动作硬裁剪到 `[-1, 1]`。因此 Isaac 训练应从新 RSL-RL run 开始；即使将来手工转换网络权重，也不能把它当作等价续训。
