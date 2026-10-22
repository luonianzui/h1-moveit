# H1-2 Gazebo 孵育箱、离心机与导航项目

ROS 2 Jazzy / MoveIt 2 / Gazebo Harmonic；H1-2、双Dex1夹爪、D435、Mid360。所有成果仅限仿真，实体机器人未授权。

## 先看这一份

**[HANDOFF_CURRENT.md：当前成果与交接总览](HANDOFF_CURRENT.md)**。不再在多个入口重复堆叠“最新状态”；累积历史和规则仍由[AGENTS.md](AGENTS.md)保存。

- 离心机完整双盒装入→关盖→旋转30sim秒→停转开盖→双盒返桌：**r283 PASS，179.129sim秒**，独立28/28物理、原片和保护核验通过；单次固定骨盆手部仿真。
- 单独双盒装篮：**r159 PASS，172.500sim秒**；不能混同完整离心流程。
- 孵育箱双循环：v9/r17动作权威、v24/r103完整雷达地图固定重放；**920箱子版本永久保留**。
- Mid360/Nav2与官方步态已有定位/短距导航成果，但严格驻足及导航后视觉操作联动尚未完成。

用户最新目标是**现实三分钟**，目前未达。原target已1.0，但七个独占被动性能
对照最快约0.135倍实时；旧r283的179.129秒不能当现实耗时。见
[墙钟性能诊断](runtime_evidence/centrifuge_48box_wallclock_perf_20261015_r3/REPORT.md)。

## 文档导航

| 文件 | 用途 |
|---|---|
| [AGENTS.md](AGENTS.md) | 唯一累积式权威，采用最新日期小节 |
| [HANDOFF_CURRENT.md](HANDOFF_CURRENT.md) | 唯一当前摘要、成果边界、证据与备份索引 |
| [HANDOFF_PLAN.md](HANDOFF_PLAN.md) | 只读/开发/新身份重放/清理/备份分支合同 |
| [QUICKSTART.md](QUICKSTART.md) | 只读校验入口 |
| [HANDOFF_PROMPT.md](HANDOFF_PROMPT.md) | 新对话复制提示词 |
| [SCENE_VERSIONS.md](SCENE_VERSIONS.md) | 箱子与离心机场景切换、920恢复合同 |
| [CENTRIFUGE_FULL_CYCLE.md](CENTRIFUGE_FULL_CYCLE.md) | 三分钟完整离心机成果、验收与新身份运行边界 |
| [H1_2_MID360_NAVIGATION_PLAN.md](H1_2_MID360_NAVIGATION_PLAN.md) | 定位导航/步态独立成果线计划 |
| [826_plan.md](826_plan.md) | AprilTag历史设计与完成覆盖 |

包级接口与模型README继续在src下保存，正式REPORT/RUN_SUMMARY、authority/run、视频、rosbag、builder链和所有备份不合并删除。NEXT_CHAT_HANDOFF只保留兼容入口。

最新完整离心机证据：[r283报告](runtime_evidence/centrifuge_48box_optional_micro_full_20261015_r283/REPORT.md)。原始连续录像按真实电脑运行时间记录，约39分33秒；179.129秒是完整动作的仿真时间，不是录像快放。
普通场景显示不自动执行任务；重放必须明确授权及全新身份，不能直接重跑任何历史run。
