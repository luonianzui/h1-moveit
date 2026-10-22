# H1-2 当前交接执行计划

> 整理说明（2026-10-10）：当前成果、live和备份统一见[HANDOFF_CURRENT.md](HANDOFF_CURRENT.md)。以下早期“当前/最新”小节是历史分支合同，不覆盖统一摘要；离心机完整通过后的提速采用第21节。本文不授权自动执行历史命令。

更新时间：2026-09-23 CST
适用范围：下一位接手者、下一次对话或新工作阶段  
默认模式：只读接管；本计划本身不授权启动 Gazebo、发布轨迹或连接实体机器人

## 2026-09-23 离心机双48box支线：分支D

用户选择摇篮不变、48box等比85%，两个料盒从桌面开始，不直接生成到篮内。
当前最新r131独立effort试验因0.6011°试抬倾斜失败；r129/r130也失败，仍0/2入篮。
先读runtime_evidence/centrifuge_48box_effort_grip_20260923_r131/REPORT.md。
原r128最大抬升148.6206mm只是失败进度，不能作为稳定带载基线。
此任务属于独立离心机功能开发，不是分支G孵育箱固定轨迹重放；旧r119按键成功不等于装载成功。

继续时先重新只读检查live及保护哈希，用普通mkdir建立未使用的新身份；不得复用旧claim。
同步原生位姿/接触已确认实际滑移或倾斜；下一步先审核原网格接触面和抗俯仰力矩，
再调整抓取姿态，不能把30种顶点诊断候选当已通过IK/FCL的动作。
若延续独立effort配置，保留原机器人/控制器文件，日常r122入口不切换；
重新审核完整空载/带载路径。必须先通过低速试抬及持续带载静止保持，才进入转移。
用户2026-09-23补充允许在摇篮正上方松爪，靠重力完全落入即为正确：
释放前验证料盒/摇篮实际姿态及开口投影，释放后须证明完整包络在篮内、实际桶接触并稳定停留。
不再要求机器人先带盒降到篮底才松爪；不得用瞬移/绑定或仅中心点在篮内代替完全落入。
禁止放宽门限凑成功、改变未授权的摇篮/机器人、瞬移或物理绑定物体；失败stop-and-hold、
不丢失夹爪已接受的位置和力前馈目标，然后正常SIGINT收尾。未包含带载转子运行或实体机器人授权。
日常bash run_lab_scene.sh centrifuge --48box只生成桌上两盒；默认不自动搬运或按压。

## 2026-09-20 场景版本补充

先读 `SCENE_VERSIONS.md`：920 是受保护的“箱子版本”，当前默认显示入口已改为
交付离心机。模型导入与静态显示是场景开发，不是分支 G 孵育箱动作重放。
原箱子源码和成果均保留；切回箱子显示不需要解压备份，但须退出另一 Gazebo。
离心机没有旧箱子的关节/标签/轨迹合同，禁止把分支 G 密封命令用于离心机。
需要完整恢复 920 时只能解压到用户指定的新空目录，不覆盖当前工作空间。

## 1. 交接目标

接手者应在不混淆证据身份、不复用一次性运行资产、不破坏保护链的前提下，能够：

1. 准确说明当前已经完成的功能和有效证据；
2. 判断用户的新请求属于只读复核、Gazebo 重放、功能开发、清理还是备份；
3. 只在用户明确要求后进入对应执行分支；
4. 为任何新结果建立非覆盖身份、日志、视频、rosbag 和终态证据；
5. 完成后更新交接文档，并在需要时创建新的受保护备份。

## 2. 当前基准快照

必须同时区分三种身份：

| 类型 | 当前身份 | 结论 |
|---|---|---|
| 动作语义权威 | `candidate3_roundtrip_831_authority_v9 / r17` | 双循环 PASS，调用至 PASS `164.523835390 s` |
| 最新完整雷达地图重放 | `candidate3_roundtrip_831_authority_v21 / r31` | 四墙 Mid360 地图内固定双循环，`167.302430689 s` PASS |
| 最新受保护恢复点 | `/home/liu/h1_moveit/backups/914.tar.zst` | 包含 v19/r29 和归档前导航成果；不包含后续 v21/r31 |

完整动作是：右手单独开门，左手放入方块，右手关门锁门；右手再次开门，左手把方块
取回出生原位，右手最终关门锁门。左手没有把手命令或接触，躯干峰值小于 45 度。

关键固定身份：

```text
v9 manifest       b9e49b32c60979eb5a97d8ed3d5dc325a5d73e10b1985a45fd99c61f8848cd29
v9 commands       aef171adfacb8b1af212caa1c193e8fabf51e06d03f8fff4ceddf9ed0a22e8fb
v19 manifest      43c765450228557c1930787e25e9661eeed5fe34ffe2b80318ed3152aab8d7ff
v19 commands      752126eac221b9c5cea15fd433811f95992aba025278b381d93034ce59109612
r29 video         7f77658d9d7cd515503fe66e432b35892d0044ac80d37f6e3f0e9b4e85c1d6c7
v21 manifest      363314e291863c2e3ed8230e2a3549fcf6c17d73f913126f937f34e787666edb
v21 commands      dd06328648e7380798136097c758edb9cb52a1d3f4f8af1a5ddf2ebb81bc8fda
r31 video         3c4dbf44dca9cba740c0998c9056c205691a5f7854d43bbc4f79a35b872be88a
914 archive       96ea710307e598a023503cc6361dd0795b924fb51e992dd3c51a1f6763f0ca48
```

826 AprilTag A--L 是独立的动态规划成果线，不能被 r29/r31 固定轨迹重放替代或外推。其
最终执行证据在 `runtime_evidence/826_apriltag_execution_20260827/REPORT.md`。

## 3. P0：接管前冻结

接手后先保持现场不变：

- 不启动、停止或重启 Gazebo；
- 不发布轨迹、不调用控制器、不 reset/detach/松夹；
- 不修改密封 authority、正式 run、模型或原装 STL；
- 不创建备份编号，也不清空系统回收站；
- 不把旧对话、`/tmp` 或历史“当前状态”当成执行入口。

2026-09-04 的快照显示 Gazebo server/GUI 已退出，只剩无数据孤立子进程，controller
manager 不可用。这是时间点信息；新接手者必须重新只读检查，不能假设仍然相同。

## 4. P1：完整阅读顺序

按以下顺序完整阅读：

1. `AGENTS.md`，只以最靠后的日期小节覆盖旧状态；
2. `HANDOFF_CURRENT.md`，取得当前摘要和证据索引；
3. `826_plan.md`，区分第 1--18 节历史设计与第 19--21 节完成状态；
4. `NEXT_CHAT_HANDOFF.md`，确认统一摘要入口和文件名兼容关系；
5. 本文件 `HANDOFF_PLAN.md`；
6. `HANDOFF_PROMPT.md`，需要新开对话时完整复制其中提示词；
7. `QUICKSTART.md`，仅在需要验证或用户明确要求重放时使用；
8. 与用户目标直接相关的 `RUN_SUMMARY.md`、`REPORT.md`、manifest 和 commands。

若文档冲突，以 `AGENTS.md` 最靠后的时间小节和机器可校验的落盘证据为准。

## 5. P2：只读身份复核

最低复核集合：

```text
runtime_assets/candidate3_roundtrip_831_r17/RUN_SUMMARY.md
runtime_assets/candidate3_roundtrip_831_r29/RUN_SUMMARY.md
runtime_assets/candidate3_roundtrip_831_r29/CAMERA_POSE_POST_MID360_R29.json
runtime_assets/candidate3_roundtrip_831_r29/flow_output.log
runtime_assets/candidate3_roundtrip_831_r29/video_contact_sheet.png
runtime_assets/candidate3_roundtrip_831_r29/rosbag_831/
runtime_assets/candidate3_roundtrip_831_r31/RUN_SUMMARY.md
runtime_assets/candidate3_roundtrip_831_r31/CAMERA_POSE_COMBINED_MAP_R31.json
runtime_assets/candidate3_roundtrip_831_r31/final_audit.json
runtime_assets/candidate3_roundtrip_831_r31/video_contact_sheet.png
runtime_assets/candidate3_roundtrip_831_r31/rosbag_831/
runtime_evidence/826_apriltag_execution_20260827/REPORT.md
BACKUP_914_RELEASE.md
```

复核通过条件：

- v21 builder `--self-test` 为 `14/14 PASS`、`writes_performed=false`；
- r31 视频 SHA 正确且完整解码；
- rosbag 控制消息为 `60 = 16/16/16/4/8`，把手接触为 0；
- r17/r29/r31 最终方块为 origin、门 locked、两个 support detached；
- r31 Mid360 定位 valid、720 点、四墙可见；
- 914 sidecar 校验 PASS，并明确其不包含后续 v21/r31。

## 6. P3：根据用户目标分流

### 分支 R：只读状态或证据复核

只读取并报告：当前功能、权威身份、计时、终态、备份边界和 live 健康状况。不得借机
重跑、修复、清理或新建备份。

### 分支 G：Gazebo 重放或重新录像

只有用户明确要求运行/录像后才能进入。现有 v21/r31 是一次性证据，不能原样重用。

1. 重新只读检查进程、窗口、控制器和夹持/support 状态；
2. 安全清理退化现场，但不得启动或复用旧 v21 launch；
3. 雷达地图内重放从当前 v21 builder 链实现下一个未使用版本；候选编号必须重新枚举，
   不能照抄；
4. 新 builder 必须产生新 run id、socket、ready claim、authority，并绑定尚不存在的
   run root 和 rosbag 路径；
5. `--self-test` PASS 后一次性 build；随后用普通 `mkdir` 独占创建 run root，禁止
   `mkdir -p`；
6. 只从新 `commands_831.json` 的完整 launch argv 启动 Gazebo，并等待 fresh 健康门；
7. 继续原样执行新 commands 的密封角色：`rosbag recorder -> support_daemon ->
   preparation`；daemon 启动后立即并发 preparation，不能先等待 socket；
8. preparation 精确 PASS 后、flow 前，以新路径启动独立 GUI 视频录制；视频 recorder
   不在 commands 的五个密封角色内；
9. 原样执行新 commands 的 flow argv；flow 后正常 SIGINT recorder，等待 MP4/SQLite
   刷盘，再做完整审计。

动作通过门必须同时满足：终端 PASS、60 条控制消息、左手把手命令 0、把手接触 0、
躯干 <45 度、方块回原位、门 locked、两个 support detached、机器人回 launch pose、
五个最终 trajectory publisher 为 0。

视频和相机属于非动作权威的交付验收：视频完整解码、相机 pose 有独立记录且前后无
漂移、日志/MP4/SQLite 均完成刷盘。它们不能替代上述动作通过门。

### 分支 D：功能或轨迹开发

先明确属于哪条成果线：

- 修改 AprilTag、动态场景或在线重规划：进入 826 动态规划线，完整保持 601 门目标、
  10 mm 环境净距、4.9 度连续性、正确铰链、正反路径、碰撞/时序和禁止 partial；
- 修改双循环姿态、速度或动作：从 v9/r17 语义线派生新版本，不能改写 v9/v19/v21；
- 只改变相机视角：动作 payload 保持不变，但仍需新 replay 身份和新证据路径。

任何源码或正式资产变化都必须按 `AGENTS.md` 的隔离构建方法重新构建、测试，并记录
源码、资产、manifest、commands 和执行结果的哈希。

#### D1：导航后 AprilTag 把手对准

D1 离线参考系实现已于 2026-09-14 完成。目标仍是：Nav2 到达名义开箱站位并完全停止后，以 Tag 10/12
恢复箱体 pose，通过刚性偏移和门/把手机构状态求出把手 pose，再走 826 动态 IK/碰撞规划。
Tag 11 只作门 pose/铰链关系交叉检查；完整双循环还要继续使用 cube tags。

按以下顺序推进，前一门未 PASS 不得进入后一门：

1. 仅离线修改 worker 的参考系建模：从冻结 PlanningScene 恢复 pelvis 相对动态对象，
   不改 v9/v19/v21 authority，不改密封固定轨迹；
2. 增加 scene/epoch 刚性一致性、有限值、对象拓扑和基座残差包络的 fail-closed 门；
3. 增加纯 SE(3) 单测，证明物体 world pose 不变时，pelvis 平移/偏航仍能产生正确相对
   cabinet/handle delta；禁止直接偏移关节角；
4. 用 attempt11 未改 CDR 派生全新 plan-only 矩阵，完整运行 IK、601 门目标、10 mm 净距、
   4.9 度连续性、正反路径、时序和 no-partial 审计；
5. 使用隔离 build/install/log，并运行受影响包测试；失败必须回退，不得把失败调参留在
   正式源码；
6. 只有用户再次明确授权 Gazebo 联调后，才建立全新身份测试分段闭环。测试顺序必须是
   导航停止 -> 直立/速度/定位门 -> 标签稳定 -> scene freeze -> plan certificate -> 执行；
7. 标签丢失、时间戳过期、姿态跳变、scene/epoch 不一致、机器人未驻足或残差超包络时
   stop-and-hold。夹持/support 期间仍禁止 reset、detach 或松夹。

旧 `(-0.45,0,0)` 仅有等待/接近意义；固定动作名义 robot root 约为 `(0,0,0)`。把该名义
站位变为 Nav2 最终目标会扩大既有导航距离，必须作为独立新合同验证，不能沿用 0.15 m
到站证据直接宣称通过。

D1 当前机器结果见
`runtime_evidence/826_post_nav_apriltag_alignment_20260914/REPORT.md`。worker 已恢复并校验
pelvis 相对动态几何，最终矩阵 16/16 预期匹配；但 0.1 mm/0.01 deg 材料残差均无法通过
attempt11 的 IK/连续性/自碰撞门，最终准入仅为 ±0.5 um 与 ±0.0001 deg 数值零容差。
因此步骤 1--5 已完成，步骤 6 尚不得执行。下一步先走本分支的新增 D1a：离线重设有
可达裕量的名义操作站位或笛卡尔路径，保持 601/10 mm/4.9 deg/正反/时序/no-partial 门，
直到材料级正负残差矩阵完整 PASS。其后才可再次请求用户授权步骤 6 的 Gazebo 联调。

### 分支 C：继续清理

先证明目标是缓存、逐字节重复副本、被正式结果取代的过程文件或失败重型数据。正式
authority、成功 run 核心、视频、rosbag、报告、builder 链和备份不进入清理范围。优先
移入系统回收站并记录精确路径与字节数；永久清空回收站需要用户另行明确确认。

### 分支 B：创建备份

必须由用户明确指定一个从未使用的新编号。先验证目标 archive/sidecar 都不存在，再以
非覆盖方式创建、校验、设为只读并编写 release 说明。不得覆盖 914 或任何保护链。

## 7. P4：失败与停止条件

遇到下列任一情况立即停止当前执行分支并保留证据：

- 权威文件哈希不匹配或目标路径已经存在；
- fresh 门、控制器、遥测、相机或 publisher 门未通过；
- preparation/ready claim 不精确匹配；
- 左手出现把手命令或接触，躯干达到 45 度，轨迹消息计数异常；
- 方块、门、latch、support 或机器人终态未通过；
- 当前处于夹持或支撑状态而恢复动作需要 reset/detach/松夹；
- 请求可能触及实体机器人但没有新的明确授权和独立安全流程。

失败时使用 stop-and-hold；不通过放宽限位、碰撞门、时序门或 partial 来伪造成功。

## 8. P5：交付与再交接

每次任务结束至少报告：

1. 实际完成和未完成内容；
2. 使用的 authority/run id 及关键 SHA-256；
3. 运行计时、控制消息计数和终态；
4. 新建、修改、移入回收站或保留的文件；
5. 当前 live 是否 fresh、是否仍在运行；
6. 是否修改源码/模型/轨迹，是否已构建测试；
7. 最新受保护备份编号，以及新成果是否已包含其中；
8. 下一步是否仍需用户授权或指定新备份编号。

随后按顺序更新 `AGENTS.md` 最末覆盖节、`HANDOFF_CURRENT.md`、`826_plan.md` 的状态节、
`NEXT_CHAT_HANDOFF.md`、本计划和必要的 `QUICKSTART.md`。历史证据不重写，只增加清晰的
最新覆盖说明。

## 9. 当前清理后的接手边界

2026-09-04 已把约 182.92 MiB 的失败/重复/可再生过程数据移入系统回收站；r26/r27 已
从工作区移除，r21--r23 只保留小型失败状态记录。r17/r24/r25/r28 正式核心、全部
authority/builder、826 A--L 证据和受保护备份均保留。接手者不得自动恢复或永久清空
这些回收站内容。

所有当前执行结论严格限于 Gazebo；实体机器人从未获得连接或执行授权。

## 10. 2026-09-14 当前执行覆盖

最新分支 G 已在完整四墙 Mid360 地图中以 v21/r31 PASS。v20/r30 在任何动作前因场景不符
停止并封存。r31 调用至 PASS `167.302430689 s`，控制消息 `60=16/16/16/4/8`，把手
接触 0，方块回原位、门 locked、两个 support detached；Mid360 终态 valid、720 点、4 墙。
视频与 rosbag 完整，最终 launch/Gazebo/recorder/support/socket 全部离线。

未来若再次要求该组合场景重放，必须从 v21 builder 链创建尚未使用的新 builder、authority、
run、run id、socket、ready claim、rosbag 和视频路径；v21/r31 不得复用或覆盖。动作语义
仍由 v9/r17 定义，914 仍是最新备份但不包含 v21/r31。任何新备份仍需用户明确给出新的
未使用编号。

## 11. 2026-09-14 D1 离线覆盖

D1 的 scene/epoch 共同刚性恢复、pelvis 相对 cabinet/door/handle/cube pose、独立 world/
base 包络、纯 SE(3) 测试、attempt11 CDR 派生 runner 和隔离构建均已完成。最终 summary
SHA-256 为 `888734ea2e561d2cce3f112a6694439a23140312d44852601fa5b3c84fb485c3`，包测试
112 passed、1 skipped、0 failed。

该结果同时证明现有 attempt11 没有材料级导航残差裕量；最终准入只相当于 nominal 数值
容差，不能进入 Gazebo 动作验证。下一步是 D1a 离线站位/路径裕量设计。未来 Gazebo 联调
仍需用户重新明确授权、全新运行身份和先行材料级矩阵 PASS；默认保持全程离线。

## 12. 2026-09-14 D1a 第一轮离线结果

D1a 的“只改最终站位”候选集已完成但无材料 PASS：g5 下 18 个非零单轴和 16 个非零
对角站位全部失败；g6--g12 的 IK 分支/局部 detour 试验也失败并回退。正式 g5/h1 最终
矩阵仍为 16/16 预期匹配，但只证明数值零包络。

后续 D1a 应继续保持离线并建立全新 plan-only 候选身份，重设整段门/把手 Cartesian 路径，
显式绕开 door raw 471 奇异区、left wrist yaw 限位和腕/肘对 torso 自碰；完整 601、10 mm、
4.9 度、正反路径、时序和 no-partial 门不可放宽。任何路径候选若 nominal 不 PASS，立即
回退并保留失败证据。

最新报告：`runtime_evidence/826_post_nav_apriltag_alignment_20260914/D1A_MARGIN_DIAGNOSIS.md`。
Gazebo 分支仍未授权；只有用户再次明确授权且材料级离线矩阵先 PASS，才可创建全新身份
进入联调。

## 13. 2026-09-14 D1b 完成覆盖与后续分支

D1b 已完成用户指定的离线固定外参链：Tag 10/12 cabinet authority -> 刚性机构偏移得到
door/handle -> Tag 11 door check -> exact-stamp scene freeze/certificate -> 当前 pelvis 相对
目标重新 IK/FCL/时序。新右手 `826-a6` plan-only admission 为 37/37 预期匹配，其中 24
完整 PASS、13 预期拒绝。报告为
`runtime_evidence/826_post_nav_apriltag_alignment_20260914/D1B_FIXED_OFFSET_OPEN_REPORT.md`。

以后继续本方向时：

1. 只读/解释/复核仍走分支 A，全程不得启动 Gazebo；
2. 继续离线开发走 D1，只能在新 plan-only 身份中扩展残差/时长裕量并重跑完整矩阵；
3. 只有用户再次明确授权 Gazebo 联调后，才可在只读 live/保护哈希复核和全新不可覆盖身份
   下实现“最终 operation goal -> 30 s 驻足 -> fresh tags -> freeze -> certificate -> exact
   plan audit -> 新三段执行 supervisor”；
4. x/y/z ±1 mm、yaw ±0.1 deg、tilt ±0.05 deg 是 prefilter，不是成功保证。包络内仍可能
   自碰，必须逐次完整审计；超包络、陈旧/丢失标签、Tag 11 不一致、未驻足或审计失败均
   stop-and-hold；
5. 完整双循环必须继续使用 cube tags，并单独完成取放/回收规划。不得把右手开门 plan-only
   当完整任务成功；旧 `(-0.45,0,0)` 不得当作已认证操作终点。

Gazebo 仍未授权，新 replay/build/run 编号尚未申请；新备份也仍需用户指定从未使用编号。

## 14. 2026-09-15 分支 G 执行覆盖

用户随后明确授权打开 Gazebo 查看开箱效果。v22/r32 在动作前因 support plugin 哈希不符
fail closed 并封存；恢复 914 内合同绑定二进制后，fresh v23/r33 完成完整四墙 Mid360
固定轨迹双循环，调用至 PASS `176.938775168 s`，动作、rosbag、视频和终态门均 PASS。

本次授权已经用于固定 v9/r17 重放，未实现 D1b 新三段执行 supervisor，也未发布 Nav2
步行。以后再次重放不得复用 v23/r33，必须从当前 builder 链重新枚举全新身份；以后进入
D1b Gazebo 联调仍须用户再次明确授权，并先实现 operation goal、30 s 驻足、fresh tags、
scene freeze、certificate、逐次 IK/FCL 和新 supervisor 的完整 fail-closed 链。

09:55 CST 的历史快照中 v23/r33 Gazebo GUI/server 与基础 launch 仍打开，flow/support/
recorders 已退出且安全终态成立；任何后续操作前必须重新只读检查，不得凭此快照假定 live。

## 15. 2026-09-15 915 模板与当前入口

用户已明确指定并完成 915；它现在是最新不可覆盖的完整工作空间恢复/开发模板：
`/home/liu/h1_moveit/backups/915.tar.zst`，SHA-256
`2148b1f083f4fa28ce647c8e8ca4debf4fc49a92ee26f0bfe5eb3428a9beea0c`。报告入口为
`runtime_evidence/backup_915_release_20260915/REPORT.md`。以后开始新开发时可从 915 的
副本/恢复工作区起步；不得修改 archive 或把模板身份当作动作 authority。

当前 live 已正常离线。分支边界不变：只读复核走 A；D1 离线继续扩展 D1b；新的固定重放
走 G 且必须新身份；D1b Gazebo 在线联调仍需用户再次明确授权并建立新执行 supervisor/
run。新备份必须另获一个从未使用编号，禁止覆盖 915 或任何历史保护链。

## 16. 2026-09-15 步态 r29 离线候选与后续入口

分支 D 的 authorized deadband zero-slew 单变量已在 fresh r29 离线身份中实现并通过纯函数
矩阵、50 项聚焦测试和 122 项隔离包测试。功能默认关闭；无 Gazebo/Nav2/控制命令，严格
连续 30 秒 0.08 m 驻足 PASS 仍为 0。入口：

```text
runtime_evidence/unitree_h12_incubator_nav2_authorized_deadband_slew_20260915_r29/REPORT.md
runtime_evidence/unitree_h12_incubator_nav2_authorized_deadband_slew_20260915_r29/SHA256SUMS
```

若用户下一步只要求解释、代码审查或证据复核，走分支 A 并保持离线。若用户明确要求
Gazebo 步态验证，才继续分支 D：先重新只读核验 live/保护哈希，再枚举并证明全新 identity
未使用；启用参数只能是 `authorized_deadband_zero_slew:=true`，其余 production policy、
controller、gain/limit、goal checker、strict supervisor 和场景必须逐字节不变。必须独立
记录输入/输出交接、物理速度、root pose、support、upright/fault 和连续 30 秒走廊，任何
gate 失败立即 exact zero/stop-and-hold。只有 strict supervisor PASS 才能宣称改善；失败须
封存新身份并恢复 915 基线文件，r29 不得重用。

Gazebo 移动验证仍需用户明确授权；真机从未授权。新备份也必须由用户另给未使用编号。

## 17. 2026-09-16 r30 失败覆盖

r29 zero-slew 的 fresh r30 Gazebo 运行已结束且被严格门拒绝：Nav2 staging goal 成功，但
`STABLE_HOLD_TIMEOUT`，连续 30 秒/0.08 m PASS 为 0。候选已完整回退至 915 基线，r30
封存且不得复用。以后不得继续调 zero-slew 或据单次较小漂移宣称改善。

## 18. 2026-09-16 r31 离线候选与下一入口

用户要求从 `/home/liu/test` 学习后，分支 D 已产生 r31：保留官方 Unitree MOVE，仅为
exact-zero STOP 增加 opt-in V12 learned BRAKE/RECOVER/HOLD，默认 false。34 项聚焦测试、
122 项隔离包测试和 r30 sensor plan-only 矩阵 PASS；未运行 Gazebo，闭环 strict PASS 仍为 0。

若只做审查/复核，走分支 A 并保持离线。若用户重新明确授权 Gazebo 移动，才继续分支 D：

1. 重新只读核验 live、915、v9/r17、v23/r33、826 和 support 哈希；
2. 重新枚举并独占一个从未使用的 identity，不得复用 r30/r31；
3. 只启用 `locomotion_profile:=unitree_h12 learned_recover_stop:=true`，其余 MOVE policy、
   gain/limit、goal checker、strict supervisor 和场景保持不变；
4. 独立记录 BRAKE/HOLD/RECOVER 状态、target、实测关节/root/IMU/support、raw/safe cmd、
   physical/localized pose 和连续 30 秒走廊；
5. 任一门失败立即 exact-zero/stop-and-hold；只有原 strict 30 秒、0.08 m supervisor PASS
   才能保留候选，否则完整回退并封存失败身份。

该分支不授权开箱动作、备份或真机。915 仍是模板且不含 r30/r31。

## 19. 2026-09-16 r60 失败覆盖与新入口

r31 learned STOP 已在 fresh r60 闭环验证中失败：Nav2 到站后 physical corridor 超限，
V12 只有 BRAKE、没有 HOLD，随后 FALL_ENVELOPE。源码已回退 915，`113/113` 测试 PASS。
不得继续启用 `learned_recover_stop`，不得放宽 corridor/fall/strict 门，也不得复用 r60。

当前分支 D 入口改为：先保持离线，从 r60 trace 诊断 MOVE->STOP 状态/动作域失配并提出
新的单变量方案；完成纯函数/回放/隔离构建以前不得再开 Gazebo。10:51 CST GUI 仍保留失败
姿态供用户查看，下一动作前必须只读核验后正常关闭，禁止 reset。未来再次 Gazebo 移动仍
需用户明确授权和全新全局未使用 identity。真机、备份和开箱动作未授权。

## 20. 2026-10-10 分支D：离心机双盒整套，当前主机图形阻塞

本次用户明确要求离心机整套流程，属于独立设备任务开发；不是分支G的孵育箱密封轨迹，
不得执行旧commands_831或改写孵育箱成果。最新r203报告/落盘状态见HANDOFF_CURRENT第80节。
GUI的NVIDIA client580.178.04与kernel580.173.02冲突，启动失败，claim前安全拒绝，无任务动作。
当前相关现场已正常退出；不要反复启动、复用身份或自主改驱动/重启电脑/显示服务。

恢复顺序：

1. 用户重启电脑或修复图形后，先只读确认GPU版本一致、进程/控制器/夹持与保护哈希；
   检查磁盘空间，不为试跑清理任何正式模型、视频/rosbag、成功run或备份。
2. r204只有已准备代码与离线测试，未seal/launch/claim，依赖r203局部物理PASS未满足；
   不直接执行，也不复用r201/r203。枚举未使用新局部身份、普通mkdir独占run与新install。
3. 保留静态腿posture JTC/完整原路径/所有原物理门，只实测末10mm落桌减速假设；
   按AGENTS六库隔离构建、测试、按钮/几何审核后一次seal，再复核成员哈希。
4. 健康和录像启动命令明确exit0才允许claim；精确六控制器active、同一launch的server/GUI
   PID/ctime、相机ACK、存活视频PID/非空MP4全部通过。不得把仍运行session当完成。
5. 局部双篮预置实际双盒卸载、各3sim秒返桌稳定及10sim秒原健康门/独立审计PASS后，
   新建完整双桌盒身份实际装入、5指尖按键、30rpm稳速不少于30sim秒、停稳开盖、双盒返桌。
6. 动作/物理、终态、正常录像SIGINT刷盘与完整解码、前后保护均PASS才称整套完成；
   失败stop-and-hold，不reset/detach/异常松夹，正常SIGINT收尾并保留失败证据。

终态预先声明末盒放稳后双空夹爪开放保持，首盒空手停车仍执行，不承诺双臂neutral。
本场景原world→pelvis固定，不能宣称自由站立、Nav2联动或真机成果。整套尚无PASS；
局部或历史装入/旋转通过不能替代新完整实跑。备份必须另给未使用编号，1009/920不覆盖。

## 21. 2026-10-10 16:43 CST 分支D：全套通过后的速度优化

以r214 REPORT/HANDOFF_CURRENT第90节覆盖第20节历史图形阻塞。两桌盒全流程同身份
物理27/27、录像/无损trace/保护PASS，509.164sim秒；speed_audit FAIL，不称提速或三分钟。
当前已正常SIGINT收尾，不自动重启旧身份。用户授权5°携带/2mm、取回制动瞬时0.1rpm
仿真合同；原2°/全程0.02严格证书未通过，其他原接触/定位/支撑/跟踪/3/10/30秒仍保持。

下一步只读分段复核，优先左盒释放前姿态修正和冗余空手折返/停车，然后独立加速度与
平滑时序验证。以r214为首个完整速度基线，不把失败r206或10倍视频播放当完整速度对比。
任何改动须新未用run/build/install身份、普通mkdir独占、隔离构建/测试/保护、一次seal，
新fresh健康门及窗口录像短测后完整实跑；失败保持，正常SIGINT，原正式资产及证据不覆盖。
原r214密封FULL_CYCLE_PLAN/代码/inputs不得修改；只读可验SHA256SUMS/27物理门/日志/视频。
机器人world→pelvis固定，双盒按持物手分桌槽，末次空夹爪保持不宣称双臂neutral；
不能外推Nav2联动/真机。920箱子版及1009全部保护，创建备份仍须用户明确新未用编号。

## 22. 2026-10-10 17:30 CST：r216提速候选离线通过，外部策略资产P0阻塞

r215只读分析确认首盒空手return指令17.447863080sim秒；r216保留完整retreat和3秒桌证书，
只省首盒空手return，第二盒根据实际另一臂/torso重做IK/FCL，离线7288点PASS。
局部400/配置46/旧装载126及两包六库隔离构建PASS；导航109通过/4失败，因外部
`/home/liu/robot/`内Unitree七项与V12三项原资产缺失。当前导出策略/黄金样本仍匹配。
未seal/launch/录像/claim/动作，完整物理基线仍r214509.164秒，不称已提速或三分钟。
见r216 REPORT/dependency_preflight；先请用户给原文件当前路径，逐项冻结哈希校验，
不得跳过测试/改合同哈希/恢复回收站。来源核验通过后派生全新身份，不复用封存r216。
旧孵育箱/正式源码模型/所有成果与备份保持，无真机、控制发布或新备份。

## 23. 2026-10-10 17:41 CST：用户授权原RL源恢复，官方已恢复、V12仍阻塞

用户确认原RL目录已删除，随后要求“帮我恢复到之前的样子”。本次只恢复该来源依赖，
不是回滚当前工作区、还原整个robot目录或恢复旧Gazebo现场；不执行导航、孵育箱或离心机动作。
已从本地`/home/liu/下载/unitree_rl_gym-main.zip`非覆盖恢复原官方目录，277文件/
150507282字节，7项冻结哈希全部匹配，未改任何正式工作区源码/模型/policy/合同/测试。
当前导航111通过/2失败；V12原checkpoint/env/agent三项仍缺，现存导出策略不能冒充原训练文件。
详见`runtime_evidence/rl_source_recovery_20261010_r1/REPORT.md`；第22节10项缺失是历史快照。

后续仅接收现有其他副本/外部备份中的原V12三文件，并逐项验证原冻结哈希；目标存在则拒绝覆盖。
没有原checkpoint时，不制造假的同名文件、改身份合同或跳过测试以伪造恢复成功。
若要另建仅导出策略的独立部署合同，须用户明确选择，再单独开发/测试/记录原来源不可再核验，
不得替换历史严格证书或宣称恢复了训练状态。全部原依赖及回归通过后才可派生新运行身份；
r216已封存，不直接续跑。新备份/回收站恢复/真机和新的Gazebo动作仍需各自明确授权。

## 24. 2026-10-10 17:53 CST：官方重新下载已核验，手部任务分线待选择

用户授权重新下载官方Unitree目录后继续工作。新官方提交276801e46c5d433564f24658bac64f254b7d2d4b
的277文件内容与当前已恢复目录一致，7项冻结身份匹配；无需覆盖目录或改变正式来源。
官方归档不含用户V12原checkpoint，复测111通过/2失败；第23节剩余三项缺失仍有效。
见`runtime_evidence/rl_official_download_20261010_r1/REPORT.md`。

已继续只读依赖与速度预算分析，并请用户选择是否单独采用固定骨盆离心机手部合同，
保留V12原来源缺失/失败事实，暂不执行腿部导航。当前尚未明确选择，不改变原P0门。
如果用户选择分线，下一新身份先完整审查传递依赖、固定world→pelvis/静态腿控制、
禁止RL/导航速度运行的边界；执行完整原回归并保留两个V12来源失败，同时单独证明
所选手部合同所有相关测试/源身份/隔离构建均通过，不能把原全回归称为全PASS。
只有新合同对应gate明确通过后才一次seal/fresh健康/录像/同身份全流程，旧r216不复用。

省首盒空手return的17.447863080秒只是原指令反事实，约491.716秒仍高于180秒；
下一步短空手路径/FCL及载荷跟踪感知时序验证，不以快放、删稳转或盲目倍率冒充提速。
正式孵育箱/离心机/导航成果及全部备份保持；真机、备份、回收站和导航动作不随本选择授权。

## 25. 2026-10-12 CST：用户结束V12原件找回，仅重新下载官方目录

用户确认训练目录放入回收站后已清空、无其他副本，随后明确“没有备份也恢复不了，
你重新下gym_sim/unitree_rl_gym-main/就行”。当前任务限定为重新获取官方来源并核验，
不再主动继续磁盘恢复、安排Windows切换或要求准备Live USB/外置盘。未执行底层恢复，
不能将用户的任务选择写成原文件绝对不可恢复的技术证明。

已再次下载官方main对应提交276801e46c5d433564f24658bac64f254b7d2d4b，277普通文件/
150507282字节与现有原路径全部逐字节一致、7项原冻结身份匹配；正确目标不覆盖。
ZIP59750482字节，SHA-256 0f43bf9a9e1310b155cb0496facc6a1cf4924a387f11cec3b73be30213c475a8。
见`runtime_evidence/rl_official_redownload_20261012_r1/REPORT.md`。本次导航完整回归为
111通过/2失败，仍因原V12来源缺失；不改测试、预期哈希或历史严格证书来消除失败。

本任务完成官方下载，不恢复用户私有训练状态，也不自动选择第24节的新独立手部合同、
启用腿部导航或授权新Gazebo动作。后续具体任务按所选成果线单独核验相关合同/依赖/
隔离构建与新fresh身份；保持原来源缺失记录，旧r216不直接续跑。原孵育箱/离心机/
导航模型、正式证据与全部备份继续保护，本次无新编号备份、回收站操作或真机连接。

## 26. 2026-10-12 CST：用户授权补写独立仿真层，r217场景启动PASS

用户“继续运行仿真，缺的东西重新写一下补上”是本轮新仿真授权；不制造原V12同名
checkpoint或改旧冻结哈希，而是按实际依赖补固定骨盆场景的独立启动合同。
r217新root/inputs/run/install独占；25父级代码/输入只换身份路径，源几何/控制payload
及二进制不变。82相关资源、27新测试、46配置Python及6项CTest、两包六库隔离构建
通过；原导航完整回归111通过/2失败原样保留，新分类只接受已知源缺失FileNotFoundError，
未知失败/节点/启用步行/自由骨盆均拒绝。原训练和腿部严格合同未解除。

真实fresh r217 server2781965/GUI2781966、六控制器active，30.000sim秒持续定位有效/
31关节有限/机器ready且转子停止；两个独立桌盒及完整原场景模型存在，GUI72帧短测
全解码通过，场景保留运行。见`runtime_evidence/centrifuge_48box_scene_resume_20261012_r217/REPORT.md`。
这只是startup-only健康证据，不是双盒动作、Nav2驻足或三分钟全套任务证据。

后续先重新只读查live及持物/支撑/机器状态；当前健康场景不能重复启动。若具体目标
进入完整双盒动作，须安全收尾不持物旧场景并建立全新身份/完整手部任务合同，完整
相关回归/碰撞审核/一次seal/fresh视频门后执行；r217启动scope不直接扩展，r216不复用。
腿部导航/原训练源重建另立成果线，明确新来源、不能冒充已删除原文件。旧孵育箱/
离心机/导航源码模型、正式证据、重要失败和全部备份继续保护；无真机/新备份/清理授权。

## 27. 2026-10-12 CST：分支D完整手部独立合同与r219实测提速PASS

用户明确继续两盒装篮/关盖/旋转/开盖/取回并尽量快，是完整手部Gazebo新授权。
独立固定world→pelvis合同审核177资源/完整Python及MoveIt/FCL传递链，无RL/导航动作；
617相关Python/6项CTest与两包六库隔离构建PASS，原导航111通过/2失败保留。
V12原checkpoint缺失不属于本范围，不再以其统一阻塞已审核手部任务；原训练/腿部严格
来源合同不变，不伪造checkpoint、不改原测试/冻结哈希，未来按实际所选完整依赖审核。

r218录像not-negotiated与后台继承SIGINT忽略导致FAIL；暂停触发原stop-and-hold后正常
SIGINT收尾，无reset/detach/松夹，264失败证据保留。r219入口显式恢复SIGINT默认处理，
守卫查/proc确认可送达，实际子进程继承忽略恢复及真实执行器前置通过；Popen父监督。
指定Gazebo窗口固定区域录像、120帧奇偶尺寸调整预检及72帧真实窗口门通过；
gzip1日志无抽样/原行字节SHA/CRC保持，轨迹/物理/3/10/30秒门不变。

r219同一fresh身份完整27/27、视频全解码/无损trace/保护/速度PASS；
471.314sim秒、wall4924.442240秒，比r214省37.850sim秒/7.433754%、wall123.100971秒。
首盒桌证书及完整retreat后省1次冗余空手return；下一手真实双臂场景重新IK/FCL。
全部改善不归因于17.4479秒单段；三分钟完整流程仍未实现，10倍视频不算动作速度。
实测携带1.548617°/独立滑移0.901623mm/取回微动0.005490rpm；
运行合同仍已授权5°/2mm及取回有限制动瞬时0.1rpm，不修改r214旧严格失败。
末次双盒分桌槽/双空爪保持10秒，不宣称每盒出生位、双臂neutral、Nav2联动或真机。

最新完整报告runtime_evidence/centrifuge_48box_fast_complete_20261012_r219/REPORT.md，
SHA-256 b86f6df5238e75f7aa1ed5c40f393fe99a81737a0da0fba3f5b346d12faca77c。
r217/r218/r219均已正常退休；先重查live，不能复用旧claim或已密封代码。
下一提速以r219为新完整基线，优先审计采样与控制采样解耦/平滑时序及短空手路径，
必须新未用身份、独占目录/构建/测试/保护/一次seal/fresh门及完整实跑。
禁止盲目倍率、删稳定证书、源资产覆盖、清理/新备份/真机；920箱子版本继续永久保护。

## 28. 2026-10-12 16:28 CST：分支D水平提速r225单次完整PASS，稳定余量仍需改善

用户“继续优化速度”。r225最新同一fresh完整27/27、速度/无损日志/视频/前后保护PASS，
465.9815sim秒（2.293→468.2745），wall4744.791356秒；比r219省5.3325sim秒/1.131411%，
wall省179.650884秒。三分钟完整仍未实现，不能把单次wall差异或快放当重复性能证书。
报告`runtime_evidence/centrifuge_48box_horizontal_rate_20261012_r225/REPORT.md`，
SHA-256 e286bbc7c8217fc1cc27358aeb3d79b9a19bbedb4774f533e74d4d6d778d54da。

只四个已认证自由水平transfer×1.15；率为left/right装载0.07475/0.1495，取回0.115rad/s。
全部原垂直/接触/试抬/抽离/落桌/抓放/按键、20ms网格/0.5ms物理、3/10/30秒保持。
保留取回合爪观察上下文hook，只显式retrieval_closing=True的grasp适用原取回机器门，
goal原phase/16N/6秒软合爪不改；21例原故障快照/边界回归和双合爪真实反馈通过。
657相关Python/6项CTest/两包六库隔离、180传递资源/178成员seal复核通过。
原导航111通过/2原V12缺源失败保留，固定骨盆手部不使用私有训练源，不改旧合同。

本轮尝试必须区分：r220五次端点无收益、r222密点只预计0.06秒，均无物理执行；
r221仅预留。r22325%/15ms装载与机器通过，但取回合爪误用装载0.02rpm门而FAIL；
r224修上下文后首盒4.117678°超原3°校正窗口，未进入IK/FCL即FAIL，不是高度问题。
r225撤回25%/15ms和装载lift提速；旧报告/视频/trace及各失败保护链不改，不推广失败参数。

当前实测携带4.222334383°（left_retrieval_setdown）/滑移1.449929176mm，
取回微动0.021285175rpm、旧0.02超限1样本；既有5°/2mm/制动取回0.1rpm合同PASS。
旧2°/全程0.02rpm本次未通过，不能宣称比r219更稳定或新严格证书。
单次新完整速度基线r225与较低角偏差回退r219并存，正式源码/模型未推广修改。

下一可验证方向：先降低左盒取回落桌累计角漂移，或审计缩短空手路径/平滑时序；
距5°只余约0.778°，不再直接提高带载倍率。必须重新IK/FCL/跟踪/载荷/机器门证明，
不得扩大0.25°释放或3°校正窗口、拆多次校正绕过边界、加假attach或缩短3/10/30秒。
未来每次仍新未用身份、普通mkdir独占、新install/完整回归/前后保护、一次seal、
fresh健康/实际录像、同身份全套与独立审计；不能复用r225 claim或修改密封输入。

r225录像与场景正常SIGINT退休，末次相关残留0；接手必须重新只读核查live。
r223/r224失败均stop-and-hold/暂停/正常SIGINT，无reset/detach/异常松夹。
10262历史保护文件/全部后续链/10备份、785正式源码前后PASS；920永久箱子版本、
孵育箱v9/r17/v24/r103、离心机/导航源码模型policy和所有重要成果保持。
本轮没有清理/回收站操作/新备份/真机/腿部导航授权。

## 29. 2026-10-12 CST：三分钟目标需结构重构，r225视频内容FAIL覆盖

用户询问全流程三分钟。按已约定sim180秒，不改变计时口径/稳转30秒或原3/10秒证书。
r226新独占离线审计身份centrifuge_48box_three_minute_budget_20261012_r226，只有封存
日志/视频分析器及结构计划，无launch/controller/builder/identity/run/claim或仿真动作。
32分析器测试PASS，原r225304封存文件及全部保护链/10备份/785正式源码前后PASS；
不把离线分析PASS当新动作/三分钟PASS，没有正式ROS源码变化，无新隔离构建。

465.9815sim秒需要减少285.9815秒/61.371857%。39臂指令225.139196秒、8按钮空手
29.506143秒，其余不变的2/3/5倍反事实338.658830/296.217940/262.265228秒；
即使这两类运动归零仍211.336161秒。这是特定其他时序不变的反事实，不是通用下界，
不证明结构重构后不可能三分钟，也没有证明大倍率跟踪/夹持安全。

计划runtime_evidence/centrifuge_48box_three_minute_budget_20261012_r226/STRUCTURAL_REDESIGN_PLAN.md：
先双持物独立守卫及同一sim时间轴双臂/双盒/全场景FCL，不能两线程直接跑旧单holding；
再从实际场景重求短连接、完整10秒/3秒观察窗口重叠、真实按钮预位与平滑控制。
173秒分配只是阶段设计目标，没有路径/控制/实跑证书，禁止宣传为已有三分钟成果。
每盒10秒/3秒必须独立完整，不能以并行名义清掉另一手状态或漏检碰撞/支撑。
只有新预算/全部相关测试/碰撞/隔离构建、前后保护/一次seal/fresh门通过才完整实跑；
实际≤180且全动作/物理/录像内容/全解码/trace/保护均PASS才能验收，旧身份不复用。

用户要求看视频后的实查纠正：r225原速和10倍预览约原速1分半之后的抽查关键阶段
主要是地面及机器人边缘，离心机/盒子/双爪在录制区外；视频展示FAIL。
起始窗与录制1200×900，末次独立截图3676×2022；推断固定左上录制未跟随后续放大，
确切resize事件未记录。原视频/codec审计/正式报告/SHA不改，旧解码PASS保留，
物理/时间独立PASS不等于录像内容PASS；末态截图不能补成完整动作录像。
新运行前录像必须四角/移动主体测试、全窗或精确窗口持续约束/主体可见性守卫、
真实关键动作分段抽帧，不能只验证MP4完整解码。

本轮只读live无相关仿真/动作进程，未启动或停止Gazebo/发布控制/reset/detach/松夹。
正式孵育箱/离心机/导航模型源码/重要成功与失败证据/builder链和全部备份不改，
920永久箱子版本继续保护；无真机/清理/回收站/新备份或腿部导航。

## 30. 用户授权连续实现180sim秒：500Hz证据与新带载候选

第29节预算/双臂同步是历史设计方向，不是只能选双持物的强制实现。
r227原站位单holding完整流程461.299sim秒/27门PASS，全窗口录像取景修复，
17关键相位原速抽帧确认主体留在画面，远桌盒/手仍有自然遮挡；盖下旋转由物理日志认证。
r228独立500Hz空臂组件实际0.8rad/s跟踪峰0.0064rad，原0.015/0.0003门通过；
原站位/P0.2/0.5ms/模型质量惯量摩擦保持，仅组件PASS，不是完整任务或带载证明。
末俯视已真实录进视频再SIGINT，两个盒/双爪可见。r227367/r228333静态文件封存受保护。

用户在本线程已明确授权直接修改/测试/fresh实跑至完整≤180sim秒，不等每步“继续”，
目标和权限不扩展成腿部导航、真机、清理/回收站或新备份，也不使旧身份可重用。
按AGENTS最末小节/最新REPORT/live只读核对接续，选择有证据的低风险根因改动，
新普通mkdir、隔离构建/相关完整回归/保护/一次seal/fresh六控制器/实际频率/录像内容门。
所有已有碰撞/跟踪/滑移/角偏差/接触/末态及3/10/30秒证书仍保留。
新带载倍率是未证候选，必须独立物理/full trace/时间/实际视频/前后保护通过；
任何失败实际stop-and-hold并按PID/ctime正常SIGINT，不用预算/空臂或快放宣称180已完成。

r229当前候选见ACTUAL_RUN_SCOPE.md与FastCycle：500Hz，空手0.8/自由带载0.5，
pilot/末10mm/姿态校正保守保留，按钮接触点20ms及实际接触偏移计算稳转停止时间，
取回软合爪2s但原16N/18N/0.2m/s/真实双指反馈不变。若仍超180，按实测改短连接/
连续时域路径，不能直接删除独立稳定证书。当前不可写成新物理或三分钟PASS。

## 31. 2026-10-15：完整三分钟验收已通过，后续仍须新身份

第29–30节历史预算/未达状态由r283正式报告覆盖：首非空sim10.1915→完整
PASS189.3205为179.1290sim秒，包含双盒实际装篮/真实指尖关盖/30.258sim秒稳转/
停稳开盖/双盒返不同桌槽与全部3/10/30证书。独立28/28/双持物/原片全解码与
20相位内容/无损trace/保护全PASS，九身份正常SIGINT退休。不是现实三分钟、
重复可靠性、自由站立或导航后自动操作证书。正式源码/模型、旧资产/所有备份保持。

接手、总结、查看录像或证据核对走只读分支；不可直接重跑r283或任一旧claim。
用户明确要求再次展示完整流程才从已验证r283链派生新未用builder/候选/run/
安装overlay和视频路径，普通mkdir独占，隔离构建/所有相关回归/保护/依赖/一次
seal/fresh六控制器和实际频率/录像首秒与持续广角门后，由连续执行器独立完成。
最终仍核验首有限非空至完整PASS≤180、28门、独立双持物、全trace及原片内容/
全解码，正常视频后场景SIGINT；失败stop-and-hold，不reset/detach/异常松夹。
原Nav111/2旧V12缺源不阻塞已审核固定骨盆手部范围，也不赋予腿部/真机许可。

本次实际桶轴退出已通过；新增可选四段退路和11目标微量alignment未触发，
只有单元/离线资格。如要验证该分支，必须新身份实际触发并完成全部逆退路和
整套物理审计，不能用离线0.597861sim预算或三段成功替代。接口见
CENTRIFUGE_FULL_CYCLE.md，正式结果不改写，清理和新备份仍需另行授权/未用编号。

## 32. 用户要求现实三分钟：先实际1:1，再新完整身份验证

旧r283179.129sim秒成功保持，不满足新现实180秒目标。原target1.0不等于实测，
七个无任务静止对照最快.135倍实时；慢引擎/求解器/渲染/编译选项未推广。
本轮三probe全部正常SIGINT/七组保护保持，详细证据见wallclock_perf_20261015_r3。
现实计时单元和连续稳转≥30wall且≥30sim单元通过，尚无新的完整实跑资格。

下一先量化单物理step/持续接触和在线程序成本；优先保留原网格/步长/增益/
路径/物理门和全部观察证书，提高实际速度。只有实际1:1前置门通过才新未用
完整身份跑原流程，不重复已知几十分钟的完整任务。不改sim时间戳、统计起点
或影片播放速度；在线规划、等待及最终返桌/终态检查必须计入monotonic墙钟。
如需改变步长/控制采样或简化非接触网格，另列精度与物理资格合同，不借用旧
500Hz/.5ms证书。旧模型/authority/run/视频/报告/rosbag/builder链和所有备份不动。

## 33. 2026-10-20：r327分开抓快版完成后的接续合同

本节按用户后续“分开抓、保护r318、放宽仿真门限并提速”的要求覆盖第29–32节
相冲突的历史当前状态和执行先决条件；不是修改旧版本的验收结果。当前摘要统一见
[HANDOFF_CURRENT.md](HANDOFF_CURRENT.md)，本轮失败链、参数及完整接口见
[CENTRIFUGE_FULL_CYCLE.md](CENTRIFUGE_FULL_CYCLE.md)。

- 新r327完整顺序装篮→实体关盖/启动→稳转30.168sim→停稳开盖→双盒返桌/空手终态
  通过，234.8065sim/615.775707219wall秒；28/28当前合同及原片/独立核验通过。
  r318的428.45sim/3894.946321249wall秒原成功完整保留；新旧合同不同。
- r327采用显式5mm/10度、抓稳1/释放0.5/终态1sim秒等快版门，稳转30秒不减；
  峰滑移2.158579mm/角偏4.218821度，不满足旧2mm/2度。原密FCL、关节限位、
  实体双指、原生反馈和不可重设锚点保持；visual-only碰撞缺口如实保留。
- 第32节“必须先证明1倍才可任何完整实跑”不再适用于这条已明确授权的新合同
  集成验证；仍须如实分别记录实际RTF、完整sim和monotonic墙钟。
  **现实180秒、当前分开抓sim180秒、稳定1倍均未达**，不能改时钟或用快放充数。
- 原r283是旧双臂同时版179.129sim完整成功，不作为新姿态的速度或严格合同证明。
  当前完整证据为[r327报告](runtime_evidence/centrifuge_48box_sequential_full_20261020_r327/REPORT.md)
  与[review_r15](runtime_evidence/centrifuge_48box_sequential_review_20261020_r15/REPORT.md)。

当前人类消息仅要求整理本轮MD：执行文档编辑和只读核验，不再启动仿真或发布动作。
新对话目标留空只读R；明确继续离心机提速/运行走D，孵育箱重放才走G。

后续D分支必须重新查live、夹持与保护，普通mkdir独占新未用builder/run/domain/
partition/overlay/日志/录像；把r318/r327及两个独立review的清单都列入保护。
按AGENTS六库隔离构建及相关回归、一次seal/fresh六控制器/真实原腿初始化与实际
首帧门，再由连续执行器整套调度，不用AI逐步临时插动作。私有脚底代理只限固定骨盆，
不能扩展为步态/真机模型；严禁复用封存run、claim或覆盖任何成功版本。

准备、测试、保护用tools/trace_archive/run包装并保留继承环境；原日志迁移清单用
archive_cli.py verify-manifest，不直接对历史raw缺失路径用sha256sum -c。
新实跑按独立物理/抓持/实际路径/全trace/原片内容及解码/终态/保护完整审计；
失败stop-and-hold，按精确身份正常SIGINT；不reset/detach/异常松夹或瞬移/物理绑定。
下一优化优先测执行器重复计算/序列化及物理接触成本，现有CPU占用不是函数级根因。

本轮MD整理原文及校验见[整理审计](runtime_evidence/documentation_roundup_20261020_r1/REPORT.md)。
本计划第1–32节保留原字节历史，不将旧“当前”覆盖本节；AGENTS仍是唯一累积权威。
正式模型、孵育箱/920箱子版、authority/run、视频、rosbag、报告、builder链及所有备份
均保护；本次无清理、恢复、新编号备份、腿部导航或实体机器人授权。
