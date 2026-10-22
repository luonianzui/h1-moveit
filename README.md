> **GitHub 同步说明（2026-10-22）**：本仓库是截至 r347 的源码、模型、配置与精选成果快照。完整录像、逐帧日志和历史大数据仍保留在原工作站，尚未上传其他存储。文件目录见 `LOCAL_DATA_INVENTORY.jsonl.gz`，范围与限制见 [SYNC_STATUS.json](SYNC_STATUS.json) 和 [LOCAL_DATA_STORAGE.json](LOCAL_DATA_STORAGE.json)。历史脚本保留绝对路径及一次性运行身份；此快照尚未验证可在新电脑直接运行，原始封存清单中的部分大文件和二进制未随仓库分发。

# H1-2 Gazebo 孵育箱、离心机与导航项目

ROS 2 Jazzy / MoveIt 2 / Gazebo Harmonic；H1-2、双Dex1、D435、Mid360。所有成果仅限仿真，实体机器人未授权。

## 当前入口

**[HANDOFF_CURRENT.md：当前成果与交接总览](HANDOFF_CURRENT.md)**，更新于2026-10-22。累积规则及历史证据仍以[AGENTS.md](AGENTS.md)最新日期小节为准。

## 0. 2026-10-22最新覆盖：r347缩短空手间隔，完整通过

[r347报告](runtime_evidence/centrifuge_48box_sequential_full_20261022_r347/REPORT.md) · [完整原速录像](runtime_evidence/centrifuge_48box_sequential_full_20261022_r347/run/full_cycle.mp4) · [独立复核](runtime_evidence/centrifuge_48box_sequential_review_20261022_r5/REPORT.md) · [间隔与性能](runtime_evidence/centrifuge_48box_sequential_review_20261022_r5/PERFORMANCE.md)。

- 完整双盒装篮→关盖→稳转30.154sim→停稳开盖→双盒返桌：**240.219sim / 584.977490836现实秒**，28门通过。现实三分钟、当前分开抓仿真三分钟及稳定1倍仍未达。
- 11次明确空手自由阶段调度0.30→0.16sim，减少1.54sim缓冲，终点观察重叠最多另省0.55sim；持物/近接触/夹爪/按键原等待保留。全程连续执行器，无AI逐动作接续。
- 相对r345完整仅少1.283wall，非受控A/B、不称显著整体提速。单planner22请求102.710wall、RTF0.4079；本次多1次右返桌正支撑校正，1.738176→0.740523mm，左0.169826mm。
- r346全局缩短在右取回试抬触发-0.171037rpm超原0.1门，470.698wall仅失败前缀；[失败报告](runtime_evidence/centrifuge_48box_sequential_full_20261022_r346/REPORT.md)及原片保留。r347收窄范围、门限不改，不宣称唯一根因已确认。
- 2240测试两次/46/122/6CTest通过，Nav111/2旧V12缺源保留；151发布、4锚点4释放、单holder、22路径24574密FCL。抓持2.602166mm/4.608764°符合现5mm/10°，非旧严格证书；原visual-only碰撞缺口仍在。
- 原速629.25秒全解码，终态俯视双盒返桌/空爪空篮/盖开；正常SIGINT退休、owned空，下次重查live。九组前后保护相同PASS；r318/r327/r331/r333/r337/r342/r345、孵育箱/920版、全部备份不改。
- 源655项SHA `12127f531a60c98a0cc0075958c894d07ffb98df2940d4f225f19549b947c985`；993核心SUM `2be5611074bf221cb7f63765fcbf300a55e2bd08075394f76a3d2c5353de06bf`。

**本节和AGENTS最末小节覆盖以下历史“当前/最新”；旧报告不改，后续必须新身份，不重跑sealed资产。**

## 历史：r343–r345（非当前）

[r345报告](runtime_evidence/centrifuge_48box_sequential_full_20261022_r345/REPORT.md) · [完整原速录像](runtime_evidence/centrifuge_48box_sequential_full_20261022_r345/run/full_cycle.mp4) · [本轮复核](runtime_evidence/centrifuge_48box_sequential_review_20261022_r3/REPORT.md) · [DART性能证据](runtime_evidence/centrifuge_48box_sequential_review_20261022_r3/PERFORMANCE.md)。

- 新r345完整分开抓双装篮→关盖→稳转30.181sim→停稳开盖→双盒返桌：**242.2705sim / 586.260130234现实秒（9分46.26秒）**，28门/4锚点4释放/最多1holder/21路径24529密FCL通过。
- 比r337完整600.442秒少14.182秒，较r342少63.456秒；非同快照受控A/B，不把全部差值归因DART。**当前本链完整较快记录为r345，但现实/当前分开抓sim三分钟和稳定1倍均未达。**
- 新隔离DART约束指针/维数缓存与同序矩阵拷贝；105组矩阵/冲量/回调和1200步接触世界逐字节对照，组件约1.1534倍。系统库/接触/步长/夹力/正式模型不改。
- r343首次撤臂跟踪FAIL；r344双装篮后定位FAIL，原视频/日志保留。r345继承0.6rad/s空手撤臂，并改整数纳秒定位周期，原1秒源龄/质量/真实戳不改。1209pose、962健康检查有效，最大源龄0.2705sim；不证明历史输入空窗已永久解决。
- 抓持峰0.931135mm/1.166645°，取回转子峰0.035353rpm<原0.1门；左/右返桌误差0.119767/0.314852mm。桌面有界校正未触发；原visual-only碰撞覆盖缺口及指速0.2旧门越界2样本保留，不外推旧严格证书或真机。
- 2205候选测试/46包/122装盒/6CTest通过，Nav111通过/2旧V12缺源不变。两个旧post定位统计因启动clock前9条None而失败，原脚本/日志保留；独立revision2仅修统计，7回归测试及全部健康/源戳核验PASS，不把原post写成全0。
- 世界RTF0.420246；53365主线程CPU样本中BoxedLcp累计52.9673%，单位脉冲/速度传播仍明显。下一步新身份优化重复取数，保留物理求解/抓稳/30秒；不靠改时钟、删接触或快放凑达标。
- 原速642.833333秒/7714帧全解码，终态俯视双盒返桌/双空篮空爪，正常SIGINT退休且owned残余空；下次仍重新查live。九组前后保护PASS，r318/r327/r331/r333/r337/r342、孵育箱/920版/三模型/十备份均不覆盖。
- 源648项SHA `d202078cba7d45d07a03b254d85f2047d0a9bdbaa63bbd1bfde6f7b54958508a`；980核心SUM `8029db4d7b0523d504eee48d4b05bfeb8571f55f8df4de76a076f6ee2e17c16a`。新库build被显式绑定，不能当无用缓存删。

**本节覆盖后文旧“最新/当前”描述；旧报告不改、任何sealed身份不得直接重跑。**

### 上轮保留结果（历史）

- **r342最新完整成功**：分开抓双装篮→关盖→稳转30.1865sim→停稳开盖→双盒返桌，234.032sim/649.716164822现实秒。转子峰0.072811329rpm通过原0.1门，28物理门/21路径24538密FCL通过；[完整原速录像](runtime_evidence/centrifuge_48box_sequential_full_20261021_r342/run/full_cycle.mp4)、[报告](runtime_evidence/centrifuge_48box_sequential_full_20261021_r342/REPORT.md)。**仍未三分钟，r337仍较快**。
- [本轮复核](runtime_evidence/centrifuge_48box_sequential_review_20261021_r10/REPORT.md)：r341返桌失败保留；r342新增有界桌面校正未触发，2164测试/46/122/6CTest通过，九组前后保护为封存门。r337及r318/r327/r331/r333、孵育箱/920版/全部模型/备份不覆盖，录像全解码并正常退休。
- [主线程性能证据](runtime_evidence/centrifuge_48box_sequential_review_20261021_r10/PERFORMANCE.md)：59609真实CPU样本，精确符号复核物理更新累计89.4%、约束求解58.2%，包含比例不可相加。世界RTF约0.355；后续测物理约束成本，不将AI等待或视频快放当性能证据。

### 保留历史（以下“最新/当前”只指当时）

- **r338/r339/r340最新性能实验未完整成功**：[汇总](runtime_evidence/centrifuge_48box_sequential_review_20261021_r8/REPORT.md)、[r340失败过程录像](runtime_evidence/centrifuge_48box_sequential_full_20261021_r340/run/full_cycle.mp4)。最终2135测试通过，但试抬转子瞬态仍超门，实际0.05rad/s时0.377062651rpm、制动达原-2N·m夹限；两盒未返桌。约0.409RTF，未达1倍/现实三分钟。三次均正常退休且旧成果保护通过，当前成功基线仍为r337。

- **r337最新完整PASS**：分开抓双装篮、关盖、稳转30.233sim、停稳开盖、双盒取回；232.1135sim/600.442257241现实秒，较r333少44.310秒，仍未三分钟。[原速录像](runtime_evidence/centrifuge_48box_sequential_full_20261021_r337/run/full_cycle.mp4)、[报告](runtime_evidence/centrifuge_48box_sequential_full_20261021_r337/REPORT.md)。定位最大源龄0.3755秒、原1秒门不改；28项物理核验与旧成果保护通过。

- **此前r334/r335/r336优化未完整成功**：复用规划进程、连接自由搬运段，r336最终2102测试PASS，但开盖定位源龄1.0025sim超1秒门，取回未执行。旧r318/r327/r331/r333及孵育箱成果不改。[本轮汇总](runtime_evidence/centrifuge_48box_sequential_review_20261021_r4/REPORT.md)、[r336原速失败过程](runtime_evidence/centrifuge_48box_sequential_full_20261021_r336/run/full_cycle.mp4)。现实三分钟仍未达，不以失败前缀作为速度结果。

- **保留r333完整PASS**：定位最新样本订阅修正、原1秒门不改，分开抓全流程239.8095sim/644.752285933现实秒；[原速录像](runtime_evidence/centrifuge_48box_sequential_full_20261021_r333/run/full_cycle.mp4)、[报告](runtime_evidence/centrifuge_48box_sequential_full_20261021_r333/REPORT.md)。末尾俯视可见双盒返桌，r318/r327/r331哈希保持；仍未达三分钟或稳定1倍。以下为保留版本。

- 离心机分开抓抬肘快版**r327完整PASS**：双盒装篮→关盖→稳转30秒→停稳开盖→双盒返桌；234.8065仿真秒、615.775707219现实秒，原速录像649.333秒。
- **r318成功版本完整保留**；r327采用另列的放宽仿真合同，旧严格门未全部通过。现实三分钟、当前分开抓仿真三分钟及稳定1倍均未达。
- **保留r331完整PASS**：双盒均已取回，239.198仿真秒/621.132718628现实秒；[完整原速录像](runtime_evidence/centrifuge_48box_sequential_full_20261020_r331/run/full_cycle.mp4)。新增预压等待实跑未触发，不能声称间歇故障已充分验证解决；r327仍较快。
- 此前r332缓存候选在首次开盖后因定位源过期停止，尚未抓盒；[原报告](runtime_evidence/centrifuge_48box_sequential_review_20261020_r20/REPORT.md)。此前r328–r330失败原件保留，不覆盖任何成功版本。
- 历史r283双臂同时版179.129仿真秒完整成功不改；不能用它替代当前姿态或现实计时。
- 孵育箱v9/r17语义权威、v24/r103完整地图重放和**920箱子版本**永久保留；导航、AprilTag动态联动单独验证。

## 文档导航

| 文件 | 用途 |
|---|---|
| [HANDOFF_CURRENT.md](HANDOFF_CURRENT.md) | 唯一简明当前状态、证据、备份与live边界 |
| [CENTRIFUGE_FULL_CYCLE.md](CENTRIFUGE_FULL_CYCLE.md) | 本轮初始化/提速/失败链、验收与后续接口 |
| [QUICKSTART.md](QUICKSTART.md) | 查看原片及只读校验，不自动运行 |
| [HANDOFF_PROMPT.md](HANDOFF_PROMPT.md) | 可复制的新对话提示词 |
| [HANDOFF_PLAN.md](HANDOFF_PLAN.md) | R只读、D开发、G孵育箱重放等分支，当前快版见第33节 |
| [SCENE_VERSIONS.md](SCENE_VERSIONS.md) | 箱子/离心机场景区别与920恢复边界 |
| [H1_2_MID360_NAVIGATION_PLAN.md](H1_2_MID360_NAVIGATION_PLAN.md) | 雷达定位/步态/导航独立成果线 |
| [826_plan.md](826_plan.md) | AprilTag历史设计与完成状态覆盖 |
| [AGENTS.md](AGENTS.md) | 唯一累积式权威，历史只追加 |

[r327正式报告](runtime_evidence/centrifuge_48box_sequential_full_20261020_r327/REPORT.md) · [独立复核](runtime_evidence/centrifuge_48box_sequential_review_20261020_r15/REPORT.md) · [完整前视原速录像](runtime_evidence/centrifuge_48box_sequential_full_20261020_r327/run/full_cycle.mp4)。

封存run不能原地重跑；后续必须明确目标并建立全新身份。此前只整理文档的原文和核验在[文档审计](runtime_evidence/documentation_roundup_20261020_r1/REPORT.md)；本轮后续性能实跑另有新证据，不删除正式模型、源码、视频、报告、rosbag、builder链或备份。
