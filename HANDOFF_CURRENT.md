# H1-2 当前成果与交接总览

更新：2026-10-22 CST，r348–r351优化均未完整通过，r347仍为完整成功基线，现实三分钟未达。本文是唯一简明状态入口；[AGENTS.md](AGENTS.md)仍为累积式权威。所有成果仅限Gazebo。

## 0. 2026-10-22最新覆盖：r348–r351优化未完整通过，r347成功基线保留

[本轮汇总与接续](runtime_evidence/centrifuge_48box_sequential_review_20261022_r9/ROUND_SUMMARY.md) · [r351失败报告](runtime_evidence/centrifuge_48box_sequential_full_20261022_r351/REPORT.md) · [旧缓存清理证据](runtime_evidence/build_cache_cleanup_20261022_r1/REPORT.md)。

- 已实现不可变规划模型复用、每次独立场景、两次装盒静止期提前规划；动作由连续执行器接续，无AI逐动作等待。22个真实快照新旧结果相同，规划组件94.192→44.441秒（减少52.82%），**不等于全流程提速**。
- r348取回规划5°姿态门拒绝；r349先出现严格关节源跳步；r350已双装篮/稳转30.1375sim/左返桌，右预压因精确1ms浮点边界拒绝；r351修正边界但首次开盖撤臂0.1548rad超原0.15rad，尚未抓盒。四次都是FAIL，不能晋升或把失败前缀当三分钟成功。
- 新DART响应短路仅小世界数值对照及约1.05–1.07倍组件收益，r349起未加载，未获得整机资格。r350失败区间RTF约0.4264；r351无有效world stats，不编造实时率。
- r351最终2268测试两轮、93专项、46包/122装盒/6CTest通过；Nav111/2旧V12来源缺失保持。原物理/抓持/密FCL及30秒稳转要求不改；原visual-only碰撞缺口不外推为绝对无碰撞。
- 四次原片、原始失败日志和九组前后保护保留；r318/r327/r331/r333/r337/r342/r345/r347、孵育箱、920箱子版、模型及十备份未改。最新完整成功仍r347：**240.219sim / 584.977490836现实秒**；现实三分钟及稳定1倍仍未达。
- 用户新授权后仅删1546个不被安装/正式证据引用的旧CMake中间文件，分配约337.21MiB；源、共享库、日志、视频和备份不删。后验约1.80GB，低于2096208487字节完整实跑预算，不再启动新运行或擅自扩展清理。
- 新查r351已正常停持/SIGINT收尾、退休无owned残余；下次仍重查live。接续先补磁盘余量，在新身份同步按钮命令/控制器reference/关节反馈，解决撤臂跟踪，再验证右取回整数纳秒修正；不盲目放宽门或复用sealed身份。

**本节及AGENTS最末小节覆盖后文历史“当前/最新”；详细失败原因、视频与哈希见本轮汇总。**

## 保留成功基线：r347缩短空手间隔，完整通过

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

## 0a. 历史r341–r342：转子瞬态通过，完整流程成功

[r342报告](runtime_evidence/centrifuge_48box_sequential_full_20261021_r342/REPORT.md) · [完整原速录像](runtime_evidence/centrifuge_48box_sequential_full_20261021_r342/run/full_cycle.mp4) · [本轮复核](runtime_evidence/centrifuge_48box_sequential_review_20261021_r10/REPORT.md) · [主线程性能证据](runtime_evidence/centrifuge_48box_sequential_review_20261021_r10/PERFORMANCE.md)。

- r341沿实际篮轴试抬/退出，转子峰0.049174055rpm通过原0.1门，但右返桌2.238481mm超2mm，完整FAIL；[r341报告](runtime_evidence/centrifuge_48box_sequential_full_20261021_r341/REPORT.md)和录像保留，642.868556054wall只是失败前缀。
- r342继承轴向取出，新增最多两次水平≤1mm桌面正支撑校正，不改原锚点/最终2mm、2°门；实际左右0.122178/1.215702mm已满足，**校正运动未触发**，不称该分支已实际验证修好r341。
- 分开抓双装篮→关盖→稳转30.1865sim→停稳开盖→双盒返桌完整PASS；**234.032sim / 649.716164822现实秒**。较r337慢49.274秒，未达现实三分钟、当前分开抓仿真三分钟或稳定1倍。
- 148发布、4锚点4释放、最多1holder、28物理门、21路径24538密FCL通过；转子峰0.072811329rpm，原0.1门不改。抓持峰2.300208mm/4.087195°符合现5mm/10°，不替代旧严格证书或补足visual-only腕部碰撞缺口。
- 主线程59609个CPU样本，经精确符号范围核验：Physics::Update累计89.4211%、BoxedLcp求解58.2395%；包含关系不相加、不当墙钟分摊。pprof的World::bake/exists近邻误标签已排除。下一步新身份测受约束组规模/成本，不直接删抓持接触。
- 2164候选测试、46/122回归/6CTest通过，Nav111/2原V12缺源不变。世界RTF0.354713，定位最大健康源龄0.4005sim，原1秒门保持。635项源identity SHA `b3ea5587e70407787a8e5ec9b8074f746d115dfd636b821e77d61532e97901be`。
- 原速视频680.583333秒/8167帧完整解码，终态俯视双盒返桌；正常SIGINT退休、owned残余为空，下次仍重新查live。九组前后保护是最终封存门，见review_r10/final_review.json。r337及r318/r327/r331/r333、孵育箱/920版/三模型/十备份不覆盖，无清理/备份/真机。

r342已完成九组before/after保护，966项核心SUM：`9702e1f49ee2b2a81d88821bd08a829a133fc97f28ad21b08d60dcd287f07ead`；[最终封存复核](runtime_evidence/centrifuge_48box_sequential_review_20261021_r10/final_review.json)。r341失败941项核心封存不改。

## 0a. 历史r338–r340：当时尚未新增完整成功

[三次实验汇总](runtime_evidence/centrifuge_48box_sequential_review_20261021_r8/REPORT.md) · [r340报告](runtime_evidence/centrifuge_48box_sequential_full_20261021_r340/REPORT.md) · [r340原速失败过程录像](runtime_evidence/centrifuge_48box_sequential_full_20261021_r340/run/full_cycle.mp4)。

- r338精确位姿/篮角点快路径组件离线约1.916/2.276倍，35919个实际位姿矩阵与r337逐字节一致；右返桌前角偏10.024783°超过当前10°门，588.337528310wall失败。
- r339仅减缓右粗下降0.4→0.2rad/s、ramp0.3→0.6sim；左试抬转子瞬态0.112222255rpm超0.1，411.774332893wall失败，未触发右下降修正。
- r340另将15mm取回试抬0.1→0.05rad/s；已双装篮、关盖、稳转30.1855sim、停转开盖，左试抬仍出现0.377062651rpm，407.193697229wall失败。实际0.05已发布，单纯降速未解决问题。
- r340日志制动输出-2N·m已到原±2N·m夹限，已核对保护源码/插件；下一步先分离试抬横向校正、篮壁接触力矩与制动响应，不能直接改门凑PASS。
- 世界采样RTF约0.373/0.408/0.409，主线程约0.93核；这是串行负载线索，非接触求解的唯一因果证明。三个失败前缀均不能与r337完整600.442秒计算提速，稳定1倍/现实三分钟仍未达。
- r340最终2135测试、46/122回归、6CTest通过；Nav111/2旧V12缺源保留。三次九组前后保护PASS，正常SIGINT退休、无owned残余；候选未提升为正式版本，旧成功/模型/孵育箱/备份均未改。
- r340源identity617项SHA：`a5abe583efb2fb4557e6dab32d8d25010cc7a4bfaafbe8543ef02654e89a045c`；
  901核心SUM：`b8440c5ade95acdfb8d68d83456b147d94c36221e8b57b41c751e38398c8bd86`。
  原速流434.916667秒/5219帧完整解码，仅失败过程；末帧左手仍在篮中持盒，右盒在篮。

## 0b. r337：保留且较快的完整成功基线

[核心报告](runtime_evidence/centrifuge_48box_sequential_full_20261021_r337/REPORT.md) · [独立复核](runtime_evidence/centrifuge_48box_sequential_review_20261021_r5/REPORT.md) · [完整原速录像](runtime_evidence/centrifuge_48box_sequential_full_20261021_r337/run/full_cycle.mp4)。

- 定位输出上限2→5Hz，原算法/720点/10Hz雷达/源时间戳/质量及1秒源龄门保持；只在新身份记录点云、TF及发布时序。
- 923条消费位姿、920次健康检查全有效，最大源龄0.3755sim；无TF错误，完整通过开盖和双盒取回。不能以一次成功证明r336唯一根因或重复可靠性。
- 分开抓双装篮→关盖→稳转30.233sim→停稳开盖→双盒返桌完整PASS；232.1135sim / **600.442257241现实秒**。比r333少44.310秒，比r327少15.333秒；仍未达现实/仿真三分钟或稳定1倍。
- 145次运动发布、4真实锚点/4正常释放、最多一手持物，28物理门、20路径24469密FCL通过。抓持峰2.390323mm/4.919090°符合既有5mm/10°，不符合旧2mm/2°；原visual-only腕部碰撞缺口保持。
- 一个planner PID、20请求、4对连续段全部完成；规划等待92.687秒，比r333少22.150秒，非受控同快照基准。世界样本区间RTF约0.384126，后续优先隔离测接触物理与重复几何计算。
- 2112候选测试、46/122回归、6CTest通过；导航111/2旧V12缺源保留。九组前后保护PASS，r318/r327/r331/r333、失败r334–r336、孵育箱和十备份未改。
- 原片738.583333秒，含首帧审核和前后画面，完整解码；末尾俯视真实看见双盒在桌、双篮空。正常SIGINT退休、零owned残余；每次仍重新查live。
- 606项源identity SHA：`2ec0b6b55d93f5765f3fb4983c845592772e0fac5797d867e9943717872a114a`。
  917核心成员SUM：`81ccc8cbeb799e110797d930f576b70d8425ca93a93c04e5088b98518410fb9f`。
  本次原文/AGENTS前缀记录在[review_r5](runtime_evidence/centrifuge_48box_sequential_review_20261021_r5/documentation_before.json)。

## 0c. 历史r334–r336失败链

- [r334](runtime_evidence/centrifuge_48box_sequential_full_20261021_r334/REPORT.md)：复用一个规划进程、连接原始密集自由搬运段；右盒取回IK失败，540.360407497现实秒仅失败前缀。
- [r335](runtime_evidence/centrifuge_48box_sequential_full_20261021_r335/REPORT.md)：右取回中间点降10mm；右盒装篮前校正1.232812720°超局部1°门，197.817187497秒失败。
- [r336](runtime_evidence/centrifuge_48box_sequential_full_20261021_r336/REPORT.md)：增加最多两次≤1°/1mm的真实快照校正，原holder及最终释放检查不变；2102项局部测试通过。双盒装篮、关盖、稳转30.1425sim及停转已执行，开盖时定位源龄1.0025sim超过保留1秒门，未取回。350.351281705现实秒仍是失败前缀。
- [本轮汇总](runtime_evidence/centrifuge_48box_sequential_review_20261021_r4/REPORT.md)、[r336原速失败过程录像](runtime_evidence/centrifuge_48box_sequential_full_20261021_r336/run/full_cycle.mp4)；不可当成功全流程或现实三分钟演示。
- 首次收尾发生父进程退出竞态，原FAIL保留；复查后只重试原retire脚本，最终SIGINT退休、无owned残余。未来仍重新检查live。
- 当时待办（现r337已作新实验）：定位原始点云/定位发布/TF的源端空窗，不伪造时间戳、不直接放宽定位门。r318/r327/r331/r333、孵育箱及所有备份保护不变。

## 1. 当前结论

**最新完整通过r345，586.260现实秒/242.2705sim；r342、r337、r318/r327/r331/r333均保留。现实三分钟、当前分开抓仿真三分钟、稳定1倍仍未达。以下为旧版本历史表，最新覆盖见第0节。**

| 身份 | 实际范围 | 仿真耗时 | 现实耗时 | 结论 |
|---|---|---:|---:|---|
| [r342最新完整成功](runtime_evidence/centrifuge_48box_sequential_full_20261021_r342/REPORT.md) | 分开抓双装篮、关盖、稳转30.1865秒、停稳开盖、双盒返桌 | 234.032秒 | 649.716164822秒 | 28物理门PASS；转子峰0.072811329rpm；主线程物理热点已测量 |
| [r337最新完整成功](runtime_evidence/centrifuge_48box_sequential_full_20261021_r337/REPORT.md) | 分开抓双装篮、关盖、稳转30.233秒、停稳开盖、双盒返桌 | 232.1135秒 | 600.442257241秒 | 28/28当前物理门PASS；定位最大源龄0.3755秒；4对连续段完成 |
| [r333保留完整成功](runtime_evidence/centrifuge_48box_sequential_full_20261021_r333/REPORT.md) | 分开抓双装篮、实体关盖/启动、稳转30.094秒、停稳开盖、双盒返桌 | 239.8095秒 | 644.752285933秒 | 28/28当前物理门PASS；最大定位源龄0.584秒，原1秒门未放宽 |
| [r331新完整成功](runtime_evidence/centrifuge_48box_sequential_full_20261020_r331/REPORT.md) | 分开抓双装篮、实体关盖/启动、稳转30.1865秒、停稳开盖、双盒返桌 | 239.198秒 | 621.132718628秒 | 28/28当前合同PASS；右预压等待分支未在实跑触发 |
| [r327快版](runtime_evidence/centrifuge_48box_sequential_full_20261020_r327/REPORT.md) | 单手依次双盒装篮、关盖、稳转30.168秒、停稳开盖、双盒返不同桌槽、空手终态1秒 | 234.8065秒 | 615.775707219秒 | 本次放宽仿真合同PASS |
| [r318保留版](runtime_evidence/centrifuge_48box_sequential_full_20261020_r318/REPORT.md) | 同一时刻一手持物的完整装取，稳转30.128秒，原终态10秒 | 428.45秒 | 3894.946321249秒 | 先前合同PASS，原件不改 |
| [r283历史速度版](runtime_evidence/centrifuge_48box_optional_micro_full_20261015_r283/REPORT.md) | 旧双臂同时装取的完整流程 | 179.1290秒 | 2324.616083秒 | 旧仿真三分钟成功，不是现分开抓/现实三分钟 |

r327比r318的任务墙钟约快6.3253倍、减少84.19%，但合同不同，不能当同严格门基准或重复性证明。其单调墙钟包含执行器启动/导入/预检/在线规划/退出，不含场景启动、人工首帧审核及录像终结。234.8065/615.7757约0.381是任务时间比例，不是纯物理引擎实时率。

## 2. r327验收、录像与局限

- 154次运动发布；4次真实抓持锚点/4次正常释放；任一时刻最多一只手持物；28/28当前物理门通过。
- 19条实际路径、24,439次原2mrad密FCL通过；1938本地测试、原46/122与6CTest通过。原模型visual-only腕部等碰撞覆盖缺口仍在，不称绝对无碰撞。
- 独立滑移峰FK/native为2.158579/2.158360mm，角偏4.218821°；符合本次5mm/10°，**未通过旧2mm/2°门**。取盒转子瞬态0.053409829rpm，8样本超过旧0.02rpm，但在此前授权0.1rpm内。
- 最终左右盒位置误差0.325218/0.960172mm；两盒返不同桌槽、双爪空、盖开、转子停。手臂停于释放位置，不是中立收臂，也不是自由站立/导航后操作证书。
- [原速完整录像](runtime_evidence/centrifuge_48box_sequential_full_20261020_r327/run/full_cycle.mp4)：649.333333秒，1200×900、12fps、7792原始帧；完整解码通过，无加速、裁剪或重复帧。含执行前后，故长于任务计时。
- 录像远侧桌面盒受设备遮挡；双盒返回由物理证据证明，不冒称画面完整可见。[独立复核](runtime_evidence/centrifuge_48box_sequential_review_20261020_r15/REPORT.md)保留实际抽帧与CPU观察。

机器证据：[28项物理核验](runtime_evidence/centrifuge_48box_sequential_full_20261020_r327/speed_physical_audit.json)、[任务计时](runtime_evidence/centrifuge_48box_sequential_full_20261020_r327/speed_task_audit.json)、[独立抓持](runtime_evidence/centrifuge_48box_sequential_full_20261020_r327/independent_speed_grip.json)、[实际路径](runtime_evidence/centrifuge_48box_sequential_full_20261020_r327/speed_actual_paths_review.json)、[最终独立汇总](runtime_evidence/centrifuge_48box_sequential_review_20261020_r15/final_review.json)。

关键SHA-256：

| 对象 | SHA-256 |
|---|---|
| r327 identity | 0694b4ea0cdd1c8448387453d7ded0ba0be4d7537fcb06b9c81f703df571cb78 |
| r327核心SUM | 8039813de5340b87c6adb2638222355fe3c2131bf6a9ba99274e9635ab6b652a |
| review_r15 SUM | 35ab3bf41c30c01e4b405ce689971097af474f188470ffe64923a9b36f3e61ad |
| r327原片 | 5045bf54504b0bfaf2b3b965c7872707a53935a5d55350a26f1ad6700e866368 |
| r318核心SUM | 4bdfd4c608f56f0ed0c7b1c4396e727f0986d6b8df0bb7d10c1d3a3c8116823f |
| review_r6 SUM | a7ebcc60dfca91c9365b6f26caaeb2267b89b3e0bdfdcc9e129f4bc059570ccf |

r327为796个核心成员、review_r15为14成员；r318为753核心、review_r6为56成员，均只读封存。r318原post首版浮点连接核验失败及日志未改，完整独立结论见[revision2](runtime_evidence/centrifuge_48box_sequential_review_20261020_r6/final_review_revision2.json)，不宣称原post全部成功。

## 3. 历史性能实验（截至r333，当前覆盖见第0节）

**此前10月21日r333覆盖：** [r333正式报告](runtime_evidence/centrifuge_48box_sequential_full_20261021_r333/REPORT.md)与[独立复核](runtime_evidence/centrifuge_48box_sequential_review_20261021_r1/final_review.json)确认完整成功。定位两条订阅改为BEST_EFFORT/KEEP_LAST深度1，原发布算法/速率/质量与1秒源龄门不改；476条pose、951次健康检查无无效项，最大源龄0.584sim秒。独立同源可靠/最新样本传输峰约41.5/0.465ms，但没有重现r332的数秒迟到，不能宣称历史唯一根因已证实或重复可靠性。

r333有2067本地测试、原46/122/6CTest通过；Nav111/2旧缺源保留。157发布、4锚点4释放、20路径24634密FCL、28物理门及独立抓持通过；独立滑移2.261860mm/4.902739度符合5mm/10度但不符合旧严格门。最终左右桌槽误差0.285251/1.122609mm，双爪空/盖开/停转。预压窄等待仍未实跑触发。

[完整原速录像](runtime_evidence/centrifuge_48box_sequential_full_20261021_r333/run/full_cycle.mp4)713.75秒/8565帧全解码，含启动审看等待；已看7个关键帧，末尾712秒俯视确见双盒返桌、双篮空、双爪空。正面远侧遮挡仍保留。源identity `4a3c61ee93c592576620e72a6410d79f79e9b4a7bd70d079cff250a5c90ed50a`；854核心SUM `a1caf853a13ef2eb852764741f8fd6b01ab9403794fc07a96f741c0dee0bd53f`。

缓存实际1733167命中、21716角点逐字节重算通过，但全程644.752秒慢于r331/r327，不能宣称缓存整体提速。44个世界样本区间RTF约0.378171；下一步隔离测物理接触求解与重复几何检查成本，不放宽定位门或取消真实抓持/FCL凑180秒。九类前后保护通过，正常SIGINT退休。以下r331/r332段落为此前实验背景。

此前[r331](runtime_evidence/centrifuge_48box_sequential_full_20261020_r331/REPORT.md)加入窄窗口预压观测等待，保留原支撑/强双指捕获，完成了双盒全部取回。2023本地测试、157发布、4锚点4释放、20实际路径/24521密FCL和28物理门通过；独立滑移FK/native为2.479099/2.477191mm，角偏5.400582°，符合5mm/10°但不符合旧严格门。**实跑未触发新增等待，不能宣称间歇缺帧已充分复现解决。** [原速完整录像](runtime_evidence/centrifuge_48box_sequential_full_20261020_r331/run/full_cycle.mp4)657.333333秒，全解码通过；[独立复核](runtime_evidence/centrifuge_48box_sequential_review_20261020_r19/final_review.json)。835核心SUM `5dedec3f942ce59e9a51bb4b2cbd15b3c1aa09eead8c0a77613880d9b9a66d6c`；review_r19的38项SUM `74439505241590e5797ae68e25a44eb4e53d518e546ede3912ad1d2754610a38`。

[r332](runtime_evidence/centrifuge_48box_sequential_full_20261020_r332/REPORT.md)只新增精确篮角点缓存，2052最终测试通过；实际首次开盖撤臂时39.901sim，定位源38.9的龄1.001秒超过原1.0门，4发布/0锚点/14.031937063wall秒FAIL。缓存尚未被调用，不能评估整段提速；停持后迟到定位成批到达，具体发布/DDS/消费队列根因未定。保留1秒门，没有放宽凑PASS。[本轮复核](runtime_evidence/centrifuge_48box_sequential_review_20261020_r20/REPORT.md)保留失败录像/日志、定位时戳和下一假设。两次原46/122/6CTest通过，Nav111/2旧缺源仍保留。

后续先对定位发布/接收/消费时间做独立观测，排除排队造成的旧样本，再验证缓存整体收益。只读部分世界观测显示r331区间RTF约0.374，非稳定1倍。maxContacts=4仅离线SDF转换探针，没有安装或实跑；不得将其视为已通过物理优化。

此前r328–r330也按新身份修改、隔离构建、测试并连续派发完整任务，均录制原速视频。以下是历史失败链，没有逐动作等待AI，没有修改r318/r327或正式源码/模型。

| 身份 | 改动 | 实际结果与墙钟 |
|---|---|---|
| [r328](runtime_evidence/centrifuge_48box_sequential_full_20261020_r328/REPORT.md) | 有界索引关节时间线、线程栈采样 | 双装篮/机器循环后，左取回试抬转子瞬态0.125302rpm超0.1；FAIL，413.145525秒 |
| [r329](runtime_evidence/centrifuge_48box_sequential_full_20261020_r329/REPORT.md) | 64KiB无损日志批量写入；取回试抬0.15→0.10rad/s | 双装篮后空手按键退臂误差0.154rad超0.15；FAIL，247.891034秒 |
| [r330](runtime_evidence/centrifuge_48box_sequential_full_20261020_r330/REPORT.md) | 512项精确位姿缓存；空手按钮非接触段上限0.8rad/s | 双装篮/30秒稳转/停稳开盖/左盒返桌；右预压支撑证据缺失，FAIL，521.840822秒 |

三个时间均为**未完成任务的失败片段**，不能与r327完整615.776秒计算提速。r330有1986本地测试及原46/122/6CTest通过，Nav111通过/2旧缺源失败保留。已观察抓持独立滑移FK/native峰0.852226/0.848499mm、角峰2.650490°，不是完整四锚点证书。

历史右预压故障时距上一正篮底力0.5ms，当前单指力为零；几何仍在当前门内。既不能断言真实掉落，也不能把零力当抓牢。详见[接触复核](runtime_evidence/centrifuge_48box_sequential_review_20261020_r18/preload_failure_context.json)。停持后下一条正力记录已晚9.5ms，不能当连续下一帧恢复证据；r331窄等待测试明确拒绝此大间隔。

本轮各身份九类before/after保护均通过，r318/r327、785正式源、三模型与十备份不改。新814成员r330核心SUM为`f8e8d80f9f48f2ac231db4683c9af7070f00d5e7ecec92b2136e4ecfba0b0e13`；review_r18的16成员SUM为`f0cbc7ecbab33651eba1e04141376c84f657a7c0a098d0a5a28804bc09cbb9b4`。[独立复核](runtime_evidence/centrifuge_48box_sequential_review_20261020_r18/final_review.json)、[最新原速失败录像](runtime_evidence/centrifuge_48box_sequential_full_20261020_r330/run/full_cycle.mp4)（565.75秒）和[本轮汇总](runtime_evidence/centrifuge_48box_sequential_runtime_perf_20261020_r1/REPORT.md)均保留。

## 4. 孵育箱与导航：独立保留

- 孵育箱动作语义权威仍是`runtime_assets/candidate3_roundtrip_831_authority_v9/`与`runtime_assets/candidate3_roundtrip_831_r17/`，双循环164.523835390秒；完整雷达地图固定重放为v24/r103，169.329060321秒。见[场景版本](SCENE_VERSIONS.md)及[r103证据目录](runtime_assets/candidate3_roundtrip_831_r103/)。
- 双循环为右手两次独立开关锁门，左手放入并取回出生原位；左手把手命令/接触0，躯干峰<45°，60控制消息=16/16/16/4/8，最终门locked、方块原位、两个support detached。固定重放不是导航后动态操作证书。
- [826 AprilTag A–L](runtime_evidence/826_apriltag_execution_20260827/REPORT.md)独立有效；箱体Tag10/12固定外参求把手、Tag11交叉检查的D1b链仍为离线plan-only，不能当完整导航接近→视觉补偿→开箱联动。
- Mid360/Nav2/官方Unitree步态有定位和短距action成果，严格30秒/8cm驻足仍未完成。当前导航测试111通过/2项旧V12来源缺失失败；不以固定骨盆离心机成功替代步态资格。详见[导航计划](H1_2_MID360_NAVIGATION_PLAN.md)。
- [官方RL重下载](runtime_evidence/rl_official_redownload_20261012_r1/REPORT.md)已恢复固定提交的277文件及7项官方身份；用户V12原`model_17250.pt`及`params/env.yaml`、`params/agent.yaml`未恢复，导出策略不等于原训练状态。用户已停止原件恢复，不主动重新做磁盘恢复/Windows操作。

## 5. 备份区别

备份均在`/home/liu/h1_moveit/backups/`，本轮没有新建或覆盖。

| 恢复点 | 保留范围 | SHA-256 |
|---|---|---|
| [920箱子版本](BACKUP_920_BOX_VERSION.md) | 永久保留孵育箱，含v24/r103，早于离心机导入 | `c4e72068e831ab0d14712261446709704012797ab81ae8948800f7a668db3ecf` |
| [1008](BACKUP_1008_RELEASE.md) | 双盒装篮阶段快照 | `92ae87377fe9a92922130df5ca30966f70551952a7243b87138976a78c0a2cf9` |
| [1009](BACKUP_1009_RELEASE.md) | 最新完整工作区快照，含r159；不含r160以后、r318/r327或本轮整理 | `9a746ba6b0f4a7fd871db63ef962cad0d58491207f0e102a85ce7f322c87ea26` |

827/828/831/901/909/914/915/920/1008/1009保护链不动。新备份必须用户另给未使用编号；恢复只能到指定的新空目录，不覆盖当前工作区。

## 6. Live、权限与接手

最新r337已正常退休、零owned残余；[本轮重新只读live快照](runtime_evidence/centrifuge_48box_sequential_review_20261021_r5/live_after.json)是其记录时刻的结果，后续必须重新检查。下段r331/r332退休与旧live链接仅为历史。当前授权为D分支定位故障修正及完整录像，未启动腿部导航、孵育箱动作、清理或备份。

r331成功后、r332失败停持后均已正常SIGINT退休，各`run/retirement.json`无剩余owned进程。最新只读可见进程快照见[本轮live核验](runtime_evidence/centrifuge_48box_sequential_review_20261020_r20/live_after.json)；不是永久live结论，也不是控制器/夹持健康证书。r327与r328–r330退休均为更早历史，下一次仍重新查现场。

当前用户要求为继续性能开发、保持分开抓/抓稳并完整运行录像，属于D及[HANDOFF_PLAN第33节](HANDOFF_PLAN.md#33-2026-10-20r327分开抓快版完成后的接续合同)，不是此前仅整理MD。本轮未清理、恢复或创建备份。新对话留空目标仍默认只读R；孵育箱重放才走G。

任何新动作必须新builder/run/domain/partition/overlay/录像身份，普通`mkdir`独占，隔离构建、回归、一次seal、fresh与真实录像门齐全；禁止复用历史claim、改写成功/失败证据、reset/detach/异常松夹或连接真机。查看入口：[QUICKSTART.md](QUICKSTART.md)；新对话复制：[HANDOFF_PROMPT.md](HANDOFF_PROMPT.md)。

此前七份根文档原文保存在[文档整理原文](runtime_evidence/documentation_roundup_20261020_r1/documentation_before.json)；本轮更新前六份入口原文及AGENTS867182字节前缀哈希见[本轮原文记录](runtime_evidence/centrifuge_48box_sequential_review_20261020_r19/documentation_before.json)。AGENTS历史仅追加，正式报告、模型、源码、视频、rosbag、builder和备份均未合并删除。
