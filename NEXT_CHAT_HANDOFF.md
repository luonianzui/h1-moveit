# H1-2 新对话兼容入口

更新：2026-10-22 CST。本文件保留兼容入口与阅读顺序，当前统一见[HANDOFF_CURRENT.md](HANDOFF_CURRENT.md)。

## 0. 2026-10-22最新覆盖：r353完整分开抓PASS，现实494.301秒

[r353报告](runtime_evidence/centrifuge_48box_sequential_full_20261022_r353/REPORT.md) · [原速完整录像](runtime_evidence/centrifuge_48box_sequential_full_20261022_r353/run/full_cycle.mp4) · [轮次汇总](runtime_evidence/centrifuge_48box_sequential_review_20261022_r11/ROUND_SUMMARY.md) · [性能分析](runtime_evidence/centrifuge_48box_sequential_review_20261022_r11/PERFORMANCE.md)。

- 完整分开抓双装篮→实体按键关盖/启动→稳转30.1345sim→停稳开盖→双盒返桌：**221.4645sim / 494.301397270现实秒**。较r347少90.676094秒（15.50%），非单因素受控A/B；**现实三分钟、当前分开抓仿真三分钟、稳定1倍仍未达**。
- 抬肘/单holder保留；28门、4捕获4正常释放、21路径24555密FCL通过。独立抓持峰0.941606mm/1.752226°，返桌误差左0.176898mm/右0.173000mm。现速度合同不替代旧严格证书，原指速0.2门2样本越界和visual-only碰撞缺口保留。
- 初始撤臂有界加减速，原0.15rad门不改，1626参考样本最大跟踪0.003405619rad；规划模型1构建/34复用/35新场景、21请求52.927wall，两次装盒观察期提前规划。全程连续脚本，无AI逐动作等待。
- r352动作前源丢帧失败、无任务录像，隔离封存；r353仅新进程树UDPv4，全程通过原0.5ms源门，但微测DEFAULT也通过，不断言唯一根因。右取回整数ns的原1ms边界本次通过。
- 隔离构建、2293测试两轮/25专项/46配置/122装盒/6CTest通过；Nav111/2旧V12缺失不变。原新增离线审计raw读取失败保留，独立gzip修正及8测试通过，执行源和物理门未改。
- RTF区间0.454443，主线程0.939核；精确符号Physics累计89.405%、BoxedLcp55.480%，重叠比例不当墙钟。下轮新身份优先DART单位脉冲/约束装配重复访问优化，不删接触/改时钟凑三分钟。
- 原速532.666667秒录像完整解码，抽查首帧/四运动帧/两终帧，俯视双盒返桌。正常SIGINT录像刷盘及场景退休，owned为空；下次重查live/空间，不重跑sealed身份。
- 九组前后保护相同PASS，r347及r318/r327/r331/r333等旧成功、孵育箱/920版/正式模型/旧视频/十备份保持。本轮未清理或备份。收尾执行既定GitHub脚本，提交号以成功回执为准；视频/原始流/备份仅本地。
- 源679 SHA `b2d9b669704f61323a3cde92c3b7615b709d4c54df7490f474fa99bb62721321`；1028核心SUM `541d8a5a1da0c245ca30ed2f84f065c1120b10e0ecf663168525d33a836d3ed2`。

**本节与AGENTS最末覆盖下列历史“当前/最新”；旧报告和成功保持。**

## 历史：r348–r351优化未完整通过（非当前）

[本轮汇总与接续](runtime_evidence/centrifuge_48box_sequential_review_20261022_r9/ROUND_SUMMARY.md) · [r351失败报告](runtime_evidence/centrifuge_48box_sequential_full_20261022_r351/REPORT.md) · [旧缓存清理证据](runtime_evidence/build_cache_cleanup_20261022_r1/REPORT.md)。

- 已实现不可变规划模型复用、每次独立场景、两次装盒静止期提前规划；动作由连续执行器接续，无AI逐动作等待。22个真实快照新旧结果相同，规划组件94.192→44.441秒（减少52.82%），**不等于全流程提速**。
- r348取回规划5°姿态门拒绝；r349先出现严格关节源跳步；r350已双装篮/稳转30.1375sim/左返桌，右预压因精确1ms浮点边界拒绝；r351修正边界但首次开盖撤臂0.1548rad超原0.15rad，尚未抓盒。四次都是FAIL，不能晋升或把失败前缀当三分钟成功。
- 新DART响应短路仅小世界数值对照及约1.05–1.07倍组件收益，r349起未加载，未获得整机资格。r350失败区间RTF约0.4264；r351无有效world stats，不编造实时率。
- r351最终2268测试两轮、93专项、46包/122装盒/6CTest通过；Nav111/2旧V12来源缺失保持。原物理/抓持/密FCL及30秒稳转要求不改；原visual-only碰撞缺口不外推为绝对无碰撞。
- 四次原片、原始失败日志和九组前后保护保留；r318/r327/r331/r333/r337/r342/r345/r347、孵育箱、920箱子版、模型及十备份未改。最新完整成功仍r347：**240.219sim / 584.977490836现实秒**；现实三分钟及稳定1倍仍未达。
- 磁盘不足已通过[五份冷日志无损归档](runtime_evidence/cold_trace_archive_20261022_r1/REPORT.md)缓解：4.45GB→0.89GB，释放3.56GB，后验约5.15GB。原31项归档/开发入口不改，九组前后保护和4732项离线测试通过；原模型、源码、共享库、视频和备份保持。五份历史raw的读取/导出见[说明](runtime_evidence/cold_trace_archive_20261022_r1/README.md)，不可删除新gzip。上轮337.21MiB编译缓存清理记录保留。
- r351已正常停持/SIGINT收尾、退休无owned残余；本轮仅离线归档未启动Gazebo，下次仍重查live和磁盘。接续在新身份同步按钮命令/控制器reference/关节反馈，解决撤臂跟踪，再验证右取回整数纳秒修正；不盲目放宽门或复用sealed身份。

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

## 历史覆盖：r342完整成功，r337当时较快

[r342报告](runtime_evidence/centrifuge_48box_sequential_full_20261021_r342/REPORT.md) · [完整原速录像](runtime_evidence/centrifuge_48box_sequential_full_20261021_r342/run/full_cycle.mp4) · [本轮复核](runtime_evidence/centrifuge_48box_sequential_review_20261021_r10/REPORT.md) · [性能证据](runtime_evidence/centrifuge_48box_sequential_review_20261021_r10/PERFORMANCE.md)。

r341轴向取出转子峰0.049174055rpm通过原0.1门，但返桌2.238481mm失败；r342继承轴向几何，完整分开抓双装篮/关盖/稳转30.1865sim/停稳开盖/双盒返桌PASS，234.032sim/649.716164822现实秒，未达三分钟或稳定1倍，r337的600.442257241秒仍较快。
r342转子峰0.072811329rpm，4锚点4释放/单holder/28门/21路径24538密FCL通过。新增≤1mm桌面正支撑反馈校正未触发，不能声称该运动分支已实际验证。2164测试/46/122回归/6CTest通过，Nav原V12缺源2失败保留。
59609主线程CPU样本经精确符号范围确认物理更新累计89.4211%、BoxedLcp求解58.2395%；近邻函数误标签已排除，不相加、不当墙钟分摊。下一步新身份测约束组成本，不直接删抓持接触。
录像680.583333秒全解码，末尾俯视两盒返桌；正常SIGINT退休、owned为空，下次仍重查live。九组前后保护是封存门，r337/r318/r327/r331/r333、全部旧成果/孵育箱/920版/三模型/十备份不覆盖。

## 历史记录（以下“最新/当前”仅指当时）

最新性能候选r338/r339/r340均失败；[本轮汇总](runtime_evidence/centrifuge_48box_sequential_review_20261021_r8/REPORT.md)、[r340失败录像](runtime_evidence/centrifuge_48box_sequential_full_20261021_r340/run/full_cycle.mp4)。r340实际0.05rad/s试抬仍触发0.377062651rpm转子门，制动达到原-2N·m夹限；407.193697229wall仅失败前缀。2135测试通过不等于物理完整成功。三次九组保护PASS、正常SIGINT退休，旧成功及全部备份不改；未达1倍实时或现实三分钟。后续先查试抬校正/接触力矩/制动响应，不复用旧身份。当前完整成功仍为下面r337。

最新[r337完整成功](runtime_evidence/centrifuge_48box_sequential_full_20261021_r337/REPORT.md)：分开抓双装篮、关盖、稳转30.233sim、停稳开盖、双盒返桌，232.1135sim/600.442257241现实秒。定位输出上限5Hz、原1秒门不改，最大源龄0.3755sim。28物理门、4锚点4释放、20路径24469密FCL通过，原速录像完整解码，仍未现实三分钟或稳定1倍。r318/r327/r331/r333、失败r334–r336及孵育箱全部保留。

[本轮复核](runtime_evidence/centrifuge_48box_sequential_review_20261021_r5/REPORT.md)记录2112测试、九组前后保护和正常退休；不是永久live结论。常驻planner/四处连续段已在r337完整执行，但不能宣称历史r336定位唯一根因或重复可靠性。新运行必须新身份。

## 阅读顺序

1. 完整阅读[AGENTS.md](AGENTS.md)，采用最靠后的日期小节及落盘证据。
2. 阅读[HANDOFF_CURRENT.md](HANDOFF_CURRENT.md)：当前摘要、证据、备份与live边界。
3. 阅读[826_plan.md](826_plan.md)，区分历史设计与末尾完成覆盖。
4. 确认本兼容入口。
5. 阅读[HANDOFF_PLAN.md](HANDOFF_PLAN.md)、[QUICKSTART.md](QUICKSTART.md)、[SCENE_VERSIONS.md](SCENE_VERSIONS.md)。
6. 离心机任务再读[CENTRIFUGE_FULL_CYCLE.md](CENTRIFUGE_FULL_CYCLE.md)及目标正式报告/manifest；新对话复制[HANDOFF_PROMPT.md](HANDOFF_PROMPT.md)。

默认只读；历史PID、已封存脚本和旧命令不构成运行许可。任何新运行、视角或参数实验均须新身份；不得破坏r318/r327/r331/r333/r337、孵育箱/920箱子版、模型、正式证据及备份。新备份须未用编号，清理须对应明确授权，实体机器人未授权。

本轮被整理的根文档完整原文及哈希在[文档整理审计](runtime_evidence/documentation_roundup_20261020_r1/REPORT.md)；此前[10月10日原文](runtime_evidence/documentation_process_cleanup_20261010_r1/documentation_before.json)也保留。AGENTS累积历史与正式报告不删。
