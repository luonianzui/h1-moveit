# H1-2 快速查看与只读核验

更新：2026-10-23 CST。默认只读，不因本文启动/停止Gazebo或重跑旧身份。

## 0. 2026-10-23最新覆盖：r354完整通过，现实484.470秒

[r354报告](runtime_evidence/centrifuge_48box_sequential_full_20261023_r354/REPORT.md) · [原速完整录像](runtime_evidence/centrifuge_48box_sequential_full_20261023_r354/run/full_cycle.mp4) · [轮次交接](runtime_evidence/centrifuge_48box_sequential_review_20261023_r1/ROUND_SUMMARY.md) · [性能分析](runtime_evidence/centrifuge_48box_sequential_review_20261023_r1/PERFORMANCE.md)。

- 分开抓双盒装篮→实体关盖/启动→稳转30.2595sim→停稳开盖→双盒返桌完整PASS：**214.241sim /484.470481546wall**。比r353少9.830916wall（1.99%），非同状态受控A/B。**现实/当前分开抓仿真三分钟、稳定1倍仍未达**；RTF0.434703低于r353的0.454443，不称实时率提升。
- 新私有DART组件只缓存一次求解内的所属骨架/可反应性查询，退出清空；数值响应/脉冲状态实时读取，算术/顺序/接触/0.5ms步长不改。5400小世界步、43200状态向量一致，组件约1.149倍不代替整机结论。
- 2302候选测试两轮、15专项、46配置/122装盒/6CTest通过；Nav111/2旧V12缺失不变。28门、4捕获4释放、单holder、21路径24459密FCL通过；独立抓持0.905259mm/2.337157°，返桌0.128403/0.436028mm。现5mm/10°不等于旧严格门，指速0.2界5样本越界及visual-only碰撞缺口保留。
- 单planner模型1构建34复用35新场景，21请求worker40.757846wall；两次装盒观察期预规划，连续执行无AI逐动作等待。主线程0.932516核，精确符号Physics累计88.2516%、BoxedLcp51.6780%、速度传播19.4210%，重叠比例不当墙钟。下一新组件测单位脉冲传播的重复惯量读取。
- 原速529.916667秒视频完整解码，实际首帧/四运动帧/两终帧抽查，俯视双返桌。正常SIGINT刷盘、退休owned空；live下次重查。未清理或备份，旧r353及此前成功、孵育箱/920版/源模型/十备份不改。
- 九组保护前后相等；r353核心1028成员全部符合旧清单。**旧review_r10/r11各自freeze_review.log存在seal先哈希空文件、后写自身回执的既有不一致**，本轮独立精确例外记录且前后不变，旧文件/清单不改，不称旧review无例外全通过。新sealer防自包含stdout的2测试通过。
- r354源682 SHA `771aa0233af8b4fe4b8b67e624880cbe1a748432e834cf9e443a363e03941b0f`；1035核心SUM `b26e2f0e94230ddd26465fee3e6ac9da68ba4d71145647c7b1830d3741cedf5d`。新版本封存/MD完成后按既定脚本同步GitHub，实际提交看同步回执；大视频、原始流、备份留本地。

**本节及AGENTS最末覆盖后续历史“当前/最新”；禁止复用r354/r353封存身份。**

## 保留成功：2026-10-22 r353完整分开抓PASS，现实494.301秒

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

## 上轮与历史结果

- **[r342最新完整原速录像](runtime_evidence/centrifuge_48box_sequential_full_20261021_r342/run/full_cycle.mp4)**：680.583333秒；任务234.032sim/649.716164822现实秒，分开抓、稳转30.1865sim、双盒返桌。**仍未三分钟，r337仍较快**。
- [r342报告](runtime_evidence/centrifuge_48box_sequential_full_20261021_r342/REPORT.md) · [本轮复核](runtime_evidence/centrifuge_48box_sequential_review_20261021_r10/REPORT.md)：28门/21路径24538密FCL通过，转子峰0.072811329rpm，原0.1门不改；桌面≤1mm校正未触发。2164测试/46/122/6CTest通过，Nav原2缺源不变；九组前后保护为封存条件，旧成功不改。
- [主线程证据](runtime_evidence/centrifuge_48box_sequential_review_20261021_r10/PERFORMANCE.md)：物理更新累计89.4%、约束求解58.2%，59609CPU样本及精确符号复核，约0.355RTF，未稳定1倍。[r341失败报告](runtime_evidence/centrifuge_48box_sequential_full_20261021_r341/REPORT.md)保留，642.869秒失败前缀不算提速。以下“最新”均为当时历史描述。

- **最新实验不是新成功**：[r338/r339/r340汇总](runtime_evidence/centrifuge_48box_sequential_review_20261021_r8/REPORT.md)。2135最终测试通过，但r340左试抬转子0.377062651rpm超0.1，制动达到原-2N·m限值；未双盒取回。
- [r340原速失败过程](runtime_evidence/centrifuge_48box_sequential_full_20261021_r340/run/full_cycle.mp4)：434.916667秒/5219帧完整解码。407.193697229wall是失败前缀，不能与完整任务比较提速。约0.409RTF，未达1倍或现实三分钟。
- 三次均停持后正常SIGINT退休，旧成功/模型/备份保护通过。下一步先分析接触力矩/试抬横向校正；当前可展示的完整成功仍是下面r337，不能直接重跑已封存身份。

- **[r337最新完整原速录像](runtime_evidence/centrifuge_48box_sequential_full_20261021_r337/run/full_cycle.mp4)**：738.583333秒，含任务前后；任务232.1135sim/600.442257241现实秒，尚未三分钟。
- [r337报告](runtime_evidence/centrifuge_48box_sequential_full_20261021_r337/REPORT.md)：最大定位源龄0.3755sim、原1秒门不改；28物理门、4锚点4释放、20路径24469密FCL通过；完整30秒旋转，末尾俯视双盒返桌。旧r318/r327/r331/r333不改。
- [本轮复核](runtime_evidence/centrifuge_48box_sequential_review_20261021_r5/REPORT.md)：2112测试和九组前后保护PASS，正常SIGINT退休。以下均是保留历史版本。

- 此前r334/r335/r336间隔优化均未完整成功；[汇总](runtime_evidence/centrifuge_48box_sequential_review_20261021_r4/REPORT.md)。r336已双装篮/关盖/旋转30秒/停转，开盖定位超时，未取回；2102项本地测试PASS不等同完整任务成功。
- [r336原速失败过程](runtime_evidence/centrifuge_48box_sequential_full_20261021_r336/run/full_cycle.mp4)410.916667秒；当时最新成功为下面r333，当前已为r337。不要重跑旧run或将失败前缀当现实三分钟结果。

- [r333保留完整原速录像](runtime_evidence/centrifuge_48box_sequential_full_20261021_r333/run/full_cycle.mp4)：713.75秒，任务239.8095sim/644.752285933现实秒；全程定位有效，**仍非三分钟**，r318/r327/r331保留。
- [r333报告](runtime_evidence/centrifuge_48box_sequential_full_20261021_r333/REPORT.md)：原1秒定位门不改、最大源龄0.584秒；28物理门/20路径24634密FCL通过，末尾俯视可见双盒返桌。以下为历史版本入口。

- [当前交接总览](HANDOFF_CURRENT.md)：唯一简明最新状态、保护与备份边界。
- [本轮离心机总结](CENTRIFUGE_FULL_CYCLE.md)：第13节为r337完整通过，第11节保留r333；r331成功与r332失败原件不改，三分钟未达。
- [r331保留完整原速录像](runtime_evidence/centrifuge_48box_sequential_full_20261020_r331/run/full_cycle.mp4)：657.333333秒；实际任务621.132718628墙秒/239.198仿真秒，双盒均已取回。见[正式报告](runtime_evidence/centrifuge_48box_sequential_full_20261020_r331/REPORT.md)。
- [r332失败报告](runtime_evidence/centrifuge_48box_sequential_full_20261020_r332/REPORT.md)：首次开盖后定位源过期，未抓盒，缓存未进入实跑。不得把14.032秒失败片段算三分钟成功。
- [r327完整原速录像](runtime_evidence/centrifuge_48box_sequential_full_20261020_r327/run/full_cycle.mp4)：649.333秒，未加速或裁剪；任务实际615.776现实秒，**不是三分钟**。
- [r327正式报告](runtime_evidence/centrifuge_48box_sequential_full_20261020_r327/REPORT.md)与[独立复核](runtime_evidence/centrifuge_48box_sequential_review_20261020_r15/REPORT.md)：234.8065sim、28/28当前合同，旧2mm/2°门不通过。
- [r318原成功](runtime_evidence/centrifuge_48box_sequential_full_20261020_r318/REPORT.md)保留，不覆盖；旧r283的179.129sim是双臂同时版，不是当前分开抓结果。
- [r330历史失败录像](runtime_evidence/centrifuge_48box_sequential_full_20261020_r330/run/full_cycle.mp4)：565.75秒，**未完成全流程**，左盒返桌后右预压支撑证据不足；见[失败报告](runtime_evidence/centrifuge_48box_sequential_full_20261020_r330/REPORT.md)。
- [性能实验汇总](runtime_evidence/centrifuge_48box_sequential_runtime_perf_20261020_r1/REPORT.md)：组件等价测试、三次实跑和独立保护；不以组件倍数代替整体实时率。

## 只读校验命令

在工作区执行；这些命令不启动仿真，不向封存目录写新报告：

```bash
cd /home/liu/h1_moveit/h1_2_ros2_ws_claude_2026-07-30/h1_2_ros2_ws
tools/trace_archive/run -- /usr/bin/python3 tools/trace_archive/archive_cli.py verify-manifest \
  runtime_evidence/centrifuge_48box_sequential_full_20261021_r337/SHA256SUMS
tools/trace_archive/run -- /usr/bin/python3 tools/trace_archive/archive_cli.py verify-manifest \
  runtime_evidence/centrifuge_48box_sequential_full_20261021_r333/SHA256SUMS
tools/trace_archive/run -- /usr/bin/python3 tools/trace_archive/archive_cli.py verify-manifest \
  runtime_evidence/centrifuge_48box_sequential_full_20261020_r331/SHA256SUMS
tools/trace_archive/run -- /usr/bin/python3 tools/trace_archive/archive_cli.py verify-manifest \
  runtime_evidence/centrifuge_48box_sequential_full_20261020_r327/SHA256SUMS
tools/trace_archive/run -- /usr/bin/python3 tools/trace_archive/archive_cli.py verify-manifest \
  runtime_evidence/centrifuge_48box_sequential_review_20261020_r15/SHA256SUMS
tools/trace_archive/run -- /usr/bin/python3 tools/trace_archive/archive_cli.py verify-manifest \
  runtime_evidence/centrifuge_48box_sequential_full_20261020_r318/SHA256SUMS
tools/trace_archive/run -- /usr/bin/python3 tools/trace_archive/archive_cli.py verify-manifest \
  runtime_evidence/centrifuge_48box_sequential_review_20261020_r6/SHA256SUMS
```

原SHA清单含部分迁移历史raw日志时，不用原生`sha256sum -c`判断缺失；[归档说明](tools/trace_archive/README.md)规定兼容读取。保留wrapper继承环境，不用Python `-I/-S`或清空`PYTHONPATH`。全清单核验会读取视频和日志，需时间；不要重跑会向旧目录`open('x')`的历史审计脚本。

## 如果要继续运行

先完整阅读AGENTS、当前总览、HANDOFF_PLAN及目标相关正式报告；检查实际进程、控制器、夹持/support，历史退休信息不能代替live。

- 离心机开发/提速：分支D及[HANDOFF_PLAN第33节](HANDOFF_PLAN.md)，必须新未用身份、隔离构建/回归、一次seal、fresh和实际首帧录像门；不直接执行r318/r327旧脚本。
- 孵育箱重放：分支G，按v24链派生新builder/authority/run，原样执行新密封commands；离心机合同不能混用。
- 场景区别/920箱子版本见[SCENE_VERSIONS.md](SCENE_VERSIONS.md)；1009是历史完整快照，不含r318/r327。
- 新对话复制[HANDOFF_PROMPT.md](HANDOFF_PROMPT.md)，目标留空仅只读。所有真机、清理和新备份都不在本入口授权内。

此前[文档整理审计](runtime_evidence/documentation_roundup_20261020_r1/REPORT.md)是只读核验；其后已授权r328–r332实跑。最新两次见[review_r20](runtime_evidence/centrifuge_48box_sequential_review_20261020_r20/REPORT.md)，均退休，不能复用。未删除证据或创建备份。
