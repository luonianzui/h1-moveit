# 离心机分开抓完整流程：本轮总结与接口

更新：2026-10-22 CST，r348–r351规划/性能候选尚未完整通过，r347成功保留，现实三分钟仍未达。当前入口为[HANDOFF_CURRENT.md](HANDOFF_CURRENT.md)，累积权威为[AGENTS.md](AGENTS.md)最新日期小节。第0节覆盖本轮，其余历史不替代原始报告。

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

## 0a. 历史r341–r342：轴向取盒、桌面反馈与主线程采样

[r342报告](runtime_evidence/centrifuge_48box_sequential_full_20261021_r342/REPORT.md)、[完整原速录像](runtime_evidence/centrifuge_48box_sequential_full_20261021_r342/run/full_cycle.mp4)、[本轮复核](runtime_evidence/centrifuge_48box_sequential_review_20261021_r10/REPORT.md)、[CPU热点](runtime_evidence/centrifuge_48box_sequential_review_20261021_r10/PERFORMANCE.md)。

- r341沿实际篮轴15mm试抬，仅必要局部X避让，保持局部Y/姿态，退出原篮沿+6mm后调平；转子峰0.049174055rpm通过，但右返桌2.238481mm超2mm，FAIL保留。
- r342入口`table_alignment_cycle`继承右侧下降稳定类：真实桌面正支撑、≤3.5mm且近水平时最多两次水平≤1mm校正，原Z/姿态/holder/2mm、2°终态门保持。实跑已满足0.122178/1.215702mm，校正运动0次，不能称该分支实跑验证。
- 完整分开抓双装篮、关盖、稳转30.1865sim、停稳开盖、双盒返桌：234.032sim/649.716164822wall，148发布、4锚点4释放、28门、21路径24538密FCL通过。取回转子0.072811329rpm，原0.1门不改。
- 抓持峰2.300208mm/4.087195°满足现5mm/10°，原visual-only腕部碰撞缺口保留。2164测试/46/122回归/6CTest通过；Nav原2缺源不变。原速680.583333秒完整解码，终态俯视双盒返桌，正常SIGINT退休。
- 原生59609主线程CPU样本，精确符号范围确认Physics::Update累计89.4211%、BoxedLcp受约束组求解58.2395%；世界RTF0.354713。比例有包含关系，不相加、不当墙钟成本，World::bake/exists近邻误标签已排除。
- 未达三分钟/稳定1倍，r337的600.442257241wall仍较快。下一步另建身份量化约束规模/求解，不直接删盒/爪/篮接触、不改时钟。九组前后保护为封存门，r337/r318/r327/r331/r333、孵育箱/920版/三模型/十备份不覆盖。

## 0a. 历史r338–r340性能排查与稳定性边界

[本轮汇总](runtime_evidence/centrifuge_48box_sequential_review_20261021_r8/REPORT.md)、[r340原报告](runtime_evidence/centrifuge_48box_sequential_full_20261021_r340/REPORT.md)、[失败过程录像](runtime_evidence/centrifuge_48box_sequential_full_20261021_r340/run/full_cycle.mp4)。

- 精确几何快路径通过字节等价和组件基准，但r338右返桌出现10.024783°抓持角偏。
- r339右粗下降减冲击尚未触发，即在左试抬因0.112222255rpm转子瞬态停持。
- r340实际将取回15mm试抬降为0.05rad/s，仍在左试抬出现0.377062651rpm，制动输出到原-2N·m下限；减速单因子不足以解决。
- 已完成双装篮/关盖/30.1855sim稳转/停转开盖，未双盒取回。r340的407.193697229wall仅失败前缀；旧完整基线r337仍为600.442257241wall。
- r340入口`right_setdown_cycle.SpeedPriorityCycle`显式继承父速度类；原PROFILE保留，新增identity单列试抬0.05及右下降0.2/ramp0.6，原5mm/10°和0.1rpm门不变。
- 2135本地测试、46/122回归/6CTest通过；Nav111/2已知缺源不变。三次九组保护PASS，视频完整解码/正常退休，旧版本不覆盖。
- 世界区间RTF0.373–0.409，Gazebo主线程约0.93核；先测量主线程内各系统成本，再针对接触/发布链做独立优化，不将GIL栈样本当因果分摊。

下一步优先离线核验试抬横向校正、篮壁接触力矩和制动输出夹限，建立可重复的单变量验证后才另建身份实跑；不重复使用r338/r339/r340，不修改旧成功和孵育箱工作。

## 1. 完成了什么

用户选择左右手**分开抓、保持抬肘姿态近似**，允许在仿真中放宽门限并加快动作，但要求保留r318成功版本。

新[r327](runtime_evidence/centrifuge_48box_sequential_full_20261020_r327/REPORT.md)实际完成：

1. 左右手依次从桌上抓取两个85%48box，分别放入两个摇篮。
2. 机器人真实指尖按按钮关盖、启动，稳速旋转30.168仿真秒。
3. 真实按键停止，确认停稳后打开盖子。
4. 左右手依次取回两盒，放到两个不同桌面位置，双爪空、终态观察1秒。

同一时刻最多一个实际holder；154次运动发布、4次真实抓持锚点及4次正常释放。不是把模型直接生成到篮内，也没有瞬移、物理attach或重设持物锚点。

**实际耗时234.8065仿真秒、615.775707219现实秒（10分15.776秒）。当前分开抓的仿真180秒、现实180秒及稳定1倍实时均未完成。**

## 2. 基线及时间口径

| 版本 | 姿态/合同 | 仿真秒 | 现实秒 | 定位 |
|---|---|---:|---:|---|
| [r283](runtime_evidence/centrifuge_48box_optional_micro_full_20261015_r283/REPORT.md) | 旧双臂同时装取 | 179.1290 | 2324.616083 | 历史仿真三分钟完整成功 |
| [r291](runtime_evidence/centrifuge_48box_elbow_full_20261016_r291/REPORT.md) | 旧双臂抬肘近似 | 207.730 | 2362.143548 | 旧姿态成果保留 |
| [r318](runtime_evidence/centrifuge_48box_sequential_full_20261020_r318/REPORT.md) | 分开抓，先前2mm/5°持物合同，终态10秒 | 428.45 | 3894.946321249 | 必须保留的原成功 |
| [r327](runtime_evidence/centrifuge_48box_sequential_full_20261020_r327/REPORT.md) | 分开抓，显式5mm/10°快版合同，终态1秒 | 234.8065 | 615.775707219 | 当前新成功，独立封存 |

r327较r318墙钟快约6.3253倍、减少84.19%；仿真时长减少45.20%。不同合同的单次比较不是同严格门的加速基准或重复可靠性证书。r159的172.500仿真秒仅装篮，不包含完整离心和取回。

r327仿真范围为25.232→260.0385。单调墙钟包含执行器启动、导入、预检、在线规划、执行和退出；不含场景启动、人工首帧审核及录像终结。任务仿真/墙钟比约0.381，不能当Gazebo纯物理RTF。录像长度及任何演示快放都不用于证明执行耗时。

## 3. 本轮根因与失败链

所有身份的失败记录、原片、哈希和封存状态保留；以下压缩过程说明，不覆盖原始结论。

| 实验 | 变化与观察 | 结论 |
|---|---|---|
| [perf_r1](runtime_evidence/centrifuge_48box_sequential_perf_20261020_r1/REPORT.md) | 原脚接触上限4：RTF约0.144554→0.401909 | 组件通过，其他基线失败，总体FAIL |
| [perf_r2](runtime_evidence/centrifuge_48box_sequential_perf_20261020_r2/REPORT.md) | 私有两足完整AABB：0.141244→0.874659 | 被动组件更快；非完整任务/1倍，腿初始化不兼容 |
| [planning_perf_r1](runtime_evidence/centrifuge_48box_sequential_planning_perf_20261020_r1/REPORT.md) | 4个真实抓前快照，跳过假想未来搬运预演，73.948854433→17.933111233wall秒 | 实际pickup字段相同、8886密FCL通过；真实抓后仍需重规划 |
| [init_r1](runtime_evidence/centrifuge_48box_sequential_init_20261020_r1/REPORT.md) | 完整AABB阻碍原腿目标，残差约89mrad | FAIL |
| [init_r2](runtime_evidence/centrifuge_48box_sequential_init_20261020_r2/REPORT.md) | 原站姿定向、底面修剪至地面的私有脚代理，残差约1.48mrad | 仍超1mrad，FAIL |
| [init_r3](runtime_evidence/centrifuge_48box_sequential_init_20261020_r3/REPORT.md) | 静止腿端点2mrad、原目标/3秒ramp/5秒保持 | 组件PASS，保持峰0.001496919rad，后续RTF约0.847 |
| [r319](runtime_evidence/centrifuge_48box_sequential_full_20261020_r319/REPORT.md) | 首次集成，腿误差2.259mrad超过2mrad | 未进入手部任务 |
| [r320](runtime_evidence/centrifuge_48box_sequential_full_20261020_r320/REPORT.md) | 静止腿5mrad通过；重复查询后时钟偏差0.1555超过旧0.15秒 | 0运动发布，FAIL |
| [r321](runtime_evidence/centrifuge_48box_sequential_full_20261020_r321/REPORT.md) | 快时序；初次开盖后的空手退臂跟踪0.0188rad超过旧0.015 | FAIL |
| [r322](runtime_evidence/centrifuge_48box_sequential_full_20261020_r322/REPORT.md) | 空手按钮跟踪0.08；空手停放时软件指速0.231819超过0.2 | FAIL |
| [r323](runtime_evidence/centrifuge_48box_sequential_full_20261020_r323/REPORT.md) | 软件指速监控0.6，原腿初始化通过；启动73.6sim超过旧45窗口 | 未进入完整任务 |
| [r324](runtime_evidence/centrifuge_48box_sequential_full_20261020_r324/REPORT.md) | 启动窗口300；左已装入、右已抓持，原生配对滞后17ms超过旧12ms | FAIL |
| [r325](runtime_evidence/centrifuge_48box_sequential_full_20261020_r325/REPORT.md) | 滞后50ms、真实关节夹逼仍12ms、gzip1；双装篮/稳转30.181/停稳开盖通过 | 左取回实际倾斜3.329°超过旧2°规划门，未派发搬运 |
| [r326](runtime_evidence/centrifuge_48box_sequential_full_20261020_r326/REPORT.md) | 取回5°，原篮轴退出后沿原高点逐步调平；实际快照密FCL通过 | 实跑在空手开盖退臂0.0836rad超过0.08时停止，未验证取回 |
| [r327](runtime_evidence/centrifuge_48box_sequential_full_20261020_r327/REPORT.md) | 相对r326仅空手按钮跟踪改0.15rad，原接触几何/端点保持 | 完整快版合同PASS，旧合同及180秒仍不通过 |

这里放宽的是**新的显式仿真合同**，不是重写r318或将历史失败改成PASS。初始化脚底代理并非完整网格的保守包络，仅固定骨盆场景可用，不能推广为步态/真机模型。

## 4. r327实际采用的合同

| 项目 | 新身份采用值 | 仍保留的检查 |
|---|---|---|
| 原路径点重定时 | 空手1.2、载物0.8、近段0.4、细段/试抬0.15rad/s | 原关节界、渐变、原密FCL及实际反馈 |
| 观察等待 | 抓稳10→1sim秒，释放3→0.5，终态10→1 | 真实两盒/释放后状态；稳转仍至少30sim秒 |
| 抓持偏差 | 5mm/10° | 原始锚点、真实双指/原生位姿与FK独立复核 |
| 空手按钮跟踪 | 0.15rad | 原按钮几何、20ms实体接触段、5mrad端点 |
| 取回姿态 | 指令倾角5°，沿实际篮轴先退出，再逐步世界调平并保持实际yaw | 原高点/实际载物路径密FCL，不直线穿出篮壁 |
| 静止腿初始化 | 原12腿目标、3秒平滑动作、5mrad端点和5sim秒保持 | 真实反馈；仅新私有脚底代理/固定骨盆 |
| 启动窗口 | 300sim秒 | 新身份、精确PID/ctime、fresh六控制器、录像首帧 |
| 指速 | 软件监控0.6m/s | 实体模型速度0.2不改 |
| 原生时序 | 最新配对滞后50ms | 真实关节插值夹逼仍12ms，不外推，仍有最新原生与FK一致性守卫 |
| 日志/规划 | gzip1无损；不算抓前假想未来搬运 | 全trace，不抽样；真实抓后新快照、新持物路径仍必需 |

不取消原关节界、真实接触/抓持、篮内完整包络、双盒终态或30秒稳转；不用模拟时钟修改、瞬移、物理绑定或异常松夹凑结果。规划中的collision attachment不等于物理绑定。

## 5. 独立验收与已知局限

- [物理审计](runtime_evidence/centrifuge_48box_sequential_full_20261020_r327/speed_physical_audit.json)28/28；[任务审计](runtime_evidence/centrifuge_48box_sequential_full_20261020_r327/speed_task_audit.json)标记`PASS_RELAXED_SIMULATION_SEQUENTIAL_TASK_NOT_OLD_CERTIFICATE`。
- [抓持复核](runtime_evidence/centrifuge_48box_sequential_full_20261020_r327/independent_speed_grip.json)：左右1451/1446次，4锚点/4结束，FK/native滑移2.158579/2.158360mm、角偏4.218821°。新门PASS，旧2mm/2°FAIL，不隐藏差别。
- [实际路径复核](runtime_evidence/centrifuge_48box_sequential_full_20261020_r327/speed_actual_paths_review.json)：19条、24,439次原2mrad密FCL，核对实际发布点。原visual-only腕部等碰撞缺口未改，不承诺绝对不碰撞。
- 取盒转子瞬态0.053409829rpm、8样本超过旧0.02，未超过授权0.1；终态左右盒位置误差0.325218/0.960172mm。双手空、盖开、停转；未要求中立收臂或原出生槽一一归位。
- 1938本地测试、原46/122和6CTest通过。Nav111通过/2项旧V12来源失败保留，未跑导航；原训练来源缺失不被伪造补齐。
- [独立最终汇总](runtime_evidence/centrifuge_48box_sequential_review_20261020_r15/final_review.json)涵盖identity、物理/抓持/路径、trace、录像、退休与九类保护。

## 6. 录像、日志和保护

[完整前视原片](runtime_evidence/centrifuge_48box_sequential_full_20261020_r327/run/full_cycle.mp4)为649.333333秒、25,787,091字节、1200×900/12fps、7792原始帧、0重复；正常SIGINT刷盘、全解码PASS，无加速/裁剪。已看首帧和230/340/500/640秒实际帧；远侧返桌盒自然遮挡，双盒返回以物理证据为准。

无损trace612,901行、原始1,597,447,827字节、gzip420,685,394字节，状态CLOSED。547源identity及796核心已封存，review_r15有14成员。关键SHA统一在[当前总览](HANDOFF_CURRENT.md)；完整清单见[r327 SHA256SUMS](runtime_evidence/centrifuge_48box_sequential_full_20261020_r327/SHA256SUMS)与[review_r15 SHA256SUMS](runtime_evidence/centrifuge_48box_sequential_review_20261020_r15/SHA256SUMS)。

本次运行九类before/after通过，25,579历史成员、785正式源、三模型及十备份保持；r318成功、旧双臂版本、孵育箱、920箱子版、所有原视频/失败记录均保留。场景已正常退休，不能复用r327的run/claim。文档整理仅复核其声明的有限保护集合，不重新声称全归档扫描。

## 7. 后续新身份接口

以下是开发接口说明，**不是重跑封存脚本的命令**。本轮继续开发已按明确授权派生r328–r330；后续不能复用这些已退休身份。

1. 用户明确继续离心机开发时，按分支D及HANDOFF_PLAN第33节，先重新只读检查live/夹持/保护，枚举未用builder/run/domain/partition/overlay/log/video，普通`mkdir`独占。
2. 只在新私有候选修改；按AGENTS六库隔离构建与相关完整回归，准备/校验经`tools/trace_archive/run --`，保留继承的`PYTHONPATH`及归档registry。
3. 一次seal后通过精确PID/ctime、fresh六控制器、真实原腿初始化和录像门。实际首帧看图；仅启动审看使用已审核的自有server冻结/一次resume，不用world reset或持物现场恢复。
4. 连续执行器执行整套动作，不等AI逐段手动派发；并行守卫、磁盘监测与正常收尾。r327入口为`SpeedPriorityCycle(SpeedFeedbackMixin, SequentialCycle)`，后续实验另有性能mixin，以各自密封源码为准。
5. 以真实monotonic计时，独立物理、抓持、实际路径、trace、原片内容/全解码、保护与终态全部核验。失败stop-and-hold，再按精确身份正常SIGINT，不reset/detach/异常松夹。
6. 新结果独立保存；把r318/r327及review清单加入保护，不修改旧identity、inputs、claim、报告或视频。正式源有改动才按权威另做隔离构建，不把私有实验覆盖日常模型。

## 8. 下一步性能假设

现有只读CPU统计：执行器约1.022核、Gazebo server约1.427核；这不是函数级归因。稳转阶段31.332sim/74.656wall秒，部分持物阶段比例0.16–0.29；继续抬速度门不能独自解决实时性能。

优先在**新身份**测量在线规划/重复计算/序列化及物理接触的分项成本，再做同条件A/B；保留全量证据、真实物理和30秒稳转。之后再实际验证全流程wall≤180秒；不能以纯被动RTF、预算、快放视频或单阶段成绩替代。

本轮文档原文与修改审计见[整理报告](runtime_evidence/documentation_roundup_20261020_r1/REPORT.md)。本文件不合并删除正式证据，不授权新备份、清理、腿部导航或实体机器人。

## 9. 最新r328–r330：组件减负，完整任务仍未通过

用户进一步授权直接修改、测试、完整运行录像，同时保护r318/r327。
本轮普通mkdir创建三个身份，隔离构建、一次seal、fresh及实际首帧检查后连续实跑；
没有每个动作等待AI，没有修改正式src/模型或继承的抓持/碰撞/正力门。

| 身份 | 性能变化 | 最新故障 | 最终本地回归 |
|---|---|---|---:|
| [r328](runtime_evidence/centrifuge_48box_sequential_full_20261020_r328/REPORT.md) | 有界关节时间线索引，避免反复复制全队列；50Hz有界线程栈采样 | 左取回试抬转子-0.125302rpm超0.1，97发布/413.145525wall秒 | 1961 |
| [r329](runtime_evidence/centrifuge_48box_sequential_full_20261020_r329/REPORT.md) | 64KiB批量gzip/哈希，原JSONL逐字节保留；取回试抬改0.10rad/s | 双装篮后空手按钮退臂-0.154rad超0.15，76发布/247.891034wall秒 | 1970 |
| [r330](runtime_evidence/centrifuge_48box_sequential_full_20261020_r330/REPORT.md) | 512项精确浮点位姿缓存及防修改副本；按钮空手非接触段上限0.8rad/s | 双装篮/30秒稳转/停稳开盖/左返桌后，右取盒预压支撑证据缺失，129发布/521.840822wall秒 | 1986 |

三次均是完整任务的**失败尝试**，不是完成了三个新全流程；全部录制原速视频并正常
SIGINT/退休，独立trace/视频/保护通过不替代任务成功。对应review_r16/r17/r18保留
原始故障和最终独立摘要。原46/122/6CTest通过，Nav111/2旧缺源失败仍保留。

组件等价基准：时间线约14.72倍，日志约1.075倍，位姿缓存约4.041845倍；
运行阶段不同且均未完整成功，不能推导或相乘为全程加速。手写刚体逆比NumPy逆慢，
未采用；线程切换间隔微测收益小且不代表实跑，未采用。r330实际位姿缓存命中
10855102/未命中34233，但返回副本仍在采样叶栈占42.86%；采样有GIL偏差，
不是CPU时间因果分摊。Gazebo server/GUI精确PID均值约1.418/1.079核，定位节点
约0.847核；未更改Mid360或定位频率，未证明稳定1倍实时。

r330的右预压故障位于241.272sim：上一正篮底力时戳241.2715、差0.5ms；
当前只剩单指0N，关节落后2ms，开始预压0.383秒，盒几何仍在当前容差内。
原单缺帧逻辑要求正力指尖，故拒绝；没有建立第四个真实锚点，也没有派发右抬升。
不能宣称盒已掉落，不能以零力/近似几何当抓稳。下一实验先区分接触转换与采样
缺帧，任何有界等待必须仍禁止创建holder/抬升，强双指抓稳资格不得混为等待资格。

最新原片565.75秒/22301320字节、全解码通过；trace574288行无损通过；
独立已观察滑移FK/native 0.852226/0.848499mm、角偏2.650490°，不是完整任务证书。
本轮所有保护哈希通过，r318/r327仍为成功基线，正式源码/模型、920箱子版和十备份
未改。下一步性能候选是同输入几何重复计算的精确缓存及固定骨盆定位负载A/B，
须新身份逐项证明，不通过削减原始证据、假造时钟或取消物理门凑现实180秒。

实验和文档来源集中于[本轮汇总](runtime_evidence/centrifuge_48box_sequential_runtime_perf_20261020_r1/REPORT.md)。

## 10. r331完整成功，r332精确缓存候选在定位门失败

按用户“先解决右盒预压支撑不足、再完整录像、保护r318/r327”的授权，新建两套
身份/domain139与140、私有overlay、录像和独立review，未覆盖正式源/模型或旧成果。

[r331报告](runtime_evidence/centrifuge_48box_sequential_full_20261020_r331/REPORT.md)：
仅在已接受的取回预压、无holder、实际篮底近邻几何、0.5ms内正篮底历史且当前
本手中带零力指尖接触时，有界消费真实新观测；最多64回调、1ms仿真、20ms协作式
墙钟预算，底层检查仍执行，不发新动作、不创建holder。结束必须再过原完整支撑，
真正抬升仍须原强双指/真实锚点。r330停持后9.5ms才到达的正力不作有效恢复证据。
37新增窄分支测试、2023最终本地测试通过；**实跑未触发等待**，不夸大复现覆盖。

r331完整双盒装篮→关盖→稳转30.1865sim→停稳开盖→双盒返不同桌槽通过，
157发布、4锚点4释放、20实际路径/24521密FCL、28物理门。
239.198sim/621.132718628wall秒，不比r327的615.775707219wall更快；
未到现实或当前姿态仿真三分钟。独立FK/native滑移2.479099/2.477191mm，
角偏5.400582度，符合5mm/10度但旧严格门FAIL；取回rpm峰0.058735082。
原片657.333333秒、1200×900/12fps/7888帧全解码，抽帧实际看过，远桌盒遮挡如实保留。
核心835项SUM 5dedec3f942ce59e9a51bb4b2cbd15b3c1aa09eead8c0a77613880d9b9a66d6c；
review_r19的38项SUM 74439505241590e5797ae68e25a44eb4e53d518e546ede3912ad1d2754610a38。

[r332报告](runtime_evidence/centrifuge_48box_sequential_full_20261020_r332/REPORT.md)：
精确篮角点512项LRU，三个实际位姿21浮点原字节键/内部只读/返回副本，原矩阵计算
与角点顺序不变；不缓存接触/力/时间/结论，不改物理、速率或抓持门。
5000随机和实际样本逐字节通过、组件约4.88倍，不是整段提速。
隔离6.84秒构建，旧来源恢复器补充精确函数替换及变异拒绝测试后2052/2052通过；
早期失败日志保留。原46/122/6CTest通过，Nav111/2旧缺源失败保留。

实跑首次开盖后39.901sim因定位源38.9龄1.001秒超过原1.0门停止；4发布/0锚点/
14.031937063wall秒为失败片段。缓存hits=misses=0，尚未进入装篮；不能宣称缓存实跑
更快或导致数学异常。596组实际trace角点离线重算通过不是运行命中证据。
停持后源39.4至41.4的定位消息成批到达，发布/DDS/消费侧原因未定，未放宽定位门。
随后42.1905sim relay重复时间戳FAIL原日志保留，不倒推为更早故障原因。
原片69.583333秒完整解码，首帧/60/68秒实际看过；不是完整双盒视频。

两次均正常退休，九类前后保护按各运行落盘核验；r318/r327/r331旧成功各自封存。
下一步先独立观测定位发布/接收/消费排队，完成可解释的时序修复，再作新身份缓存
与接触计算性能验证。maxContacts=4仅离线转换探针，未改本次模型、未验证DART采纳。
不以快放、放宽定位门、删证据、改变时钟或把失败耗时当完整速度凑180秒。
本轮MD原文及AGENTS前缀身份在review_r19/documentation_before.json，
最终文档核验见[review_r20](runtime_evidence/centrifuge_48box_sequential_review_20261020_r20/REPORT.md)。

## 11. r333：定位订阅修正后完整运行与录像通过

[r333正式报告](runtime_evidence/centrifuge_48box_sequential_full_20261021_r333/REPORT.md)与[本轮复核](runtime_evidence/centrifuge_48box_sequential_review_20261021_r1/REPORT.md)：新domain141/partition/overlay/run，只改执行器两条定位订阅为BEST_EFFORT/KEEP_LAST深度1，记录DDS发送/接收/回调时间，原定位算法、传感器频率、质量门和1秒源龄门不改。真实DDS资格、2067本地测试、原46/122/6CTest通过；Nav111/2旧缺源保留。

实际完成分开抓双装篮、实体关盖/启动、30.094sim秒稳转、停稳开盖、双盒返不同桌槽、双爪空1秒。157发布、4真实锚点4释放、20条路径24634密FCL、28物理门和独立抓持通过。239.8095sim/644.752285933现实秒，慢于r331和r327，**现实三分钟、当前分开抓仿真三分钟、稳定1倍仍未达**。

定位476条pose、951次健康检查无无效项，最大源龄0.584sim秒；执行器DDS传输峰0.462ms、回调排队峰26.850ms。独立同源可靠/最新样本传输峰约41.5/0.465ms，但本次没有复现历史r332数秒延迟，不能据单次成功确认唯一根因或长期可靠性。真正过期仍失败，不回填旧样本。

独立滑移2.261860mm/4.902739度符合当前5mm/10度，旧严格门未过；最终左右桌槽误差0.285251/1.122609mm。原visual-only碰撞缺口、固定骨盆范围保持。右预压本次正常通过，但等待分支依然未触发。精确角点缓存1733167命中/15694未命中、21716组实际角点逐字节重算通过，不代表整机提速。

[完整原速录像](runtime_evidence/centrifuge_48box_sequential_full_20261021_r333/run/full_cycle.mp4)713.75秒/8565帧全解码，实际看7个关键帧；正面远侧遮挡如实保留，末尾712秒俯视可见双盒均返桌、双篮及双爪空。原片SHA `e413ae451efa24fff659fc78c96b3aab47ca0d5c04c0491738a1547e9f739a25`。

580源identity `4a3c61ee93c592576620e72a6410d79f79e9b4a7bd70d079cff250a5c90ed50a`；854核心SUM `a1caf853a13ef2eb852764741f8fd6b01ab9403794fc07a96f741c0dee0bd53f`。九类前后保护通过，正常SIGINT退休、零owned残余，r318/r327/r331及正式源/模型/孵育箱/920箱子版/十备份未改。

下一步依44个世界样本区间RTF约0.378171，隔离测物理接触求解和重复几何成本；不把GIL叶栈占比当因果证据、不靠降低定位门凑180秒。所有后续实验仍新身份，本轮无清理、新备份或导航/真机。

## 12. r334–r336常驻规划与连续段（均未完整成功）

[本轮独立汇总](runtime_evidence/centrifuge_48box_sequential_review_20261021_r4/REPORT.md)记录三个全新身份，旧成功原件不改。常驻planner每次仍使用真实快照、独立场景、原IK和密FCL；连接load_lift→load_transfer与retrieve_outboard→retrieve_transfer，但不跨接触/抓持/释放门，不缩短30秒旋转。运行期间不等待AI逐动作确认。

r334右取回IK失败；r335修正右取回悬空高度但在右装入前1.23°校正门失败；r336加入最多两次≤1°/1mm校正，最终2102项本地测试及46/122/6回归通过，导航V12缺失2失败保持。r336实际双盒装篮、关盖、稳转30.1425sim、停转后在开盖中定位源龄1.0025sim超1秒门失败，350.351281705现实秒仅失败前缀。未完成取回，不晋升为成功版本。

[r336报告](runtime_evidence/centrifuge_48box_sequential_full_20261021_r336/REPORT.md)与[原速失败录像](runtime_evidence/centrifuge_48box_sequential_full_20261021_r336/run/full_cycle.mp4)410.916667秒完整解码；最后盖子在打开中、双盒仍在篮中。真实局部滑移峰FK/native0.969033/0.967689mm、角偏0.426801°，但完整抓持证书FAIL。>1°双步分支本次未触发，右取回高度修复也未执行；离线资格不能代替实跑。

下一步须同步记录点云源戳、localizer发布间隔、TF等待区分生产空窗与未收到样本；本次收到样本低DDS延迟不证明无丢帧。仍不改1秒定位门、不伪造新戳。原收尾NoSuchProcess竞态保留，精确身份重试原retire后零残余。所有后续实验仍新身份；r318/r327/r331/r333和正式成果、视频、日志、备份不覆盖。

## 13. r337：定位修正后完整成功（当前覆盖）

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
