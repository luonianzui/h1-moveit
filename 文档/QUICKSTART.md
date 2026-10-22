# H1-2 当前只读复核入口

## 最新目标改为现实三分钟：当前仍未实现

旧r283179.129秒只代表仿真时间。world原本已target1.0，七个新静止性能对照
最快仍约0.135倍实时，未再执行完整动作；所有被动场景已正常退出/保护PASS。
只读查看：

```bash
root=runtime_evidence/centrifuge_48box_wallclock_perf_20261015_r3
cat "$root/REPORT.md" "$root/PERFORMANCE_SUMMARY.json"
```

必须使实际速度达到1:1，再以monotonic墙钟≤180验收新全流程；不能仅改配置或
视频快放冒称达成。旧成果、模型与备份未改。下面仿真目标的成功结论仍保留。

## 2026-10-15 最新完整结果：r283，179.1290sim秒

最新完整双盒装篮、指尖关盖、稳转30.258sim秒、停稳开盖、双盒返桌及末态
10秒观察为**179.1290仿真秒**，独立28/28/双持物/录像内容与全解码/保护PASS。
真实wall约38分45秒，不能当现实三分钟；单次固定骨盆手部成功，不是Nav2联动。
旧结果全部保留，下面“最新r227/r225”和“180未达”等只作历史。

只读查看，不启动仿真，也不复用已用run/claim：

```bash
cd /home/liu/h1_moveit/h1_2_ros2_ws_claude_2026-07-30/h1_2_ros2_ws
root=runtime_evidence/centrifuge_48box_optional_micro_full_20261015_r283
cat "$root/REPORT.md" "$root/complete_speed_audit.json"
cat "$root/cycle_audit.json" "$root/independent_dual_grip.json"
cat "$root/run/terminal_integrity_review.json" "$root/video_audit.json"
(cd "$root" && sha256sum -c SHA256SUMS)
```

原片在`run/full_cycle.mp4`，20相位拼图在`run/actual_phase_contact_sheet.png`。
接口/再运行边界见[CENTRIFUGE_FULL_CYCLE.md](CENTRIFUGE_FULL_CYCLE.md)。九身份
已正常SIGINT退休；所有旧孵育箱/离心机/导航成果与10备份保持，920箱子版本仍可用。
原Nav111/2项V12缺源事实不变；可选四退路/微对齐未在本次触发，不能外推物理资格。

## 2026-10-14 旧日志无损归档后的入口

r161–r211 精确 31 份 raw 日志已核验后改为无损 gzip，释放约 54.09 GB。
新候选准备、Python 回归及前后保护均应通过以下启动器运行，以兼容旧路径和旧 SHA：

下面是不同阶段的入口示例，不要整段顺序执行。`prepare_candidate.sh` 已包含
before 保护；准备成功后不能再次运行 before，以免重复创建已存在的审计输出。

```bash
tools/trace_archive/run -- bash runtime_evidence/NEW_CANDIDATE/prepare_candidate.sh
tools/trace_archive/run -- /usr/bin/python3 runtime_evidence/NEW_CANDIDATE/verify_dual_protection.py before
tools/trace_archive/run -- /usr/bin/python3 runtime_evidence/NEW_CANDIDATE/verify_dual_protection.py after
```

`NEW_CANDIDATE` 只表示新身份占位符，不授权重跑旧身份。启动器环境须继承到子进程。
全部归档检查：`python3 tools/trace_archive/archive_cli.py doctor`。
含已归档 raw 路径的旧 SHA 清单使用
`python3 tools/trace_archive/archive_cli.py verify-manifest 清单路径`；原生
`sha256sum -c` 不支持该逻辑路径。现有 MP4 展示不受影响。
详见 [使用说明](tools/trace_archive/README.md) 和
[迁移结果](runtime_evidence/trace_archive_20261014_r1/migration_result.json)。
原保护链前后校验及 31 份归档完整性检查均通过，相关测试共 **1,341 项通过**。
压缩范围、验证结果和兼容边界见 [Markdown 更新报告](runtime_evidence/trace_archive_20261014_r1/REPORT.md)。
备份或迁移时须同时保留归档 `data/`、`registry.json`、`inventory.json` 和
`tools/trace_archive/`，不能只复制旧 run 目录。

更新：2026-10-14 CST。当前状态统一见[HANDOFF_CURRENT.md](HANDOFF_CURRENT.md)，权威规则见[AGENTS.md](AGENTS.md)。本文不授权启动仿真或执行历史命令。

## 0. 最新终态与候选

最新[r269终态](runtime_evidence/centrifuge_48box_native_gripper_feedback_20261014_r269/REPORT.md)
实际新反馈原精度修复通过、双装篮52.691sim/稳转30.468sim/停稳开盖通过，但取盒
实际IK/密FCL拒绝，125.0915sim非完整。物理16/28/完整持物/速度FAIL；原片19510
真实/0重复/正常SIGINT全解码/15前缀格已看，483封存/九角色退休/保护保持。
当前无Gazebo。[r270](runtime_evidence/centrifuge_48box_dense_shortcut_20261014_r270/REPORT.md)
仅plan-only，同一失败请求以2mrad shortcut检查后几何PASS，7文件封存，不含运行
合同/身份/输入，不能直接启动。磁盘约1.61GB不足下次约2.10GB，已请求腾出至少
1GB；新完整候选须另立从未用身份及完整测试/fresh证据。基线185.695sim，180未达。
下文旧live/PID/待启动是历史，不复用r269/r270，也不额外删除重要成果。

当前live：r269原完整前检退出0/1072局部/17反馈专项/31时序/6.90s隔离构建，
318一次seal/277依赖复验/最新合格r266输出预算/fresh六控制器/0.6/500Hz/D6/
splines/72帧短测/原片首秒及首帧通过；首帧审阅实际晚于同工具脚本collector启动，
独立timing_note如实保留。独立collector1271223/ctime1791953056.35连续执行，
九身份及318成员见[检查点](runtime_evidence/centrifuge_48box_native_gripper_feedback_20261014_r269/run/live_checkpoint1.json)。
首6.3395<25、最新sim35.0085仍双持盒10sim前缀，尚无result，不是完整180证书。
接续重查同handle/progress/result，不重启/复用claim/活动中改源构建或另开场景。
下文未启动/当前无Gazebo皆历史，完整合格基线仍185.695sim。

最新覆盖：[r268](runtime_evidence/centrifuge_48box_schedule_gripper_20261014_r268/REPORT.md)
0.25松爪action成功后旧native误差0.000295628超原0.00025而FAIL，49.362sim仅前缀。
独立物理9/28/完整持物/速度FAIL；原片11607真实/0重复/正常SIGINT完整解码与前缀
视觉通过，469封存/314未变/274依赖不变/九角色退休/保护保持；当前无Gazebo。
新[r269](runtime_evidence/centrifuge_48box_native_gripper_feedback_20261014_r269/CANDIDATE_DELTA.md)
保持原精度，只成功结果后等实际新反馈，≤1sim/30wall及原safety。17新专项/
1072局部/6.90s两包隔离构建通过，完整前检一次进行中，尚未seal/claim/实跑。
下文r268 live皆历史，不复用；完整合格基线仍185.695sim，180未达。

当前live：r2681055局部/31新专项、6.87s隔离构建及全部前检退出0；314一次seal/
274资源SHA复验/fresh六控制器/0.6/500Hz/D6/splines/72帧短测/原片首秒和首帧通过。
独立collector971446/ctime1791950203.30连续执行，九角色和314成员不变已记录于
[检查点](runtime_evidence/centrifuge_48box_schedule_gripper_20261014_r268/run/live_checkpoint1.json)。
首6.3675<25，最新sim36.3935仍双手持盒10sim前缀，不是完整结果或180证明。
接续重查同handle与progress/result，不重启/复用claim/活动中改源构建或另开场景；
下文未seal/未运行、无live与旧PID都是历史。完整合格基线仍r266185.695sim。

最新候选：[r267离线失败](runtime_evidence/centrifuge_48box_direct_retrieval_20261014_r267/REPORT.md)
IK/RRT拒绝home-skip，370成员封存、未仿真。新[r268](runtime_evidence/centrifuge_48box_schedule_gripper_20261014_r268/CANDIDATE_DELTA.md)
保留原路线/final home，只有合格后起时0.16与原≤1mm零末力短行程末点0.25；
31新专项通过，正在一次隔离构建/完整前检，尚未seal/claim/fresh执行。预算非180。
两份旧安装包已按单独明确许可精确删除约1.12GB，当前安装/新更新包与旧保护保持，
45成员封存；该白名单结束。下文尚未处理安装包、旧live/候选均历史，不能复跑。

当前覆盖：[r266完整终态](runtime_evidence/centrifuge_48box_retrieval_rate_restore_20261014_r266/REPORT.md)
双盒装篮、关盖、稳转30.543sim、停转开盖、双盒返不同桌槽及末10sim完成，
首6.2395→191.9345为185.695sim；独立物理28/28、持盒/完整原片/日志/保护PASS，
速度FAIL，180仍未达。29838真实/0填充/正常SIGINT/全解码，20实际阶段已复核。
506成员0444封存、九角色退休；新只读检查未见Gazebo/执行器/collector，不复用。
用户要求先清理无用录像，本轮只删1个逐字节重复下载副本和33个pytest占位文件，
约5.4MB，其余363媒体SHA/权限保持，正式原片/失败诊断/所有备份保留，回收站不动。
见[定点清理报告](runtime_evidence/video_cleanup_20261014_r1/REPORT.md)。两份另获授权的旧
安装下载包尚未处理。下文旧候选/等待/live均历史，不能据此重跑准备或复用claim。

最新覆盖：[r264实际FAIL](runtime_evidence/centrifuge_48box_paired_ramp_20261014_r264/REPORT.md)，
双装篮53.690sim/稳转30.5485sim/停稳开盖通过，但取回右肘0.003120712rad超原
0.002，131.304sim只是失败前缀。20367原片真实/0填充/正常SIGINT/全解码、14阶段
已看；494封存/309未变/九角色退休/前后保护保持，不复用。新[r266](runtime_evidence/centrifuge_48box_retrieval_rate_restore_20261014_r266/CANDIDATE_DELTA.md)
不继承失败坡度，只恢复空手取回0.4，原路径/精度/物理/3、10、30证书不改；
7针对/1024局部及6.95s隔离构建/完整前检退出0，六库/272依赖/before保护通过，
未seal/claim/fresh实跑。[准备接续](runtime_evidence/centrifuge_48box_retrieval_rate_restore_20261014_r266/PREPARATION_STATUS.md)
明确禁止重跑整个open-x准备。磁盘约
1.5GiB，等待腾出至少2GB或另行限定旧缓存清理授权，当前无live。r265只派生失败，
不续用。完整合格基线r261216.0305sim，180未达；下文旧live/候选为历史。

最新覆盖：[r263实际FAIL](runtime_evidence/centrifuge_48box_button_ramp_20261014_r263/REPORT.md)，
双装篮53.909sim/五真实按钮/稳转30.591sim/停转开盖通过，取回空手0.6接近指爪
0.209453680410超原0.2；130.409sim仅失败前缀，未取回。19957真实帧/0填充/正常
SIGINT/全解码/14阶段复核通过，只前缀；477封存/306未变/九角色退休/旧保护保持。
新[r264候选](runtime_evidence/centrifuge_48box_paired_ramp_20261014_r264/CANDIDATE_DELTA.md)
保持原点/组/positions-only折线/0.6上限，仅空手取回接近首末0.2sim时间坡度；
1041离线通过，正新隔离构建/完整前检，尚未seal/claim/fresh实跑。原物理与3/10/30
证书不改，解析弧长加速度不是实际连续加速度证明。完整合格基线r261216.0305sim，
完整180目标未达；以下旧候选/live为历史，不复跑旧claim或盲信旧PID。

最新[r262终态](runtime_evidence/centrifuge_48box_complete_rate_profile_20261014_r262/REPORT.md)FAIL：
双装篮53.065sim/真实关盖通过；未平滑1.2启动接近原生指爪0.246517/0.252176超0.2，
未旋转/开盖/取回。完整物理10/28FAIL，原片14733真实/0填充/正常SIGINT/全解码，
15实际阶段已看，九角色退休/303未变/462封存/旧保护保持，不复跑旧claim。
新[r263平滑候选](runtime_evidence/centrifuge_48box_button_ramp_20261014_r263/CANDIDATE_DELTA.md)
28专项/1017局部通过；仅软件。同原折线/顶点有限加减速，原物理/接触/20ms/
追踪/终点及3、10、30证书不改，完整目标不缩减；新实际两臂splines回读必需。
26输入只身份绑定，下一步隔离构建/完整前检/一次seal/fresh实跑，180未达。

最新[r261完整终态](runtime_evidence/centrifuge_48box_leg_fresh_endpoint_20261014_r261/REPORT.md)：
完整动作/28项独立物理/双抓持/合格完整原片PASS，首6.0125→222.043=216.0305sim，
速度FAIL、180未达。稳转30.435sim、双返桌及末10sim；35409真实/0填充/正常SIGINT/
全解码，20实际阶段已看。467封存/292未变/九角色退休/10262历史/10备份/785源码保持。
新[r262完整时间候选](runtime_evidence/centrifuge_48box_complete_rate_profile_20261014_r262/CANDIDATE_DELTA.md)
137针对/989局部通过，仅离线；原速率预算仍189.7，不作达成。仅新私有盖子速度
0.35→0.8及声明的阶段时序，120°/30Nm/原物理/3、10、30证书保持，旧模型不写。
父级已封存，须新隔离构建/完整前检/一次seal/fresh完整实测；没有r262物理/180结论。
以下旧活动是历史快照，不能复用PID/claim或覆盖既有身份。

历史[r260终态](runtime_evidence/centrifuge_48box_zero_effort_timing_20261014_r260/REPORT.md)FAIL：
腿1sim action成功后独立source8.92误差0.001015253572超原0.001，source8.93已在
原门内；立即断言导致2.434sim准备前缀失败，未手部动作，D6/新时序未物理证明。
623真实/0重复/正常SIGINT/全解码/四收尾PASS，412封存/292未变/九角色退休/保护保持。
新[r261候选](runtime_evidence/centrifuge_48box_leg_fresh_endpoint_20261014_r261/CANDIDATE_DELTA.md)
只加结果后的原精度新反馈等待3sim/30wall上界，原轨迹/精度/偏差/1sim观察保持；
852局部/6.97s隔离构建/六库恢复/完整前检退出0；292一次seal/266依赖复核，
[r261检查点](runtime_evidence/centrifuge_48box_leg_fresh_endpoint_20261014_r261/run/live_checkpoint2.json)
已fresh连续实跑，六控制器/.6/500Hz/四D6/原片首秒通过，九角色存活/292未变，
collector2888952/ctime1791928101.67。新反馈门8.94及原1sim观察9.94通过，初始
D6原生空手26.774通过，两10sim保持51.0045通过，52.2805只装载运输前缀。
查同进程/trace_progress/result/原片，不观察超时重启；活动中不改密封源/移动
六库构建/另开Gazebo。下文旧活动均历史，完整180未达，不清理/备份/真机/腿部导航。

最新[r259终态](runtime_evidence/centrifuge_48box_camera_deadline_20261014_r259/REPORT.md)FAIL：
实际装篮70.4635sim/机器稳转30.4785sim与密交接/开盖通过；取回接近原生
左finger1速度0.2208555>0.2而保护，164.4395sim仅失败前缀，未实际取回。
录像25543真实/0重复/正常SIGINT/全解码/相机通过，14帧原片已看；四收尾通过/
九角色退休/446封存/287身份未变/保护保持。旧RUNNING检查点不再是live。
新[r260候选](runtime_evidence/centrifuge_48box_zero_effort_timing_20261014_r260/CANDIDATE_DELTA.md)
原模型/物理门不改，仅低幅空爪时刻/fine速率与四D3→6；fresh四D6实际回读与
当前原生接近资格必需。845局部/6.71s隔离构建/六库恢复/26输入审核/before保护
通过，原51592完整前检进行中，尚未seal/claim/Gazebo，以下旧活动均历史。
完整180未达，不复跑已有open-x或复用claim，不清理/备份/真机/腿部导航。

最新[r258终态](runtime_evidence/centrifuge_48box_gripper_damping_20261014_r258/REPORT.md)FAIL，
[独立物理计数勘误](runtime_evidence/centrifuge_48box_gripper_damping_20261014_r258/REPORT_ERRATUM.md)
为20/28。实际装篮75.620sim、稳转30.538sim、实际停转交接和两盒桌槽释放完成；
录像三次ACK超时导致守卫SIGINT，缺返桌稳定/末端10sim/完整PASS。215.461sim为
失败前缀；36076真实帧/全解码但录像FAIL，原446及2项勘误封存/九角色退休/保护保持。
新[r259候选](runtime_evidence/centrifuge_48box_camera_deadline_20261014_r259/CANDIDATE_DELTA.md)
仅有界相机期限3000ms/5s/join16s与静止腿初始化5→1sim；三次真实ACK、原目标/
容差/物理门/1sim观察、其他速率和模型保持。63针对/821局部、6.93s隔离构建/
六库恢复通过，Nav111/2V12缺源保留；完整前检退出0，287一次seal/263依赖复核。
[r259检查点](runtime_evidence/centrifuge_48box_camera_deadline_20261014_r259/run/live_checkpoint1.json)
已fresh连续运行，六控制器/.6/500Hz/四D3/72帧短测/原片首秒通过，九角色存活/
287未变，collector2431297/ctime1791923654.04。fresh门7.479、新初始化9.9345/
误差峰0.000153670rad通过，原生初始空手路径28.417、双10sim保持52.6475通过。
最新[第二检查点](runtime_evidence/centrifuge_48box_camera_deadline_20261014_r259/run/live_checkpoint2.json)
首6.531/sim53.887装载运输中，九角色仍活/287未变，尚无result，仅前缀。
查同collector/trace_progress/result/原片，不观察超时重启；活动中不改密封源/
移动六库构建/另开Gazebo。下文全部旧活动均历史，完整180未达，不清理/备份/真机/腿部导航。

最新[r255终态](runtime_evidence/centrifuge_48box_empty_return_gain_20261014_r255/REPORT.md)FAIL：
装篮/关盖/真实30.5055sim稳转/实际按停完成，回home指爪-0.202595m/s>原0.2而保护。
未最终开盖/取回，152.517sim非完整；421封存，25576真实+2重复帧/全解码/正常退休。
当前[r256检查点](runtime_evidence/centrifuge_48box_stop_handover_20261014_r256/run/live_checkpoint1.json)
fresh连续实跑，省停转后多余right home、保留实际撤离/rearm，左开盖前必须新
实际快照密FCL；四段自由搬运试0.4，物理硬门不改。799局部/7.10s构建/六库恢复/
260依赖/before含r255421通过，281一次seal，九角色身份存活/成员未变，sim51.716
双抓取中尚无result，不能预称完整180或录像收尾成功。下文旧活动均历史，接续
重查同collector1377929/ctime1791913542.04，不复用claim或观察超时重启。

最新活动覆盖：[r255检查点](runtime_evidence/centrifuge_48box_empty_return_gain_20261014_r255/run/live_checkpoint1.json)。
前检退出0，779局部/64双/46配置/122旧装载/6CTest与7.20s构建/六库恢复/before
含r254413通过。272一次seal、256依赖一致，fresh六控制器/真实0.6/500Hz/72帧短测
及同原片首秒已看。collector944370/ctime1791909272.63连续执行，九角色身份存活/
272成员未变，sim25.174只装载前缀，尚无result，不能预称完整180/终态视频。
下文未启动描述均历史；接续重查同进程/原片/日志，不观察超时重启或复用claim。
当前活动中禁止改密封源/移动六库构建/另开Gazebo，失败按原守卫正常收尾。

最新[r254终态](runtime_evidence/centrifuge_48box_gain_full_cycle_20261013_r254/REPORT.md)FAIL：
gain0.6真实双盒装篮75.721sim通过，0.8rad/s空手neutral RRT的手指0.223796>0.2
触发原保护，未关盖/旋转/取回，87.714sim非完整。17718真实帧/0重复/正常SIGINT/
全解码及正常场景退休PASS，413封存/272密封/前后10262历史文件/10备份/785源码保持。
新[r255候选](runtime_evidence/centrifuge_48box_empty_return_gain_20261014_r255/FULL_CYCLE_PLAN.md)
仅故障空手neutral限至0.4/原2mrad，其余速度/模型/3、10、30sim与物理门不变；
779局部/7.20s隔离构建/六库恢复通过，完整前检继续，尚未seal/claim/Gazebo。
接续重查当前进程/日志，不复用旧claim或因观察超时重复open-x；下方旧活动
快照均历史。完整180未达，r253小幅校准不能外推到本次大幅RRT或完整加载。

最新覆盖：[r253部件报告](runtime_evidence/centrifuge_48box_gain_reverse_calibration_20261013_r253/REPORT.md)
PASS，gain0.6/500Hz的小幅0.4rad/s原生误差0.533333mrad<原2mrad，手指峰0.128312<0.2。
只COMPONENT_PASS/full_cycle=false，17.365sim不是完整180，未抓盒/按按钮。原电影2667
真实/0重复/正常SIGINT/全解码及正分支俯视录入后退休通过，403封存/remaining0；
275密封/前后保护保持。下文r253前检和活动均历史；下一完整加载必须新身份。

r252正肘小幅离线密查碰左右盒，未seal/claim/Gazebo；323失败封存/前后保护PASS。
负肘候选两组61点几何通过不是物理；[r253新候选](runtime_evidence/centrifuge_48box_gain_reverse_calibration_20261013_r253/FULL_CYCLE_PLAN.md)
只gain.6空臂负肘校准，原2mrad等门保持，771局部/隔离构建及before保护含r252323
通过，原20648前检中，尚未seal/claim/Gazebo，不观察超时复跑。下文r252前检活动已覆盖。

最新[r251终态](runtime_evidence/centrifuge_48box_retrieval_medium_20261013_r251/REPORT.md)FAIL：
.4空手取回接近右肩roll2.413634mrad超原2mrad，未取回抓取。装篮76.354sim/稳转
30.5325sim通过，首至失败158.1055sim非完整；26918真实电影/0重复/正常SIGINT/全解码
及场景退休PASS，417封存/前后保护保持，新收尾器仅失败分支被实际证明。
[r252候选](runtime_evidence/centrifuge_48box_gain_calibration_20261013_r252/FULL_CYCLE_PLAN.md)
只私有XML位置gain.2→.6独立空臂校准，严格其余字段不变；不抓盒/不按按钮，
COMPONENT_PASS/full_cycle=false不等于180。20针对/771局部/隔离构建通过，24441
前检中，尚未seal/claim/Gazebo；读同作业/日志，不观察超时重启。下文r251活动已覆盖。

当前活动为[r251检查点](runtime_evidence/centrifuge_48box_retrieval_medium_20261013_r251/run/live_checkpoint1.json)：
761局部/64双执行器/46配置/122旧装载/6CTest及隔离构建/六库恢复、255依赖与before
保护含r250443通过，原Nav111/2V12缺源保持。269成员一次seal/0444，fresh六控制器
active/500Hz/72帧短测；同原电影首秒已看，抽帧ss格式失败原记录保留，未重启录像。
九角色PID/ctime/collector真实父关系核验，首sim20.435/启动窗21.0445/腿姿态27.463
PASS，误差.0004586842rad<原.001。sim44.1345只装载前缀，不是完整180或录像收尾
成功。继续查同collector/进度/实际进程，不复用claim/观察超时重启；活动中不改密封
源、不移动六库构建/另开Gazebo。下文r251前检中描述已经被此活动检查点覆盖。

最新[r250终态](runtime_evidence/centrifuge_48box_table_clearance_20261013_r250/REPORT.md)：
完整物理28/28及独立双持盒PASS，首8.355→PASS245.4155=237.0605sim，稳转30.5595sim，
双返回/末端10sim保持通过；180速度FAIL。原电影38597真实/0重复/完整解码0，17阶段
主体可见，但终态相机ACK超时使监督先退休场景、未SIGINT录像，**录像交付FAIL**。
OS0/0/0/1/正常场景退休剩余0，443封存/264密封未变/前后保护PASS。
[r251候选](runtime_evidence/centrifuge_48box_retrieval_medium_20261013_r251/FULL_CYCLE_PLAN.md)
只empty approach .4/.002及3次终态相机重试/强制先收录像再退休；模型.2m/s、路径与
物理门/3、10、30sim保持，28针对测试通过，隔离前检原39242作业中，尚未seal/claim/
Gazebo。读retry1前检日志，不观察超时重启；下面r250活动描述已被终态覆盖。

最新活动为[r250真实检查点](runtime_evidence/centrifuge_48box_table_clearance_20261013_r250/run/live_checkpoint1.json)：
752局部/65双执行器/46配置/122旧装载/6CTest及隔离构建/恢复六库、253依赖/before保护
通过；原Nav111/2V12缺源保持。264成员一次seal，fresh六控制器active/500Hz/72帧短测，
MP4首秒已看，远手自然遮挡。首sim8.355/启动窗9.046/腿姿态15.584 PASS，九角色
实际PID/ctime核验，四角色真实父为独立collector3286095。本次重查sim51.7385
paired_load_static_grip_hold仍活，只是前缀，不是完整180。接续读取result/trace_progress/
collector及活进程；不观察超时重启，不改密封源/活动中构建或另开Gazebo。
下文r250准备描述及r249活动快照已被本段覆盖，完整任务仍未达三分钟。

最新[r249终态](runtime_evidence/centrifuge_48box_retrieval_approach_20261013_r249/REPORT.md)FAIL：
取回接近/双夹持抬离/10sim保持/122点制动实际进入，但桌面下降误用篮口高度条件。
212.145sim只是失败前缀，未完整180。quiet222.3485/pause222.3995/正常SIGINT退休剩余0，
395封存/保护PASS，电影36091真实+1重复，不是零重复证书。
r250仅桌面两阶段验证原collision双篮投影间距≥原3mm且无篮触，其他竖直3mm不变；
独立审计另式重算/匹配原contact流，19集成/752局部/隔离构建通过。
[r250新前检](runtime_evidence/centrifuge_48box_table_clearance_20261013_r250/preparation_pipeline.log)
原18691作业中，尚未seal/claim/Gazebo；不观察超时重启/复跑open-x。下文活动均为历史。

最新r249前检733局部/隔离构建/保护通过，255成员一次seal，fresh六控制器active/500Hz，
实际首帧已看，原四角色由独立collector session连续执行。
[r249活动检查点](runtime_evidence/centrifuge_48box_retrieval_approach_20261013_r249/run/live_checkpoint1.json)
九角色PID/ctime和255成员不变，collector真实父关系确认；首sim9.6225/启动窗10.0515/
原腿姿态16.5945 PASS，最大误差.0002434542rad<原.001。sim31.4415仅装载前缀，非完整。
empty retrieval approach .13/.002和paired制动实际仍待验证，原Nav111/2缺源保持。
接续读result/trace_progress/collector.log和活进程，不以旧CLI观察结束当任务终止、
不复用claim/观察超时重启；活动中不改密封源/移动六库构建/开第二Gazebo。完整180未达。

最新[r248终态](runtime_evidence/centrifuge_48box_paired_setdown_20261013_r248/REPORT.md)FAIL：
装载76.486sim/真实机器稳转30.5sim/停稳开盖通过，取回空手接近夹爪速度-.2011199m/s
超原.2；未返回抓取/paired制动/放回，158.732sim仅失败前缀。quiet169.971/pause170.0735/
正常SIGINT退休剩余0，388封存/保护PASS。录像26165真实+6重复，不是零重复证书。
r249仅收紧该接近段.13/.002、不改模型限速，19测试通过，
[新前检](runtime_evidence/centrifuge_48box_retrieval_approach_20261013_r249/preparation_pipeline.log)
流水线中，尚未seal/claim/Gazebo；检查原4366作业，不因观察超时重启/重复open-x。
旧observer143不是角色当时已停止，缺失OS退出码不伪造。下文r248活动均已覆盖。

最新r248前检730局部/隔离构建/保护通过，254成员一次seal，fresh六控制器active/500Hz，
实际电影前帧已看，四角色完整流程连续执行。
[r248活动检查点](runtime_evidence/centrifuge_48box_paired_setdown_20261013_r248/run/live_checkpoint1.json)
八角色实际PID/ctime和254成员不变；首sim10.5185，启动窗11.166/原腿姿态17.669 PASS，
最大误差.0002645746rad<原.001。sim34.6365只是装载前缀，不是完整或180证书。
新paired122点制动仍待本run实际放下；原Nav111/2缺源保持。首次header短观察FAIL
保留，同活录像重读成功，未重启。先查result/trace_progress及实际进程，不复用claim/
观察超时重启，活动中不改密封源/移动六库构建/开第二Gazebo。下面准备描述已被覆盖。

最新[r247终态](runtime_evidence/centrifuge_48box_startup_window_20261013_r247/REPORT.md)为FAIL：
双装篮76.034sim/真实空手撤回/关盖/稳转30.5855sim/停稳开盖/双取出至桌面上空通过，
旧121点放下接口拒绝新paired122点，未两盒桌面释放。192.57sim只是失败前缀，非完整。
双holder保持/quiet200.424/pause200.507/正常SIGINT退休剩余0，393封存/前后保护PASS。
r248新严格paired制动保留所有点与末10mm/.03，17测试/730局部/隔离构建及旧计划FK
分段验证通过，[前检流水线](runtime_evidence/centrifuge_48box_paired_setdown_20261013_r248/preparation_pipeline.log)
仍须核对作业，尚未seal/claim/Gazebo。不复用任何旧claim或因观察超时重启。
下面r247活动描述均被终态覆盖；完整≤180未达，原完整基线仍r227461.299sim。

最新r247前检713局部/隔离构建/保护通过，249成员一次seal，fresh六控制器active/500Hz，
实际录像首帧已看，四角色完整流程连续执行。
[r247当前检查点](runtime_evidence/centrifuge_48box_startup_window_20261013_r247/run/live_checkpoint1.json)
八角色PID/ctime存活及249成员不变，首sim7.2045/启动窗7.4885/原腿姿态13.862 PASS，
最大误差.0001851257rad<原.001。sim34.1355只是装载下降前缀，不是完整180证书。
暂时MP4 header观察失败已保留，同活录像重读成功，未重启。原Nav111/2缺源保持。
接续读result/trace_progress和实际进程，不复用claim、不因观察超时重启；活动中不得
改密封源/移动六库构建/开第二Gazebo。下面r247准备与r246活动描述均为覆盖后的历史。

最新[r246终态](runtime_evidence/centrifuge_48box_empty_retreat_20261013_r246/REPORT.md)为FAIL，
原静态腿姿态action超过.001rad门，未任务轨迹/装篮/机器/取回，不称完整15.705sim。
6195真实帧/正常SIGINT/解码PASS，quiet65.5825/pause65.725/场景退休剩余0，371封存/
前后保护PASS。r247只增加测得sim≤25早接管门，原目标/容差/增益/物理证书不放宽；
15测试通过，[新前检](runtime_evidence/centrifuge_48box_startup_window_20261013_r247/preparation_pipeline.log)
流水线中，尚未seal/claim/Gazebo。下面r246活动描述均已被终态覆盖，不能照抄PID。
先查原作业/最新输出，不因观察超时重启或重复open-x；完整基线r227461.299未达180。

最新r246前检698局部/隔离构建/保护通过，250成员一次seal，fresh六控制器active/500Hz，
录像门及实际MP4前帧已看；四角色已经连续执行完整双盒任务，尚无完整/180证书。
[r246当前活动检查点](runtime_evidence/centrifuge_48box_empty_retreat_20261013_r246/run/live_checkpoint1.json)
核验八角色实际PID/ctime和250成员不变；首非空sim49.1335已直接读正确gzip，检查点1
的null是观察器glob未匹配，不是没有sim。原Nav111/2缺源仍保留。先重读result/
trace_progress与活进程，不复用claim、不因观察超时重启；活动中不得改密封源/移动
六库构建/开第二Gazebo。下面r246未seal/claim/场景的准备描述均为历史覆盖。

最新[r245终态](runtime_evidence/centrifuge_48box_clock_budget_20261013_r245/REPORT.md)整体FAIL，
双真实夹持/各10sim保持/自然落入两篮及各3sim落稳实际完成，上料76.1125sim；空手
RRT撤回7740查询未找到路径，未关盖/转/取回，不能把上料当完整≤180。
录像17751真实帧/正常SIGINT解码通过；quiet91.598/场景正常退休剩余0，368封存保护。
r246当前run指南结构化空手撤回3062密查通过，已接入新隔离构建/回归/保护/前检，
未seal/claim/新Gazebo；先查preparation_pipeline.log及实际作业，不复跑open-x，不因
观察超时重启。旧固定腕四方案原IK拒绝不执行，下面r245活动状态均为已覆盖历史。

最新r245已封存241成员/真实fresh全流程连续执行，686测试、隔离构建/恢复/保护/几何/
录像前门通过；实际六控制器active/500Hz，241成员未变。
[当前现场检查点](runtime_evidence/centrifuge_48box_clock_budget_20261013_r245/run/live_checkpoint1.json)
不是完整/180证书；先读当前result/trace_progress与PID/ctime，不复用claim、不因观察
超时重启，活动中不改密封源/移动六库构建/开第二Gazebo。运动、10sim保持及定时停键
等待共用有限实际clock进展监视，全部3/10/30sim/物理门/速度保持。下面准备中是历史。

最新[r244终态](runtime_evidence/centrifuge_48box_camera_retry_20261013_r244/REPORT.md)为FAIL，
双实际夹持前缀独立通过、录像5977帧/正常SIGINT解码通过，但试抬墙钟预算漏起止余量/
倍率不足，未完整试抬/装篮/稳转/取回。已保持/quiet46.518/场景正常SIGINT退休剩余0。
r245有限墙钟预算/实际clock停滞守卫已实现，原指令速度和全部物理门不变；前检流水线
在[重试日志](runtime_evidence/centrifuge_48box_clock_budget_20261013_r245/preparation_pipeline_retry1.log)，
旧preparation_pipeline.log是输出身份初始错误拒绝，旧成果未变；未seal/claim/新Gazebo。
先检查真实作业及最新输出，不复跑open-x，不把12初测/实际前缀复放当完整/180证书。
下文r244活动快照及更早准备/失败状态均为历史，完整基线仍r227461.299。

最新r244原流水线已退出0、666测试及隔离构建/保护/几何/录像前门PASS；235成员一次seal，
真实fresh六控制器active/500Hz，四角色与录像已连续执行。
[当前活动检查点](runtime_evidence/centrifuge_48box_camera_retry_20261013_r244/run/live_checkpoint1.json)
非完整/180证书，须重读result/trace_progress及PID/ctime；不复用claim、不因观察超时重启。
活动中不得修改密封源、移动六库构建或开第二Gazebo。下面r243及r244准备中是历史覆盖。

最新[r243终态](runtime_evidence/centrifuge_48box_empty_contact_fix_20261013_r243/REPORT.md)为FAIL，
不是旧RUNNING_CHECKPOINT：空手前缀右盒位移峰5um，camera单次超时后录像退出/守卫SIGINT，
ROS绑定异常停止，未装篮/转/取回。紧急连续静止门未确认；独立pause/正常SIGINT退休剩余0。
3972帧解码通过但录像总体FAIL，183613行无损日志/240密封成员/前后保护PASS，355静态0444。
r244只对同一camera请求最多3次有界重试，6初测通过；隔离构建/回归/保护/几何/录像前检
流水线运行中，未seal/claim/新Gazebo。先重查实际作业/输出，不能复跑已有open-x输出。
实际全任务仍须独立物理/双参考/计时/录像及保护全部通过，完整基线r227461.299未达180。
本轮无清理/备份/真机/腿部导航。下文r236/r240与r243活动快照均为已覆盖历史。

[r236终态](runtime_evidence/centrifuge_48box_alignment_cap_20261013_r236/REPORT.md)为FAIL，
双落篮/机器稳转30.0505sim已过，首次取回缺配对故障、两盒未返桌，已正常SIGINT退休。
原27门16/27为真，不是完整三分钟；禁止复用旧claim或把RUNNING_CHECKPOINT当live。
[r240报告](runtime_evidence/centrifuge_48box_coupled_path_20261013_r240/REPORT.md)三条同步
带盒全段FCL与空臂路径通过，22测试；实验初始robot x=.12/device x=.52只离线，未实跑。
名义约14.21sim带盒单程不是真实耗时；停转取回须按actual姿态再规划，不直接倒放。
r238正确23mm高度为1.07525，保留初版1.03025错误输出；r239容量假设未证实际根因。
下一新完整身份须双独立物理夹持、反馈配对诊断、重生成按键、持续广角录像与所有fresh门。
原始模型/孵育箱与10备份/785源码不改；此处“无live”是r240历史，完整基线仍为r227。

## 1. 查看最新完整离心机结果

最新完整物理/录像修复回归是r227461.299sim秒，未达180；r228是500Hz空臂组件，
0.8rad/s实际跟踪0.0064rad，不是带载或整套三分钟验收。两者已正常SIGINT退休。
只读查看：

```bash
cat runtime_evidence/centrifuge_48box_dual_sync_20261012_r227/REPORT.md
cat runtime_evidence/centrifuge_48box_servo_rate_20261012_r228/REPORT.md
```

下面保留r225/r226前一基线与录像纠正，旧成果不改。新提速r229准备中，不直接执行旧身份。

```bash
cd /home/liu/h1_moveit/h1_2_ros2_ws_claude_2026-07-30/h1_2_ros2_ws
root=runtime_evidence/centrifuge_48box_horizontal_rate_20261012_r225
cat "$root/REPORT.md"
cat "$root/run/result.json" "$root/cycle_audit.json" "$root/trace_audit.json" "$root/video_audit.json"
cat "$root/complete_speed_audit.json" "$root/protected_after.json" "$root/source_1009_after.json"
cat "$root/actual_profile_context_audit.json" "$root/change_scope_audit.json"
(cd "$root" && sha256sum -c SHA256SUMS)
review=runtime_evidence/centrifuge_48box_three_minute_budget_20261012_r226
cat "$review/REPORT.md" "$review/video_review.json" "$review/timing_budget.json"
```

动作27/27、完整速度、无损日志/保护PASS。完整465.9815sim秒，比r219省5.3325秒/1.13%，仍未达三分钟。
录像只解码PASS，内容复核FAIL：约原速1分半后所查阶段设备被裁出画面。原video_audit不包含主体可见性，不把其PASS当完整展示验收。
合同采用已授权actual携带5°/2mm及仅取回有限制动瞬时0.1rpm；实测4.222334°/1.449929mm/0.021285rpm，旧2°/全程0.02rpm本次未通过。
仅四个水平自由搬运段×1.15，原接触/垂直/抓放/按键/20ms及3/10/30秒不变；取回合爪观察上下文修正不改物理数值。
单次速度PASS，携带角较r219更大；r219471.314秒保留为较低角偏差回退基线。失败25%/15ms方案不推广。
原完整录像full_cycle.mp4和10倍播放预览preview_10x.mp4在该目录run下；快放不是动作提速。
新r226仅耗时审计/结构计划，32分析器测试通过，无新物理run；173秒设计预算不是实测。

## 2. 区分单独上料与箱子成果

- [r159报告](runtime_evidence/centrifuge_48box_global_speed_20261009_r159/REPORT.md)：双盒上料172.500sim秒，不包含关盖旋转取回。
- [v24/r103摘要](runtime_assets/candidate3_roundtrip_831_r103/RUN_SUMMARY.md)：雷达四墙地图内孵育箱固定双循环169.329060321秒。
- [826 A–L报告](runtime_evidence/826_apriltag_execution_20260827/REPORT.md)：独立动态规划线。
- [D1b报告](runtime_evidence/826_post_nav_apriltag_alignment_20260914/D1B_FIXED_OFFSET_OPEN_REPORT.md)：导航后固定Tag外参开门仅离线plan-only。

## 3. 备份只读校验

```bash
cd /home/liu/h1_moveit/backups
sha256sum -c 920.tar.zst.sha256
sha256sum -c 1008.tar.zst.sha256
sha256sum -c 1009.tar.zst.sha256
```

920是永久箱子版本；1009是截至r159阶段的最新完整快照，不含r214及之后r219/r225。本轮不创建新备份。
恢复/显示切换合同见[SCENE_VERSIONS.md](SCENE_VERSIONS.md)，禁止覆盖当前工作区。
只切换静态场景不意味着可以执行旧固定轨迹。

## 4. 当前live与新的动作

r227完整任务与r228组件录像已正常SIGINT结束；r217保留运行是历史。每次接手仍需重新只读检查进程/窗口/控制器/夹持/support，不能照抄历史PID。

- 离心机提速：HANDOFF_PLAN第29节，r225最新单次速度基线、r219稳定回退；三分钟需双持物/双臂时域FCL、短路径与完整观察重叠，不盲目加倍率。
- 原训练/腿部导航仍缺V12 checkpoint/env/agent，原回归111通过/2失败原样保留；官方源7项冻结身份已匹配。用户已停止原件恢复，不主动开展磁盘恢复或要求设备，不能冒充恢复训练状态。
- 固定骨盆完整手部任务r225独立核验180资源/657相关Python/6项CTest/178密封成员及实际27/27 PASS；原私有训练源不用于此范围，不能继续据此阻塞该已审核任务。任何旧身份均不能续跑或扩大scope。
- r218录像尺寸/忽略SIGINT故障已保留；新的执行器必须证明SIGINT实际送达，录像须尺寸变化测试和新路径。固定指定窗口区域不是放大后的整窗，快放不是动作提速。
- r225内容复核失败：新录像须验证四角/主体、持续检查窗口和动作可见性，关键动作分段抽帧；只解码或末态截图不能证明过程合格。
- 孵育箱双循环：分支G，从v24 builder链派生新身份；只原样执行新密封commands。
- 导航/视觉适配：单独成果线，不能把固定骨盆手部操作当作Nav2联动。
- 新运行须普通mkdir独占、隔离构建测试/保护、一次seal、fresh健康/视频门；不得复用旧claim。
- 失败stop-and-hold，禁止reset、detach或异常松夹；真实机器人未授权。

## 5. 本轮文档和清理证据

[整理报告](runtime_evidence/documentation_process_cleanup_20261010_r1/REPORT.md)记录旧原文、移动清单及前后校验。
旧Quickstart全部原文保存在documentation_before.json；其中旧运行命令仅为历史，不能直接复制重跑。
只把无引用且未受保护的过程日志/可再生缓存移入回收站，不永久清空。

## 6. 2026-10-15 最新少翻臂录像（仅查看，不复用运行）

- 新r285完整物理28门/双持物/实际转动路径核验PASS，装盒轴向转动少35.43%、取盒少4.33%。
- 演示剪辑：`/home/liu/离心机全流程_少翻臂优化_前方视角_20261015.mp4`，约2分57秒，非实际执行耗时。
- [原始证据报告](runtime_evidence/centrifuge_48box_minimum_turn_full_20261015_r285/REPORT.md)；原片在该身份`run/full_cycle.mp4`。
- 本轮193.979sim/2537.422484wall秒，未过180sim速度门；旧r283179.129sim和r284成果仍保留，现实三分钟未达。
- 场景已正常退出；后续执行必须派生未用新身份并完整验证，不能重用r285或旧claim。无清理/新备份/真机许可。
