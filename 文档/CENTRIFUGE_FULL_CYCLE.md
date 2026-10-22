# 离心机完整双盒流程：接口、成果与边界

更新：2026-10-15 CST。累积权威仍为`AGENTS.md`最末日期小节。

## 最新要求：现实三分钟，尚未完成

用户要求实际仿真1:1，再运行原179.129sim秒流程。配置原本已target1.0，
七个保持模型的独占静止性能对照最快约0.135倍实时，未采用慢配置，也未再跑
完整任务。实际1:1和完整wall≤180均未达；[诊断报告](runtime_evidence/centrifuge_48box_wallclock_perf_20261015_r3/REPORT.md)
与[原始量化表](runtime_evidence/centrifuge_48box_wallclock_perf_20261015_r3/PERFORMANCE_SUMMARY.json)保留。
新验收必须用monotonic墙钟计入在线规划、等待及最终返桌/检查；旋转同时至少
30真实秒和30仿真秒。计时单元已测试，但尚未接入已资格化的新完整流程。
下面成功结论只代表此前仿真时间目标，不外推现实三分钟；全部旧证据不改。

## 已验收的仿真时间成果

完整新身份：`runtime_evidence/centrifuge_48box_optional_micro_full_20261015_r283/`。
同身份桌面两个85%48box实际双指抓取→分别自然落篮→机器人指尖关盖→30rpm稳转
**30.258仿真秒**→指尖停转、实际停稳→开盖120°→按实际姿态取回两个盒子→
正常放回两个不同桌槽。含完整装载/取回各10秒保持、各3秒落篮/返桌和末态10秒。
首有限非空sim10.1915→完整实际PASS189.3205，**179.1290仿真秒**，独立28/28、
双持物、无损trace、完整录像/20相位内容和保护全部PASS。

这是单次fresh、固定骨盆的Gazebo手部成果，不是现实三分钟、重复可靠性认证、
自由站立、RL步态、Nav2到站后操作或真机许可。真实wall约38分45秒；原连续录像
包括启动/末态取景约39分33秒。未用影片快放制造动作提速。

## 只读入口

- [正式报告](runtime_evidence/centrifuge_48box_optional_micro_full_20261015_r283/REPORT.md)
- [独立速度核验](runtime_evidence/centrifuge_48box_optional_micro_full_20261015_r283/complete_speed_audit.json)
- [28项物理核验](runtime_evidence/centrifuge_48box_optional_micro_full_20261015_r283/cycle_audit.json)
- [独立实际双持物核验](runtime_evidence/centrifuge_48box_optional_micro_full_20261015_r283/independent_dual_grip.json)
- [终态、内容与保护总核验](runtime_evidence/centrifuge_48box_optional_micro_full_20261015_r283/run/terminal_integrity_review.json)
- [完整原始录像](runtime_evidence/centrifuge_48box_optional_micro_full_20261015_r283/run/full_cycle.mp4)
- [20实际阶段拼图](runtime_evidence/centrifuge_48box_optional_micro_full_20261015_r283/run/actual_phase_contact_sheet.png)
- [全目录只读哈希清单](runtime_evidence/centrifuge_48box_optional_micro_full_20261015_r283/SHA256SUMS)

REPORT SHA-256：`f40a4d741ed3040ff969626be3a416996f76d0113606337ab0b0d434e1709404`。
原九PID/ctime和四运行角色均已正常退休/exit0；接手仍重新只读核查live。
原片末俯视已真实录入并见两桌槽盒/空双篮；远侧手/盒有部分自然遮挡，关盖时
转子不可见，其时长/留篮由完整物理日志认证。没有以末态截图替代缺失过程。

## 运行接口与验收合同

本身份运行链为密封`inputs/lab_dual.launch.py`、fresh健康/控制器与广角原片门、
`execute_cycle.py`连续调度`EndpointOverlapCycle`，并行guard/disk/supervisor。
之后独立trace→cycle→dual grip→speed审计及原片抽帧/全解码、七组保护复核。
以上是接口说明，**不是允许直接执行旧脚本的命令**。run/claim/overlay/视频路径
已经使用；后续用户授权再展示时从本链派生未用身份，普通mkdir独占、隔离构建/
回归、最终依赖与源SHA/一次seal、新fresh现场和新录像后才能运行。

- 蓝按钮由实际机器人指尖接触控制开/关盖；绿按钮实际接触启动/停止。正常关盖互锁
  保留，本次五事件开→关→启→停→开全部来自Gazebo物理接触，不直接调用功能替代。
- 用当前run actual poses和冻结双holder参考重规划取回，不反播旧轨迹、不瞬移物体，
  规划碰撞attachment不等于物理绑定。桶轴退出已通过真实lift/完整保持/双返桌。
- 原2mm、力/速度/碰撞/跟踪门及全部3/10/30秒证书保持；合同使用之前用户授权
  携带5°及仅取回制动瞬时0.1rpm。实测角.531668°/取回残速.001351936rpm，也满足
  旧2°/0.02rpm数值，但不冒称旧严格证书重新全认证。
- 本次未触发loading alignment；完整可选四逆退路与11目标微量对齐仍只单元/离线
  资格，不将其预算当实际节省，也不能跳过实际guide的任何逆退路。
- 任何失败stop-and-hold并正常SIGINT；夹持期间禁止reset、detach或异常松夹。
  最终空夹爪/两不同桌槽保持，不要求双臂neutral，也不宣称原出生槽一一回位。

## 保护与其他成果线

1320局部测试、两包隔离构建和相关回归通过；原导航111通过/2项V12缺源保持。
399密封成员/315资源/36归档依赖、七组前后保护、10备份/785正式src/10262历史
及后续链均通过，原模型/源码、孵育箱动作和所有重要视频/报告/rosbag/builder链不改。
包内原README/PROVENANCE与日常入口仍保持正式原字节，本文件描述独立验收身份。
920永久箱子版本和旧完整r279184.358sim秒等成果保留，不从当前工作区覆盖恢复。
本轮无清理、回收站操作、新备份、腿部导航、真机连接或运行。

## 2026-10-15：少翻臂版本r285（独立保留，不替换速度版本）

用户随后要求减少不必要翻臂并重新录像。新r285保持原模型/物理/真实夹持与所有观察门，
只比较原对称接触框架的多个IK分支、密集无碰撞空手路径和连续携物IK种子，旧规划可回退。
已完整实际装篮、关盖、30sim秒稳转、停稳开盖、取回分桌释放；物理28门/双持物/新实际路径审计PASS。
装盒抓取肩/腕轴向累计转动减少35.43%，取盒减少4.33%；装盒左腕roll约-132°→+4.11°。
取盒保留原必要抓取方向，未承诺消除所有翻臂或全局最优。native滑移峰1.275mm、角0.814°。

本轮多候选规划更耗时：193.979sim秒、2537.422484wall秒，原180sim速度专项FAIL如实保留。
旧r283的179.129sim速度成功及r284前视版本不改，现实1:1/三分钟仍未达，不用视频长度替代耗时。
新演示剪辑：`/home/liu/离心机全流程_少翻臂优化_前方视角_20261015.mp4`，177.467秒；原片2594.75秒保留。
报告与机器核验：`runtime_evidence/centrifuge_48box_minimum_turn_full_20261015_r285/REPORT.md`。
632结果只读封存、436输入/329资源/36外依赖及七组前后保护通过；10备份/920箱子版本保持。
当前场景已正常SIGINT退休，不得原地重跑r285/复用claim；新运行仍须新身份及原fresh/完整验证。
