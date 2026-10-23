# r357：DART查询缓存启动试验失败，未进入任务

新 lookup 组件离线两轮5400步/43200向量逐位一致，组件约1.0880/1.0909倍；2341候选测试两轮、46配置/122装盒/6CTest通过。Nav111/2旧V12缺源保留。
源698项SHA 2ef78c0eada9cabdcd2aeac600035359a99d455a912bc6879583168d5d2cb57a。
实际新Gazebo加载库与lookup计数验证通过，六原控制器随后active。

## 失败链
原关节relay报 JOINT_SOURCE_STEP:70980000000:70981000000:1000000，即sim70.980到70.981之间缺少70.9805这一帧。原0.5ms完整性门正确拒绝，没有插值、放宽或忽略。
这引起launch shutdown，腿初始化等待不到完整/joint_states，25.039038085wall超时、samples=[]。
腿初始化无goal发出；双盒/盖/转子完整任务执行器未启动，尚无execution.claim、捕获锚点或完整任务录像，不能计算完整流程速度。
不能仅凭此日志确定源生产、Gazebo transport、bridge或DDS哪层丢帧，也不能直接归因于新DART查询。

## 收尾与边界
launch先SIGINT，Gazebo包装进程超5秒后由launch自身升级SIGTERM；不声称所有进程仅SIGINT退出。
独立retire_scene再次按原身份正常SIGINT退休残留server/gui，remaining=[]；无reset/detach/松夹。
动作源/物理资产与r356保持；r356仍最新完整成功。启动失败不推广为性能成果。
下一次只能全新身份复测原完整性门，不复用r357 run，也不修改失败证据。
