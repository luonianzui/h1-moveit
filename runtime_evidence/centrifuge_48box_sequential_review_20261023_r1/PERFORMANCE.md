# r354性能结论
完整214.241sim / 484.470481546wall，比r353少9.830915724wall（1.988850%），但区间RTF从0.454443到0.434703，不能称稳定1倍或整机实时率提高。
查询缓存小世界两轮约1.149倍，仅组件结论；真实在线快照、接触数量、规划计算耗时不同，不做单因素因果归因。

44040主线程样本，主线程平均0.932516核。精确符号累计：
- Physics::Update 88.2516%；
- BoxedLcp约束组51.6780%；
- Skeleton::updateVelocityChange 19.4210%；
- ContactConstraint::applyUnitImpulse 19.2870%；
- BodyNode::getArticulatedInertia 7.4069%。

包含子调用，不能相加或直接当墙钟。1073抽样组，维数中位37/p95为175，平均组装147.266us、LCP51.694us；抽样fallback3，非完整普查。
同一规划模型1构建34复用35新场景，worker40.757846wall，低于r35352.927339wall；这也可能影响整体差异。
本次两盒已全部返桌且28门通过，流程完整性没有拿掉；全流程仍超过现实和仿真180秒。

下一低风险假设：缓存同次求解内不变的惯量/骨架查询，重点测单位脉冲传播。必须先数值对照、私有编译、新身份完整验证，不删真实接触或改变时间。原r353留作回退/对照，不重跑封存身份。
原始机器证据：performance_conclusion.json、native_symbol_audit.json、native_impulse_audit.json、dart_solver_audit.json、world_stats_summary.json、phase_performance.json。更多限制见[运行报告](../centrifuge_48box_sequential_full_20261023_r354/REPORT.md)。
