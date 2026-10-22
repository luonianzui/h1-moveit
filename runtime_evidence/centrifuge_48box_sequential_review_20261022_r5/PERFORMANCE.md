# r347：间隔与整体耗时复核

- 实测11个空手自由阶段0.30→0.16sim，减少启动缓冲1.54sim；终点观察重叠最多另省0.55sim。
- r346全局缩短未完整成功，右试抬转子-0.171036512rpm触发原0.1门；不拿其470.698秒失败前缀算提速。
- r347完整584.977490836wall/240.219sim，相对r345完整586.260130234wall仅少1.282639398秒。
- 不同实际接触与规划快照，本次额外一次有正桌面支撑的校正；不是受控A/B，不能宣称显著整体收益。
- 一个planner、22请求/4对连接段，规划102.710188627wall；执行无AI逐动作等待。
- 世界stats区间RTF0.4079013898；主线程均值0.937383核，53572实际PreUpdate线程CPU样本。
- 精确映射/二进制SHA/导出符号长度核验：Physics::Update累计88.3465%、BoxedLcp53.5018%。
- 比例包含子调用，不能相加、不能当墙钟分摊；采样有开销，不声称物理1倍实时或现实180秒达标。
- 下一步只用全新身份测规划中的重复静态模型构建或单位脉冲响应重复取数；保留原路径、抓持和30秒稳转。

原始证据：interval_audit.json在run根目录；本review保留world_stats_summary.json、performance_conclusion.json、
native_symbol_audit.json、native_impulse_audit.json、dart_solver_audit.json和逐秒线程样本。

