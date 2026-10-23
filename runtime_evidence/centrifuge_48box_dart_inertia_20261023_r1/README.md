# DART求解作用域内惯量访问优化（2026-10-23）

仅用于全新r355固定骨盆离心机仿真；r354及之前版本、系统DART、正式模型和动作合同保持不变。

## 实现
惯量数值本身原已由DART缓存，本轮不重复发明数值缓存或省略重算。只在单次约束组求解内复用已锁定的骨架所属关系，减少单位脉冲传播调用惯量getter时反复锁定shared/weak骨架。
两个getter每次仍读原mTreeCache[mTreeIndex].mDirty.mArticulatedInertia，需要时仍调用原updateArticulatedInertia，返回原mArtInertia或mArtInertiaImplicit。
64槽惯量所属关系直达表只缓存BodyNode指针到现有作用域BodyQuery引用，碰撞退回原map；RAII进入/退出清空，不跨步保存。
作用域外和显式关闭优化均调用原安装函数；不改惯量、矩阵算术、关节传播顺序、物理步长、接触、摩擦、质量或抓持门。

## 离线证据
- 原105矩阵/输出/回调案例和1200接触世界步逐字节一致；原查询5400步/43200向量一致。
- 相对r354（查询缓存始终开启，仅切换新惯量访问）：5400世界步/43200位置速度向量、5088接触步逐字节一致，两轮12447858次惯量查询。
- 覆盖多骨架/同骨架多根/自碰撞/关节限位，步间mobile、LOCKED/PASSIVE、质量、COM、阻尼、时间步变化。
- 求解内额外探针1266次显式/隐式惯量逐字节对照，152次真实dirty重算、30次拒绝作用域内改模式、退出引用零。
- 第一版只复用map查询，两轮约1.0130/1.0138倍；增加直达表后两轮约1.0340/1.0338倍（组件耗时下降约3.3%）。首轮并行了隔离编译，不能拿首轮对第二轮作受控因果比较；最终两轮独立交错测量相对同进程r354模式。
- revision1源码与日志保持，最终revision2源、日志和实际共享库由新run依赖清单绑定；组件测量不代替实际完整任务或1倍实时证明。
- 两次六库隔离构建均前后逐字节恢复，无全局LD_PRELOAD，仅新Gazebo子树注入。

## 来源
沿r354组件派生。官方DART v6.13.2原文：
[BodyNode.cpp](https://raw.githubusercontent.com/dartsim/dart/v6.13.2/dart/dynamics/BodyNode.cpp)、
[Skeleton.cpp](https://raw.githubusercontent.com/dartsim/dart/v6.13.2/dart/dynamics/Skeleton.cpp)。
随目录保留原上游源码及原许可证，系统安装库/头文件哈希由r355绑定；不替换系统库。
build及私有install是后续运行依赖，不能当作无用缓存删除。
