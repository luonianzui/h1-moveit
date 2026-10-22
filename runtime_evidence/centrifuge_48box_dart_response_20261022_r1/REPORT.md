# DART 接触速度响应短路优化：组件对照

日期：2026-10-22。独立身份 centrifuge_48box_dart_response_20261022_r1。

## 范围

仅为 Gazebo 新候选 r348 提供进程级私有预加载库，不安装或覆盖系统 DART。
保留原 DART 6.13.2 ContactConstraint::getVelocityChange 的算术、矩阵顺序、
CFM 和滑动项，仅交换两个只读布尔判断的短路顺序：非 reactive 体先退出，
避免不需要的 skeleton/shared_ptr 获取。原接触、约束数、摩擦、步长均不改。
官方来源：[DART 6.13.2 ContactConstraint.cpp](https://raw.githubusercontent.com/dartsim/dart/v6.13.2/dart/constraint/ContactConstraint.cpp)。
完整上游源和许可证保留于 UpstreamContactConstraint.cpp。
上游 SHA-256：f66bb3dbf267709582d6eab84f48a5a37d85f218d820257704f8d586786fd32b。

## 独立验证

- 原安装库符号与候选符号地址不同，baseline 使用 RTLD_NEXT 调用原实现。
- 两个相同三盒/地面世界逐步交替执行；每轮 2400 步、2400 接触步，
  全部位置和速度逐字节相同。
- 无摩擦/有摩擦、1/3 维、各单位脉冲和有无 CFM 共 96 响应对照逐字节相同。
- 六次独立进程重复均 PASS，组件加速比分别
  1.0707132993、1.0467926089、1.0592701148、1.0726053402、
  1.0660300993、1.0561177897。计数器证明真实调用候选。
- 六 Conda 库隔离构建前后核验一致。测试程序不是机器人全任务资格。
- 循环外展示命令曾因 tail 多文件参数语法退出 1；六个测试程序和日志本身均成功，
  原输出保留，不以展示命令的返回值替代逐项测试。

## 限制与交接

这是小场景数值一致性和微基准，不证明任意约束状态全部相同，也不证明整机提速、
稳定实时率或现实三分钟。完整机器人运行结果另见 r348 和 review_r6。
新模型资源缓存/提前规划同时存在于 r348，因此整机比较不是单变量因果实验。
库、构建输出和原始对照日志均保留；不得清理仍被引用的 build/安装 symlink。
