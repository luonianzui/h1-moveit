# DART 单次约束求解查询优化（2026-10-23）

仅供新身份 r357 的私有 Gazebo DART 6.13.2 注入库使用，不覆盖系统库、r356 或任何旧成果。

## 改动

在 r356 的求解作用域中，为 BodyNode owner/reactivity 查询增加 256 槽 thread_local 直接寻址缓存。命中时复用本次求解的记录；地址冲突回退原 unordered_map；进出求解均清空，异常退出同样清空。保留原共享所有权寿命、即时 impulse flag/velocity、惯量 dirty check、数值计算顺序、接触/摩擦/质量/步长、LCP fallback。模式开关只能在求解之外改变。

## 数值和范围验证

- `lookup.log`、`lookup_repeat.log`：各 5400 world steps、43200 个位置/速度向量逐位一致、5088 contact steps；包含多根、自碰撞、关节限制及跨步 mass/COM/damping/timestep/mobile/LOCKED/PASSIVE 变化。相对同一库关闭新 lookup 分支，组件速度比 1.088001663 / 1.090934079。不是完整任务提速证明，也不是两个独立库的受控整机对照。
- `lookup_scope.log`：1296 checks、31 次作用域内模式修改拒绝、157 次惯量 dirty refresh；回调异常注入后 retained=lookup_retained=0。
- `solver_revision3.log`：105 矩阵、1200 world steps；`queries_revision3.log` 和两份 `inertia*revision3.log` 保留原层级验证；`dirty_revision3.log` 1266 checks/30 rejections/152 refreshes。
- 最初命令漏设 `H1_DART_CACHE_MODE=cached`，`queries_revision2.log` 在查询覆盖门拒绝；该未激活实验保留，不当作数值失败或成功。正确设置后独立 revision3 全套通过，未改动算法绕过测试。
- 编译 `-O3 -fno-fast-math -ffp-contract=off`；独立 install；六项 Conda 库构建前后逐字节一致。正式全流程、实际计数、握持与录像核验另见 r357/review_r4。

运行身份、二进制与源文件绑定由 r357 的 `dart_cache_dependencies.json`、`identity.json` 和该组件 `SHA256SUMS` 记录。此目录不能覆盖重跑；后续改动另建身份。
