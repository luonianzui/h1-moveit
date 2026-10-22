# r291独立只读合同、参考系与完整性复核

日期：2026-10-16 CST。没有启动仿真、发布动作或修改任何实跑源、轨迹、模型、原始日志/视频。

## 来源与结论

目标 `../centrifuge_48box_elbow_full_20261016_r291/`，identity
`37ce8db3843f900936566180c5c6164b7edc4029dc83d384df70c6bdf85918bf`，460封存源保持。
实际执行/28物理门/独立双持物/无损trace/正常录像与场景退休通过。原posthoc姿态脚本exit1、旧180sim速度exit1仍保留。

原姿态脚本将0.20000000001764634与无容差0.2比较，忽略了原运行源码既有1e-8数值容差。
本复核直接使用原 `velocity_violation` 函数，不改其源码；0.2019737463与0.20000002仍判违规。
首版外部复核只纠正数值判断，随后因规划矩阵字典与holder完整快照结构错误比较而失败，日志保留。

修订复核从当前请求关节和pose重算每个规划FK变换，并从原 `dual_holder_begin` 精确恢复全部4个运行锚点。
规划专用变换不要求与首次抓持矩阵数值相等；原运行holder快照必须逐项等于首次锚点，禁止重置滑移基准。
5组实际请求全部通过，篡改规划变换、holder矩阵或payload的测试均拒绝。原2mm/5°合同保持。

9271次原2mrad密FCL、发布样本、原关节余量、篮口前臂角及独立28门/双持物都通过；
原 `command_samples` 和原 `audit_plan` 几何检查主体保持，后者只有上述错误接口断言替换为独立核对。
源字节差异与破坏性测试见21项专项测试和额外闭包派生测试；初版尾换行、锚点来源夹具错误的失败日志保留。
锚点初版误取数值重新表达的coherent参考（约2e-16差），已改为真正 `dual_holder_begin.frozen`，并重新独立FK验证。

## 证据

- `elbow_actual_audit_revision2.json`：7组实际规划、9271密FCL及20778速度样本。
- `independent_contract_review.json`：原运行谓词、原/新源码hash、全部实际输出峰17.826489N和七组保护hash。
- `reference_review_tests_revision2.log`：21通过；`closure_review_tests.log`另含原封存闭包精确派生测试。
- `finalizer_derivation.json`：对原封存freezer的每项精确替换与原/新hash，不覆盖原freezer。
- `finalize_elbow_with_external_review.py`：只允许上述有证据的审计exit1、旧速度exit1，其余原完整性门必须通过。

完整实际207.730sim/2362.143548wall秒；本复核不是新速度证书、真实机器人或新增碰撞模型覆盖资格。
原模型部分visual-only link局限保持，不通过删模型、放宽碰撞或忽略持物保护制造成功。

## 演示录像的清单修订

演示片164.633333秒是10倍速/缩短长观察的汇报剪辑，不是实际运行时长，原始2391.416667秒连续录像不改。
制作器原先把自己的stdout纳入清单，最后打印PASS后该stdout多一行；完整stdout无损移到本目录。
`verify_briefing.py`证明旧stdout前缀精确匹配原hash、其他原成员全相等、视频全解码与原片不变。
保留原清单作诊断，不覆盖它；新稳定权威为媒体目录 `SHA256SUMS_revision2` 和 `media_integrity_revision2.json`。
视频本身未重编码或修改；演示片已实际抽帧审看，无混入其他运行、无以剪辑长度冒充运行耗时。
