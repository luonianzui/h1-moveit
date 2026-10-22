# r352–r353独立复核

r352动作前遥测源缺帧失败；r353新身份完整PASS。完整事实与边界见[r353报告](../centrifuge_48box_sequential_full_20261022_r353/REPORT.md)。

- 新完整记录221.4645sim / 494.301397270wall；比r347缩短15.500783%，并非现实三分钟。
- 分开抓姿态与最多单holder保留，实体按键操作，稳转30.1345sim，双盒返桌，28门通过。
- 2293候选测试两轮与25专项通过；导航2项旧V12缺失失败不隐藏。
- 原速532.666667秒视频完整解码；实际所看7帧详见image_review.json，未逐帧观看。
- 离线控制器审计原raw读取失败保留；revision2仅gzip/root定位修正，8项测试通过，原执行源未修改。
- 九组前后保护相等，r347及全部旧成功/失败/正式源模型/孵育箱/备份不改；未清理或创建备份。
- 场景与录像正常SIGINT退休，owned剩余为空；live为当时快照，下次重查。
- final_review.json记录新核心manifest、独立验收与视频哈希；本目录SHA256SUMS封存该复核。
- 根六MD原文在documentation_before.json；AGENTS只追加，原930174字节前缀SHA
  7d875248f07bbd80088ae5a877b437950cfe41e09923cae0dd056fd224514a39保持。
- [性能分析](PERFORMANCE.md)说明DART仍为瓶颈；[轮次汇总](ROUND_SUMMARY.md)提供接续入口。
