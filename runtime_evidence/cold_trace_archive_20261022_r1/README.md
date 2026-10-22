# 五份冷日志的无损归档与读取

本目录只处理r144/r147/r148/r149/r150的历史 `run/loading_trace.jsonl`；
精确路径、原SHA、字节数、权限、原manifest在inventory.json。
不是修改/删除模型、控制脚本、轨迹、视频、rosbag或备份；不影响已有31份归档登记表。

## 开发和仿真入口不变

继续使用工作区原来的 `tools/trace_archive/run -- 原命令`。
原tools/trace_archive代码、registry、31个旧gzip及36项运行依赖不改。
迁移前后以**原入口和原保护代码**核验当前九组保护，记录cold_target_reads为空；
再运行r347、r351候选回归和原配置/装盒测试。结果在本目录，不重写旧审计。

冷日志并非当前模型/控制器输入。不要将新归档添加进旧31项registry，
不要修改旧sealed脚本、旧身份SHA或新建全局sitecustomize。
本轮没有启动Gazebo、执行机器人动作或改变运动参数。

## 校验历史日志及旧清单

在工作区运行：

```bash
audit="runtime_evidence/cold_trace_archive_20261022_r1"
tools/trace_archive/run -- /usr/bin/python3 "$audit/cold_access.py" doctor
tools/trace_archive/run -- /usr/bin/python3 "$audit/cold_access.py" verify-manifest \
  runtime_evidence/centrifuge_48box_left_reorient_20261008_r150/SHA256SUMS
```

该显式工具在本Python进程内叠加只读五路径视图，原31路径适配器仍独立原样保留。
使用经SHA锁定的旧兼容代码，不改旧文件；对指定冷日志的open/Path读取和stat透明返回原字节/元数据。
SHA、大小、行数、gzip CRC均验证；缺档/错误数据必须失败，不能跳过原manifest成员。
仅校验时调用，并不变更默认运行环境；不会自动传播这层冷日志视图给子进程。

## 需要原始文件的旧分析或原生工具

五个raw路径物理文件已用独立gzip保存，普通cat、原生sha256sum或直接旧分析会看到原路径缺失；
这不是内容损坏。旧五份清单请使用上面的专用校验工具，当前运行保护仍用原入口。
某个旧离线分析若要求真实文件，先检查磁盘，在**新的非封存临时目录**导出，例如：

```bash
tools/trace_archive/run -- /usr/bin/python3 \
  runtime_evidence/cold_trace_archive_20261022_r1/cold_access.py export \
  runtime_evidence/centrifuge_48box_left_reorient_20261008_r150/run/loading_trace.jsonl \
  /你选择的全新临时目录/loading_trace.jsonl
```

输出父目录须已存在，目标须不存在，且不能位于任何含SHA256SUMS的封存目录内。
export保留至少512MiB余量，完成后再次核验SHA/字节/行数并设为只读；不覆盖原路径或已有文件，
也不删除gzip。分析工具显式读取新导出路径；无需恢复旧脚本、覆盖run或改变当前开发入口。
若未来确需按原路径读取，可在该离线Python分析入口显式调用cold_access.install()，
但其子进程/原生描述符不虚拟化，不能冒称系统级透明压缩。

## 事务与中断

archive_cold.py固定五路径白名单、普通独占目录和锁；
读取源SHA与旧清单一致后压缩，系统gzip完整往返解码，fsync/独占提交及登记后才移除重复raw。
每次删除前再核对原inode/元数据、原manifest和可读进程FD，权限盲区如实记录。
任何异常保留原raw及未完成partial；已有目标不覆盖。不要直接重跑已完成迁移。
inventory初次因旧r150清单的./前缀拒绝，规范化路径后重试成功，原失败日志保留。
