# 一键同步到 GitHub

本机入口：

```bash
/home/liu/h1_moveit/sync_to_github.sh
```

命令会检查、同步、提交并推送到 `luonianzui/h1-moveit` 的 `main` 分支。
凭据由本机 GitHub CLI 与系统密钥环提供，不在脚本、Git URL 或仓库内保存 token。
没有变化时不产生空提交；如果上次推送失败但本地提交成功，重跑会补推该提交。

先看将同步哪些文件：

```bash
/home/liu/h1_moveit/sync_to_github.sh --dry-run
```

预览不联网、不修改发布工作树；它会在发布仓库的 Git 元数据目录加锁，并在仓库旁使用临时目录。
自定义提交说明：

```bash
/home/liu/h1_moveit/sync_to_github.sh -m "更新规划代码和报告"
```

从 GitHub 下载此工具后，也可运行 `python3 tools/github_sync/sync.py --config 配置文件路径`。
`config.json` 包含这台工作站的路径，换机器时需要修改 `source`、`repository` 和目标仓库。

## 文件范围

- 原工程：`/home/liu/h1_moveit/h1_2_ros2_ws_claude_2026-07-30/h1_2_ros2_ws`。
- Git 发布副本：`/home/liu/h1_moveit/.github_sync/repository`。
- 自动扫描 `src/`、`tools/`、`文档/`、`runtime_assets/` 和根级文档/入口脚本。
- 保留初次精确选出的历史证据文件；新增实验目录按代码、配置、报告及小型结果文件的规则纳入。当前 r348–r351 和关联复核/组件目录已包含。成功与失败结果原样保存，不把同步当作成功认证。
- 为避免把全部旧实验上传，配置中的 `historical_evidence_dirs` 固定记录早期历史目录。没有精确选中的旧文件继续留本地。以后需要发布某个旧目录的新文件，可把该目录名称加入 `extra_evidence_dirs`。新实验请沿用项目惯例使用新目录，脚本会自动发现。
- 常规单文件上限 50 MiB；新实验中的小型证据另有 5 MiB 上限。超限的新文件跳过，已发布文件变为超限或被规则排除时停止并指出文件。
- 视频（含小视频）、rosbag、逐帧 JSONL、压缩历史归档、profile、build/install/log/cache、编译二进制和凭据文件不上传。符号链接不跟随。早期精确选出的两个小日志作为已有证据保留。

筛选细则见 `selection.py`；新增文件被跳过时会显示分类计数。`LOCAL_DATA_INVENTORY.jsonl.gz` 仅记录未发布运行数据的路径、大小、修改时间，不读取或计算约 100 GB 原始数据的哈希，也不包含被跳过的构建目录。此清单不是异地备份。

## 数据保护和失败处理

原工程始终只读；脚本只改 Git 发布副本。源文件删除会对应为发布副本的 Git 删除，历史提交仍保留原内容；整个必要源目录缺失会停止，避免误判为批量删除。

脚本会检查 Git 目录、origin 地址、分支、并发锁、源文件读取稳定性、已暂存内容和常见凭据模式。推送前还检查所有待推送提交中的新增内容，包含之后已被删除的文件，防止遗漏中间提交里的凭据或大文件。凭据扫描是启发式检查，不能保证发现所有敏感数据；命中时只显示路径与类型。

发布目录有未提交/未跟踪改动、远端领先/分叉、源文件正在编辑或空间不足时会停止。它不会自动 stash、reset、clean、强制推送，也不会启动 ROS、Gazebo、构建或历史实验。请在保存并结束本轮编辑后运行。

如果同一路径从文件变成目录或相反，会在写入之前停止，需要先处理发布副本的结构冲突。

网络或 GitHub 授权失败：先恢复网络/登录，再重跑脚本。GitHub 授权状态可用 `gh auth status --hostname github.com` 检查。

远端领先或分叉：先检查发布副本里的 `git status` / `git log`，妥善合并远端更改，再重跑。发布副本有未提交修改或复制/提交中断：先检查保留下来的文件，完成或处理该次更改，再重跑；脚本不会擅自丢弃它们。可直接请助手协助处理。

生成的 `SYNC_STATUS.json` 保留初始历史基线，并记录实际同步时间和范围；最新工程结论以原工程同步来的交接文档为准。历史原始清单仍可能引用未分发的大文件或二进制，未验证在另一台机器完整复现。

## 独立测试

```bash
python3 -m unittest discover -s /home/liu/h1_moveit/github_sync -p test_sync.py -v
```

测试只使用临时源目录与本地临时 bare Git 仓库，不访问真实工程或 GitHub。
