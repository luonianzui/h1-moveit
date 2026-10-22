#!/usr/bin/env python3
"""Read-only source export, explicit Git staging, commit and ordinary push."""
from __future__ import annotations

import argparse
import collections
import datetime as dt
import fcntl
import gzip
import hashlib
import json
import os
from pathlib import Path, PurePosixPath
import re
import shutil
import stat
import subprocess
import sys
import tempfile

from selection import collect_candidates, blocked_path

TOOL_DIR = Path(__file__).resolve().parent
PUBLISHED_TOOLS = "tools/github_sync"
GENERATED = {"SYNC_FILES.json", "SYNC_STATUS.json", "LOCAL_DATA_STORAGE.json",
             "LOCAL_DATA_INVENTORY.jsonl.gz"}
SECRET_PATTERNS = {
    "private_key": re.compile(rb"-----BEGIN (?:RSA |EC |OPENSSH |DSA |ENCRYPTED )?PRIVATE KEY-----"),
    "github_token": re.compile(rb"\b(?:gh[pousr]_[A-Za-z0-9]{30,}|github_pat_[A-Za-z0-9_]{30,})\b"),
    "aws_access_key": re.compile(rb"\b(?:AKIA|ASIA)[A-Z0-9]{16}\b"),
    "api_key": re.compile(rb"\bsk-(?:proj-)?[A-Za-z0-9_-]{32,}\b"),
    "credential_url": re.compile(rb"https?://[^\s/:]+:[^\s/@]+@"),
}
NOTICE = (
    "> **GitHub 自动同步说明**：本仓库保存源码、模型、配置、文档和精选小型成果。"
    "完整录像、逐帧日志、历史大数据和构建产物继续保留在原工作站；"
    "未配置异地大文件备份。使用方法和范围见 [一键同步说明](tools/github_sync/README.md)，"
    "本地数据清单见 `LOCAL_DATA_INVENTORY.jsonl.gz`。"
    "同步不代表实验成功，也不代表已经验证异机复现；原始清单引用的部分大文件和二进制未分发。\n\n"
)
IGNORE = """
# Managed by tools/github_sync: full data stays in the original workspace.
**/__pycache__/
**/.pytest_cache/
**/.git/
**/build*/
**/install*/
**/log/
**/logs/
**/colcon_log/
*.py[cod]
*.mp4
*.mkv
*.avi
*.mov
*.webm
*.db3
*.mcap
*.bag
*.cdr
*.jsonl
*.jsonl.gz
*.tar
*.tar.gz
*.tar.zst
*.zip
*.7z
*.so
*.so.*
*.o
*.a
.env
.env.*
*.pem
*.key
!LOCAL_DATA_INVENTORY.jsonl.gz
"""


class SyncError(RuntimeError):
    pass


def command(args, *, cwd=None, check=True, data=None):
    env = os.environ.copy()
    env.update(GIT_TERMINAL_PROMPT="0", GIT_ASKPASS="/bin/false",
               SSH_ASKPASS="/bin/false", GH_PROMPT_DISABLED="1")
    try:
        result = subprocess.run(args, cwd=cwd, input=data, stdout=subprocess.PIPE,
                                stderr=subprocess.PIPE, env=env, timeout=180)
    except subprocess.TimeoutExpired as exc:
        raise SyncError(f"命令超时：{args[0]} {args[1]}；已有本地文件和提交保留。") from exc
    if check and result.returncode:
        # Never include helper output, credentials, or a possibly credential-bearing remote.
        raise SyncError(f"命令失败：{' '.join(args[:3])}（退出码 {result.returncode}）。"
                        "请检查网络、GitHub 登录或 Git 分支状态；已有提交保留。")
    return result


def git(repo, *args, check=True):
    return command(["git", *args], cwd=repo, check=check)


def text_git(repo, *args):
    return git(repo, *args).stdout.decode().strip()


def safe_relative(name):
    p = PurePosixPath(name)
    if not name or p.is_absolute() or ".." in p.parts or ".git" in p.parts or "\\" in name:
        raise SyncError(f"不安全的相对路径：{name!r}")
    if p.as_posix() != name or any(ord(c) < 32 for c in name):
        raise SyncError(f"不支持的路径：{name!r}")
    return p


def load_json(path, default):
    return json.loads(path.read_text()) if path.exists() else default


def json_bytes(value):
    return (json.dumps(value, ensure_ascii=False, indent=2) + "\n").encode()


def blob(data):
    return hashlib.sha1(b"blob " + str(len(data)).encode() + b"\0" + data).hexdigest()


def scan_secret(name, data):
    if data.startswith((b"\x7fELF", b"MZ")) or data[4:8] == b"ftyp":
        raise SyncError(f"检测到编译二进制或录像内容，未提交或推送：{name}")
    findings = [kind for kind, pattern in SECRET_PATTERNS.items() if pattern.search(data)]
    if findings:
        raise SyncError(f"发现疑似凭据，未提交或推送：{name}（{', '.join(findings)}）。"
                        "请先移除凭据或调整发布范围；不会显示匹配值。")


def read_stable(root, name, limit):
    """Open every path component with O_NOFOLLOW; never follow source symlinks."""
    parts = safe_relative(name).parts
    directory = os.open(root, os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW)
    try:
        for part in parts[:-1]:
            child = os.open(part, os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW, dir_fd=directory)
            os.close(directory)
            directory = child
        fd = os.open(parts[-1], os.O_RDONLY | os.O_NOFOLLOW, dir_fd=directory)
        with os.fdopen(fd, "rb") as stream:
            before = os.fstat(stream.fileno())
            if not stat.S_ISREG(before.st_mode) or before.st_size > limit:
                raise SyncError(f"文件类型或大小不适合上传：{name}")
            data = stream.read(limit + 1)
            after = os.fstat(stream.fileno())
        current = os.stat(parts[-1], dir_fd=directory, follow_symlinks=False)
        signature = lambda s: (s.st_dev, s.st_ino, s.st_size, s.st_mtime_ns, s.st_ctime_ns, s.st_mode)
        if signature(before) != signature(after) or signature(after) != signature(current) or len(data) != before.st_size:
            raise SyncError(f"文件在读取时发生变化，请停止编辑后重试：{name}")
        return data, before
    finally:
        os.close(directory)


def source_signature(path):
    s = path.lstat()
    return (s.st_dev, s.st_ino, s.st_size, s.st_mtime_ns, s.st_ctime_ns, s.st_mode)


def index_entries(repo):
    result = {}
    for record in git(repo, "ls-files", "--stage", "-z").stdout.split(b"\0"):
        if not record:
            continue
        meta, name = record.split(b"\t", 1)
        mode, sha, stage = meta.decode().split()
        if stage != "0" or mode not in {"100644", "100755"}:
            raise SyncError("发布仓库包含未合并文件、符号链接或子模块，需要先人工处理。")
        result[name.decode()] = (mode, sha)
    return result


def assert_clean(repo):
    if git(repo, "status", "--porcelain=v1", "--untracked-files=all").stdout:
        raise SyncError("GitHub 同步目录存在未提交/未跟踪的修改。为避免覆盖，请先检查该目录的 git status；原工程未改动。")


def verify_repository(repo, source, config):
    if repo == source or source in repo.parents or repo in source.parents:
        raise SyncError("原工程与发布仓库必须是互不包含的不同目录。")
    if text_git(repo, "rev-parse", "--show-toplevel") != str(repo):
        raise SyncError("repository 必须指向发布仓库根目录。")
    if text_git(repo, "symbolic-ref", "--short", "HEAD") != config["branch"]:
        raise SyncError(f"发布仓库必须位于 {config['branch']} 分支。")
    for direction in ([], ["--push"]):
        urls = text_git(repo, "remote", "get-url", *direction, "--all", "origin").splitlines()
        if urls != [config["expected_remote"]]:
            raise SyncError("origin 地址与配置不一致，已停止以防推送到错误仓库。")
    for name in config.get("source_trees", ["src", "tools", "文档", "runtime_assets"]) + ["runtime_evidence"]:
        p = source / name
        if not p.is_dir() or p.is_symlink():
            raise SyncError(f"原工程必要目录缺失或是符号链接：{name}；不会把它当作批量删除。")
    if not (source / "README.md").is_file():
        raise SyncError("原工程 README.md 缺失。")
    gitdir = Path(text_git(repo, "rev-parse", "--absolute-git-dir"))
    for marker in ("MERGE_HEAD", "CHERRY_PICK_HEAD", "REVERT_HEAD", "rebase-merge", "rebase-apply"):
        if (gitdir / marker).exists():
            raise SyncError("发布仓库有尚未完成的 Git 合并/变基操作。")
    return gitdir


def check_remote(repo, branch):
    git(repo, "fetch", "--quiet", "origin", f"refs/heads/{branch}:refs/remotes/origin/{branch}")
    result = git(repo, "merge-base", "--is-ancestor", f"origin/{branch}", "HEAD", check=False)
    if result.returncode:
        raise SyncError("GitHub 上有本地尚未包含的提交（落后或分叉）。请先检查并合并远端修改，再运行；脚本不会强制推送或自动覆盖。")


def check_outgoing(repo, config):
    """Audit every new blob in outgoing history, even a later-deleted secret."""
    revision = f"origin/{config['branch']}..HEAD"
    objects = git(repo, "rev-list", "--objects", "--no-object-names", revision).stdout
    if not objects.strip():
        return
    details = command(["git", "cat-file", "--batch-check=%(objectname) %(objecttype) %(objectsize)"], cwd=repo, data=objects).stdout
    new_blobs = {sha: int(size) for sha, kind, size in (line.split() for line in details.decode().splitlines()) if kind == "blob"}
    legacy = {x["path"] if isinstance(x, dict) else x for x in config.get("historical_evidence_files", []) + config.get("legacy_source_files", [])}
    checked = set()
    for commit in text_git(repo, "rev-list", revision).splitlines():
        tree = git(repo, "ls-tree", "-r", "-z", commit).stdout
        for entry in tree.split(b"\0"):
            if not entry:
                continue
            fields, raw_name = entry.split(b"\t", 1)
            mode, kind, sha = fields.decode().split()
            name = raw_name.decode()
            if sha not in new_blobs:
                continue
            safe_relative(name)
            reason = blocked_path(name)
            inventory = name == "LOCAL_DATA_INVENTORY.jsonl.gz"
            legacy_log = name in legacy and name.endswith(".log") and reason == "excluded_file_type"
            if mode not in {"100644", "100755"} or (reason and not inventory and not legacy_log):
                raise SyncError(f"待推送历史包含不允许发布的文件：{name}。已有提交保留，请先处理这些提交。")
            limit = config["max_file_bytes"]
            if name.startswith("runtime_evidence/"):
                limit = min(limit, config.get("evidence_max_file_bytes", 5 * 1024 * 1024))
            if new_blobs[sha] > limit:
                raise SyncError(f"待推送历史包含超限文件：{name}。已有提交保留，未推送。")
            if sha in checked:
                continue
            data = git(repo, "cat-file", "blob", sha).stdout
            if inventory:
                # A generated gzip must contain only inventory records, not arbitrary payloads.
                import io
                total = 0
                with gzip.GzipFile(fileobj=io.BytesIO(data)) as stream:
                    while True:
                        line = stream.readline(65537)
                        if not line:
                            break
                        total += len(line)
                        if len(line) > 65536 or total > 256 * 1024 * 1024:
                            raise SyncError("待推送的数据清单格式或解压大小异常。")
                        item = json.loads(line)
                        if set(item) != {"path", "bytes", "mtime_ns", "storage", "checksum_status"} or item["storage"] != "original_local_only" or item["checksum_status"] != "not_computed_by_sync":
                            raise SyncError("待推送的数据清单含非元数据内容。")
                        safe_relative(item["path"])
                        scan_secret(name, line)
            else:
                scan_secret(name, data)
            checked.add(sha)


def local_inventory(source, selected, output):
    counts, totals = collections.Counter(), collections.Counter()
    # Metadata only: no opening or hashing large historical payloads.
    with output.open("wb") as raw, gzip.GzipFile(filename="", fileobj=raw, mode="wb", mtime=0) as gz:
        for top in ("runtime_assets", "runtime_evidence", "runtime_dependencies", "runtime_logs"):
            if not (source / top).is_dir():
                continue
            for base, dirs, files in os.walk(source / top, followlinks=False):
                dirs[:] = sorted(d for d in dirs if not (Path(base) / d).is_symlink()
                                 and not d.startswith(("build", "install"))
                                 and d not in {".git", "__pycache__", ".pytest_cache", "log", "colcon_log"})
                for filename in sorted(files):
                    p = Path(base) / filename
                    rel = p.relative_to(source).as_posix()
                    if rel in selected or p.is_symlink():
                        continue
                    reason = blocked_path(rel)
                    if reason and any(word in reason.lower() for word in ("secret", "credential", "sensitive")):
                        continue
                    s = p.stat()
                    if not stat.S_ISREG(s.st_mode):
                        continue
                    item = {"path": rel, "bytes": s.st_size, "mtime_ns": s.st_mtime_ns,
                            "storage": "original_local_only", "checksum_status": "not_computed_by_sync"}
                    gz.write((json.dumps(item, ensure_ascii=False, separators=(",", ":")) + "\n").encode())
                    counts[top] += 1
                    totals[top] += s.st_size
    return {
        "source_workspace": str(source), "state": "local_only_pending_destination",
        "inventory": "LOCAL_DATA_INVENTORY.jsonl.gz",
        "checksum_status": "Metadata inventory only; large payloads were not read or hashed.",
        "inventory_scope": "Omitted runtime files, excluding build/install/cache directories and credential paths.",
        "file_counts": dict(counts), "bytes": dict(totals),
        "original_files_modified": False, "large_files_copied": False,
    }


def create_plan(source, repo, config, stage, config_path):
    candidates, excluded = collect_candidates(source, config)
    previous = load_json(repo / "SYNC_FILES.json", [])
    managed = {str(safe_relative(x["path"])) for x in previous}
    entries = index_entries(repo)
    writes, modes, all_blobs, source_stats = {}, {}, {}, {}

    def offer(name, data, mode="100644", scan=True):
        safe_relative(name)
        if scan:
            scan_secret(name, data)
        info = (mode, blob(data))
        all_blobs[name] = info
        if entries.get(name) != info:
            target = stage / name
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_bytes(data)
            target.chmod(0o755 if mode == "100755" else 0o644)
            writes[name] = target
            modes[name] = mode

    for name in sorted(managed - candidates.keys()):
        p = source / name
        if p.exists() or p.is_symlink():
            raise SyncError(f"已发布文件现在被筛选规则排除，已停止以免静默丢失：{name}。请检查文件大小、类型或规则。")

    records = []
    for name, reason in sorted(candidates.items()):
        if name in GENERATED or name == PUBLISHED_TOOLS or name.startswith(PUBLISHED_TOOLS + "/"):
            raise SyncError(f"原工程文件与同步工具保留路径冲突：{name}")
        limit = config["max_file_bytes"]
        if name.startswith("runtime_evidence/"):
            limit = min(limit, config.get("evidence_max_file_bytes", 5 * 1024 * 1024))
        data, info = read_stable(source, name, limit)
        source_stats[name] = (info.st_dev, info.st_ino, info.st_size, info.st_mtime_ns, info.st_ctime_ns, info.st_mode)
        original_sha = hashlib.sha256(data).hexdigest()
        if name == "README.md":
            data = NOTICE.encode() + data
        elif name == ".gitignore":
            data = data.rstrip() + b"\n" + IGNORE.encode()
        mode = "100755" if info.st_mode & 0o111 else "100644"
        offer(name, data, mode)
        records.append({"path": name, "bytes": len(data), "sha256": hashlib.sha256(data).hexdigest(),
                        "source_sha256": original_sha, "git_blob_sha": blob(data), "mode": mode, "reason": reason})
    if ".gitignore" not in candidates:
        offer(".gitignore", IGNORE.encode())
    # Publish this helper alongside the snapshot; the original workspace stays untouched.
    for p in sorted(TOOL_DIR.iterdir()):
        if p.is_file() and p.suffix in {".py", ".sh", ".md", ".json"} and p.name != "config.json":
            offer(f"{PUBLISHED_TOOLS}/{p.name}", p.read_bytes(), "100755" if p.name in {"sync.py", "sync.sh"} else "100644")
    offer(f"{PUBLISHED_TOOLS}/config.json", config_path.read_bytes())
    offer("SYNC_FILES.json", json_bytes(records))
    inventory = stage / "inventory.gz"
    storage = local_inventory(source, candidates, inventory)
    offer("LOCAL_DATA_INVENTORY.jsonl.gz", inventory.read_bytes(), scan=False)
    offer("LOCAL_DATA_STORAGE.json", json_bytes(storage))
    deletes = sorted((managed - candidates.keys() - all_blobs.keys()) & entries.keys())
    prior_status = load_json(repo / "SYNC_STATUS.json", {})
    status = dict(prior_status)
    status.pop("snapshot_baseline", None)
    status.update(
        repository="https://github.com/luonianzui/h1-moveit" if config["expected_remote"].endswith("luonianzui/h1-moveit.git") else config["expected_remote"],
        initial_evidence_baseline=prior_status.get("initial_evidence_baseline", prior_status.get("snapshot_baseline", "initial snapshot")),
        snapshot_kind="Source and selected evidence archive; not a portable verified release. Sync does not certify experiment success.",
        sync_policy="tools/github_sync/config.json", source_files=len(records), source_bytes=sum(r["bytes"] for r in records),
        data_storage="Full videos and historical large data remain at original local locations; no offsite backup configured.",
        checks={"source_copy_sha256": "Every selected file hashed; source left unchanged.",
                "secret_pattern_scan_findings": [], "builds_or_simulation_run": False},
    )
    # Avoid a fresh commit merely because the command was run again.
    if writes or deletes or status != prior_status:
        status["last_synced_at"] = dt.datetime.now(dt.timezone.utc).isoformat()
    offer("SYNC_STATUS.json", json_bytes(status))
    return writes, modes, deletes, all_blobs, source_stats, excluded, storage


def verify_publication(repo, expected, paths):
    entries = index_entries(repo)
    for name in paths:
        if name in expected:
            if entries.get(name) != expected[name]:
                raise SyncError(f"暂存内容与已检查的快照不一致：{name}；请检查发布目录，尚未提交。")
        elif name in entries:
            raise SyncError(f"删除项仍在暂存区：{name}")
    actual = set(x.decode() for x in git(repo, "diff", "--cached", "--name-only", "-z").stdout.split(b"\0") if x)
    if actual != set(paths):
        raise SyncError("暂存区包含计划以外的改动；停止提交，请检查发布目录。")


def execute(config_path, dry_run=False, message=None):
    config_path = config_path.resolve()
    config = load_json(config_path, None)
    if not config:
        raise SyncError("同步配置不存在或为空。")
    source, repo = (Path(config[k]).expanduser().resolve() for k in ("source", "repository"))
    if not source.is_dir() or not repo.is_dir():
        raise SyncError("原工程或 GitHub 发布目录不存在。")
    gitdir = verify_repository(repo, source, config)
    with (gitdir / "h1-sync.lock").open("a") as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            raise SyncError("已有同步正在运行，请等待结束后再试。") from exc
        assert_clean(repo)
        head = text_git(repo, "rev-parse", "HEAD")
        if not dry_run:
            check_remote(repo, config["branch"])
            check_outgoing(repo, config)
        with tempfile.TemporaryDirectory(prefix="h1-sync-", dir=repo.parent) as temp:
            stage = Path(temp)
            writes, modes, deletes, expected, signatures, excluded, storage = create_plan(source, repo, config, stage, config_path)
            print(f"检查完成：{len(signatures)} 个源文件；新增/修改 {len(writes)} 个，发布副本删除 {len(deletes)} 个。", flush=True)
            print(f"本地数据清单：{sum(storage['file_counts'].values())} 个文件，{sum(storage['bytes'].values()) / 1024**3:.2f} GiB；仅记录元数据。", flush=True)
            print("跳过统计：" + json.dumps(excluded, ensure_ascii=False), flush=True)
            for name in sorted(writes)[:25]:
                print("  + " + name)
            for name in deletes[:25]:
                print("  - " + name)
            if len(writes) + len(deletes) > 25:
                print("  （仅显示前 25 个新增/修改和前 25 个删除路径）")
            if dry_run:
                print("预览完成：未修改原工程、发布工作树或远端；未联网。")
                return
            if text_git(repo, "rev-parse", "HEAD") != head:
                raise SyncError("准备期间 Git HEAD 被其他操作修改，请重试。")
            assert_clean(repo)
            for name, signature in signatures.items():
                if source_signature(source / name) != signature:
                    raise SyncError(f"准备期间源文件被修改，请停止编辑后重试：{name}")
            paths = sorted(set(writes) | set(deletes))
            if paths:
                need = sum(p.stat().st_size for p in writes.values())
                if shutil.disk_usage(repo).free < need + 64 * 1024 * 1024:
                    raise SyncError("磁盘剩余空间不足以安全暂存 Git 对象；原工程未改动。")
                # Reject file/directory transitions before touching any worktree file.
                for name in writes:
                    dest = repo / name
                    if dest.is_dir() or any((repo / p).is_file() for p in dest.relative_to(repo).parents):
                        raise SyncError(f"源文件/目录结构发生切换，需要先处理发布副本中的冲突路径：{name}")
                for name, prepared in sorted(writes.items()):
                    dest = repo / name
                    for parent in dest.relative_to(repo).parents:
                        if (repo / parent).is_symlink():
                            raise SyncError(f"发布路径含符号链接：{name}")
                    if dest.is_symlink():
                        raise SyncError(f"发布目标是符号链接：{name}")
                    dest.parent.mkdir(parents=True, exist_ok=True)
                    os.replace(prepared, dest)
                for name in deletes:
                    (repo / name).unlink()
                # Stage only our explicit plan, including the generated inventory exception.
                payload = b"\0".join(p.encode() for p in paths) + b"\0"
                command(["git", "--literal-pathspecs", "add", "-f", "--pathspec-from-file=-", "--pathspec-file-nul"], cwd=repo, data=payload)
                verify_publication(repo, expected, paths)
                git(repo, "commit", "-m", message or ("Sync H1 project sources " + dt.datetime.now().strftime("%Y-%m-%d %H:%M:%S")))
                print("已创建提交：" + text_git(repo, "rev-parse", "--short", "HEAD"), flush=True)
            else:
                print("内容没有变化，不创建空提交。", flush=True)
            assert_clean(repo)
            check_outgoing(repo, config)
            result = git(repo, "push", "origin", f"HEAD:refs/heads/{config['branch']}", check=False)
            if result.returncode:
                raise SyncError("推送失败，本地提交已保留。检查网络/GitHub 授权或远端冲突后重跑同一脚本；不会丢弃提交或强制推送。")
            remote = text_git(repo, "ls-remote", "--exit-code", "origin", f"refs/heads/{config['branch']}").split()[0]
            if remote != text_git(repo, "rev-parse", "HEAD"):
                raise SyncError("推送后远端出现新的并发提交，请检查分支状态。")
            print("同步完成：GitHub 与本地提交一致；原工程、录像和历史大文件未改动。", flush=True)


def main():
    parser = argparse.ArgumentParser(description="同步源码、提交并推送；原工程和大文件保持本地。")
    parser.add_argument("--config", type=Path, default=TOOL_DIR / "config.json")
    parser.add_argument("--dry-run", action="store_true", help="只预览，不改工作树、不提交、不联网")
    parser.add_argument("-m", "--message", help="自定义提交说明")
    args = parser.parse_args()
    try:
        execute(args.config, args.dry_run, args.message)
    except (SyncError, OSError, ValueError, KeyError) as exc:
        print(f"同步停止：{exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
