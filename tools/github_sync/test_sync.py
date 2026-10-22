#!/usr/bin/env python3
"""Exercise the sync CLI using disposable source trees and local Git remotes."""
import fcntl
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest


ENGINE = Path(__file__).with_name('sync.py')


class SyncIntegrationTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix='h1-sync-test-')
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.source = self.root / 'original project'
        self.repository = self.root / 'published repository'
        self.remote = self.root / 'remote.git'
        self.source.mkdir()
        for name in ('src', 'tools', '文档', 'runtime_assets', 'runtime_evidence'):
            (self.source / name).mkdir()
        self.write_source('README.md', '# Temporary test project\n')
        self.write_source('src/main.py', 'print("original")\n')
        self.env = os.environ.copy()
        self.env.update({
            'GIT_CONFIG_NOSYSTEM': '1',
            'GIT_CONFIG_GLOBAL': os.devnull,
            'GIT_TERMINAL_PROMPT': '0',
            'GH_PROMPT_DISABLED': '1',
            'GIT_AUTHOR_NAME': 'Sync integration test',
            'GIT_AUTHOR_EMAIL': 'sync-test@example.invalid',
            'GIT_COMMITTER_NAME': 'Sync integration test',
            'GIT_COMMITTER_EMAIL': 'sync-test@example.invalid',
        })
        self.git('init', '--bare', '--initial-branch=main', str(self.remote), cwd=self.root)
        self.git('clone', str(self.remote), str(self.repository), cwd=self.root)
        (self.repository / 'README.md').write_text('# Temporary test project\n')
        (self.repository / 'SYNC_FILES.json').write_text('[]\n')
        self.git('add', '.', cwd=self.repository)
        self.git('commit', '-m', 'Seed disposable test repository', cwd=self.repository)
        self.git('push', '-u', 'origin', 'main', cwd=self.repository)
        self.config = self.root / 'config.json'
        self.settings = {
            'source': str(self.source),
            'repository': str(self.repository),
            'expected_remote': str(self.remote),
            'branch': 'main',
            'max_file_bytes': 1024 * 1024,
            'evidence_max_file_bytes': 512 * 1024,
            'source_trees': ['src', 'tools', '文档', 'runtime_assets'],
            'historical_evidence_files': [],
            'historical_evidence_dirs': [],
            'extra_evidence_dirs': [],
        }
        self.save_config()

    def save_config(self):
        self.config.write_text(json.dumps(self.settings, ensure_ascii=False))

    def git(self, *args, cwd=None, check=True):
        result = subprocess.run(
            ['git', *args], cwd=cwd or self.repository, env=self.env,
            text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
        )
        if check and result.returncode:
            self.fail(f'Git fixture failed: {args!r}\n{result.stdout}\n{result.stderr}')
        return result.stdout.strip() if check else result

    def run_sync(self, *args, success=True):
        result = subprocess.run(
            [sys.executable, str(ENGINE), '--config', str(self.config), *args],
            cwd=self.root, env=self.env, text=True,
            stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=60,
        )
        if success:
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        else:
            self.assertNotEqual(result.returncode, 0, result.stdout + result.stderr)
        return result

    def write_source(self, rel, data):
        target = self.source / rel
        target.parent.mkdir(parents=True, exist_ok=True)
        if isinstance(data, bytes):
            target.write_bytes(data)
        else:
            target.write_text(data)
        return target

    @staticmethod
    def file_snapshot(root):
        return {
            str(p.relative_to(root)): hashlib.sha256(p.read_bytes()).hexdigest()
            for p in root.rglob('*') if p.is_file() and not p.is_symlink()
        }

    def remote_head(self):
        return self.git('rev-parse', 'refs/heads/main', cwd=self.remote)

    def assert_pushed_clean(self):
        self.assertEqual(self.git('rev-parse', 'HEAD'), self.remote_head())
        self.assertEqual(self.git('status', '--porcelain'), '')

    def create_remote_commit(self):
        other = self.root / 'other clone'
        self.git('clone', str(self.remote), str(other), cwd=self.root)
        (other / 'remote-note.md').write_text('An independent remote update.\n')
        self.git('add', '.', cwd=other)
        self.git('commit', '-m', 'Independent remote change', cwd=other)
        self.git('push', cwd=other)
        return self.remote_head()

    def test_add_modify_delete_and_repeated_noop(self):
        self.run_sync('-m', 'Initial source synchronization')
        self.assertEqual((self.repository / 'src/main.py').read_text(), 'print("original")\n')
        self.write_source('src/main.py', 'print("updated")\n')
        added = self.write_source('tools/new entry.sh', '#!/bin/sh\necho updated\n')
        added.chmod(0o755)
        self.write_source('文档/新说明.md', '同步 Unicode 文件名。\n')
        self.write_source('src/deleted.py', '# temporary\n')
        self.run_sync()
        (self.source / 'src/deleted.py').unlink()
        before = self.file_snapshot(self.source)
        self.run_sync('-m', 'Apply source updates and deletion')
        self.assertEqual(self.file_snapshot(self.source), before)
        self.assertFalse((self.repository / 'src/deleted.py').exists())
        self.assertEqual((self.repository / 'src/main.py').read_text(), 'print("updated")\n')
        self.assertEqual(self.git('ls-files', '-s', '--', 'tools/new entry.sh').split()[0], '100755')
        self.assertEqual((self.repository / '文档/新说明.md').read_text(), '同步 Unicode 文件名。\n')
        self.assert_pushed_clean()
        head = self.remote_head()
        self.run_sync()
        self.assertEqual(self.remote_head(), head, 'A no-op must not create a timestamp-only commit')
        self.assert_pushed_clean()

    def test_videos_large_files_archives_and_credentials_stay_local(self):
        names = ['src/recording.mp4', 'tools/capture.mkv', '文档/replay.webm',
                 'runtime_assets/raw.jsonl', 'src/model.tar.gz', 'src/compiled.so',
                 'src/.env', 'src/.ssh/id_ed25519']
        for name in names:
            self.write_source(name, b'local data only\n')
        self.write_source('src/large.bin', b'x' * (1024 * 1024 + 1))
        self.write_source('runtime_evidence/old_run/video.mp4', b'preserve historical original')
        before = self.file_snapshot(self.source)
        self.run_sync()
        tracked = set(self.git('ls-files', '-z').split('\0'))
        for name in names + ['src/large.bin', 'runtime_evidence/old_run/video.mp4']:
            self.assertNotIn(name, tracked)
        self.assertEqual(self.file_snapshot(self.source), before)
        self.assert_pushed_clean()

    def test_secret_in_source_aborts_before_publish_without_echoing_secret(self):
        self.run_sync()
        secret = 'ghp_' + 'A' * 40
        self.write_source('src/config.py', 'TOKEN = "' + secret + '"\n')
        head = self.remote_head()
        before = self.file_snapshot(self.repository)
        result = self.run_sync(success=False)
        self.assertNotIn(secret, result.stdout + result.stderr)
        self.assertEqual(self.remote_head(), head)
        # Fetch may update Git's remote-tracking bookkeeping; published files must stay unchanged.
        after = self.file_snapshot(self.repository)
        self.assertEqual({k: v for k, v in before.items() if not k.startswith('.git/')},
                         {k: v for k, v in after.items() if not k.startswith('.git/')})

    def test_dirty_worktree_and_untracked_files_are_preserved(self):
        self.run_sync()
        (self.repository / 'src/main.py').write_text('# local edit must survive\n')
        local = self.repository / 'personal-note.txt'
        local.write_text('A local note must survive.\n')
        self.write_source('src/main.py', '# concurrent source change\n')
        head = self.remote_head()
        self.run_sync(success=False)
        self.assertEqual((self.repository / 'src/main.py').read_text(), '# local edit must survive\n')
        self.assertEqual(local.read_text(), 'A local note must survive.\n')
        self.assertEqual(self.remote_head(), head)

    def test_wrong_remote_stops_before_worktree_changes(self):
        before = self.file_snapshot(self.repository)
        self.settings['expected_remote'] = str(self.root / 'different.git')
        self.save_config()
        self.run_sync(success=False)
        after = self.file_snapshot(self.repository)
        self.assertEqual({k: v for k, v in before.items() if k != '.git/h1-sync.lock'},
                         {k: v for k, v in after.items() if k != '.git/h1-sync.lock'})

    def test_remote_ahead_does_not_get_overwritten(self):
        self.run_sync()
        local_head = self.git('rev-parse', 'HEAD')
        remote_head = self.create_remote_commit()
        self.write_source('src/main.py', '# source pending remote reconciliation\n')
        self.run_sync(success=False)
        self.assertEqual(self.git('rev-parse', 'HEAD'), local_head)
        self.assertEqual(self.remote_head(), remote_head)
        self.assertEqual((self.repository / 'src/main.py').read_text(), 'print("original")\n')

    def test_diverged_history_is_not_rebased_reset_or_force_pushed(self):
        self.run_sync()
        remote_head = self.create_remote_commit()
        (self.repository / 'local-note.md').write_text('Independent local commit.\n')
        self.git('add', '.', cwd=self.repository)
        self.git('commit', '-m', 'Independent local commit', cwd=self.repository)
        local_head = self.git('rev-parse', 'HEAD')
        self.run_sync(success=False)
        self.assertEqual(self.git('rev-parse', 'HEAD'), local_head)
        self.assertEqual(self.remote_head(), remote_head)
        self.assertTrue((self.repository / 'local-note.md').exists())

    def test_rejected_push_is_retained_and_next_run_retries(self):
        self.run_sync()
        old_remote = self.remote_head()
        hook = self.remote / 'hooks/pre-receive'
        hook.write_text('#!/bin/sh\nprintf "test push rejection\\n" >&2\nexit 1\n')
        hook.chmod(0o755)
        self.write_source('src/main.py', '# offline update must remain committed\n')
        self.run_sync(success=False)
        pending = self.git('rev-parse', 'HEAD')
        self.assertNotEqual(pending, old_remote)
        self.assertEqual(self.remote_head(), old_remote)
        self.assertEqual(self.git('status', '--porcelain'), '')
        hook.unlink()
        self.run_sync()
        self.assertEqual(self.remote_head(), pending, 'Retry must push the saved commit without duplicating it')
        self.assert_pushed_clean()

    def test_unpushed_secret_history_is_not_published_even_if_later_deleted(self):
        self.run_sync()
        remote_before = self.remote_head()
        secret = 'ghp_' + 'B' * 40
        accidental = self.repository / 'accidental-secret.txt'
        accidental.write_text('An accidentally committed credential: ' + secret + '\n')
        self.git('add', '--', accidental.name)
        self.git('commit', '-m', 'Accidental local sensitive commit')
        self.git('rm', '--', accidental.name)
        self.git('commit', '-m', 'Remove sensitive file from working tree')
        local_head = self.git('rev-parse', 'HEAD')
        result = self.run_sync(success=False)
        self.assertNotIn(secret, result.stdout + result.stderr)
        self.assertEqual(self.remote_head(), remote_before)
        self.assertEqual(self.git('rev-parse', 'HEAD'), local_head)

    def test_unpushed_local_video_commit_is_not_published(self):
        self.run_sync()
        remote_before = self.remote_head()
        (self.repository / 'accidental-video.mp4').write_bytes(b'local video, even when small')
        self.git('add', '-f', '--', 'accidental-video.mp4')
        self.git('commit', '-m', 'Accidental local video commit')
        local_head = self.git('rev-parse', 'HEAD')
        self.run_sync(success=False)
        self.assertEqual(self.remote_head(), remote_before)
        self.assertEqual(self.git('rev-parse', 'HEAD'), local_head)

    def test_concurrent_invocation_is_rejected_without_source_changes(self):
        lock = self.repository / '.git/h1-sync.lock'
        source_before = self.file_snapshot(self.source)
        with lock.open('a') as stream:
            fcntl.flock(stream.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
            self.run_sync(success=False)
        self.assertEqual(self.file_snapshot(self.source), source_before)
        self.assertFalse((self.repository / 'src/main.py').exists())

    def test_dry_run_does_not_mutate_publish_files_or_need_remote_network(self):
        before = self.file_snapshot(self.repository)
        remote_offline = self.root / 'temporarily-offline.git'
        self.remote.rename(remote_offline)
        self.run_sync('--dry-run')
        after = self.file_snapshot(self.repository)
        self.assertEqual({k: v for k, v in before.items() if k != '.git/h1-sync.lock'},
                         {k: v for k, v in after.items() if k != '.git/h1-sync.lock'})

    def test_symlinks_do_not_copy_external_bytes(self):
        outside = self.root / 'outside-private.txt'
        outside.write_text('Never publish bytes outside the original project.\n')
        (self.source / 'src/outside.py').symlink_to(outside)
        (self.source / 'tools/outside-dir').symlink_to(self.root, target_is_directory=True)
        self.run_sync()
        self.assertNotIn('src/outside.py', self.git('ls-files', '-z').split('\0'))
        self.assertFalse((self.repository / 'tools/outside-dir').exists())
        self.assertEqual(outside.read_text(), 'Never publish bytes outside the original project.\n')

    def test_missing_source_tree_does_not_mass_delete_published_files(self):
        self.run_sync()
        (self.source / 'src/main.py').unlink()
        (self.source / 'src').rmdir()
        head = self.remote_head()
        self.run_sync(success=False)
        self.assertEqual(self.remote_head(), head)
        self.assertTrue((self.repository / 'src/main.py').exists())


if __name__ == '__main__':
    unittest.main(verbosity=2)
