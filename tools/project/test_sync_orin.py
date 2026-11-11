"""Offline receiver tests. Every project is temporary; no SSH or hardware runs."""
import hashlib
import io
import json
import os
from pathlib import Path
import subprocess
import sys
import tarfile
import tempfile
import unittest
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parent))
import sync_orin as sync


def manifest(contents, executable=()):
    return {name: {'sha256': hashlib.sha256(data).hexdigest(), 'size': len(data),
                   'mode': 0o755 if name in executable else 0o644}
            for name, data in contents.items()}


class PathAndDiscoveryTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(dir=os.environ.get('TMPDIR'))
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name) / 'source'
        self.root.mkdir()

    def test_relative_rejects_escape_and_reserved_paths(self):
        invalid = ('', '/etc/passwd', '../outside', 'a/../outside', 'a//b', './a',
                   'a\\b', 'C:/outside', '.git/config', '.local/token',
                   'ros2_ws/build/file', 'mujoco/ref01/output/result.json',
                   'config/parallel_exo/servo_pair.local.json', 'a/__pycache__/x.pyc')
        for name in invalid:
            with self.subTest(name=name), self.assertRaises(RuntimeError):
                sync.relative(name)
        self.assertEqual(sync.relative('mujoco/ref01/model.xml'), 'mujoco/ref01/model.xml')

    def test_local_working_tree_includes_new_and_omits_deleted_and_generated(self):
        (self.root / 'AGENTS.md').write_bytes(b'project')
        (self.root / 'run.sh').write_bytes(b'#!/bin/sh\n')
        (self.root / 'new.txt').write_bytes(b'new unsaved-to-git source')
        (self.root / '.local').mkdir()
        (self.root / '.local/cache').write_bytes(b'not a source')

        def git(command, **_):
            arguments = command[3:]
            if arguments == ['rev-parse', '--show-toplevel']:
                return str(self.root).encode() + b'\n'
            if arguments == ['ls-files', '-z', '--cached', '--others', '--exclude-standard']:
                return b'AGENTS.md\0run.sh\0deleted.txt\0new.txt\0.local/cache\0'
            if arguments == ['ls-files', '--stage', '-z']:
                return (b'100644 ' + b'0' * 40 + b' 0\tAGENTS.md\0'
                        b'100755 ' + b'1' * 40 + b' 0\trun.sh\0'
                        b'100644 ' + b'2' * 40 + b' 0\tdeleted.txt\0')
            self.fail(f'Unexpected Git query: {arguments}')

        with mock.patch.object(sync.subprocess, 'check_output', side_effect=git):
            result = sync.local_files(self.root)
        self.assertEqual(set(result), {'AGENTS.md', 'run.sh', 'new.txt'})
        self.assertEqual(result['run.sh']['mode'], 0o755)
        self.assertEqual(result['new.txt']['sha256'], hashlib.sha256(b'new unsaved-to-git source').hexdigest())

    def test_report_policy_keeps_runtime_docs_and_receiver_visibility(self):
        for name in (*sync.LOCAL_ONLY_REPORTS, 'reports/new.md',
                     'docs/parallel_exo/reports/new.json'):
            with self.subTest(name=name):
                self.assertTrue(sync.local_only_report(name))
                self.assertEqual(sync.relative(name), name)
        for name in ('AGENTS.md', 'README.md', 'docs/parallel_exo/local_orin.md',
                     'docs/parallel_exo/project_management.md',
                     'docs/parallel_exo/restore.md', 'docs/parallel_exo/realtime_state.md',
                     'tools/project/benchmark_mujoco_orin.py'):
            self.assertFalse(sync.local_only_report(name))

    def test_local_git_report_stays_on_disk_but_is_not_uploaded(self):
        name = 'docs/parallel_exo/mujoco_timing_20261106.md'
        report = self.root / name
        report.parent.mkdir(parents=True)
        report.write_bytes(b'local report')
        (self.root / 'AGENTS.md').write_bytes(b'project')
        def git(command, **_):
            if '--show-toplevel' in command:
                return str(self.root).encode() + b'\n'
            if '--stage' in command:
                return b''
            return b'AGENTS.md\0' + name.encode() + b'\0'
        with mock.patch.object(sync.subprocess, 'check_output', side_effect=git):
            self.assertEqual(set(sync.local_files(self.root)), {'AGENTS.md'})
        self.assertEqual(report.read_bytes(), b'local report')

    def test_report_removal_plan_preserves_remote_changes_and_unknown_files(self):
        report = 'docs/parallel_exo/imu_timing_20261106.md'
        old = manifest({'AGENTS.md': b'project', report: b'original'})
        wanted = manifest({'AGENTS.md': b'project'})
        self.assertEqual(sync.sync_plan(old, old, wanted, True)['delete'], [report])
        changed = manifest({'AGENTS.md': b'project', report: b'remote edit'})
        with self.assertRaisesRegex(RuntimeError, 'Remote source changes'):
            sync.sync_plan(old, changed, wanted, True)
        with self.assertRaisesRegex(RuntimeError, 'Remote source changes'):
            sync.sync_plan(wanted, old, wanted, True)

    def test_remote_inventory_omits_machine_outputs_and_local_config(self):
        files = {'source.py': b'x', 'config/parallel_exo/servo_pair.ref01.json': b'{}',
                 '.local/cache/file': b'x', 'ros2_ws/build/file': b'x',
                 'mujoco/ref01/output/file': b'x',
                 'config/parallel_exo/servo_pair.local.json': b'{}'}
        for name, data in files.items():
            path = self.root / name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(data)
        self.assertEqual(set(sync.remote_files(self.root)),
                         {'source.py', 'config/parallel_exo/servo_pair.ref01.json'})

    @unittest.skipUnless(os.name == 'posix', 'Linux symlink behavior')
    def test_safe_path_rejects_links_even_when_target_is_inside_root(self):
        (self.root / 'real').mkdir()
        (self.root / 'alias').symlink_to(self.root / 'real', target_is_directory=True)
        with self.assertRaises(RuntimeError):
            sync.safe_path(self.root, 'alias/new.txt')
        with self.assertRaises(RuntimeError):
            sync.safe_path(self.root, '../outside.txt')


@unittest.skipUnless(os.name == 'posix', 'Receiver uses Linux flock')
class ReceiverTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(dir=os.environ.get('TMPDIR'))
        self.addCleanup(self.temporary.cleanup)
        self.project = Path(self.temporary.name) / 'project'
        self.project.mkdir()
        self.root = self.project / 'dev/jetson_mech_control'
        self.sentinel = self.project / 'existing_release.txt'
        self.sentinel.write_bytes(b'original release')
        guard = mock.patch.object(sync.subprocess, 'run',
                                  side_effect=AssertionError('No build, SSH, or hardware subprocess is allowed in receiver tests'))
        self.runner = guard.start()
        self.addCleanup(guard.stop)

    def tearDown(self):
        self.assertEqual(self.sentinel.read_bytes(), b'original release')
        self.runner.assert_not_called()

    def request(self, action, files, **extra):
        return {'project_root': str(self.project), 'remote_root': str(self.root),
                'action': action, 'files': files, **extra}

    def status(self, wanted):
        return sync.remote_operation(self.request('status', wanted))

    def transfer(self, contents, executable=(), overrides=None, mutate_archive=None):
        wanted = manifest(contents, executable)
        plan = sync.remote_operation(self.request('prepare', wanted))
        archive = Path(plan['archive'])
        with tarfile.open(archive, 'w:gz') as tar:
            for name in plan['update']:
                data = (overrides or {}).get(name, contents[name])
                entry = tarfile.TarInfo(name)
                entry.size = len(data)
                entry.mode = wanted[name]['mode']
                if mutate_archive:
                    mutate_archive(entry)
                tar.addfile(entry, io.BytesIO(data) if entry.isfile() else None)
        request = self.request('apply', wanted, token=plan['token'],
                               archive_sha256=hashlib.sha256(archive.read_bytes()).hexdigest(),
                               source_git={'head': '0' * 40, 'working_tree_included': True})
        return wanted, plan, request

    def synchronize(self, contents, executable=()):
        wanted, plan, request = self.transfer(contents, executable)
        result = sync.remote_operation(request)
        self.assertEqual(self.status(wanted)['files'], wanted)
        self.assertFalse(self.status(wanted)['incomplete'])
        return result, plan

    def test_first_sync_repeat_update_delete_and_runtime_preservation(self):
        self.assertFalse(self.status({})['exists'])
        self.assertFalse(self.root.exists())
        (self.root / '.local').mkdir(parents=True)
        (self.root / '.local/keep').write_bytes(b'local environment')
        first = {'AGENTS.md': b'project', 'obsolete.txt': b'old', 'run.sh': b'#!/bin/sh\n'}
        result, _ = self.synchronize(first, executable=('run.sh',))
        self.assertEqual((result['updated'], result['deleted']), (3, 0))
        self.assertEqual(sync.file_record(self.root / 'run.sh')['mode'], 0o755)
        result, _ = self.synchronize(first, executable=('run.sh',))
        self.assertEqual((result['updated'], result['deleted']), (0, 0))
        second = {'AGENTS.md': b'project', 'new.txt': b'new', 'run.sh': b'#!/bin/sh\nexit 0\n'}
        result, _ = self.synchronize(second, executable=('run.sh',))
        self.assertEqual((result['updated'], result['deleted']), (2, 1))
        self.assertFalse((self.root / 'obsolete.txt').exists())
        self.assertEqual((self.root / '.local/keep').read_bytes(), b'local environment')

    def test_old_managed_report_is_backed_up_removed_and_repeat_is_noop(self):
        report = 'docs/parallel_exo/imu_timing_20261106.md'
        self.synchronize({'AGENTS.md': b'project', report: b'original report'})
        result, plan = self.synchronize({'AGENTS.md': b'project'})
        self.assertEqual(result['deleted'], 1)
        self.assertFalse((self.root / report).exists())
        backup = self.root / '.local/sync/incoming' / plan['token'] / 'before' / report
        self.assertEqual(backup.read_bytes(), b'original report')
        result, _ = self.synchronize({'AGENTS.md': b'project'})
        self.assertEqual((result['updated'], result['deleted']), (0, 0))

    def test_unregistered_nonempty_target_is_not_overwritten(self):
        self.root.mkdir(parents=True)
        (self.root / 'existing.txt').write_bytes(b'owned by someone else')
        with self.assertRaisesRegex(RuntimeError, 'unregistered'):
            sync.remote_operation(self.request('prepare', manifest({'AGENTS.md': b'project'})))
        self.assertEqual((self.root / 'existing.txt').read_bytes(), b'owned by someone else')

    def test_remote_edit_or_deletion_conflict_is_preserved(self):
        original = {'AGENTS.md': b'project', 'a.txt': b'original'}
        self.synchronize(original)
        for changed in (b'remote-only edit', None):
            with self.subTest(changed=changed):
                target = self.root / 'a.txt'
                if changed is None:
                    target.unlink()
                else:
                    target.write_bytes(changed)
                    target.chmod(0o644)
                with self.assertRaisesRegex(RuntimeError, 'Remote source changes'):
                    sync.remote_operation(self.request('prepare', manifest(original)))
                self.assertEqual(target.read_bytes() if target.exists() else None, changed)

    def test_remote_unregistered_same_name_is_never_adopted_silently(self):
        self.synchronize({'AGENTS.md': b'project'})
        (self.root / 'new.txt').write_bytes(b'local intention')
        desired = manifest({'AGENTS.md': b'project', 'new.txt': b'local intention'})
        with self.assertRaisesRegex(RuntimeError, 'Remote source changes'):
            sync.remote_operation(self.request('prepare', desired))
        self.assertEqual((self.root / 'new.txt').read_bytes(), b'local intention')

    def test_changed_remote_file_cannot_be_deleted_by_sync(self):
        self.synchronize({'AGENTS.md': b'project', 'remove.txt': b'original'})
        (self.root / 'remove.txt').write_bytes(b'remote work')
        with self.assertRaisesRegex(RuntimeError, 'Remote source changes'):
            sync.remote_operation(self.request('prepare', manifest({'AGENTS.md': b'project'})))
        self.assertEqual((self.root / 'remove.txt').read_bytes(), b'remote work')

    def test_corrupt_archive_preserves_all_old_source(self):
        original = {'AGENTS.md': b'project', 'a.txt': b'old'}
        self.synchronize(original)
        wanted, _, request = self.transfer({'AGENTS.md': b'project', 'a.txt': b'new'})
        request['archive_sha256'] = '0' * 64
        with self.assertRaisesRegex(RuntimeError, 'archive checksum'):
            sync.remote_operation(request)
        self.assertEqual(sync.remote_files(self.root), manifest(original))
        self.assertFalse(self.status(wanted)['incomplete'])

    def test_wrong_member_hash_preserves_all_old_source(self):
        original = {'AGENTS.md': b'project', 'a.txt': b'old'}
        self.synchronize(original)
        wanted, _, request = self.transfer({'AGENTS.md': b'project', 'a.txt': b'new'},
                                           overrides={'a.txt': b'bad'})
        with self.assertRaisesRegex(RuntimeError, 'File checksum mismatch'):
            sync.remote_operation(request)
        self.assertEqual(sync.remote_files(self.root), manifest(original))
        self.assertFalse(self.status(wanted)['incomplete'])

    def test_tar_symlink_cannot_escape_into_other_project_content(self):
        def symlink(entry):
            entry.type = tarfile.SYMTYPE
            entry.linkname = str(self.sentinel)
        wanted, _, request = self.transfer({'AGENTS.md': b'project'}, mutate_archive=symlink)
        with self.assertRaisesRegex(RuntimeError, 'Invalid archive entry'):
            sync.remote_operation(request)
        self.assertFalse((self.root / 'AGENTS.md').exists())
        self.assertFalse(self.status(wanted)['incomplete'])

    def test_linked_destination_or_metadata_is_rejected(self):
        self.root.mkdir(parents=True)
        elsewhere = self.project / 'elsewhere'
        elsewhere.mkdir()
        (self.root / '.local').symlink_to(elsewhere, target_is_directory=True)
        with self.assertRaisesRegex(RuntimeError, 'Link/reparse|Path escapes project'):
            sync.remote_operation(self.request('prepare', manifest({'AGENTS.md': b'project'})))
        self.assertEqual(list(elsewhere.iterdir()), [])

    def test_shared_lock_blocks_another_sync_or_build(self):
        self.root.mkdir(parents=True)
        with sync.locked(self.root):
            for action in ('prepare', 'build', 'test'):
                with self.subTest(action=action), self.assertRaisesRegex(RuntimeError, 'Another sync/build/test'):
                    sync.remote_operation(self.request(action, manifest({'AGENTS.md': b'project'}), profile='imu'))

    def test_apply_interruption_keeps_backup_and_blocks_followup(self):
        self.synchronize({'AGENTS.md': b'project', 'a.txt': b'old a', 'b.txt': b'old b'})
        wanted, plan, request = self.transfer({'AGENTS.md': b'project', 'a.txt': b'new a'})
        replace = sync.os.replace

        def interrupted(source, destination):
            if Path(destination) == self.root / 'a.txt':
                raise OSError('injected interruption')
            return replace(source, destination)

        with mock.patch.object(sync.os, 'replace', side_effect=interrupted):
            with self.assertRaisesRegex(OSError, 'injected interruption'):
                sync.remote_operation(request)
        self.assertTrue(self.status(wanted)['incomplete'])
        before = self.root / '.local/sync/incoming' / plan['token'] / 'before'
        self.assertEqual((before / 'a.txt').read_bytes(), b'old a')
        self.assertEqual((before / 'b.txt').read_bytes(), b'old b')
        for action in ('prepare', 'apply', 'build', 'test'):
            with self.subTest(action=action), self.assertRaisesRegex(RuntimeError, 'interrupted'):
                sync.remote_operation(self.request(action, wanted, profile='imu'))

    def test_test_rejects_source_snapshot_without_matching_build(self):
        contents = {'AGENTS.md': b'project'}
        self.synchronize(contents)
        with self.assertRaisesRegex(RuntimeError, 'Build this exact source snapshot'):
            sync.remote_operation(self.request('test', manifest(contents), profile='imu'))

    def test_directory_to_file_change_has_same_result_as_clean_checkout(self):
        self.synchronize({'AGENTS.md': b'project', 'package/nested/old.txt': b'old'})
        self.synchronize({'AGENTS.md': b'project', 'package': b'now a file'})
        self.assertEqual((self.root / 'package').read_bytes(), b'now a file')

    def test_file_to_directory_change_has_same_result_as_clean_checkout(self):
        self.synchronize({'AGENTS.md': b'project', 'package': b'old file'})
        self.synchronize({'AGENTS.md': b'project', 'package/nested/new.txt': b'new'})
        self.assertEqual((self.root / 'package/nested/new.txt').read_bytes(), b'new')


if __name__ == '__main__':
    unittest.main(verbosity=2)
