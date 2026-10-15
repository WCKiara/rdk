"""No robot imports, no network calls: validation, guards, and error handling."""
import ast
import hashlib
import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch, Mock

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
from common import validate_ip, validate_serial, positive_seconds, run_command
from network_diagnostics import ping_command, classify, main as network_main
from run_official_state_example import RELATIVE, verify_source, build_command, main, run_child


class OfflineChecks(unittest.TestCase):
    def test_real_serial_format(self):
        self.assertEqual(validate_serial('Enlight-L-654321'), 'Enlight-L-654321')

    def test_serial_placeholders_and_injection_rejected(self):
        for serial in (None, '', 'Enlight-L-123456', 'REPLACE_WITH_ACTUAL_SERIAL', 'Enlight L-654321', 'foo;bar', '../robot'):
            with self.subTest(serial=serial), self.assertRaises(ValueError):
                validate_serial(serial)

    def test_ip_validation(self):
        self.assertEqual(validate_ip('192.168.2.100'), '192.168.2.100')
        for address in ('robot.local', '192.168.999.1', '0.0.0.0', '127.0.0.1', '224.0.0.1', '255.255.255.255', '1.2.3.4;echo'):
            with self.subTest(address=address), self.assertRaises(ValueError):
                validate_ip(address)

    def test_finite_positive_timeouts(self):
        for value in ('nan', 'inf', '-1', '0'):
            with self.assertRaises(ValueError):
                positive_seconds(value)

    def test_ping_commands_are_bounded_argv(self):
        self.assertEqual(ping_command('192.168.2.100', 2, 'Linux'), ['ping', '-4', '-n', '-c', '4', '-W', '2', '192.168.2.100'])
        self.assertEqual(ping_command('192.168.2.100', 2, 'Windows'), ['ping', '-4', '-n', '4', '-w', '2000', '192.168.2.100'])

    def test_failure_diagnostics(self):
        self.assertTrue(classify({'code': 1, 'stderr': 'Network is unreachable'}).startswith('NO_ROUTE'))
        self.assertTrue(classify({'code': 1, 'stdout': '100% packet loss'}).startswith('NO_REPLY'))
        self.assertTrue(classify({'code': None, 'error': 'timeout'}).startswith('NO_REPLY'))
        self.assertTrue(classify({'code': 0}).startswith('PING_SUCCESS'))
        self.assertTrue(classify({'code': 1, 'stderr': 'Operation not permitted'}).startswith('UNAVAILABLE'))

    @patch('common.subprocess.run', side_effect=subprocess.TimeoutExpired(['ping'], 1))
    def test_subprocess_timeout(self, mocked):
        self.assertEqual(run_command(['ping'], 1)['error'], 'timeout')
        self.assertNotIn('shell', mocked.call_args.kwargs)

    @patch('common.subprocess.run', side_effect=FileNotFoundError('missing tool'))
    def test_missing_tool(self, mocked):
        self.assertIn('missing tool', run_command(['missing'])['error'])

    @patch('network_diagnostics.run_command', return_value={'code': 0, 'stdout': '', 'stderr': ''})
    def test_passive_default_never_pings(self, mocked):
        with patch('builtins.print'):
            self.assertEqual(network_main([]), 0)
        self.assertFalse(any(call.args[0][0] == 'ping' for call in mocked.call_args_list))

    @patch('network_diagnostics.run_command', return_value={'code': 1, 'stdout': '', 'stderr': 'Operation not permitted'})
    def test_unavailable_passive_inspection_warns(self, mocked):
        with patch('builtins.print') as printed:
            self.assertEqual(network_main([]), 1)
        self.assertIn('WARN_UNAVAILABLE', printed.call_args.args[0])

    def test_missing_official_example(self):
        with tempfile.TemporaryDirectory() as folder:
            with self.assertRaisesRegex(ValueError, 'Missing official'):
                verify_source(Path(folder))

    def fixture(self, root, code, approved=True):
        source = root / RELATIVE
        source.parent.mkdir(parents=True)
        source.write_text(code)
        (root / 'docs').mkdir()
        (root / 'docs/source_review.json').write_text(json.dumps({
            'path': RELATIVE.as_posix(), 'sha256': hashlib.sha256(source.read_bytes()).hexdigest(),
            'commit': 'reviewed-commit', 'readonly_approved': approved, 'reason': 'ServoOn found'}))
        return source

    def test_unapproved_upstream_is_refused(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            self.fixture(root, 'robot.ServoOn()\n', approved=False)
            with self.assertRaisesRegex(ValueError, 'NOT read-only'):
                verify_source(root)

    def test_changed_source_is_refused(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            source = self.fixture(root, 'robot.states()\n')
            source.write_text('robot.ServoOn()\n')
            with self.assertRaisesRegex(ValueError, 'Source changed'):
                verify_source(root)

    def test_unapproved_api_refused_even_with_matching_hash(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            self.fixture(root, 'robot.ServoOn()\n')
            with self.assertRaisesRegex(ValueError, 'Unapproved robot API'):
                verify_source(root)

    @patch('run_official_state_example.subprocess.run')
    def test_command_uses_same_interpreter_and_exact_script(self, mocked):
        mocked.return_value = Mock(stdout='reviewed-commit\n')
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            source = self.fixture(root, 'robot.states()\n')
            self.assertEqual(build_command('Enlight-L-654321', root), [sys.executable, '-u', str(source), 'Enlight-L-654321'])

    @patch('run_official_state_example.subprocess.run')
    def test_changed_commit_is_refused(self, mocked):
        mocked.return_value = Mock(stdout='different-commit\n')
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            self.fixture(root, 'robot.states()\n')
            with self.assertRaisesRegex(ValueError, 'commit changed'):
                verify_source(root)

    @patch('run_official_state_example.subprocess.Popen')
    def test_hardware_opt_in_required(self, popen):
        with patch('sys.stderr'):
            self.assertEqual(main(['--serial', 'Enlight-L-654321']), 2)
        popen.assert_not_called()

    @patch('run_official_state_example.subprocess.Popen')
    def test_timeout_reaps_child(self, popen):
        child = popen.return_value
        child.wait.side_effect = [subprocess.TimeoutExpired(['python'], 1), 0]
        with patch('builtins.print'):
            self.assertEqual(run_child(['python', 'example.py'], ROOT, 1), 0)
        self.assertEqual(child.wait.call_count, 2)
        self.assertTrue(child.send_signal.called or child.terminate.called)

    def test_created_scripts_have_no_actuation_calls(self):
        for path in (ROOT / 'scripts').glob('*.py'):
            allowed = {'info', 'states', 'connected'}
            if path.name == 'test_request_reply.py':
                # 已有只读请求/回复诊断使用的两个 getter；继续禁止任何运动或 setter。
                allowed |= {'mode', 'global_variables'}
            for node in ast.walk(ast.parse(path.read_text())):
                if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute) and isinstance(node.func.value, ast.Name) and node.func.value.id == 'robot':
                    self.assertIn(node.func.attr, allowed, str(path))


if __name__ == '__main__':
    unittest.main()
