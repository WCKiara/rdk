"""Offline supervisor checks: readiness, recorder failure and child cleanup."""

import csv
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import Mock, patch


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
import run_realtime_cartesian_with_contact as launcher


class RealtimeContactLauncherTests(unittest.TestCase):
    def args(self, *extra):
        return launcher.parse_args(["Enlight-LL-654321", "50", "--delta", "0.01", "0", "-0.01",
                                    "--tilt", "0", "15", "0", "--return-home", *extra])

    def child(self, pid, poll_values):
        child = Mock(pid=pid)
        child.poll.side_effect = poll_values
        return child

    def test_arm2_rejected_before_starting_any_process(self):
        with patch.object(launcher.subprocess, "Popen") as spawn, patch("sys.stderr"):
            with self.assertRaises(SystemExit):
                launcher.main(["Enlight-LL-654321", "--arm", "ARM_2"])
        spawn.assert_not_called()

    def test_motion_arguments_keep_frequency_delta_and_tilt(self):
        args = self.args("--contact-rate", "25", "--record-contact")
        command = launcher.motion_command(args)
        self.assertEqual(command[1:3], ["Enlight-LL-654321", "50"])
        self.assertEqual(command[-2:], ["--arm", "ARM_1"])
        self.assertIn("--return-home", command)
        self.assertNotIn("--contact-rate", command)
        self.assertIn("-0.01", command)
        self.assertEqual(args.contact_rate, 25)

    def test_separator_cannot_hide_arm2(self):
        with patch("sys.stderr"), self.assertRaises(SystemExit):
            self.args("--", "--arm", "ARM_2")

    def test_contact_rate_must_be_finite_positive(self):
        for value in ("0", "-1", "inf", "nan"):
            with self.subTest(value=value), patch("sys.stderr"), self.assertRaises(SystemExit):
                self.args("--contact-rate", value)

    def test_ready_requires_finite_arm1_csv_data(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "contact.csv"
            header = ["sample", "joint_group", "link_index", "force_x_N", "force_y_N", "force_z_N",
                      "position_x_m", "position_y_m", "position_z_m"]
            for arm, fx, expected in (("ARM_2", "1", False), ("ARM_1", "nan", False), ("ARM_1", "1", True)):
                with output.open("w", newline="") as stream:
                    writer = csv.writer(stream)
                    writer.writerow(header)
                    writer.writerow([0, arm, 0, fx, 0, 0, 0, 0, 0])
                self.assertEqual(launcher.has_contact_row(output), expected)

    def test_recorder_failure_before_first_sample_never_starts_motion(self):
        args = self.args()
        recorder = self.child(123, [7, 7])
        with patch.object(launcher.subprocess, "Popen", return_value=recorder) as spawn, \
             patch.object(launcher, "recorder_identity", return_value={}), \
             self.assertRaisesRegex(RuntimeError, "before motion"):
            launcher.supervise(args, launcher.StopState())
        self.assertEqual(spawn.call_count, 1)

    def test_recorder_timeout_never_starts_motion(self):
        args = self.args()
        recorder = Mock(pid=123)
        recorder.poll.return_value = None
        with patch.object(launcher.subprocess, "Popen", return_value=recorder) as spawn, \
             patch.object(launcher, "recorder_identity", return_value={}), \
             patch.object(launcher, "has_contact_row", return_value=False), \
             patch.object(launcher.time, "monotonic", side_effect=[0, 16]), \
             patch.object(launcher.os, "killpg") as killpg, \
             self.assertRaisesRegex(RuntimeError, "valid ARM_1 data row"):
            launcher.supervise(args, launcher.StopState())
        self.assertEqual(spawn.call_count, 1)
        killpg.assert_called_once_with(123, signal.SIGTERM)

    def test_recorder_failure_during_motion_stops_and_reaps_motion(self):
        args = self.args()
        recorder = self.child(123, [None, 9, 9])
        motion = Mock(pid=456)
        motion.poll.return_value = None
        with patch.object(launcher.subprocess, "Popen", side_effect=[recorder, motion]) as spawn, \
             patch.object(launcher, "recorder_identity", return_value={}), \
             patch.object(launcher, "has_contact_row", return_value=True), \
             patch.object(launcher.os, "killpg") as killpg, \
             patch("builtins.print"), self.assertRaisesRegex(RuntimeError, "during motion"):
            launcher.supervise(args, launcher.StopState())
        self.assertEqual(spawn.call_count, 2)
        killpg.assert_called_once_with(456, signal.SIGINT)
        motion.wait.assert_called_once_with(timeout=5)

    def test_success_stops_recorder_and_returns_motion_status(self):
        args = self.args()
        recorder = Mock(pid=123)
        recorder.poll.return_value = None
        motion = self.child(456, [0, 0])
        with patch.object(launcher.subprocess, "Popen", side_effect=[recorder, motion]), \
             patch.object(launcher, "recorder_identity", return_value={}), \
             patch.object(launcher, "has_contact_row", return_value=True), \
             patch.object(launcher.os, "killpg") as killpg, patch("builtins.print"):
            self.assertEqual(launcher.supervise(args, launcher.StopState()), 0)
        killpg.assert_called_once_with(123, signal.SIGTERM)

    def test_preview_never_starts_recorder(self):
        motion = self.child(456, [0, 0])
        with patch.object(launcher.subprocess, "Popen", return_value=motion) as spawn, \
             patch.object(launcher, "recorder_identity") as identity:
            self.assertEqual(launcher.supervise(self.args("--preview"), launcher.StopState()), 0)
        self.assertEqual(spawn.call_count, 1)
        self.assertIn("--preview", spawn.call_args.args[0])
        identity.assert_not_called()

    def test_interruption_after_spawn_cleans_up_motion(self):
        stop = launcher.StopState()
        motion = Mock(pid=456)
        motion.poll.return_value = None

        def spawned(*_args, **_kwargs):
            stop.request(signal.SIGTERM)
            return motion

        with patch.object(launcher.subprocess, "Popen", side_effect=spawned), \
             patch.object(launcher.os, "killpg") as killpg, self.assertRaises(launcher.StopRequested):
            launcher.supervise(self.args("--preview"), stop)
        killpg.assert_called_once_with(456, signal.SIGINT)

    def test_stubborn_motion_is_terminated_and_reaped(self):
        child = Mock(pid=456)
        child.poll.return_value = None
        child.wait.side_effect = [subprocess.TimeoutExpired("fake", 5), 0]
        with patch.object(launcher.os, "killpg") as killpg:
            launcher.stop_process(child)
        self.assertEqual([call.args for call in killpg.call_args_list],
                         [(456, signal.SIGINT), (456, signal.SIGTERM)])
        self.assertEqual(child.wait.call_count, 2)

    def test_existing_csv_is_not_overwritten_or_used_as_readiness(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "previous.csv"
            output.write_text("previous data")
            with patch.object(launcher.subprocess, "Popen") as spawn, \
                 self.assertRaisesRegex(ValueError, "already exists"):
                launcher.supervise(self.args("--contact-output", str(output)), launcher.StopState())
            spawn.assert_not_called()

    def test_recorder_uses_venv_interpreter_and_arm1(self):
        command = launcher.recorder_command(self.args(), Path("/tmp/contact.csv"))
        self.assertEqual(command[0], sys.executable)
        self.assertEqual(command[command.index("--arm") + 1], "ARM_1")
        self.assertEqual(command[-1], "/tmp/contact.csv")

    def test_recorder_drops_to_original_sudo_user(self):
        account = Mock(pw_gid=1001, pw_name="operator")
        with patch.object(launcher.os, "geteuid", return_value=0), \
             patch.dict(launcher.os.environ, {"SUDO_UID": "1001"}), \
             patch.object(launcher.pwd, "getpwuid", return_value=account), \
             patch.object(launcher.os, "getgrouplist", return_value=[1001, 1002]):
            self.assertEqual(launcher.recorder_identity(),
                             {"user": 1001, "group": 1001, "extra_groups": [1001, 1002]})

    def test_real_local_children_are_reaped_without_hardware(self):
        # The fixtures contain no SDK import or network operation. This checks
        # process sessions and actual group signals beyond mocked Popen calls.
        with tempfile.TemporaryDirectory() as directory:
            folder = Path(directory)
            recorder = folder / "fake_recorder.py"
            recorder.write_text(
                "import os, sys, time\nfrom pathlib import Path\n"
                "p = Path(sys.argv[sys.argv.index('--output') + 1])\n"
                "p.with_suffix('.pid').write_text(str(os.getpid()))\n"
                "p.write_text('sample,joint_group,link_index,force_x_N,force_y_N,force_z_N,position_x_m,position_y_m,position_z_m\\n0,ARM_1,0,0,0,0,0,0,0\\n')\n"
                "while True: time.sleep(1)\n")
            motion = folder / "fake_motion"
            motion.write_text(f"#!{sys.executable}\nraise SystemExit(7)\n")
            motion.chmod(0o755)
            output = folder / "contact.csv"
            args = self.args("--rt-binary", str(motion), "--contact-output", str(output))
            with patch.object(launcher, "RECORDER", recorder), \
                 patch.object(launcher, "recorder_identity", return_value={}), patch("builtins.print"):
                self.assertEqual(launcher.supervise(args, launcher.StopState()), 7)
            recorder_pid = int(output.with_suffix(".pid").read_text())
            with self.assertRaises(ProcessLookupError):
                os.kill(recorder_pid, 0)


if __name__ == "__main__":
    unittest.main()
