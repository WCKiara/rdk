"""纯离线测试：模拟 SDK，检查单臂选择、位置边界、姿态规划及停止；不连接机器人。"""
import importlib.util
import math
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import numpy as np
from scipy.spatial.transform import Rotation

SOURCE = (Path(__file__).resolve().parents[1] / 'third_party/flexiv_rdk/example_py'
          / 'intermediate4_non_realtime_cartesian_motion_force_control.py')
spec = importlib.util.spec_from_file_location('single_arm_cartesian', SOURCE)
motion = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = motion
spec.loader.exec_module(motion)

START = [0.40, 0.20, 0.12, 1.0, 0.0, 0.0, 0.0]


class Clock:
    def __init__(self):
        self.now = 0.0

    def monotonic(self):
        return self.now

    def sleep(self, seconds):
        self.now += seconds


class FakeRobot:
    def __init__(self):
        self.poses = {'ARM_1': START.copy(), 'ARM_2': [0.4, -0.2, 0.12, 1, 0, 0, 0]}
        self.current_mode = 'IDLE'
        self.calls = []
        self.home_started = False
        self.home_polls = 0
        self.violate_on_motion = False
        self.interrupt_on_motion = False
        self.never_reach = False
        self.home_failure = False
        self.fault_status = None
        self.clear_fault_succeeds = True
        self.active_primitive = 'Home'
        self.active_group = 'ARM_1'
        self.movel_failure = False
        self.tcp_wrench = [0.0] * 6

    def connected(self):
        return True

    def fault(self):
        return self.fault_status is not None

    def operational_status(self):
        return self.fault_status or 'READY'

    def ClearFault(self, timeout):
        self.calls.append(('ClearFault', timeout))
        if self.clear_fault_succeeds:
            self.fault_status = None
        return self.clear_fault_succeeds

    def operational(self):
        return True

    def mode(self):
        return self.current_mode

    def all_stopped(self):
        return True

    def info(self):
        return SimpleNamespace(single_arm_groups=['ARM_1', 'ARM_2'])

    def states(self):
        if self.violate_on_motion and any(c[0] in ('Send', 'Home', 'MoveL') for c in self.calls):
            self.poses['ARM_1'][0] = 0.7
        return {g: SimpleNamespace(tcp_pose=p.copy(), tcp_wrench=self.tcp_wrench.copy())
                for g, p in self.poses.items()}

    def SwitchMode(self, mode):
        self.calls.append(('SwitchMode', mode))
        self.current_mode = mode

    def SetForceControlFrame(self, group, frame):
        self.calls.append(('Frame', group, frame))

    def SetForceControlAxis(self, group, axes):
        self.calls.append(('Axes', group, axes))

    def SetMaxContactWrench(self, group, wrench):
        self.calls.append(('Wrench', group, wrench))

    def SendCartesianMotionForce(self, commands):
        self.calls.append(('Send', commands))
        if self.interrupt_on_motion:
            raise KeyboardInterrupt()
        if not self.never_reach:
            for group, command in commands.items():
                self.poses[group] = list(command.pose_d)

    def ExecutePrimitive(self, commands, wait_started):
        group, primitive = next(iter(commands.items()))
        self.active_primitive = primitive.pt_name
        self.active_group = group
        self.home_polls = 0
        self.calls.append((primitive.pt_name, commands, wait_started))
        if primitive.pt_name == 'MoveL':
            if self.interrupt_on_motion:
                raise KeyboardInterrupt()
            if not self.never_reach:
                target = primitive.params['target']
                qx, qy, qz, qw = Rotation.from_euler('xyz', target.orientation, degrees=True).as_quat()
                self.poses[group] = [*target.position, qw, qx, qy, qz]
        else:
            self.home_started = True

    def primitive_states(self):
        self.home_polls += 1
        failure = self.home_failure if self.active_primitive == 'Home' else self.movel_failure
        reached = self.home_polls >= 2 and not self.never_reach and not failure
        return {self.active_group: SimpleNamespace(pt_name=self.active_primitive, names_and_values={
            'reachedTarget': int(reached), 'terminated': int(failure)})}

    def Stop(self):
        self.calls.append(('Stop',))
        self.current_mode = 'IDLE'


def fake_sdk(robot):
    return SimpleNamespace(
        Robot=Mock(return_value=robot),
        JointGroup=SimpleNamespace(ARM_1='ARM_1', ARM_2='ARM_2'),
        Mode=SimpleNamespace(IDLE='IDLE', NRT_PRIMITIVE_EXECUTION='PRIMITIVE',
                             NRT_CARTESIAN_MOTION_FORCE='CARTESIAN'),
        CoordType=SimpleNamespace(WORLD='WORLD'),
        OperationalStatus=SimpleNamespace(MINOR_FAULT='MINOR_FAULT'),
        Coord=lambda position, orientation, ref_frame: SimpleNamespace(
            position=position, orientation=orientation, ref_frame=ref_frame),
        PrimitiveArgs=lambda name, params: SimpleNamespace(pt_name=name, params=params),
        NrtCartesianCmd=lambda pose, wrench, twist, *limits: SimpleNamespace(
            pose_d=pose, wrench_d=wrench, twist_d=twist, limits=limits),
    )


class SingleArmCartesianTests(unittest.TestCase):
    def args(self, *extra):
        return motion.build_parser().parse_args(['Enlight-L-test', '50', *extra])

    def run_robot(self, args, robot):
        clock = Clock()
        with patch.object(motion.time, 'monotonic', clock.monotonic), \
             patch.object(motion.time, 'sleep', clock.sleep):
            motion.run(args, sdk=fake_sdk(robot), logger=Mock())

    def test_diagonal_and_relative_tool_tilt_from_rotated_start(self):
        qx, qy, qz, qw = Rotation.from_euler('z', 90, degrees=True).as_quat()
        start = [0.4, 0.2, 0.12, qw, qx, qy, qz]
        args = self.args('--delta', '0.01', '0', '-0.01', '--tilt', '0', '15', '0')
        plan = motion.build_plan(start, args, motion.validate_args(args))
        np.testing.assert_allclose(plan.target[:3], [0.41, 0.2, 0.11])
        expected = Rotation.from_euler('z', 90, degrees=True) * Rotation.from_euler('y', 15, degrees=True)
        self.assertLess((expected.inv() * motion.pose_rotation(plan.target)).magnitude(), 1e-10)
        midpoint = plan.sample(plan.duration / 2)
        np.testing.assert_allclose(midpoint[:3], [0.405, 0.2, 0.115])
        self.assertAlmostEqual((motion.pose_rotation(start).inv() * motion.pose_rotation(midpoint)).magnitude(),
                               math.radians(7.5))
        np.testing.assert_allclose(plan.sample(plan.duration), plan.target, atol=1e-12)

    def test_absolute_position_and_orientation(self):
        args = self.args('--position', '0.41', '0.21', '0.13', '--rpy', '10', '20', '30')
        plan = motion.build_plan(START, args, motion.validate_args(args))
        np.testing.assert_allclose(plan.target[:3], [0.41, 0.21, 0.13])
        expected = Rotation.from_euler('xyz', [10, 20, 30], degrees=True)
        self.assertLess((expected.inv() * motion.pose_rotation(plan.target)).magnitude(), 1e-10)

    def test_limits_and_nonfinite_inputs_rejected_before_connect(self):
        for flags in [('--delta', 'nan', '0', '0'),
                      ('--home-only', '--position-min', '1', '1', '1'),
                      ('--home-only', '--limit-margin', '-1'),
                      ('--home-only', '--max-linear-vel', 'inf')]:
            sdk = fake_sdk(FakeRobot())
            with self.subTest(flags=flags), self.assertRaises(ValueError):
                motion.run(self.args(*flags), sdk=sdk)
            sdk.Robot.assert_not_called()

    def test_minor_fault_without_opt_in_is_reported_without_robot_writes(self):
        robot = FakeRobot()
        robot.fault_status = 'MINOR_FAULT'
        with self.assertRaisesRegex(RuntimeError, 'MINOR_FAULT'):
            self.run_robot(self.args('--delta', '0.01', '0', '0'), robot)
        self.assertEqual(robot.calls, [])

    def test_startup_minor_fault_is_cleared_once_before_movement(self):
        robot = FakeRobot()
        robot.fault_status = 'MINOR_FAULT'
        self.run_robot(self.args('--delta', '0.01', '0', '0', '--clear-minor-fault'), robot)
        self.assertEqual(robot.calls[0], ('ClearFault', 5))
        self.assertEqual(sum(c[0] == 'ClearFault' for c in robot.calls), 1)
        self.assertTrue(any(c[0] == 'Send' for c in robot.calls))
        self.assertEqual(robot.calls[-1], ('Stop',))

    def test_critical_fault_or_failed_minor_recovery_never_moves(self):
        for status, succeeds in [('CRITICAL_FAULT', True), ('MINOR_FAULT', False)]:
            robot = FakeRobot()
            robot.fault_status = status
            robot.clear_fault_succeeds = succeeds
            with self.subTest(status=status), self.assertRaises(RuntimeError):
                self.run_robot(self.args('--delta', '0.01', '0', '0', '--clear-minor-fault'), robot)
            self.assertFalse(any(c[0] in ('SwitchMode', 'Home', 'Send') for c in robot.calls))
            if status == 'CRITICAL_FAULT':
                self.assertEqual(robot.calls, [])

    def test_fault_during_motion_stops_without_recovery_or_home(self):
        robot = FakeRobot()
        robot.fault = lambda: any(c[0] == 'Send' for c in robot.calls)
        with self.assertRaisesRegex(RuntimeError, '机器人报告故障'):
            self.run_robot(self.args('--delta', '0.01', '0', '0', '--clear-minor-fault', '--return-home'), robot)
        self.assertFalse(any(c[0] in ('ClearFault', 'Home') for c in robot.calls))
        self.assertEqual(robot.calls[-1], ('Stop',))

    def test_preview_never_clears_a_fault(self):
        sdk = fake_sdk(FakeRobot())
        with self.assertRaisesRegex(ValueError, '只读'):
            motion.run(self.args('--home-only', '--preview', '--clear-minor-fault'), sdk=sdk)
        sdk.Robot.assert_not_called()

    def test_disconnection_has_its_own_message(self):
        robot = FakeRobot()
        robot.connected = lambda: False
        with self.assertRaisesRegex(RuntimeError, '连接已断开'):
            self.run_robot(self.args('--home-only'), robot)
        self.assertEqual(robot.calls, [])

    def test_target_outside_limits_prevents_every_robot_write(self):
        robot = FakeRobot()
        with self.assertRaises(ValueError):
            self.run_robot(self.args('--delta', '0.2', '0', '0'), robot)
        self.assertEqual(robot.calls, [])

    def test_rejects_current_position_inside_stop_margin(self):
        robot = FakeRobot()
        robot.poses['ARM_1'][0] = 0.351
        with self.assertRaises(ValueError):
            self.run_robot(self.args('--home-only'), robot)
        self.assertEqual(robot.calls, [])

    def test_new_z_floor_is_fixed_midpoint_and_blocks_old_low_position(self):
        expected = (motion.Z_BEFORE_HOME + motion.Z_AT_LIMIT_UPDATE) / 2
        self.assertAlmostEqual(motion.POSITION_MIN[2], expected)
        for z in [motion.Z_BEFORE_HOME, expected - 0.001]:
            robot = FakeRobot()
            robot.poses['ARM_1'][2] = z
            with self.subTest(z=z), self.assertRaises(ValueError):
                self.run_robot(self.args('--home-only'), robot)
            self.assertEqual(robot.calls, [])

    def test_downward_target_crossing_midpoint_is_rejected_before_motion(self):
        robot = FakeRobot()
        with self.assertRaises(ValueError):
            self.run_robot(self.args('--delta', '0', '0', '-0.08'), robot)
        self.assertEqual(robot.calls, [])

    def test_only_selected_arm_gets_motion_and_always_stops(self):
        robot = FakeRobot()
        other_before = robot.poses['ARM_2'].copy()
        self.run_robot(self.args('--delta', '0.01', '0', '-0.01', '--tilt', '0', '15', '0'), robot)
        commands = [c[1] for c in robot.calls if c[0] == 'Send']
        self.assertEqual(len(commands), 1)  # 非实时运动只发最终目标一次，禁止每周期重发中间目标。
        np.testing.assert_allclose(commands[0]['ARM_1'].pose_d[:3], [0.41, 0.2, 0.11])
        self.assertTrue(all(set(cmd) == {'ARM_1'} for cmd in commands))
        self.assertEqual(robot.poses['ARM_2'], other_before)
        self.assertIn(('Axes', 'ARM_1', [False] * 6), robot.calls)
        self.assertEqual(robot.calls[-1], ('Stop',))
        self.assertTrue(all(cmd['ARM_1'].wrench_d == [0] * 6 for cmd in commands))

    def test_arm2_selection_requires_its_own_limits(self):
        robot = FakeRobot()
        args = self.args('--arm', 'ARM_2', '--delta', '0.01', '0', '-0.01',
                         '--position-min', '0.35', '-0.25', '-0.06',
                         '--position-max', '0.46', '-0.15', '0.18')
        self.run_robot(args, robot)
        self.assertTrue(all(set(c[1]) == {'ARM_2'} for c in robot.calls if c[0] == 'Send'))
        self.assertEqual(robot.poses['ARM_1'], START)

    def test_preview_has_no_robot_writes(self):
        robot = FakeRobot()
        self.run_robot(self.args('--delta', '0.01', '0', '0', '--preview'), robot)
        self.assertEqual(robot.calls, [])

    def test_successful_motion_stops_then_returns_only_selected_arm_home(self):
        robot = FakeRobot()
        other_before = robot.poses['ARM_2'].copy()
        self.run_robot(self.args('--delta', '0.01', '0', '-0.01', '--tilt', '0', '15', '0',
                                 '--return-home'), robot)
        sends = [i for i, c in enumerate(robot.calls) if c[0] == 'Send']
        homes = [(i, c) for i, c in enumerate(robot.calls) if c[0] == 'Home']
        stops = [i for i, c in enumerate(robot.calls) if c[0] == 'Stop']
        self.assertEqual(len(homes), 1)
        self.assertLess(sends[-1], stops[0])
        self.assertLess(stops[0], homes[0][0])
        self.assertLess(homes[0][0], stops[-1])
        self.assertEqual(set(homes[0][1][1]), {'ARM_1'})
        self.assertEqual(robot.poses['ARM_2'], other_before)
        self.assertEqual(robot.calls[-1], ('Stop',))

    def test_failed_or_interrupted_motion_does_not_return_home(self):
        for failure, exception in [('interrupt_on_motion', KeyboardInterrupt),
                                   ('violate_on_motion', ValueError),
                                   ('never_reach', TimeoutError)]:
            robot = FakeRobot()
            setattr(robot, failure, True)
            with self.subTest(failure=failure), self.assertRaises(exception):
                self.run_robot(self.args('--delta', '0.01', '0', '-0.01', '--return-home'), robot)
            self.assertFalse(any(c[0] == 'Home' for c in robot.calls))
            self.assertEqual(robot.calls[-1], ('Stop',))

    def test_return_home_preview_has_no_robot_writes(self):
        robot = FakeRobot()
        self.run_robot(self.args('--delta', '0.01', '0', '-0.01', '--return-home', '--preview'), robot)
        self.assertEqual(robot.calls, [])

    def test_movel_sends_one_world_target_with_degree_angles_then_returns_home(self):
        robot = FakeRobot()
        other_before = robot.poses['ARM_2'].copy()
        self.run_robot(self.args('--delta', '0.01', '0', '-0.01', '--tilt', '0', '15', '0',
                                 '--motion-method', 'movel', '--return-home'), robot)
        moves = [c for c in robot.calls if c[0] == 'MoveL']
        self.assertEqual(len(moves), 1)
        self.assertEqual(set(moves[0][1]), {'ARM_1'})
        params = moves[0][1]['ARM_1'].params
        np.testing.assert_allclose(params['target'].position, [0.41, 0.2, 0.11])
        np.testing.assert_allclose(params['target'].orientation, [0, 15, 0], atol=1e-10)
        self.assertEqual(params['target'].ref_frame, ['WORLD', 'WORLD_ORIGIN'])
        self.assertAlmostEqual(params['angVel'], math.degrees(0.15))
        self.assertEqual(params['vel'], 0.01)
        self.assertEqual(params['acc'], 0.1)
        self.assertEqual(params['zoneRadius'], 'ZFine')
        self.assertFalse(any(c[0] in ('Send', 'Axes', 'Frame', 'Wrench') for c in robot.calls))
        names = [c[0] for c in robot.calls]
        self.assertLess(names.index('MoveL'), names.index('Stop'))
        self.assertLess(names.index('Stop'), names.index('Home'))
        self.assertEqual(names[-1], 'Stop')
        self.assertEqual(robot.poses['ARM_2'], other_before)

    def test_movel_guard_failures_stop_without_home(self):
        for failure, exception in [('interrupt_on_motion', KeyboardInterrupt),
                                   ('violate_on_motion', ValueError),
                                   ('never_reach', TimeoutError),
                                   ('movel_failure', RuntimeError)]:
            robot = FakeRobot()
            setattr(robot, failure, True)
            with self.subTest(failure=failure), self.assertRaises(exception):
                self.run_robot(self.args('--delta', '0.01', '0', '-0.01', '--motion-method', 'movel',
                                         '--return-home'), robot)
            self.assertEqual(sum(c[0] == 'MoveL' for c in robot.calls), 1)
            self.assertFalse(any(c[0] == 'Home' for c in robot.calls))
            self.assertEqual(robot.calls[-1], ('Stop',))

    def test_movel_force_threshold_exceeded_stops(self):
        robot = FakeRobot()
        original_states = robot.states
        def excessive_force():
            if any(c[0] == 'MoveL' for c in robot.calls):
                robot.tcp_wrench = [11.0, 0, 0, 0, 0, 0]
            return original_states()
        robot.states = excessive_force
        with self.assertRaisesRegex(RuntimeError, '软件接触阈值'):
            self.run_robot(self.args('--delta', '0.01', '0', '-0.01', '--motion-method', 'movel',
                                     '--return-home'), robot)
        self.assertEqual(sum(c[0] == 'MoveL' for c in robot.calls), 1)
        self.assertFalse(any(c[0] == 'Home' for c in robot.calls))
        self.assertEqual(robot.calls[-1], ('Stop',))

    def test_movel_invalid_primitive_ranges_rejected_before_connection(self):
        for flags in [('--movel-acc', '0.05'), ('--max-angular-vel', '0.001'),
                      ('--max-linear-vel', '0.0001')]:
            sdk = fake_sdk(FakeRobot())
            args = self.args('--delta', '0.01', '0', '0', '--motion-method', 'movel', *flags)
            with self.subTest(flags=flags), self.assertRaises(ValueError):
                motion.run(args, sdk=sdk)
            sdk.Robot.assert_not_called()

    def test_movel_preview_sends_no_commands(self):
        robot = FakeRobot()
        self.run_robot(self.args('--delta', '0.01', '0', '0', '--motion-method', 'movel',
                                 '--return-home', '--preview'), robot)
        self.assertEqual(robot.calls, [])

    def test_return_home_termination_failure_stops_and_does_not_retry(self):
        robot = FakeRobot()
        robot.home_failure = True
        with self.assertRaisesRegex(RuntimeError, '没有确认'):
            self.run_robot(self.args('--delta', '0.01', '0', '-0.01', '--return-home'), robot)
        self.assertEqual(sum(c[0] == 'Home' for c in robot.calls), 1)
        self.assertEqual(robot.calls[-1], ('Stop',))

    def test_home_only_with_return_home_is_rejected_before_connect(self):
        sdk = fake_sdk(FakeRobot())
        with self.assertRaises(ValueError):
            motion.run(self.args('--home-only', '--return-home'), sdk=sdk)
        sdk.Robot.assert_not_called()

    def test_live_boundary_violation_stops_home_and_motion(self):
        for flags in [('--home-only',), ('--delta', '0.01', '0', '-0.01')]:
            robot = FakeRobot()
            robot.violate_on_motion = True
            with self.subTest(flags=flags), self.assertRaises(ValueError):
                self.run_robot(self.args(*flags), robot)
            self.assertEqual(robot.calls[-1], ('Stop',))
            if not flags[0] == '--home-only':
                self.assertEqual(sum(c[0] == 'Send' for c in robot.calls), 1)

    def test_keyboard_interrupt_stops(self):
        robot = FakeRobot()
        robot.interrupt_on_motion = True
        with self.assertRaises(KeyboardInterrupt):
            self.run_robot(self.args('--delta', '0.01', '0', '0'), robot)
        self.assertEqual(robot.calls[-1], ('Stop',))

    def test_unreached_home_and_motion_timeout_stop(self):
        for flags in [('--home-only',), ('--delta', '0.01', '0', '0')]:
            robot = FakeRobot()
            robot.never_reach = True
            with self.subTest(flags=flags), self.assertRaises(TimeoutError):
                self.run_robot(self.args(*flags), robot)
            self.assertEqual(robot.calls[-1], ('Stop',))

    def test_home_only_never_sends_cartesian_motion_or_arm2_home(self):
        robot = FakeRobot()
        self.run_robot(self.args('--home-only'), robot)
        homes = [c for c in robot.calls if c[0] == 'Home']
        self.assertEqual(len(homes), 1)
        self.assertEqual(set(homes[0][1]), {'ARM_1'})
        self.assertEqual(homes[0][1]['ARM_1'].pt_name, 'Home')
        self.assertTrue(homes[0][2])
        self.assertFalse(any(c[0] == 'Send' for c in robot.calls))
        self.assertEqual(robot.calls[-1], ('Stop',))

    def test_home_terminated_without_reaching_is_failure(self):
        robot = FakeRobot()
        robot.home_failure = True
        with self.assertRaisesRegex(RuntimeError, '没有确认'):
            self.run_robot(self.args('--home-only'), robot)
        self.assertEqual(robot.calls[-1], ('Stop',))

    def test_trajectory_speed_and_acceleration_limits(self):
        args = self.args('--delta', '0.01', '0.01', '0.01', '--tilt', '0', '30', '0', '--duration', '0.1')
        plan = motion.build_plan(START, args, motion.validate_args(args))
        samples = np.array([plan.sample(t)[:3] for t in np.linspace(0, plan.duration, 1001)])
        dt = plan.duration / 1000
        speed = np.linalg.norm(np.diff(samples, axis=0) / dt, axis=1)
        acc = np.linalg.norm(np.diff(samples, n=2, axis=0) / dt**2, axis=1)
        self.assertLessEqual(speed.max(), args.max_linear_vel + 1e-8)
        self.assertLessEqual(acc.max(), args.max_linear_acc + 1e-8)
        rotation_angle = math.radians(30)
        self.assertLessEqual(1.875 * rotation_angle / plan.duration, args.max_angular_vel + 1e-10)
        self.assertLessEqual(10 / math.sqrt(3) * rotation_angle / plan.duration**2, args.max_angular_acc + 1e-10)


if __name__ == '__main__':
    unittest.main()
