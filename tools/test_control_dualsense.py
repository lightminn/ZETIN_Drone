"""Regression tests for the ground-station and bench failsafe review fixes."""

import builtins
import contextlib
import importlib.util
import io
import pathlib
import socket
import sys
import threading
import types
import unittest
from unittest import mock


REPO_ROOT = pathlib.Path(__file__).resolve().parents[1]
SCRIPTS_DIR = REPO_ROOT / "scripts"
CONTROL_PATH = SCRIPTS_DIR / "control_dualsense.py"


class _LoopComplete(BaseException):
    pass


class _FakeSocket:
    def __init__(self, packets=()):
        self.packets = iter(packets)
        self.sent = []

    def bind(self, _address):
        pass

    def settimeout(self, _timeout):
        pass

    def setsockopt(self, *_args):
        pass

    def sendto(self, data, address):
        self.sent.append((data, address))

    def recvfrom(self, _size):
        try:
            return next(self.packets), ("192.168.4.1", 4210)
        except StopIteration:
            raise _LoopComplete

    def close(self):
        pass


class _FakeThread:
    def __init__(self, *_, **__):
        pass

    def start(self):
        pass


class _LoopEvent:
    def __init__(self, iterations):
        self.iterations = iterations
        self.calls = 0

    def pump(self):
        self.calls += 1
        if self.calls > self.iterations:
            raise _LoopComplete


class _Joystick:
    def __init__(self, event, *, start=False, reset=False, hats=None):
        self.event = event
        self.start = start
        self.reset = reset
        self.hats = hats or [(0, 0)]

    def init(self):
        pass

    def get_name(self):
        return "test joystick"

    def get_button(self, index):
        if index == 0:
            return self.start
        if index == 12:
            return self.reset
        return False

    def get_axis(self, _index):
        return 0.0

    def get_hat(self, _index):
        loop_index = max(0, self.event.calls - 1)
        return self.hats[min(loop_index, len(self.hats) - 1)]


def _pygame_for(joystick, event):
    pygame = types.ModuleType("pygame")
    pygame.init = lambda: None
    pygame.event = event
    pygame.joystick = types.SimpleNamespace(
        init=lambda: None,
        get_count=lambda: 1,
        Joystick=lambda _index: joystick,
    )
    return pygame


def _telemetry_sample(**overrides):
    sample = {
        "Roll": 0.0,
        "Pitch": 0.0,
        "Yaw": 0.0,
        "Gyro_Z": 0.0,
        "Throttle": 1000,
        "Fault_RC": 0,
        "Fault_Critical": 0,
        "RC_Total_Pkts": 0,
        "RC_Dropped_Pkts": 0,
        "Armed": 1,
        "Trim_Roll": 0.0,
        "Trim_Pitch": 0.0,
    }
    sample.update(overrides)
    return sample


class ControlDualsenseRegressionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        sys.path.insert(0, str(SCRIPTS_DIR))

    @classmethod
    def tearDownClass(cls):
        sys.path.remove(str(SCRIPTS_DIR))

    def setUp(self):
        module_name = "_control_dualsense_under_test"
        sys.modules.pop(module_name, None)
        spec = importlib.util.spec_from_file_location(module_name, CONTROL_PATH)
        module = importlib.util.module_from_spec(spec)
        sys.modules[module_name] = module

        fake_pygame = types.ModuleType("pygame")
        fake_socket = _FakeSocket()
        fake_csv_file = io.StringIO()
        with (
            mock.patch.dict(sys.modules, {"pygame": fake_pygame}),
            mock.patch.object(socket, "socket", return_value=fake_socket),
            mock.patch.object(pathlib.Path, "mkdir"),
            mock.patch.object(pathlib.Path, "open", return_value=fake_csv_file),
            mock.patch.object(threading, "Thread", _FakeThread),
            mock.patch.object(builtins, "input", side_effect=KeyboardInterrupt),
            contextlib.redirect_stdout(io.StringIO()),
        ):
            spec.loader.exec_module(module)
        self.module = module

    def tearDown(self):
        sys.modules.pop(self.module.__name__, None)

    def _run_controller(self, *, iterations, start=False, reset=False, hats=None):
        event = _LoopEvent(iterations)
        joystick = _Joystick(
            event,
            start=start,
            reset=reset,
            hats=hats,
        )
        self.module.pygame = _pygame_for(joystick, event)
        with (
            mock.patch.object(self.module.time, "sleep", return_value=None),
            contextlib.redirect_stdout(io.StringIO()),
            self.assertRaises(_LoopComplete),
        ):
            self.module.controller_thread()

    def _run_telemetry(self, samples):
        self.module.sock = _FakeSocket([b"packet"] * len(samples))
        self.module.csv_writer = types.SimpleNamespace(writerow=lambda _row: None)
        self.module.csv_file = types.SimpleNamespace(flush=lambda: None)
        with (
            mock.patch.object(
                self.module,
                "parse_telemetry_packet",
                side_effect=samples,
            ),
            mock.patch.object(
                self.module,
                "sample_to_csv_row",
                return_value=[],
            ),
            mock.patch.object(
                self.module,
                "active_fault_names",
                return_value=["Fault_Critical"],
            ),
            contextlib.redirect_stdout(io.StringIO()),
            self.assertRaises(_LoopComplete),
        ):
            self.module.telemetry_thread()

    def test_x_button_disarms_after_streaming_stops(self):
        commands = []
        self.module.reliable_send = commands.append
        self.module.is_armed = True
        self.module.is_streaming = True

        self.module.stop_streaming_only("test")
        self._run_controller(iterations=1, start=True)

        self.assertEqual(commands, ["stop"])
        self.assertFalse(self.module.is_armed)
        self.assertFalse(self.module.is_streaming)

    def test_critical_fault_disarms_after_streaming_stops(self):
        commands = []
        self.module.reliable_send = commands.append
        self.module.is_armed = True
        self.module.is_streaming = True

        self.module.stop_streaming_only("test")
        self._run_telemetry([
            _telemetry_sample(Fault_Critical=1),
        ])

        self.assertEqual(commands, ["stop"])
        self.assertFalse(self.module.is_armed)
        self.assertFalse(self.module.is_streaming)

    def test_stop_streaming_only_does_not_send_stop(self):
        commands = []
        self.module.reliable_send = commands.append
        self.module.is_armed = True
        self.module.is_streaming = True

        self.module.stop_streaming_only("test")

        self.assertEqual(commands, [])
        self.assertTrue(self.module.is_armed)
        self.assertFalse(self.module.is_streaming)

    def test_arm_preserves_sequence_and_does_not_send_trim(self):
        commands = []
        self.module.reliable_send = commands.append
        self.module.rc_seq = 73
        self.module.is_armed = False
        self.module.is_streaming = False

        self.module.arm()

        self.assertEqual(commands, ["mag 1", "start"])
        self.assertEqual(self.module.rc_seq, 73)
        self.assertTrue(self.module.is_armed)
        self.assertTrue(self.module.is_streaming)

    def test_first_complete_telemetry_trim_is_adopted_once(self):
        self.module.trim_roll = 0.0
        self.module.trim_pitch = 0.0
        self.module.trim_synced = False
        self.module.is_armed = False
        self.module.is_streaming = False

        self._run_telemetry([
            _telemetry_sample(Trim_Roll=None, Trim_Pitch=None),
            _telemetry_sample(Trim_Roll=1.5, Trim_Pitch=-2.5),
            _telemetry_sample(Trim_Roll=4.0, Trim_Pitch=3.0),
        ])

        self.assertTrue(self.module.trim_synced)
        self.assertEqual(
            (self.module.trim_roll, self.module.trim_pitch),
            (1.5, -2.5),
        )

    def test_dpad_trim_is_clamped_to_firmware_limit(self):
        commands = []
        self.module.reliable_send = commands.append
        self.module.is_armed = True
        self.module.is_streaming = True
        self.module.trim_roll = 9.9
        self.module.trim_pitch = 0.0
        self.module.last_hat_state = (0, 0)

        self._run_controller(
            iterations=2,
            hats=[(1, 0), (0, 0)],
        )

        self.assertEqual(self.module.trim_roll, 10.0)
        self.assertEqual(commands, ["trim 10.00 0.00"])

    def test_trim_reset_button_is_edge_triggered(self):
        commands = []
        self.module.reliable_send = commands.append
        self.module.is_armed = True
        self.module.is_streaming = True
        self.module.trim_roll = 1.0
        self.module.trim_pitch = -1.0
        self.module.last_btn_trim_reset = False

        self._run_controller(iterations=3, reset=True)

        self.assertEqual(
            commands,
            ["trim 0.00 0.00"],
        )


class BenchTrimResetTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        sys.path.insert(0, str(SCRIPTS_DIR))

    @classmethod
    def tearDownClass(cls):
        sys.path.remove(str(SCRIPTS_DIR))

    def _load_bench(self, filename):
        path = SCRIPTS_DIR / filename
        module_name = f"_{path.stem}_under_test"
        sys.modules.pop(module_name, None)
        spec = importlib.util.spec_from_file_location(module_name, path)
        module = importlib.util.module_from_spec(spec)
        sys.modules[module_name] = module
        with (
            mock.patch.object(socket, "socket", return_value=_FakeSocket()),
            mock.patch.object(sys, "argv", [str(path)]),
            contextlib.redirect_stdout(io.StringIO()),
        ):
            spec.loader.exec_module(module)
        self.addCleanup(sys.modules.pop, module_name, None)
        return module

    def test_bench_senders_clear_trim_before_start(self):
        for filename in (
            "bench_thrust_ramp.py",
            "bench_sign_test.py",
            "bench_yaw_test.py",
        ):
            with self.subTest(filename=filename):
                module = self._load_bench(filename)
                commands = []
                module._running = False
                module.send = commands.append
                with mock.patch.object(module.time, "sleep", return_value=None):
                    module.sender()

                self.assertIn("trim 0 0", commands)
                self.assertLess(
                    commands.index("trim 0 0"),
                    commands.index("start"),
                )


if __name__ == "__main__":
    unittest.main()
