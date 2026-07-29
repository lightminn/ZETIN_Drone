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


class _BlockingThrottle(float):
    """Pause a controller tick after it has loaded the old throttle value."""

    def __new__(cls, value, stale_read_started, resume_completed):
        instance = super().__new__(cls, value)
        instance.stale_read_started = stale_read_started
        instance.resume_completed = resume_completed
        return instance

    def __lt__(self, other):
        self.stale_read_started.set()
        self.resume_completed.wait(timeout=0.5)
        return super().__lt__(other)


class _Joystick:
    def __init__(
        self,
        event,
        *,
        start=False,
        resume=False,
        reset=False,
        hats=None,
    ):
        self.event = event
        self.start = start
        self.resume = resume
        self.reset = reset
        self.hats = hats or [(0, 0)]

    def init(self):
        pass

    def get_name(self):
        return "test joystick"

    def get_button(self, index):
        if index == 0:
            return self.start
        if index == 3:
            return self.resume
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
        "Motor_M1": 1001,
        "Motor_M2": 1002,
        "Motor_M3": 1003,
        "Motor_M4": 1004,
        "TgtRate_Roll": 0.0,
        "TgtRate_Pitch": 0.0,
        "TgtRate_Yaw": 0.0,
        "Fault_RC": 0,
        "Fault_Critical": 0,
        "RC_Total_Pkts": 0,
        "RC_Dropped_Pkts": 0,
        "Armed": 1,
        "Trim_Roll": 0.0,
        "Trim_Pitch": 0.0,
        "Failsafe_Phase": 1,
        "Hover_Est": 1360.0,
        "Hover_Valid": 1,
        "Failsafe_Probe_State": 0,
        "Failsafe_Probe_NoResponse": 0,
        "Failsafe_Probe_Response_G": 0.0,
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
        self.module.last_telem_time = self.module.time.monotonic()

    def tearDown(self):
        sys.modules.pop(self.module.__name__, None)

    def _run_controller(
        self,
        *,
        iterations,
        start=False,
        resume=False,
        reset=False,
        hats=None,
    ):
        event = _LoopEvent(iterations)
        joystick = _Joystick(
            event,
            start=start,
            resume=resume,
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
        output = io.StringIO()
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
            contextlib.redirect_stdout(output),
            self.assertRaises(_LoopComplete),
        ):
            self.module.telemetry_thread()
        return output.getvalue()

    def _autoland_lines(self, samples):
        output = self._run_telemetry(samples)
        return [
            line.strip()
            for line in output.splitlines()
            if "[AUTO-LAND]" in line
        ]

    def _rc_timeout_fault_lines(self, samples):
        output = self._run_telemetry(samples)
        return [
            line.strip()
            for line in output.splitlines()
            if "[FAULT] 드론: RC 타임아웃" in line
        ]

    def test_resume_request_restarts_streaming_before_sending_resume(self):
        commands = []
        self.module.reliable_send = commands.append
        self.module.is_armed = True
        self.module.is_streaming = False
        self.module.telem_failsafe_phase = 1
        self.module.telem_hover_est = 1360.0
        self.module.telem_hover_valid = True
        self.module.telem_total_pkts = 40
        self.module.telem_dropped_pkts = 3

        request_resume = getattr(self.module, "request_resume", None)
        self.assertIsNotNone(request_resume, "resume must be reachable from the ground station")
        accepted = request_resume("stdin")

        self.assertTrue(accepted)
        self.assertTrue(self.module.is_streaming)
        self.assertEqual(commands, [])

    def test_resume_request_rejects_stale_cached_telemetry(self):
        commands = []
        self.module.reliable_send = commands.append
        self.module.is_armed = True
        self.module.is_streaming = False
        self.module.telem_failsafe_phase = 1
        self.module.telem_hover_est = 1360.0
        self.module.telem_hover_valid = True
        self.module.telem_total_pkts = 40
        self.module.telem_dropped_pkts = 3
        self.module.last_telem_time = 1.0

        output = io.StringIO()
        with (
            mock.patch.object(self.module.time, "monotonic", return_value=10.0),
            contextlib.redirect_stdout(output),
        ):
            accepted = self.module.request_resume("test")

        self.assertFalse(accepted)
        self.assertFalse(self.module.is_streaming)
        self.assertEqual(commands, [])
        self.assertIn("오래됨", output.getvalue())

    def test_resume_is_sent_only_after_telemetry_confirms_an_accepted_rc(self):
        commands = []
        self.module.reliable_send = commands.append
        self.module.is_armed = True
        self.module.is_streaming = False
        self.module.telem_failsafe_phase = 1
        self.module.telem_hover_est = 1360.0
        self.module.telem_hover_valid = True
        self.module.telem_total_pkts = 40
        self.module.telem_dropped_pkts = 3
        with mock.patch.object(self.module.time, "monotonic", return_value=10.0):
            self.module.request_resume("test")

        advance = getattr(self.module, "advance_resume_attempt", None)
        self.assertIsNotNone(
            advance,
            "telemetry must gate resume on an RC packet accepted by firmware",
        )
        advance(
            _telemetry_sample(RC_Total_Pkts=41, RC_Dropped_Pkts=4),
            now=10.1,
        )
        self.assertEqual(commands, [])

        advance(
            _telemetry_sample(RC_Total_Pkts=43, RC_Dropped_Pkts=5),
            now=10.2,
        )
        self.assertEqual(commands, ["resume"])

    def test_resume_wait_rc_times_out_when_every_new_packet_is_dropped(self):
        commands = []
        self.module.reliable_send = commands.append
        self.module.is_armed = True
        self.module.is_streaming = False
        self.module.telem_failsafe_phase = 1
        self.module.telem_hover_est = 1360.0
        self.module.telem_hover_valid = True
        self.module.telem_total_pkts = 40
        self.module.telem_dropped_pkts = 3
        with mock.patch.object(self.module.time, "monotonic", return_value=10.0):
            self.module.request_resume("test")

        output = io.StringIO()
        with contextlib.redirect_stdout(output):
            self.module.advance_resume_attempt(
                _telemetry_sample(RC_Total_Pkts=45, RC_Dropped_Pkts=8),
                now=12.1,
            )

        self.assertEqual(self.module.resume_state, "idle")
        self.assertEqual(commands, [])
        self.assertIn("실패", output.getvalue())
        self.assertIn("RC 수락", output.getvalue())

    def test_resume_rejects_rc_counter_rollback_instead_of_treating_it_as_wrap(self):
        commands = []
        self.module.reliable_send = commands.append
        self.module.is_armed = True
        self.module.is_streaming = False
        self.module.telem_failsafe_phase = 1
        self.module.telem_hover_est = 1360.0
        self.module.telem_hover_valid = True
        self.module.telem_total_pkts = 10
        self.module.telem_dropped_pkts = 100
        self.module.request_resume("test")

        output = io.StringIO()
        with contextlib.redirect_stdout(output):
            self.module.advance_resume_attempt(
                _telemetry_sample(RC_Total_Pkts=0, RC_Dropped_Pkts=0),
                now=self.module.resume_deadline - 0.1,
            )

        self.assertEqual(commands, [])
        self.assertEqual(self.module.resume_state, "idle")
        self.assertIn("카운터", output.getvalue())

    def test_resume_accepts_a_small_forward_uint32_counter_wrap(self):
        commands = []
        self.module.reliable_send = commands.append
        self.module.is_armed = True
        self.module.is_streaming = False
        self.module.telem_failsafe_phase = 1
        self.module.telem_hover_est = 1360.0
        self.module.telem_hover_valid = True
        self.module.telem_total_pkts = 0xFFFFFFFE
        self.module.telem_dropped_pkts = 7
        self.module.request_resume("test")

        self.module.advance_resume_attempt(
            _telemetry_sample(RC_Total_Pkts=0, RC_Dropped_Pkts=7),
            now=self.module.resume_deadline - 0.1,
        )

        self.assertEqual(commands, ["resume"])
        self.assertEqual(self.module.resume_state, "wait_phase")

    def test_resume_success_syncs_local_throttle_to_hover_est_and_warns_operator(self):
        commands = []
        self.module.reliable_send = commands.append
        self.module.is_armed = True
        self.module.is_streaming = False
        self.module.current_throttle = 1000
        self.module.throttle_f = 1000.0
        self.module.telem_failsafe_phase = 1
        self.module.telem_hover_est = 1360.0
        self.module.telem_hover_valid = True
        self.module.telem_total_pkts = 40
        self.module.telem_dropped_pkts = 3
        with mock.patch.object(self.module.time, "monotonic", return_value=10.0):
            self.module.request_resume("test")
        self.module.advance_resume_attempt(
            _telemetry_sample(RC_Total_Pkts=41, RC_Dropped_Pkts=3),
            now=10.1,
        )

        output = io.StringIO()
        with contextlib.redirect_stdout(output):
            self.module.advance_resume_attempt(
                _telemetry_sample(
                    Failsafe_Phase=0,
                    Hover_Est=1378.6,
                    Hover_Valid=1,
                    RC_Total_Pkts=42,
                    RC_Dropped_Pkts=3,
                ),
                now=10.2,
            )

        self.assertEqual(commands, ["resume"])
        self.assertEqual(self.module.resume_state, "idle")
        self.assertEqual(self.module.current_throttle, 1379)
        self.assertAlmostEqual(self.module.throttle_f, 1378.6)
        self.assertIn("성공", output.getvalue())
        self.assertIn("즉시 스로틀을 올리", output.getvalue())

    def test_resume_success_sync_is_not_overwritten_by_a_controller_tick(self):
        direct_commands = []
        stale_read_started = threading.Event()
        resume_completed = threading.Event()
        controller_errors = []
        self.module.send_cmd = direct_commands.append
        self.module.is_armed = True
        self.module.is_streaming = True
        self.module.current_throttle = 1000
        self.module.throttle_f = _BlockingThrottle(
            1000.0,
            stale_read_started,
            resume_completed,
        )
        self.module.resume_state = "wait_phase"
        self.module.resume_result_total_baseline = 41
        self.module.resume_result_dropped_baseline = 3

        def run_controller_tick():
            try:
                self._run_controller(iterations=1)
            except BaseException as error:
                controller_errors.append(error)

        controller = threading.Thread(target=run_controller_tick)
        controller.start()
        try:
            self.assertTrue(
                stale_read_started.wait(timeout=1.0),
                "controller did not reach the throttle read-modify-write",
            )
            self.module.advance_resume_attempt(
                _telemetry_sample(
                    Failsafe_Phase=0,
                    Hover_Est=1340.0,
                    RC_Total_Pkts=42,
                    RC_Dropped_Pkts=3,
                ),
                now=10.2,
            )
        finally:
            resume_completed.set()
            controller.join(timeout=2.0)

        self.assertFalse(controller.is_alive())
        self.assertEqual(controller_errors, [])
        self.assertEqual(self.module.current_throttle, 1340)
        self.assertEqual(self.module.throttle_f, 1340.0)
        self.assertNotIn("th 1000", direct_commands)

    def test_stale_phase_zero_packet_cannot_confirm_resume_success(self):
        commands = []
        self.module.reliable_send = commands.append
        self.module.is_armed = True
        self.module.is_streaming = False
        self.module.current_throttle = 1000
        self.module.throttle_f = 1000.0
        self.module.telem_failsafe_phase = 1
        self.module.telem_hover_est = 1360.0
        self.module.telem_hover_valid = True
        self.module.telem_total_pkts = 40
        self.module.telem_dropped_pkts = 3
        self.module.request_resume("test")
        self.module.advance_resume_attempt(
            _telemetry_sample(RC_Total_Pkts=41, RC_Dropped_Pkts=3),
            now=10.1,
        )

        output = io.StringIO()
        with contextlib.redirect_stdout(output):
            self.module.advance_resume_attempt(
                _telemetry_sample(
                    Failsafe_Phase=0,
                    Hover_Est=1378.6,
                    Hover_Valid=1,
                    RC_Total_Pkts=39,
                    RC_Dropped_Pkts=3,
                ),
                now=10.2,
            )

        self.assertEqual(commands, ["resume"])
        self.assertEqual(self.module.resume_state, "idle")
        self.assertEqual(self.module.current_throttle, 1000)
        self.assertEqual(self.module.throttle_f, 1000.0)
        self.assertIn("실패", output.getvalue())
        self.assertNotIn("성공", output.getvalue())

    def test_resume_refusal_is_reported_when_phase_stays_descending(self):
        commands = []
        self.module.reliable_send = commands.append
        self.module.is_armed = True
        self.module.is_streaming = False
        self.module.current_throttle = 1000
        self.module.throttle_f = 1000.0
        self.module.telem_failsafe_phase = 1
        self.module.telem_hover_est = 1360.0
        self.module.telem_hover_valid = True
        self.module.telem_total_pkts = 40
        self.module.telem_dropped_pkts = 3
        with mock.patch.object(self.module.time, "monotonic", return_value=10.0):
            self.module.request_resume("test")
        self.module.advance_resume_attempt(
            _telemetry_sample(RC_Total_Pkts=41, RC_Dropped_Pkts=3),
            now=10.1,
        )

        output = io.StringIO()
        with contextlib.redirect_stdout(output):
            self.module.advance_resume_attempt(
                _telemetry_sample(
                    Failsafe_Phase=1,
                    RC_Total_Pkts=42,
                    RC_Dropped_Pkts=3,
                ),
                now=11.2,
            )

        self.assertEqual(commands, ["resume"])
        self.assertEqual(self.module.resume_state, "idle")
        self.assertEqual(self.module.current_throttle, 1000)
        self.assertEqual(self.module.throttle_f, 1000.0)
        self.assertIn("실패", output.getvalue())
        self.assertIn("Failsafe_Phase=1", output.getvalue())
        self.assertNotIn("성공", output.getvalue())

    def test_resume_timeout_is_checked_without_a_valid_telemetry_sample(self):
        commands = []
        self.module.reliable_send = commands.append
        self.module.is_armed = True
        self.module.is_streaming = False
        self.module.telem_failsafe_phase = 1
        self.module.telem_hover_est = 1360.0
        self.module.telem_hover_valid = True
        self.module.telem_total_pkts = 40
        self.module.telem_dropped_pkts = 3
        with mock.patch.object(self.module.time, "monotonic", return_value=10.0):
            self.module.request_resume("test")

        check_timeout = getattr(self.module, "check_resume_timeout", None)
        self.assertIsNotNone(
            check_timeout,
            "resume deadlines must advance even during malformed/absent telemetry",
        )
        output = io.StringIO()
        with contextlib.redirect_stdout(output):
            check_timeout(now=12.1)

        self.assertEqual(self.module.resume_state, "idle")
        self.assertEqual(commands, [])
        self.assertIn("실패", output.getvalue())
        self.assertIn("시간 초과", output.getvalue())

    def test_rc_timeout_phase_one_reports_autoland_descent_once(self):
        samples = [
            _telemetry_sample(Fault_RC=1, Failsafe_Phase=1, Armed=1),
            _telemetry_sample(Fault_RC=1, Failsafe_Phase=1, Armed=1),
        ]

        lines = self._rc_timeout_fault_lines(samples)

        self.assertEqual(len(lines), 1)
        self.assertIn("자동착륙", lines[0])
        self.assertIn("하강", lines[0])
        self.assertIn("DESCENDING", lines[0])

    def test_rc_timeout_phase_zero_disarmed_reports_immediate_cut_once(self):
        samples = [
            _telemetry_sample(Fault_RC=1, Failsafe_Phase=0, Armed=0),
            _telemetry_sample(Fault_RC=1, Failsafe_Phase=0, Armed=0),
        ]

        lines = self._rc_timeout_fault_lines(samples)

        self.assertEqual(len(lines), 1)
        self.assertNotIn("자동착륙", lines[0])
        self.assertIn("즉시 컷", lines[0])
        self.assertIn("호버 추정치 없음", lines[0])
        self.assertIn("지상 스로틀", lines[0])
        self.assertIn("시리얼", lines[0])

    def test_rc_timeout_terminal_phases_report_the_named_end_state_once(self):
        for phase, phase_name in (
            (2, "CUT_LANDED"),
            (3, "CUT_TIMEOUT"),
            (4, "CUT_ABORT"),
        ):
            with self.subTest(phase=phase):
                samples = [
                    _telemetry_sample(
                        Fault_RC=1,
                        Failsafe_Phase=phase,
                        Armed=0,
                    ),
                    _telemetry_sample(
                        Fault_RC=1,
                        Failsafe_Phase=phase,
                        Armed=0,
                    ),
                ]

                lines = self._rc_timeout_fault_lines(samples)

                self.assertEqual(len(lines), 1)
                self.assertIn("이미 종료", lines[0])
                self.assertIn(phase_name, lines[0])

    def test_rc_timeout_unknown_phase_reports_observed_values_once(self):
        samples = [
            _telemetry_sample(Fault_RC=1, Failsafe_Phase=None, Armed=0),
            _telemetry_sample(Fault_RC=1, Failsafe_Phase=None, Armed=0),
        ]

        lines = self._rc_timeout_fault_lines(samples)

        self.assertEqual(len(lines), 1)
        self.assertIn("상태 확정 불가", lines[0])
        self.assertIn("Failsafe_Phase=None", lines[0])
        self.assertIn("Armed=0", lines[0])
        self.assertNotIn("자동착륙 진행", lines[0])

    def test_autoland_line_contains_named_phase_probe_and_all_stage_e_fields(self):
        lines = self._autoland_lines([
            _telemetry_sample(
                Failsafe_Phase=1,
                Failsafe_Probe_State=4,
                Failsafe_Probe_NoResponse=2,
                Failsafe_Probe_Response_G=0.05786,
                Hover_Est=1360.4,
                Throttle=1302,
                Motor_M1=1291,
                Motor_M2=1298,
                Motor_M3=1306,
                Motor_M4=1313,
            ),
        ])

        self.assertEqual(len(lines), 1)
        line = lines[0]
        self.assertIn("[PROBE BLOCKED]", line)
        self.assertIn("Failsafe_Phase=DESCENDING", line)
        self.assertIn("Failsafe_Probe_State=BLOCKED", line)
        self.assertNotIn("Failsafe_Phase=1", line)
        self.assertNotIn("Failsafe_Probe_State=4", line)
        for expected in (
            "Failsafe_Probe_NoResponse=2",
            "Throttle=1302",
            "Motor_M1=1291",
            "Motor_M2=1298",
            "Motor_M3=1306",
            "Motor_M4=1313",
        ):
            self.assertIn(expected, line)
        self.assertRegex(
            line,
            r"(?:^| )Failsafe_Probe_Response_G=0\.0579(?: |$)",
        )
        self.assertRegex(line, r"(?:^| )Hover_Est=1360\.4(?: |$)")

    def test_phase_zero_does_not_print_an_autoland_line(self):
        lines = self._autoland_lines([
            _telemetry_sample(Failsafe_Phase=0),
        ])

        self.assertEqual(lines, [])

    def test_autoland_line_is_printed_for_every_sample_without_decimation(self):
        self.module.is_armed = True
        lines = self._autoland_lines([
            _telemetry_sample(Failsafe_Phase=1),
            _telemetry_sample(Failsafe_Phase=1),
            _telemetry_sample(Failsafe_Phase=1),
        ])

        self.assertEqual(len(lines), 3)

    def test_cut_landed_autoland_line_has_the_highest_priority_abort_marker(self):
        lines = self._autoland_lines([
            _telemetry_sample(
                Failsafe_Phase=2,
                Failsafe_Probe_State=4,
            ),
        ])

        self.assertEqual(len(lines), 1)
        self.assertTrue(
            lines[0].startswith(
                "[AUTO-LAND][!!! CUT_LANDED: IMMEDIATE ABORT !!!]"
            )
        )

    def test_autoland_line_survives_missing_stage_e_fields(self):
        lines = self._autoland_lines([
            _telemetry_sample(
                Failsafe_Phase=1,
                Failsafe_Probe_State=None,
                Failsafe_Probe_NoResponse=None,
                Failsafe_Probe_Response_G=None,
                Hover_Est=None,
                Throttle=None,
                Motor_M1=None,
                Motor_M2=None,
                Motor_M3=None,
                Motor_M4=None,
            ),
        ])

        self.assertEqual(len(lines), 1)
        line = lines[0]
        for field in (
            "Failsafe_Probe_State",
            "Failsafe_Probe_NoResponse",
            "Failsafe_Probe_Response_G",
            "Hover_Est",
            "Throttle",
            "Motor_M1",
            "Motor_M2",
            "Motor_M3",
            "Motor_M4",
        ):
            self.assertIn(f"{field}=-", line)

    def test_armed_diag_line_contains_hover_estimate_and_validity(self):
        self.module.is_armed = True
        output = self._run_telemetry([
            _telemetry_sample(Failsafe_Phase=0, Hover_Est=1423.56, Hover_Valid=1),
            _telemetry_sample(Failsafe_Phase=0, Hover_Est=1423.56, Hover_Valid=1),
            _telemetry_sample(Failsafe_Phase=0, Hover_Est=1423.56, Hover_Valid=1),
            _telemetry_sample(Failsafe_Phase=0, Hover_Est=1423.56, Hover_Valid=1),
        ])

        diag_lines = [
            line for line in output.splitlines() if "[DIAG]" in line
        ]
        self.assertEqual(len(diag_lines), 1)
        self.assertIn("Hover_Est=1423.6", diag_lines[0])
        self.assertIn("Hover_Valid=1", diag_lines[0])

    def test_telemetry_thread_advances_resume_attempt(self):
        commands = []
        self.module.reliable_send = commands.append
        self.module.is_armed = True
        self.module.is_streaming = False
        self.module.telem_failsafe_phase = 1
        self.module.telem_hover_est = 1360.0
        self.module.telem_hover_valid = True
        self.module.telem_total_pkts = 40
        self.module.telem_dropped_pkts = 3
        self.module.request_resume("test")

        self._run_telemetry([
            _telemetry_sample(
                Failsafe_Phase=1,
                Hover_Est=1360.0,
                Hover_Valid=1,
                RC_Total_Pkts=41,
                RC_Dropped_Pkts=3,
            ),
        ])

        self.assertEqual(commands, ["resume"])
        self.assertEqual(self.module.resume_state, "wait_phase")

    def test_critical_fault_sample_cancels_resume_before_command_send(self):
        commands = []
        self.module.reliable_send = commands.append
        self.module.is_armed = True
        self.module.is_streaming = False
        self.module.telem_failsafe_phase = 1
        self.module.telem_hover_est = 1360.0
        self.module.telem_hover_valid = True
        self.module.telem_total_pkts = 40
        self.module.telem_dropped_pkts = 3
        self.module.request_resume("test")

        output = io.StringIO()
        with contextlib.redirect_stdout(output):
            self.module.advance_resume_attempt(
                _telemetry_sample(
                    Fault_Critical=1,
                    RC_Total_Pkts=41,
                    RC_Dropped_Pkts=3,
                ),
                now=10.0,
            )

        self.assertEqual(commands, [])
        self.assertEqual(self.module.resume_state, "idle")
        self.assertIn("실패", output.getvalue())
        self.assertIn("Fault_Critical", output.getvalue())

    def test_disarmed_sample_cancels_resume_before_command_send(self):
        commands = []
        self.module.reliable_send = commands.append
        self.module.is_armed = True
        self.module.is_streaming = False
        self.module.telem_failsafe_phase = 1
        self.module.telem_hover_est = 1360.0
        self.module.telem_hover_valid = True
        self.module.telem_total_pkts = 40
        self.module.telem_dropped_pkts = 3
        self.module.request_resume("test")

        output = io.StringIO()
        with contextlib.redirect_stdout(output):
            self.module.advance_resume_attempt(
                _telemetry_sample(
                    Armed=0,
                    RC_Total_Pkts=41,
                    RC_Dropped_Pkts=3,
                ),
                now=10.0,
            )

        self.assertEqual(commands, [])
        self.assertEqual(self.module.resume_state, "idle")
        self.assertIn("실패", output.getvalue())
        self.assertIn("Armed=0", output.getvalue())

    def test_triangle_button_starts_resume_without_disarming(self):
        commands = []
        self.module.reliable_send = commands.append
        self.module.is_armed = True
        self.module.is_streaming = False
        self.module.telem_failsafe_phase = 1
        self.module.telem_hover_est = 1360.0
        self.module.telem_hover_valid = True
        self.module.telem_total_pkts = 40
        self.module.telem_dropped_pkts = 3

        self._run_controller(iterations=2, resume=True)

        self.assertTrue(self.module.is_armed)
        self.assertTrue(self.module.is_streaming)
        self.assertEqual(self.module.resume_state, "wait_rc")
        self.assertEqual(commands, [])

    def test_stdin_resume_uses_the_guarded_sequence_instead_of_direct_send(self):
        direct_commands = []
        reliable_commands = []
        self.module.send_cmd = direct_commands.append
        self.module.reliable_send = reliable_commands.append
        self.module.is_armed = True
        self.module.is_streaming = False
        self.module.telem_failsafe_phase = 1
        self.module.telem_hover_est = 1360.0
        self.module.telem_hover_valid = True
        self.module.telem_total_pkts = 40
        self.module.telem_dropped_pkts = 3

        handler = getattr(self.module, "handle_stdin_command", None)
        self.assertIsNotNone(handler, "stdin resume must use the guarded resume path")
        handler("  resume  ")

        self.assertEqual(self.module.resume_state, "wait_rc")
        self.assertTrue(self.module.is_streaming)
        self.assertEqual(direct_commands, [])
        self.assertEqual(reliable_commands, [])

    def test_stdin_stop_cancels_resume_before_sending_stop(self):
        direct_commands = []
        reliable_commands = []
        self.module.send_cmd = direct_commands.append
        self.module.reliable_send = reliable_commands.append
        self.module.is_armed = True
        self.module.is_streaming = False
        self.module.telem_failsafe_phase = 1
        self.module.telem_hover_est = 1360.0
        self.module.telem_hover_valid = True
        self.module.telem_total_pkts = 40
        self.module.telem_dropped_pkts = 3
        self.module.request_resume("test")

        self.module.handle_stdin_command("stop")
        self.module.advance_resume_attempt(
            _telemetry_sample(RC_Total_Pkts=41, RC_Dropped_Pkts=3),
            now=10.0,
        )

        self.assertEqual(self.module.resume_state, "idle")
        self.assertEqual(direct_commands, [])
        self.assertEqual(reliable_commands, ["stop"])

    def test_disarm_cancels_a_ground_station_resume_attempt(self):
        commands = []
        self.module.reliable_send = commands.append
        self.module.is_armed = True
        self.module.is_streaming = False
        self.module.telem_failsafe_phase = 1
        self.module.telem_hover_est = 1360.0
        self.module.telem_hover_valid = True
        self.module.telem_total_pkts = 40
        self.module.telem_dropped_pkts = 3
        self.module.request_resume("test")

        self.module.disarm("test")
        self.module.advance_resume_attempt(
            _telemetry_sample(RC_Total_Pkts=41, RC_Dropped_Pkts=3),
            now=10.0,
        )

        self.assertEqual(self.module.resume_state, "idle")
        self.assertEqual(commands, ["stop"])

    def test_streaming_loss_reports_that_the_resume_attempt_failed(self):
        commands = []
        self.module.reliable_send = commands.append
        self.module.is_armed = True
        self.module.is_streaming = False
        self.module.telem_failsafe_phase = 1
        self.module.telem_hover_est = 1360.0
        self.module.telem_hover_valid = True
        self.module.telem_total_pkts = 40
        self.module.telem_dropped_pkts = 3
        self.module.request_resume("test")

        output = io.StringIO()
        with contextlib.redirect_stdout(output):
            self.module.stop_streaming_only("텔레메트리 끊김")

        self.assertEqual(self.module.resume_state, "idle")
        self.assertEqual(commands, [])
        self.assertIn("[RESUME] 실패", output.getvalue())
        self.assertIn("텔레메트리 끊김", output.getvalue())

    def test_concurrent_disarm_cannot_be_followed_by_resume(self):
        commands = []
        resume_send_started = threading.Event()
        release_resume_send = threading.Event()
        stop_sent = threading.Event()

        def controlled_reliable_send(command):
            if command == "resume":
                resume_send_started.set()
                release_resume_send.wait(timeout=2.0)
            commands.append(command)
            if command == "stop":
                stop_sent.set()

        self.module.reliable_send = controlled_reliable_send
        self.module.is_armed = True
        self.module.is_streaming = False
        self.module.telem_failsafe_phase = 1
        self.module.telem_hover_est = 1360.0
        self.module.telem_hover_valid = True
        self.module.telem_total_pkts = 40
        self.module.telem_dropped_pkts = 3
        self.module.request_resume("test")

        resume_thread = threading.Thread(
            target=self.module.advance_resume_attempt,
            args=(_telemetry_sample(RC_Total_Pkts=41, RC_Dropped_Pkts=3),),
            kwargs={"now": 10.0},
        )
        resume_thread.start()
        self.assertTrue(resume_send_started.wait(timeout=1.0))

        disarm_thread = threading.Thread(
            target=self.module.disarm,
            args=("test",),
        )
        disarm_thread.start()
        stop_sent.wait(timeout=0.2)
        release_resume_send.set()
        resume_thread.join(timeout=2.0)
        disarm_thread.join(timeout=2.0)

        self.assertFalse(resume_thread.is_alive())
        self.assertFalse(disarm_thread.is_alive())
        self.assertEqual(commands, ["resume", "stop"])
        self.assertEqual(self.module.resume_state, "idle")

    def test_resume_request_is_serialized_behind_an_in_progress_stop(self):
        stop_send_started = threading.Event()
        release_stop_send = threading.Event()
        request_done = threading.Event()
        request_results = []

        def controlled_reliable_send(command):
            if command == "stop":
                stop_send_started.set()
                release_stop_send.wait(timeout=2.0)

        self.module.reliable_send = controlled_reliable_send
        self.module.is_armed = True
        self.module.is_streaming = False
        self.module.telem_failsafe_phase = 1
        self.module.telem_hover_est = 1360.0
        self.module.telem_hover_valid = True
        self.module.telem_total_pkts = 40
        self.module.telem_dropped_pkts = 3

        disarm_thread = threading.Thread(
            target=self.module.disarm,
            args=("test",),
        )
        disarm_thread.start()
        self.assertTrue(stop_send_started.wait(timeout=1.0))

        def request_in_thread():
            request_results.append(self.module.request_resume("test"))
            request_done.set()

        request_thread = threading.Thread(target=request_in_thread)
        request_thread.start()
        finished_before_stop = request_done.wait(timeout=0.2)
        release_stop_send.set()
        disarm_thread.join(timeout=2.0)
        request_thread.join(timeout=2.0)

        self.assertFalse(finished_before_stop)
        self.assertFalse(disarm_thread.is_alive())
        self.assertFalse(request_thread.is_alive())
        self.assertEqual(request_results, [False])
        self.assertEqual(self.module.resume_state, "idle")

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

    def test_arm_serializes_start_against_concurrent_disarm(self):
        commands = []
        start_send_started = threading.Event()
        release_start_send = threading.Event()
        stop_sent = threading.Event()

        def controlled_reliable_send(command):
            commands.append(command)
            if command == "start":
                start_send_started.set()
                release_start_send.wait(timeout=2.0)
            elif command == "stop":
                stop_sent.set()

        self.module.reliable_send = controlled_reliable_send
        self.module.is_armed = False
        self.module.is_streaming = False

        arm_thread = threading.Thread(target=self.module.arm)
        arm_thread.start()
        self.assertTrue(start_send_started.wait(timeout=1.0))

        disarm_thread = threading.Thread(
            target=self.module.disarm,
            args=("test",),
        )
        disarm_thread.start()
        stop_raced_start = stop_sent.wait(timeout=0.2)
        release_start_send.set()
        arm_thread.join(timeout=2.0)
        disarm_thread.join(timeout=2.0)

        self.assertFalse(stop_raced_start)
        self.assertFalse(arm_thread.is_alive())
        self.assertFalse(disarm_thread.is_alive())
        self.assertEqual(commands, ["mag 1", "start", "stop"])
        self.assertFalse(self.module.is_armed)
        self.assertFalse(self.module.is_streaming)

    def test_arm_updates_grace_time_before_publishing_armed(self):
        armed_state_seen_by_clock = []
        self.module.reliable_send = lambda _command: None
        self.module.is_armed = False
        self.module.is_streaming = False

        def observe_armed_state():
            armed_state_seen_by_clock.append(self.module.is_armed)
            return 123.0

        with mock.patch.object(
            self.module.time,
            "monotonic",
            side_effect=observe_armed_state,
        ):
            self.module.arm()

        self.assertEqual(armed_state_seen_by_clock, [False])
        self.assertEqual(self.module.last_arm_time, 123.0)
        self.assertTrue(self.module.is_armed)

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
