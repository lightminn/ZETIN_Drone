import csv
import datetime
import math
import socket
import threading
import time
from pathlib import Path

import pygame

from telemetry_schema import (
    CSV_FIELDS,
    active_fault_names,
    is_gains_packet,
    parse_telemetry_packet,
    sample_to_csv_row,
)

# === 설정 ===
UDP_IP        = "192.168.4.1"
UDP_PORT      = 4210
CTRL_LOOP_HZ  = 20          # 제어 루프 주기 (50ms)
MAX_ANGLE     = 15.0
YAW_RATE_MAX_DPS = 90.0     # 스틱 최대 편향 시 yaw 각속도 (dps). 조종감은 여기서 조정
THROTTLE_RATE = 200.0        # 오른쪽 스틱 최대 편향 시 스로틀 변화율 (µs/s)
TRIM_STEP     = 0.2
TRIM_MAX_DEG  = 10.0
STOP_RETRIES  = 5            # stop/start 재전송 횟수
STOP_INTERVAL = 0.02         # 재전송 간격 (초)
RESUME_RC_TIMEOUT_SEC = 2.0   # 새 RC가 드론에 수락됐음을 기다리는 시간
RESUME_RESULT_TIMEOUT_SEC = 1.0  # resume 후 phase=0 확인 대기

# --- 고장진단 상수 ---
TELEM_TIMEOUT_SEC = 1.5
TILT_WARN_DEG     = 30.0
TILT_WARN_PERIOD  = 1.0      # 과도 기울기 경고 최소 간격 (초)

# 송신/수신 단일 소켓 사용 (드론이 송신자 포트로 텔레메트리를 응답하므로 동일 소켓을 사용해야 함)
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("", UDP_PORT))
sock.settimeout(0.1)

# --- 비행 CSV 로그 (receive/monitor와 같은 스키마, logs/에 저장) ---
SCRIPT_DIR = Path(__file__).resolve().parent
LOG_DIR = SCRIPT_DIR.parent / "logs"
LOG_DIR.mkdir(parents=True, exist_ok=True)
log_path = LOG_DIR / f"flight_log_{datetime.datetime.now():%Y-%m-%d_%H%M%S}.csv"
csv_file = log_path.open("w", newline="", encoding="utf-8")
csv_writer = csv.writer(csv_file)
csv_writer.writerow(CSV_FIELDS)
print(f"[LOG] 비행 로그: {log_path}")

# === 상태 변수 ===
current_throttle = 1000
throttle_f       = 1000.0   # 아날로그 스로틀 적분용 (µs, float)
trim_roll        = 0.0
trim_pitch       = 0.0
trim_synced      = False
is_armed         = False
is_streaming     = False
last_arm_time    = 0.0      # 드론 Armed 필드와 대조할 grace 기준 시각

# [FIX] RC 시퀀스 번호 — 지연 도착한 낡은 패킷을 드론 측에서 폐기할 수 있도록
rc_seq = 0

# 텔레메트리 상태
# 락 순서: safety_cmd_lock -> telem_lock만 허용한다. telem_lock 안에서는
# send_cmd/reliable_send 같은 블로킹 가능 I/O를 호출하지 않는다.
telem_lock        = threading.Lock()
safety_cmd_lock   = threading.Lock()
last_telem_time   = 0.0
telem_angle_x     = 0.0
telem_angle_y     = 0.0
telem_throttle    = 0
fault_rc_drone    = False
fault_critical_drone = False   # Fault_Critical: IMU 상실/과도 기울기/캘리브레이션 실패
telem_total_pkts  = 0
telem_dropped_pkts = 0
telem_failsafe_phase = None
telem_hover_est = None
telem_hover_valid = False

# 명시적 resume 시도 상태. 텔레메트리로 RC 수락과 phase 전이를 확인한다.
resume_state = "idle"  # idle | wait_rc | send_pending | wait_phase
resume_deadline = 0.0
resume_rc_total_baseline = 0
resume_rc_dropped_baseline = 0
resume_result_total_baseline = 0
resume_result_dropped_baseline = 0
resume_hover_est_snapshot = None
resume_attempt_id = 0

# 버튼 엣지 감지용
last_btn_start = False
last_btn_resume = False
last_btn_R1    = False
last_btn_L1    = False
last_trig_R2   = False
last_trig_L2   = False
last_hat_state = (0, 0)
last_btn_trim_reset = False


# ==========================================================
# 송신
# ==========================================================
def send_cmd(cmd: str):
    try:
        sock.sendto(cmd.encode(), (UDP_IP, UDP_PORT))
    except OSError:
        pass


def reliable_send(cmd: str):
    """중요 명령(start/stop)을 패킷 손실에 대비해 여러 번 전송."""
    for _ in range(STOP_RETRIES):
        send_cmd(cmd)
        time.sleep(STOP_INTERVAL)


def disarm(reason: str = "수동"):
    global is_armed, is_streaming, current_throttle, throttle_f
    with safety_cmd_lock:
        cancelled_resume = _invalidate_resume_attempt()
        is_armed         = False
        is_streaming     = False
        with telem_lock:
            current_throttle = 1000
            throttle_f       = 1000.0
        reliable_send("stop")
    if cancelled_resume:
        print(f"\n[RESUME] 실패: {reason}으로 복구 시도 취소")
    print(f"\n>>> [SYSTEM] DISARMED ({reason})")


def send_trim():
    """트림을 절대값으로 드론에 보낸다. 손실되면 자동착륙 방향이 틀어지므로
    start/stop과 같은 급으로 반복 전송한다."""
    reliable_send(f"trim {trim_roll:.2f} {trim_pitch:.2f}")


def stop_streaming_only(reason: str):
    """드론이 자동착륙 중일 때 쓰는 로컬 해제. stop을 보내지 않는다.

    stop을 보내면 하강 중인 드론의 모터를 공중에서 꺼버린다. 업링크만 죽는
    비대칭 고장에서는 링크가 간헐적으로 열려 그 stop이 실제로 도착한다.
    """
    global is_streaming, current_throttle, throttle_f
    with safety_cmd_lock:
        cancelled_resume = _invalidate_resume_attempt()
        is_streaming = False
        with telem_lock:
            current_throttle = 1000
            throttle_f = 1000.0
    if cancelled_resume:
        print(f"\n[RESUME] 실패: {reason}으로 복구 시도 취소")
    print(f"\n>>> [SYSTEM] 로컬 해제 ({reason}) - stop 미전송")


def _invalidate_resume_attempt():
    """safety_cmd_lock 보유자가 현재 resume 세대를 무효화한다."""
    global resume_state, resume_attempt_id
    with telem_lock:
        was_active = resume_state != "idle"
        resume_state = "idle"
        resume_attempt_id += 1
    return was_active


def cancel_resume_attempt():
    """킬/링크 재상실 뒤 늦은 텔레메트리가 resume을 보내지 못하게 한다."""
    with safety_cmd_lock:
        _invalidate_resume_attempt()


def _forward_counter_delta(current, baseline):
    """uint32 정방향(wrap 포함)만 작은 delta로 인정하고 rollback은 거부한다."""
    if current is None:
        return None
    delta = (int(current) - int(baseline)) & 0xFFFFFFFF
    return delta if delta < 0x80000000 else None


def check_resume_timeout(now=None):
    """수신 성공 여부와 무관하게 resume 상태의 wall-clock deadline을 집행한다."""
    global resume_state, resume_attempt_id
    if now is None:
        now = time.monotonic()
    expired_state = None
    with safety_cmd_lock:
        with telem_lock:
            if resume_state != "idle" and now >= resume_deadline:
                expired_state = resume_state
                resume_state = "idle"
                resume_attempt_id += 1
    if expired_state is not None:
        print(f"[RESUME] 실패: {expired_state} 확인 시간 초과")


def request_resume(source: str) -> bool:
    """명시적 resume을 시작하되, RC 수락 확인 전에는 명령을 보내지 않는다."""
    global is_streaming, resume_state, resume_deadline
    global resume_rc_total_baseline, resume_rc_dropped_baseline
    global resume_hover_est_snapshot, resume_attempt_id

    with safety_cmd_lock:
        now = time.monotonic()
        with telem_lock:
            if resume_state != "idle":
                print("\n[RESUME] 이미 복구 시도 중")
                return False
            if not is_armed:
                print("\n[RESUME] 실패: 지상국이 드론을 무장 상태로 보지 않음")
                return False
            if telem_failsafe_phase != 1:
                print(f"\n[RESUME] 실패: Failsafe_Phase={telem_failsafe_phase!r}, 하강 중이 아님")
                return False
            if last_telem_time <= 0 or now - last_telem_time > TELEM_TIMEOUT_SEC:
                print("\n[RESUME] 실패: 마지막 텔레메트리가 너무 오래됨")
                return False
            if (not telem_hover_valid or telem_hover_est is None
                    or not math.isfinite(telem_hover_est)):
                print("\n[RESUME] 실패: 유효한 Hover_Est 텔레메트리가 없음")
                return False

            resume_attempt_id += 1
            resume_state = "wait_rc"
            resume_deadline = now + RESUME_RC_TIMEOUT_SEC
            resume_rc_total_baseline = telem_total_pkts
            resume_rc_dropped_baseline = telem_dropped_pkts
            resume_hover_est_snapshot = float(telem_hover_est)
            is_streaming = True

    print(f"\n[RESUME] {source}: RC 스트리밍 재개, 드론 수락 확인 대기")
    return True


def advance_resume_attempt(sample, now=None):
    """텔레메트리 확인을 진행하고, 수락된 RC가 생긴 뒤에만 resume을 보낸다."""
    global resume_state, resume_deadline, resume_attempt_id
    global resume_result_total_baseline, resume_result_dropped_baseline
    global current_throttle, throttle_f

    if now is None:
        now = time.monotonic()
    send_resume = False
    send_token = None
    resumed_hover = None
    failure = None
    with telem_lock:
        if resume_state != "idle" and sample.get("Armed") != 1:
            resume_state = "idle"
            resume_attempt_id += 1
            failure = f"Armed={sample.get('Armed')!r}"
        elif resume_state != "idle" and sample.get("Fault_Critical") != 0:
            resume_state = "idle"
            resume_attempt_id += 1
            failure = f"Fault_Critical={sample.get('Fault_Critical')!r}"

        if resume_state == "wait_rc":
            phase = sample.get("Failsafe_Phase")
            if phase != 1:
                resume_state = "idle"
                resume_attempt_id += 1
                failure = f"Failsafe_Phase={phase!r}, 하강 중이 아님"
            else:
                total = sample.get("RC_Total_Pkts")
                dropped = sample.get("RC_Dropped_Pkts")
                accepted_rc = False
                if total is not None and dropped is not None:
                    total_delta = _forward_counter_delta(
                        total, resume_rc_total_baseline
                    )
                    dropped_delta = _forward_counter_delta(
                        dropped, resume_rc_dropped_baseline
                    )
                else:
                    total_delta = dropped_delta = None
                if ((total is not None and total_delta is None)
                        or (dropped is not None and dropped_delta is None)):
                    resume_state = "idle"
                    resume_attempt_id += 1
                    failure = "RC 텔레메트리 카운터 rollback/reset 감지"
                elif (total_delta is not None and dropped_delta is not None
                        and total_delta > dropped_delta):
                    resume_state = "send_pending"
                    resume_deadline = now + RESUME_RESULT_TIMEOUT_SEC
                    resume_result_total_baseline = int(total)
                    resume_result_dropped_baseline = int(dropped)
                    send_resume = True
                    send_token = resume_attempt_id
                elif now >= resume_deadline:
                    resume_state = "idle"
                    resume_attempt_id += 1
                    failure = "RC 수락을 텔레메트리로 확인하지 못함"
        elif resume_state == "wait_phase":
            phase = sample.get("Failsafe_Phase")
            total = sample.get("RC_Total_Pkts")
            dropped = sample.get("RC_Dropped_Pkts")
            total_delta = _forward_counter_delta(
                total, resume_result_total_baseline
            )
            dropped_delta = _forward_counter_delta(
                dropped, resume_result_dropped_baseline
            )
            if ((total is not None and total_delta is None)
                    or (dropped is not None and dropped_delta is None)):
                resume_state = "idle"
                resume_attempt_id += 1
                failure = "resume 확인 텔레메트리 카운터 rollback/reset 감지"
            elif not (total_delta is not None and dropped_delta is not None
                      and total_delta > dropped_delta):
                if now >= resume_deadline:
                    resume_state = "idle"
                    resume_attempt_id += 1
                    failure = "resume 후 수락된 RC/phase 확인 시간 초과"
            elif phase == 0:
                hover = sample.get("Hover_Est")
                if hover is None or not math.isfinite(float(hover)):
                    hover = resume_hover_est_snapshot
                resumed_hover = max(1000.0, min(1900.0, float(hover)))
                throttle_f = resumed_hover
                current_throttle = int(round(resumed_hover))
                resume_state = "idle"
            elif phase is not None and phase != 1:
                resume_state = "idle"
                resume_attempt_id += 1
                failure = f"Failsafe_Phase={phase}, 자동착륙이 이미 종료됨"
            elif now >= resume_deadline:
                resume_state = "idle"
                resume_attempt_id += 1
                failure = f"Failsafe_Phase={phase!r} 유지 (resume 거부 또는 확인 시간 초과)"

    if send_resume:
        with safety_cmd_lock:
            with telem_lock:
                token_is_current = (
                    resume_state == "send_pending"
                    and resume_attempt_id == send_token
                )
            if token_is_current:
                reliable_send("resume")
                with telem_lock:
                    if (resume_state == "send_pending"
                            and resume_attempt_id == send_token):
                        resume_state = "wait_phase"
                print("[RESUME] RC 수락 확인됨, resume 전송 - Failsafe_Phase 확인 대기")
    if resumed_hover is not None:
        print(
            f"[RESUME] 성공: Failsafe_Phase=0, Hover_Est={resumed_hover:.1f}µs 동기화. "
            "고도는 복구되지 않습니다. 즉시 스로틀을 올리십시오!"
        )
    if failure is not None:
        print(f"[RESUME] 실패: {failure}")


def arm():
    global is_armed, is_streaming, last_arm_time
    global current_throttle, throttle_f
    is_armed         = True
    is_streaming     = True
    last_arm_time    = time.monotonic()
    with telem_lock:
        current_throttle = 1100
        throttle_f       = 1100.0
    # mag 융합을 start '전에' 보낸다: armed 상태에선 최초 mag init이 거부되므로
    # 아직 disarmed인 이때 보내야 부팅 후 첫 arm에서도 init이 통과한다.
    reliable_send("mag 1")   # 자기계 yaw 융합 ON (추정값 드리프트 보정). heading-hold는 켜지 않음
    reliable_send("start")
    print("\n>>> [SYSTEM] ARMED (시동 ON, mag ON)")


def deadzone(v: float, dz: float = 0.05) -> float:
    return v if abs(v) > dz else 0.0


def handle_stdin_command(msg: str):
    """stdin resume은 RC 선행 확인 경로로 보내고, 나머지는 기존대로 전달한다."""
    command = msg.strip()
    if not command:
        return
    if command.lower() == "resume":
        request_resume("stdin")
    elif command.lower() == "stop":
        disarm("stdin stop")
    else:
        send_cmd(command)


# ==========================================================
# 텔레메트리 수신 + 고장진단 스레드
# ==========================================================
def telemetry_thread():
    global last_telem_time
    global telem_angle_x, telem_angle_y, telem_throttle
    global fault_rc_drone, fault_critical_drone
    global telem_total_pkts, telem_dropped_pkts
    global telem_failsafe_phase, telem_hover_est, telem_hover_valid
    global trim_roll, trim_pitch, trim_synced

    print("[TELEM] 수신 스레드 시작")
    prev_fault_rc = False
    prev_fault_critical = False
    last_connect = 0.0
    last_tilt_warn = 0.0
    packet_count = 0

    while True:
        check_resume_timeout()
        # 시동 전에도 텔레메트리를 받도록 주기적으로 목적지를 등록한다.
        now = time.monotonic()
        if now - last_connect >= 1.0:
            send_cmd("connect")
            last_connect = now

        try:
            data, _ = sock.recvfrom(512)
        except socket.timeout:
            if is_streaming and last_telem_time > 0:
                elapsed = time.monotonic() - last_telem_time
                if elapsed > TELEM_TIMEOUT_SEC:
                    print(f"\n[FAULT] 텔레메트리 {elapsed:.1f}s 수신 없음 "
                          f"- rc 중단, 드론 자동착륙에 맡김")
                    stop_streaming_only("텔레메트리 끊김")
            continue
        except OSError as e:
            print(f"[TELEM ERR] 소켓 오류: {e}")
            continue

        try:
            line = data.decode("utf-8", errors="strict")
            if is_gains_packet(line):
                continue
            sample = parse_telemetry_packet(line)
        except (UnicodeDecodeError, ValueError):
            continue

        with telem_lock:
            telem_angle_x      = sample["Roll"]
            telem_angle_y      = sample["Pitch"]
            telem_throttle     = sample["Throttle"] or 0
            fault_rc_drone     = sample["Fault_RC"] == 1
            fault_critical_drone = sample["Fault_Critical"] == 1
            telem_total_pkts   = sample["RC_Total_Pkts"] or 0
            telem_dropped_pkts = sample["RC_Dropped_Pkts"] or 0
            telem_failsafe_phase = sample["Failsafe_Phase"]
            telem_hover_est = sample["Hover_Est"]
            telem_hover_valid = sample["Hover_Valid"] == 1
            last_telem_time    = time.monotonic()
            if not trim_synced:
                received_trim_roll = sample["Trim_Roll"]
                received_trim_pitch = sample["Trim_Pitch"]
                if received_trim_roll is not None and received_trim_pitch is not None:
                    trim_roll = received_trim_roll
                    trim_pitch = received_trim_pitch
                    trim_synced = True

        now_str = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]
        csv_writer.writerow(sample_to_csv_row(now_str, sample))
        packet_count += 1
        if packet_count % 20 == 0:
            csv_file.flush()

        # [DIAG C-2] 틸트 복원 진단: 무장 중 모터µs/자세/목표레이트 라이브 출력(~5Hz).
        # 누른 쪽 모터 µs가 오르고 aX/aY(추정 자세)가 틸트를 따라가는지 확인용.
        if is_armed and packet_count % 4 == 0:
            m = [sample[k] for k in ("Motor_M1", "Motor_M2", "Motor_M3", "Motor_M4")]
            if all(v is not None for v in m):
                tr = sample["TgtRate_Roll"] or 0.0
                tp = sample["TgtRate_Pitch"] or 0.0
                ty = sample["TgtRate_Yaw"] or 0.0
                gz = sample["Gyro_Z"] or 0.0
                az = sample["Yaw"] or 0.0
                cw = m[0] + m[1]   # M1(FL)+M2(RR) = CW 대각쌍
                ccw = m[2] + m[3]  # M3(FR)+M4(RL) = CCW 대각쌍 (차이 = yaw축)
                print(f" [DIAG] M1(FL)={m[0]} M2(RR)={m[1]} M3(FR)={m[2]} M4(RL)={m[3]} "
                      f"[yaw축 CW-CCW={cw - ccw:+d}] | "
                      f"aX={sample['Roll']:+5.1f} aY={sample['Pitch']:+5.1f} aZ={az:+6.1f} | "
                      f"gZ={gz:+6.1f} tR={tr:+5.1f} tP={tp:+5.1f} tY={ty:+6.1f} th={sample['Throttle']}")

        # 드론 Armed 필드와 대조: 시동 명령이 거부됐거나(START REFUSED는
        # 시리얼에만 출력됨) 드론이 스스로 disarm한 것을 감지한다.
        # start 재전송+텔레메트리 주기를 고려해 1.5초 grace를 둔다.
        if (is_armed and sample["Armed"] == 0
                and time.monotonic() - last_arm_time > 1.5):
            print("\n[FAULT] 드론이 시동 상태가 아님 (시동 거부 또는 드론 측 disarm)")
            disarm("드론 측 미시동 감지")

        # 드론 측 고장 플래그 — fault는 latch되므로 상승 엣지에서만 알린다.
        if fault_rc_drone and not prev_fault_rc:
            print("\n[FAULT] 드론: RC 타임아웃 - 자동착륙 진행 중")
            if is_streaming:
                stop_streaming_only("드론 RC 타임아웃")
        prev_fault_rc = fault_rc_drone

        if fault_critical_drone and not prev_fault_critical:
            detail = ", ".join(active_fault_names(sample)) or "원인 미상"
            print(f"\n[FAULT] 드론: 치명 고장 감지됨 ({detail})")
            if is_armed:
                disarm("드론 치명 고장")
        prev_fault_critical = fault_critical_drone

        # Armed/critical/RC 안전 처리가 resume보다 항상 우선한다.
        advance_resume_attempt(sample)

        # 패킷 드롭률 출력 (10% 초과 시 경고)
        if telem_total_pkts > 100 and telem_total_pkts % 50 == 0:
            drop_rate = telem_dropped_pkts / telem_total_pkts * 100
            if drop_rate > 10.0:
                print(f"\n[WARN] 패킷 드롭률 {drop_rate:.1f}% ({telem_dropped_pkts}/{telem_total_pkts}) - 간섭 의심")

        # 과도 기울기 경고 (1초에 한 번만)
        ax, ay = abs(telem_angle_x), abs(telem_angle_y)
        if (is_armed and (ax > TILT_WARN_DEG or ay > TILT_WARN_DEG)
                and now - last_tilt_warn >= TILT_WARN_PERIOD):
            print(f"\n[WARN] 과도 기울기 - Roll:{telem_angle_x:.1f}° Pitch:{telem_angle_y:.1f}°")
            last_tilt_warn = now


# ==========================================================
# 컨트롤러 처리 스레드
# ==========================================================
def controller_thread():
    global current_throttle, throttle_f, trim_roll, trim_pitch, trim_synced, rc_seq
    global last_btn_start, last_btn_resume
    global last_btn_R1, last_btn_L1, last_btn_trim_reset
    global last_trig_R2, last_trig_L2, last_hat_state

    pygame.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() == 0:
        print("\n[ERR] 컨트롤러가 없습니다!")
        return

    joy = pygame.joystick.Joystick(0)
    joy.init()

    print("========== DRONE CONTROLLER ==========")
    print(f" [X]        Arm / Disarm")
    print(f" [△]        Resume (자동착륙 중 명시적 복구)")
    print(f" [R2/L2]    Throttle ↑/↓ (누르는 동안 연속, {THROTTLE_RATE:.0f}µs/s)")
    print(f" [R1/L1]    Throttle +1 / -1 (정밀)")
    print(f" [DPAD]     Trim  |  [PS] Trim Reset")
    print(f" [L-Stick]  Roll / Pitch (max ±{MAX_ANGLE}°),  [R-Stick↔] Yaw 각속도 (max ±{YAW_RATE_MAX_DPS:.0f}°/s)")
    print("======================================")

    loop_dt = 1.0 / CTRL_LOOP_HZ
    next_loop = time.monotonic()
    last_th_print = 0.0
    last_axis_print = 0.0

    while True:
        # [FIX] sleep 누적 오차 제거: monotonic 기반 정밀 타이밍
        now = time.monotonic()
        if now < next_loop:
            time.sleep(next_loop - now)
        next_loop += loop_dt

        pygame.event.pump()

        # --- 컨트롤러 분리 감지: RC 송신을 중단하고 자동착륙에 맡김 ---
        if pygame.joystick.get_count() == 0:
            if is_streaming:
                stop_streaming_only("컨트롤러 분리")
            print("\n[ERR] 컨트롤러 분리됨 - 재연결 대기 중...")
            while pygame.joystick.get_count() == 0:
                time.sleep(0.5)
                pygame.event.pump()
            joy = pygame.joystick.Joystick(0)
            joy.init()
            next_loop = time.monotonic()
            print(f"[OK] 컨트롤러 재연결됨: {joy.get_name()}")
            continue

        # --- 시동 토글 ---
        btn_start = joy.get_button(0)
        if btn_start and not last_btn_start:
            if is_armed:
                disarm()
            else:
                arm()
        last_btn_start = btn_start

        # --- 자동착륙 복구 (X와 분리된 명시적 입력) ---
        btn_resume = joy.get_button(3)
        if btn_resume and not last_btn_resume:
            request_resume("△ 버튼")
        last_btn_resume = btn_resume

        if is_streaming:
            # 실측 매핑(gamepad_probe): R1=b5, L1=b4, R2=a5, L2=a2 (트리거 휴지 -1.0)
            curr_R1 = joy.get_button(5)
            curr_L1 = joy.get_button(4)
            curr_R2 = joy.get_axis(5) > 0.0   # a5 = R2 (휴지 -1.0 → 누르면 +1.0)
            curr_L2 = joy.get_axis(2) > 0.0   # a2 = L2 (휴지 -1.0 → 누르면 +1.0)

            # [DEBUG] 트리거 원시 축값 — L2/R2 동작·휴지값 확인용(벤치).
            # 휴지 상태에서 a2·a5 모두 ≈ -1.00 이어야 정상.
            now_dbg = time.monotonic()
            if now_dbg - last_axis_print >= 0.5:
                with telem_lock:
                    displayed_throttle = current_throttle
                print(f" [AXIS] a2(L2)={joy.get_axis(2):+.2f} a5(R2)={joy.get_axis(5):+.2f} "
                      f"th={displayed_throttle}")
                last_axis_print = now_dbg

            # --- 스로틀: 트리거 전용 (오른쪽 스틱 매핑 제거 → 스틱 드리프트 자기감소 없음) ---
            # R2/L2 누르고 있는 동안 연속 램프(±THROTTLE_RATE µs/s), 짧게 누르면 미세.
            # R1/L1 = ±1 정밀. 트리거는 해제 시 확실히 안 눌린 상태라 스틱과 달리 드리프트가 없다.
            delta = 0
            if   curr_R1 and not last_btn_R1: delta = +1
            elif curr_L1 and not last_btn_L1: delta = -1
            throttle_to_send = None
            with telem_lock:
                if curr_R2:
                    throttle_f += THROTTLE_RATE * loop_dt
                if curr_L2:
                    throttle_f -= THROTTLE_RATE * loop_dt
                if delta:
                    throttle_f += delta

                throttle_f = max(1000.0, min(1900.0, throttle_f))
                new_throttle = int(round(throttle_f))
                if new_throttle != current_throttle:
                    current_throttle = new_throttle
                    throttle_to_send = new_throttle

            if throttle_to_send is not None:
                send_cmd(f"th {throttle_to_send}")
                now_mono = time.monotonic()
                if delta or now_mono - last_th_print >= 0.5:
                    print(f" [TH] -> {throttle_to_send}")
                    last_th_print = now_mono

            last_btn_R1  = curr_R1; last_btn_L1  = curr_L1

            # --- DPAD 트림 ---
            hat = joy.get_hat(0)
            if hat != last_hat_state:
                trim_changed = False
                if hat in ((0, 1), (0, -1), (-1, 0), (1, 0)):
                    with telem_lock:
                        previous_trim = (trim_roll, trim_pitch)
                        if   hat == (0,  1): trim_pitch -= TRIM_STEP
                        elif hat == (0, -1): trim_pitch += TRIM_STEP
                        elif hat == (-1, 0): trim_roll  -= TRIM_STEP
                        elif hat == (1,  0): trim_roll  += TRIM_STEP
                        trim_roll = max(-TRIM_MAX_DEG, min(TRIM_MAX_DEG, trim_roll))
                        trim_pitch = max(-TRIM_MAX_DEG, min(TRIM_MAX_DEG, trim_pitch))
                        trim_changed = (trim_roll, trim_pitch) != previous_trim
                        if trim_changed:
                            trim_synced = True
                if trim_changed:
                    print(f" [TRIM] Roll:{trim_roll:.1f}  Pitch:{trim_pitch:.1f}")
                    send_trim()
            last_hat_state = hat

            btn_trim_reset = bool(joy.get_button(12))
            if btn_trim_reset and not last_btn_trim_reset:
                with telem_lock:
                    trim_synced = True
                    trim_roll = trim_pitch = 0.0
                print(" [TRIM] RESET (0.0, 0.0)")
                send_trim()
            last_btn_trim_reset = btn_trim_reset

            # --- RC 명령 전송 (yaw는 각속도 dps) ---
            rc_seq += 1
            final_roll  = deadzone(joy.get_axis(0))  * MAX_ANGLE
            final_pitch = deadzone(-joy.get_axis(1)) * MAX_ANGLE
            yaw_rate    = deadzone(joy.get_axis(3), 0.12) * YAW_RATE_MAX_DPS

            send_cmd(f"rcr {rc_seq} {final_roll:.2f} {final_pitch:.2f} {yaw_rate:.1f}")


# ==========================================================
# 메인
# ==========================================================
t_telem = threading.Thread(target=telemetry_thread, daemon=True)
t_ctrl  = threading.Thread(target=controller_thread, daemon=True)
t_telem.start()
t_ctrl.start()

try:
    while True:
        try:
            msg = input()
            if msg:
                handle_stdin_command(msg)
        except KeyboardInterrupt:
            if is_armed:
                disarm("키보드 인터럽트")
            break
finally:
    csv_file.flush()
    csv_file.close()
    print(f"[LOG] 저장 완료: {log_path}")
