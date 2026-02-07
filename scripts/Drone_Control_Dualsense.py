import socket
import threading
import time
import pygame

# === 설정 ===
UDP_IP = "192.168.4.1"
UDP_PORT = 4210
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# 조종 상태 변수
current_throttle = 1000
target_roll = 0.0
target_pitch = 0.0
target_yaw = 0.0
is_armed = False

# 트림(영점) 변수
trim_roll = 0.0
trim_pitch = 0.0
TRIM_STEP = 0.2

# 버튼 상태 저장용 (Edge Detection)
last_btn_start = False
last_btn_L1 = False
last_btn_R1 = False
last_trig_L2 = False
last_trig_R2 = False
last_hat_state = (0, 0)

# 튜닝 변수
MAX_ANGLE = 10.0
YAW_RATE = 1.0

def send_cmd(cmd):
    sock.sendto(cmd.encode(), (UDP_IP, UDP_PORT))

# [수정] 수신 스레드 삭제됨 (이제 안 받음)

# === 컨트롤러 처리 스레드 ===
def controller_thread():
    global current_throttle, target_roll, target_pitch, target_yaw, is_armed
    global trim_roll, trim_pitch
    global last_btn_start, last_btn_L1, last_btn_R1, last_trig_L2, last_trig_R2, last_hat_state
    
    pygame.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() == 0:
        print("\n[ERR] 컨트롤러가 없습니다!")
        return
    
    joy = pygame.joystick.Joystick(0)
    joy.init()
    
    print("========== DRONE CONTROLLER (SEND ONLY) ==========")
    print(f" [Mode] Hovering Practice (+/- {MAX_ANGLE} deg)")
    print(" [Trim] DPAD to adjust Zero Point")
    print(" [Throttle] L2/R2 (+/-10), L1/R1 (+/-1)")
    print("==================================================")

    while True:
        pygame.event.pump() 

        # 1. 시동 토글 (BTN_0: X/A)
        btn_start = joy.get_button(0)
        if btn_start and not last_btn_start: 
            is_armed = not is_armed
            cmd = "start" if is_armed else "stop"
            send_cmd(cmd)
            print(f"\n>>> [SYSTEM] {'ARMED (시동 ON) 🔥' if is_armed else 'DISARMED (시동 OFF) 🛑'}")
            
            if not is_armed:
                current_throttle = 1000 
                target_roll = 0; target_pitch = 0; target_yaw = 0
            else:
                current_throttle = 1100 # 시동 초기값 (살짝 높임)
        last_btn_start = btn_start

        if is_armed:
            # 2. 스로틀 제어
            curr_btn_R1 = joy.get_button(10)
            curr_btn_L1 = joy.get_button(9)
            curr_trig_R2 = joy.get_axis(5) > 0.0 
            curr_trig_L2 = joy.get_axis(4) > 0.0 

            throttle_changed = False
            change_msg = ""

            if curr_trig_R2 and not last_trig_R2:
                current_throttle += 10
                change_msg = "▲▲ (+10)"
                throttle_changed = True
            elif curr_trig_L2 and not last_trig_L2:
                current_throttle -= 10
                change_msg = "▼▼ (-10)"
                throttle_changed = True
            elif curr_btn_R1 and not last_btn_R1:
                current_throttle += 1
                change_msg = "▲ (+1)"
                throttle_changed = True
            elif curr_btn_L1 and not last_btn_L1:
                current_throttle -= 1
                change_msg = "▼ (-1)"
                throttle_changed = True

            last_trig_R2 = curr_trig_R2; last_trig_L2 = curr_trig_L2
            last_btn_R1 = curr_btn_R1; last_btn_L1 = curr_btn_L1

            if throttle_changed:
                current_throttle = max(1000, min(1900, current_throttle))
                send_cmd(f"th {int(current_throttle)}")
                print(f" [TH] {change_msg} -> {current_throttle}")

            # 3. DPAD 트림 조절
            current_hat = joy.get_hat(0)
            if current_hat != last_hat_state:
                trim_changed = False
                
                if current_hat == (0, 1): trim_pitch -= TRIM_STEP; trim_changed = True
                elif current_hat == (0, -1): trim_pitch += TRIM_STEP; trim_changed = True
                elif current_hat == (-1, 0): trim_roll -= TRIM_STEP; trim_changed = True
                elif current_hat == (1, 0): trim_roll += TRIM_STEP; trim_changed = True
                
                if trim_changed:
                    print(f" [TRIM] Roll:{trim_roll:.1f} Pitch:{trim_pitch:.1f}")

            last_hat_state = current_hat

            if joy.get_button(12): 
                trim_roll = 0.0; trim_pitch = 0.0
                print(" [TRIM] RESET (0.0, 0.0)")

            # 4. 자세 제어
            stick_lx = joy.get_axis(0) if abs(joy.get_axis(0)) > 0.05 else 0
            stick_ly = joy.get_axis(1) if abs(joy.get_axis(1)) > 0.05 else 0
            stick_rx = joy.get_axis(2) if abs(joy.get_axis(2)) > 0.05 else 0

            final_roll = (stick_lx * MAX_ANGLE) + trim_roll
            final_pitch = (stick_ly * MAX_ANGLE) + trim_pitch
            target_yaw += stick_rx * YAW_RATE 

            # [핵심] 명령만 보냄 (수신 대기 없음)
            send_cmd(f"rc {final_roll:.2f} {final_pitch:.2f} {target_yaw:.2f}")

        time.sleep(0.05) 

# === 메인 실행 ===
t_ctrl = threading.Thread(target=controller_thread)
t_ctrl.daemon = True
t_ctrl.start()

while True:
    try:
        # 키보드 입력으로도 명령 보낼 수 있음 (PID 튜닝 등)
        msg = input() 
        if msg:
            sock.sendto(msg.encode(), (UDP_IP, UDP_PORT))
    except KeyboardInterrupt:
        break