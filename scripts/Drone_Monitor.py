import socket
import csv
import time
import datetime
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from collections import deque
import threading

# ==========================================
# 설정
# ==========================================
UDP_PORT = 4210          # 수신 포트
DRONE_IP = "192.168.4.1" # 드론 IP (또는 현재 연결된 IP)
MAX_LEN = 100            # 그래프 데이터 개수

# ------------------------------------------
# 데이터 저장소 (Deque) 초기화
# ------------------------------------------
# 1. 자세 (Attitude)
roll_data = deque(maxlen=MAX_LEN)
pitch_data = deque(maxlen=MAX_LEN)
yaw_data = deque(maxlen=MAX_LEN)

# 2. 자이로 (Gyro Raw) - 진동 확인용
gyro_x_data = deque(maxlen=MAX_LEN)
gyro_y_data = deque(maxlen=MAX_LEN)
gyro_z_data = deque(maxlen=MAX_LEN)

# 3. 가속도 (Accel Raw) - 밀림/충격 확인용
accel_x_data = deque(maxlen=MAX_LEN)
accel_y_data = deque(maxlen=MAX_LEN)
accel_z_data = deque(maxlen=MAX_LEN)

# 4. 입력
throttle_data = deque(maxlen=MAX_LEN)

# ------------------------------------------
# CSV 파일 준비
# ------------------------------------------
filename = f"flight_log_{datetime.datetime.now().strftime('%H%M%S')}.csv"
csv_file = open(filename, 'w', newline='')
csv_writer = csv.writer(csv_file)

# CSV 헤더 작성 (데이터 순서대로)
csv_writer.writerow([
    "Timestamp", 
    "Roll", "Pitch", "Yaw", 
    "Gyro_X", "Gyro_Y", "Gyro_Z", 
    "Accel_X", "Accel_Y", "Accel_Z", 
    "Throttle"
])

# ------------------------------------------
# 소켓 설정
# ------------------------------------------
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("0.0.0.0", UDP_PORT)) 
sock.settimeout(0.02)

print(f"📡 확장 모니터링 시작! (Port: {UDP_PORT})")
print(f"💾 로그 파일: {filename}")

# ------------------------------------------
# 그래프 설정 (3단 구성)
# ------------------------------------------
# figsize=(가로, 세로) 크기 조절
fig, (ax1, ax2, ax3) = plt.subplots(3, 1, sharex=True, figsize=(10, 12))
fig.suptitle('Real-time Drone Telemetry')

last_handshake = 0

def update_plot(frame):
    global last_handshake
    
    # 1. Handshake (1초마다 전송으로 변경 - 네트워크 부하 감소)
    if time.time() - last_handshake > 1.0:
        try:
            sock.sendto(b"connect", (DRONE_IP, UDP_PORT))
            last_handshake = time.time()
        except: pass

    # 2. 데이터 수신 및 파싱
    while True:
        try:
            data, _ = sock.recvfrom(2048) # 버퍼 크기 약간 늘림
            line = data.decode('utf-8', errors='ignore').strip()
            
            # 파싱: 10개 데이터 + 타임스탬프
            parts = line.split(',')
            
            # 데이터 개수가 맞는지 확인 (최소 10개)
            if len(parts) >= 10:
                # 1. 자세
                r = float(parts[0])
                p = float(parts[1])
                y = float(parts[2])
                # 2. 자이로
                gx = float(parts[3])
                gy = float(parts[4])
                gz = float(parts[5])
                # 3. 가속도
                ax = float(parts[6])
                ay = float(parts[7])
                az = float(parts[8])
                # 4. 스로틀
                th = int(parts[9])
                
                # CSV 저장
                now = datetime.datetime.now().strftime('%H:%M:%S.%f')[:-3]
                csv_writer.writerow([now, r, p, y, gx, gy, gz, ax, ay, az, th])
                
                # 그래프용 Deque에 추가
                roll_data.append(r)
                pitch_data.append(p)
                yaw_data.append(y)
                
                gyro_x_data.append(gx)
                gyro_y_data.append(gy)
                gyro_z_data.append(gz)
                
                accel_x_data.append(ax)
                accel_y_data.append(ay)
                accel_z_data.append(az)
                
                throttle_data.append(th)
                
        except socket.timeout:
            break 
        except Exception as e:
            # print(f"Err: {e}") # 디버깅 필요시 주석 해제
            break

    # 3. 그래프 그리기 (3단)
    
    # [Top] Attitude (각도)
    ax1.cla()
    ax1.plot(roll_data, label='Roll', color='red')
    ax1.plot(pitch_data, label='Pitch', color='blue')
    ax1.plot(yaw_data, label='Yaw', color='green', linestyle='--')
    ax1.set_ylabel('Angle (deg)')
    ax1.set_title(f'Attitude (Th: {throttle_data[-1] if len(throttle_data) else 0})')
    ax1.legend(loc='upper right', fontsize='small')
    ax1.grid(True)
    ax1.set_ylim(-60, 60) 

    # [Middle] Gyro Raw (각속도 - 진동 확인용)
    ax2.cla()
    ax2.plot(gyro_x_data, label='Gyro X', color='red', alpha=0.7)
    ax2.plot(gyro_y_data, label='Gyro Y', color='blue', alpha=0.7)
    ax2.plot(gyro_z_data, label='Gyro Z', color='green', alpha=0.7)
    ax2.set_ylabel('Gyro (dps)')
    ax2.set_title('Gyroscope Raw (Vibration Check)')
    ax2.legend(loc='upper right', fontsize='small')
    ax2.grid(True)
    # 범위는 센서에 따라 다르지만 보통 노이즈 보면 +/- 20 정도 튐

    # [Bottom] Accel Raw (가속도 - 쏠림 확인용)
    ax3.cla()
    ax3.plot(accel_x_data, label='Accel X', color='red', alpha=0.7)
    ax3.plot(accel_y_data, label='Accel Y', color='blue', alpha=0.7)
    ax3.plot(accel_z_data, label='Accel Z', color='green', alpha=0.7)
    ax3.set_ylabel('Accel (g)')
    ax3.set_title('Accelerometer Raw')
    ax3.legend(loc='upper right', fontsize='small')
    ax3.grid(True)
    ax3.set_ylim(-2.0, 2.0) # 중력가속도 1G 기준 위아래

# 실행
ani = FuncAnimation(fig, update_plot, interval=50, cache_frame_data=False)
plt.show()

csv_file.close()
sock.close()