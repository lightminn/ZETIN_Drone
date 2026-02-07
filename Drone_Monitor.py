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
DRONE_IP = "192.168.4.1" # 드론 IP
MAX_LEN = 100            # 그래프 데이터 개수

# 데이터 저장소
roll_data = deque(maxlen=MAX_LEN)
pitch_data = deque(maxlen=MAX_LEN)
yaw_data = deque(maxlen=MAX_LEN)
throttle_data = deque(maxlen=MAX_LEN)

# CSV 준비
filename = f"flight_log_{datetime.datetime.now().strftime('%H%M%S')}.csv"
csv_file = open(filename, 'w', newline='')
csv_writer = csv.writer(csv_file)
csv_writer.writerow(["Timestamp", "Roll", "Pitch", "Yaw", "Throttle"])

# 소켓 (수신 전용)
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("0.0.0.0", UDP_PORT)) # 모든 IP에서 오는 데이터 수신
sock.settimeout(0.02) # 타임아웃 아주 짧게 (그래프 갱신 위해)

print(f"📡 모니터링 시작! (Port: {UDP_PORT})")

# 그래프 설정
fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True)
fig.suptitle('Real-time Flight Monitor')

last_handshake = 0

def update_plot(frame):
    global last_handshake
    
    # 1. 드론에게 1초마다 "데이터 줘!" 신호 보내기 (Handshake)
    if time.time() - last_handshake > 0.05:
        try:
            sock.sendto(b"connect", (DRONE_IP, UDP_PORT))
            last_handshake = time.time()
        except: pass

    # 2. 쌓인 데이터 다 읽기
    while True:
        try:
            data, _ = sock.recvfrom(1024)
            line = data.decode('utf-8', errors='ignore').strip()
            
            # 파싱 ("Roll,Pitch,Yaw,Throttle")
            parts = line.split(',')
            if len(parts) >= 3:
                r = float(parts[0])
                p = float(parts[1])
                y = float(parts[2])
                th = int(parts[3]) if len(parts) > 3 else 0
                
                # CSV 저장
                now = datetime.datetime.now().strftime('%H:%M:%S.%f')[:-3]
                csv_writer.writerow([now, r, p, y, th])
                
                # 그래프 데이터 추가
                roll_data.append(r)
                pitch_data.append(p)
                yaw_data.append(y)
                throttle_data.append(th)
        except socket.timeout:
            break # 데이터 없으면 그리기 단계로 이동
        except:
            break

    # 3. 그래프 그리기
    ax1.cla()
    ax2.cla()
    
    ax1.plot(roll_data, label='Roll', color='red')
    ax1.plot(pitch_data, label='Pitch', color='blue')
    ax1.set_ylabel('Angle (deg)')
    ax1.legend(loc='upper right')
    ax1.grid(True)
    ax1.set_ylim(-45, 45) # Y축 고정

    ax2.plot(yaw_data, label='Yaw', color='green')
    ax2.set_ylabel('Heading')
    ax2.legend(loc='upper right')
    ax2.grid(True)
    
    if len(throttle_data) > 0:
        ax1.set_title(f"Throttle: {throttle_data[-1]}")

# 실행
ani = FuncAnimation(fig, update_plot, interval=50, cache_frame_data=False)
plt.show()

csv_file.close()
sock.close()