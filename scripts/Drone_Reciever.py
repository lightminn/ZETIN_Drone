import socket
import csv
import time
import datetime
import os
import sys

# ==========================================
# 설정
# ==========================================
UDP_PORT = 4210          
DRONE_IP = "192.168.4.1" 
LOG_DIR = "../logs"      # 저장할 폴더 이름

# ==========================================
# 1. 파일 및 소켓 준비
# ==========================================
# 폴더 없으면 생성
if not os.path.exists(LOG_DIR):
    os.makedirs(LOG_DIR)
    print(f"📂 '{LOG_DIR}' 폴더 생성 완료.")

# 파일명 생성 (날짜_시간)
filename = f"flight_log_{datetime.datetime.now().date()}_{datetime.datetime.now().strftime('%H%M%S')}.csv"
file_path = os.path.join(LOG_DIR, filename)

try:
    csv_file = open(file_path, 'w', newline='')
    csv_writer = csv.writer(csv_file)
    
    # 헤더 작성 (아두이노 전송 순서와 일치해야 함)
    csv_writer.writerow([
        "Timestamp", 
        "Roll", "Pitch", "Yaw", 
        "Gyro_X", "Gyro_Y", "Gyro_Z", 
        "Accel_X", "Accel_Y", "Accel_Z", 
        "Throttle"
    ])
    
    print(f"💾 로그 파일 생성됨: {file_path}")

except Exception as e:
    print(f"❌ 파일 생성 실패: {e}")
    sys.exit()

# 소켓 설정
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("0.0.0.0", UDP_PORT)) 
sock.settimeout(0.05) # 타임아웃 0.05초

print(f"📡 데이터 수신 대기 중... (Port: {UDP_PORT})")
print("🛑 종료하려면 'Ctrl + C'를 누르세요.")

# ==========================================
# 2. 메인 루프 (무한 반복)
# ==========================================
last_handshake = 0
packet_count = 0

try:
    while True:
        # 1. Handshake (1초마다 드론에게 '나 살아있다' 신호 전송)
        if time.time() - last_handshake > 1.0:
            try:
                sock.sendto(b"connect", (DRONE_IP, UDP_PORT))
                last_handshake = time.time()
            except: pass

        # 2. 데이터 수신
        try:
            data, addr = sock.recvfrom(2048)
            line = data.decode('utf-8', errors='ignore').strip()
            
            parts = line.split(',')
            
            # 데이터 개수 확인 (10개)
            if len(parts) >= 10:
                # 파싱 (화면에 예쁘게 출력하기 위해 변수에 담음)
                r, p, y = float(parts[0]), float(parts[1]), float(parts[2])
                gx, gy, gz = float(parts[3]), float(parts[4]), float(parts[5])
                ax, ay, az = float(parts[6]), float(parts[7]), float(parts[8])
                th = int(parts[9])
                
                # 현재 시간
                now_str = datetime.datetime.now().strftime('%H:%M:%S.%f')[:-3]
                
                # CSV 쓰기
                csv_writer.writerow([now_str, r, p, y, gx, gy, gz, ax, ay, az, th])
                
                # 화면 출력 (너무 빠르면 눈 아프니까 10번에 1번만 출력하거나, 그냥 출력)
                print(f"[{now_str}] "
                      f"R:{r:6.2f} P:{p:6.2f} Y:{y:6.2f} | "  # 자세 (Roll, Pitch, Yaw)
                      f"GX:{gx:4.0f} GY:{gy:4.0f} GZ:{gz:4.0f} | " # 자이로 (소수점 버림, 정수만 봐도 됨)
                      f"AX:{ax:5.2f} AY:{ay:5.2f} AZ:{az:5.2f} | " # 가속도 (소수점 2자리 중요)
                      f"Thr:{th:4d}")
                packet_count += 1
                
        except socket.timeout:
            continue # 데이터 안 오면 다시 루프
        except Exception as e:
            print(f"⚠️ 에러: {e}")

except KeyboardInterrupt:
    # Ctrl + C 눌렀을 때 실행
    print("\n\n🛑 로그 저장 종료!")
    print(f"📊 총 {packet_count}개 데이터가 저장되었습니다.")

finally:
    # 안전하게 닫기
    csv_file.close()
    sock.close()
    print("✅ 파일이 안전하게 닫혔습니다.")