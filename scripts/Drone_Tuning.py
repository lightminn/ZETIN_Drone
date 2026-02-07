import socket
import threading
import time

# ESP32 설정
UDP_IP = "192.168.4.1" # ESP32 SoftAP 기본 IP
UDP_PORT = 4210

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

def receive_thread():
    while True:
        try:
            data, addr = sock.recvfrom(1024)
            print(f"\n[Drone]: {data.decode()}")
        except:
            pass

# 수신 스레드 시작
t = threading.Thread(target=receive_thread)
t.daemon = True
t.start()

print("========== DRONE TUNING CONSOLE ==========")
print(" Commands:")
print("  pp <val> : Pitch P Gain (ex: pp 2.5)")
print("  dp <val> : Pitch D Gain (ex: dp 1.2)")
print("  pr <val> : Roll P Gain")
print("  dr <val> : Roll D Gain")
print("  th <val> : Base Throttle (ex: th 1150)")
print("  start    : ARM & Start Motors")
print("  stop     : DISARM (Emergency)")
print("==========================================")

while True:
    msg = input("Command > ")
    if msg:
        sock.sendto(msg.encode(), (UDP_IP, UDP_PORT))