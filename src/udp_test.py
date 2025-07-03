import socket

# --- 설정 부분 ---
LISTEN_PORT = 12345  # ESP32가 데이터를 보내는 포트와 동일해야 함
# --- 설정 끝 ---

# 1. UDP 소켓 생성 및 설정
# 0.0.0.0은 '이 컴퓨터의 모든 네트워크 인터페이스로부터 오는 데이터를 듣겠다'는 의미
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(('0.0.0.0', LISTEN_PORT))

print(f"Listening for UDP packets on port {LISTEN_PORT}...")
print("Press Ctrl+C to exit.")

# 2. 메인 무한 루프
try:
    while True:
        # 데이터가 도착할 때까지 대기
        data, addr = sock.recvfrom(1024)  # 1024바이트 버퍼

        # 받은 데이터와 보낸 곳의 IP 주소 출력
        print(
            f"Received from {addr[0]}: {data.decode(encoding='utf-8', errors='ignore')}")

except KeyboardInterrupt:
    print("\nExiting program.")
finally:
    # 3. 프로그램 종료 시 소켓을 깔끔하게 닫음
    sock.close()
    print("Socket closed.")
