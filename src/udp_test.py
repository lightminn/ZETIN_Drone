import socket
import time

# --- ★★★★★ 사용자 설정 부분 ★★★★★ ---
# ESP32의 IP 주소 (Arduino IDE 시리얼 모니터에서 확인)
SERVER_IP = "192.168.1.30"  # 예시 IP, 실제 ESP32의 IP로 변경해야 합니다.
SERVER_PORT = 8888         # ESP32의 TCP 서버 포트 (main.cpp와 동일하게)
# --- 설정 끝 ---


def send_command(command):
    """ESP32 TCP 서버에 명령어를 전송하고 응답을 수신합니다."""
    # 1. TCP 소켓 생성
    # AF_INET: IPv4 주소 체계 사용, SOCK_STREAM: TCP 프로토콜 사용
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        try:
            # 2. 서버에 연결 시도 (연결 타임아웃 5초)
            sock.settimeout(5)
            print(f"Connecting to {SERVER_IP}:{SERVER_PORT}...")
            sock.connect((SERVER_IP, SERVER_PORT))
            print("Connected to server.")

            # 3. 명령어 전송
            #   - 명령어를 UTF-8 바이트로 인코딩
            #   - ESP32의 readStringUntil('\n')에 맞춰 끝에 개행 문자(\n) 추가
            message = command + '\n'
            sock.sendall(message.encode('utf-8'))
            print(f"Sent: {command}")

            # 4. 서버로부터 응답 수신 대기 (수신 타임아웃 5초)
            sock.settimeout(5)
            response = sock.recv(1024)  # 최대 1024바이트 데이터 수신
            if response:
                print(
                    f"Received: {response.decode(encoding='utf-8', errors='ignore').strip()}")
            else:
                print("No response from server.")

        except socket.timeout:
            print("Connection or response timed out.")
        except ConnectionRefusedError:
            print("Connection refused. Is the server running and the IP correct?")
        except Exception as e:
            print(f"An error occurred: {e}")
        finally:
            # 5. 소켓 연결 종료 (with 구문으로 자동 처리)
            print("Connection closed.")


if __name__ == "__main__":
    print("TCP Client for ESP32. Type 'exit' to quit.")
    try:
        while True:
            # 사용자로부터 보낼 명령어 입력받기
            cmd_to_send = input("Enter command to send: ")

            if cmd_to_send.lower() == 'exit':
                break
            if not cmd_to_send:
                print("Cannot send empty command.")
                continue

            send_command(cmd_to_send)
            print("-" * 20)
            time.sleep(0.5)  # 다음 명령 전송 전 잠시 대기

    except KeyboardInterrupt:
        print("\nExiting program.")
