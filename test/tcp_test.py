import socket
import time

# ESP32의 IP 주소와 포트 설정
SERVER_IP = "192.168.0.101"
SERVER_PORT = 8888


def send_command(command):
    """ESP32 TCP 서버에 명령어를 전송하고 응답을 수신합니다."""
    # IPv4, TCP 소켓 생성
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        try:
            # 서버 연결 (타임아웃 5초)
            sock.settimeout(5)
            print(f"Connecting to {SERVER_IP}:{SERVER_PORT}...")
            sock.connect((SERVER_IP, SERVER_PORT))
            print("Connected to server.")

            # UTF-8로 인코딩 후 전송
            message = command + '\n'
            sock.sendall(message.encode('utf-8'))
            print(f"Sent: {command}")

            # 서버로부터 응답 수신 (타임아웃 5초)
            sock.settimeout(5)
            response = sock.recv(1024)
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
            # with 구문이 소켓을 자동으로 닫습니다.
            print("Connection closed.")


if __name__ == "__main__":
    print("TCP Client for ESP32. Type 'exit' to quit.")
    try:
        while True:
            # 사용자로부터 명령어 입력
            cmd_to_send = input("Enter command to send: ")

            if cmd_to_send.lower() == 'exit':
                break
            if not cmd_to_send:
                print("Cannot send empty command.")
                continue

            send_command(cmd_to_send)
            print("-" * 20)
            time.sleep(0.5)

    except KeyboardInterrupt:
        print("\nExiting program.")
