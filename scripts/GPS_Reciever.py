import socket

UDP_IP = "0.0.0.0"
UDP_PORT = 4210

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

print(f"📡 WiFi에 연결하고 GPS 데이터를 기다리는 중... (Port: {UDP_PORT})")

while True:
    data, addr = sock.recvfrom(1024)
    # 깨진 문자라도 일단 다 출력해서 확인
    try:
        print(data.decode('utf-8'), end='') 
    except:
        print(data, end='') # 디코딩 안되면 바이트 그대로 출력