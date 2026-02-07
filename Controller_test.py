import pygame
import time

# Pygame 초기화
pygame.init()
pygame.joystick.init()

# 연결된 조이스틱 확인
joystick_count = pygame.joystick.get_count()
if joystick_count == 0:
    print("❌ 컨트롤러를 찾을 수 없습니다. 연결을 확인해주세요.")
    exit()
else:
    # 첫 번째 컨트롤러 선택 (DualSense)
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    print(f"✅ 연결됨: {joystick.get_name()}")
    print("Ctrl+C를 눌러 종료하세요.")

try:
    while True:
        # 이벤트 처리 (이걸 호출해야 내부 상태가 갱신됨)
        pygame.event.pump()

        # === 1. 아날로그 스틱 & 트리거 (AXIS) 읽기 ===
        # DualSense 기준 (OS마다 다를 수 있으니 직접 돌려보고 확인 필요)
        # 보통: 0(L-X), 1(L-Y), 2(R-X), 3(R-Y), 4(L2), 5(R2)
        
        axis_0 = joystick.get_axis(0) # 왼쪽 스틱 좌우 (-1.0 ~ 1.0)
        axis_1 = joystick.get_axis(1) # 왼쪽 스틱 상하 (-1.0 ~ 1.0)
        axis_2 = joystick.get_axis(2) # 오른쪽 스틱 좌우
        axis_3 = joystick.get_axis(3) # 오른쪽 스틱 상하 (또는 L2/R2 일수 있음)
        
        # L2, R2 트리거 (보통 -1.0이 뗀 상태, 1.0이 꽉 누른 상태)
        # Windows/Mac 환경에 따라 매핑이 다를 수 있어 4, 5번도 확인해봐야 함
        axis_4 = joystick.get_axis(4) 
        axis_5 = joystick.get_axis(5)

        # === 2. 버튼 (BUTTON) 읽기 ===
        # 0:X, 1:O, 2:□, 3:△ ... (직접 눌러보며 매핑 확인)
        buttons = []
        for i in range(joystick.get_numbuttons()):
            if joystick.get_button(i):
                buttons.append(f"BTN_{i}")

        # === 3. 방향키 (HAT) 읽기 ===
        # (0, 0) 중앙 / (1, 0) 우 / (-1, 0) 좌 / (0, 1) 상 / (0, -1) 하
        hat = joystick.get_hat(0)

        # === 출력 ===
        # 화면 지저분해지는 것 방지하기 위해 '\r' 사용
        print(f"\rLX:{axis_0:.2f} LY:{axis_1:.2f} | RX:{axis_2:.2f} RY:{axis_3:.2f} | L2:{axis_4:.2f} R2:{axis_5:.2f} | HAT:{hat} | PUSH:{buttons}      ", end="")
        
        time.sleep(0.05) # 너무 빠른 출력 방지

except KeyboardInterrupt:
    print("\n종료합니다.")
    pygame.quit()