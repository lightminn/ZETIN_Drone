#!/usr/bin/env python3
"""게임패드 축/버튼/햇 매핑 진단 도구 (control_dualsense.py 와 무관, 단독 실행).

목적: 어떤 물리 컨트롤(스틱/트리거/버튼/DPAD)이 어떤 axis/button/hat 인덱스에
매핑되는지, 트리거의 휴지값이 -1인지 0인지 등을 실측해서 매핑을 확정한다.

사용법:
    python gamepad_probe.py

1) 시작 시 조이스틱 이름 + 축/버튼/햇 개수 + '휴지 스냅샷' 출력
   (첫 0.7초간 아무것도 만지지 말 것 → 각 축의 rest 값 기록).
2) 이후 값이 변한 항목만 출력:
       AXIS  4:  -1.00 -> +1.00     (예: L2 트리거)
       BUTTON 0:  DOWN / UP
       HAT   0:  (0,0) -> (0,1)
   각 컨트롤을 하나씩 움직여 반응하는 인덱스를 확인한다.
3) Ctrl-C 종료 시 전체 스냅샷을 다시 출력.
"""
import sys
import time

import pygame

AXIS_CHANGE_THRESHOLD = 0.15  # 스틱 노이즈 무시용


def snapshot(joy):
    axes = [joy.get_axis(i) for i in range(joy.get_numaxes())]
    btns = [joy.get_button(i) for i in range(joy.get_numbuttons())]
    hats = [joy.get_hat(i) for i in range(joy.get_numhats())]
    return axes, btns, hats


def print_full(joy):
    axes, btns, hats = snapshot(joy)
    print("  AXES :", "  ".join(f"a{i}={v:+.2f}" for i, v in enumerate(axes)))
    print("  BTNS :", "  ".join(f"b{i}={v}" for i, v in enumerate(btns)))
    print("  HATS :", "  ".join(f"h{i}={v}" for i, v in enumerate(hats)))


def main():
    pygame.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() == 0:
        print("[ERR] 조이스틱이 없습니다. 게임패드를 연결하세요.")
        return 1

    joy = pygame.joystick.Joystick(0)
    joy.init()
    print("=" * 56)
    print(f" 조이스틱: {joy.get_name()}")
    print(f" axes={joy.get_numaxes()}  buttons={joy.get_numbuttons()}  hats={joy.get_numhats()}")
    print("=" * 56)

    # 휴지값 측정: 첫 0.7초 만지지 말 것
    print("\n[휴지값 측정] 0.7초간 아무것도 만지지 마세요...")
    t_end = time.monotonic() + 0.7
    while time.monotonic() < t_end:
        pygame.event.pump()
        time.sleep(0.02)

    prev_axes, prev_btns, prev_hats = snapshot(joy)
    print("[휴지 스냅샷]")
    print_full(joy)
    print("\n이제 각 컨트롤을 하나씩 천천히 움직이세요 (반응한 항목만 출력).")
    print("확인 권장 순서: 왼쪽스틱↔↕ → 오른쪽스틱↔↕ → L2 → R2 → L1/R1 →")
    print("                X/O/△/□ → DPAD상하좌우 → 기타 버튼.")
    print("(Ctrl-C 로 종료)\n")

    try:
        while True:
            pygame.event.pump()
            axes, btns, hats = snapshot(joy)

            for i in range(len(axes)):
                if abs(axes[i] - prev_axes[i]) > AXIS_CHANGE_THRESHOLD:
                    print(f" AXIS  {i}:  {prev_axes[i]:+.2f} -> {axes[i]:+.2f}")
                    prev_axes[i] = axes[i]
            for i in range(len(btns)):
                if btns[i] != prev_btns[i]:
                    print(f" BUTTON {i}:  {'DOWN' if btns[i] else 'UP'}")
                    prev_btns[i] = btns[i]
            for i in range(len(hats)):
                if hats[i] != prev_hats[i]:
                    print(f" HAT   {i}:  {prev_hats[i]} -> {hats[i]}")
                    prev_hats[i] = hats[i]

            time.sleep(0.02)
    except KeyboardInterrupt:
        print("\n\n[종료 스냅샷]")
        print_full(joy)
        print()
    return 0


if __name__ == "__main__":
    sys.exit(main())
