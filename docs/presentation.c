void setup()
{
    // 복잡한 연산을 담당할 'PID 태스크'를 Core 0에 독립적으로 생성
    xTaskCreatePinnedToCore(
        pid_task,   // 실행할 함수 (PID 계산)
        "PID_Task", // 태스크 이름
        4096,       // 스택 메모리 크기
        NULL,       // 파라미터
        1,          // 우선순위
        NULL,       // 핸들
        0           // Core 0에 강제 할당 (계산 전담)
    );
    // 이후 기본 loop() 함수는 자동으로 Core 1에서 실행되어
    // 모터 통신(DShot)만 전담하게 됨 -> '병렬 처리' 구현
}

// [Core 0] 자세 제어 루프
void pid_task(void *pvParameters)
{
    for (;;)
    {
        // 1. 목표(0도)와 현재 기울기(angle_pitch)의 차이 계산
        float error = 0.0 - angle_pitch;

        // 2. PID 알고리즘 적용
        // [P] 기울어진 만큼 즉각적인 반발력을 생성
        float P = Kp * error;
        // [I] 미세하게 남은 오차(정상 상태 오차)를 누적하여 제거
        integral += error * dt;
        float I = Ki * integral; // (초기 튜닝 시 Ki=0으로 설정)
        // [D] 급격한 움직임과 진동을 억제
        float D = Kd * (error - prev_error);

        // 3. 최종 모터 출력 계산 (기본 스로틀 + PID 보정값)
        int final_throttle = BASE_THROTTLE + (P + I + D);

        // 4. 계산된 결과를 Core 1이 볼 수 있는 '공유 변수'에 저장
        // volatile 키워드를 사용하여 실시간성 보장
        global_throttle_percent = final_throttle;

        vTaskDelay(1); // 1ms 대기
    }
}

// [Core 1] 모터 신호 전송을 전담하는 고속 루프
void loop()
{
    // 1. Core 0에서 계산해둔 최신 스로틀 값을 즉시 읽어옴
    int throttle = global_throttle_percent;

    // 2. ESC에 DShot600 디지털 신호 전송
    motor.sendThrottlePercent(throttle);

    // 3. 초고속 제어 루프 유지
    // 계산 로직(Core 0)이 아무리 복잡해도 모터 신호는 끊기지 않음
    delayMicroseconds(75);
}