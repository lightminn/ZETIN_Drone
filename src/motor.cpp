#include "../lib/motor.h"
#include <Arduino.h>
#include <algorithm>
#include <cmath>

namespace
{
    constexpr float vMax = 12.6f;
    constexpr float dVmax = 0.05f;

    /*  S : 6×4  — “모터 1 V 변화 시 센서 6축 응답”  (실험값으로 교체) */
    constexpr float S[6][4] = {
        {0.12f, -0.12f, -0.12f, 0.12f},     // v_x
        {-0.12f, 0.12f, -0.12f, 0.12f},     // v_y
        {0.25f, 0.25f, 0.25f, 0.25f},       // v_z
        {-0.015f, 0.015f, 0.015f, -0.015f}, // w_x
        {0.015f, -0.015f, 0.015f, -0.015f}, // w_y
        {-0.02f, 0.02f, -0.02f, 0.02f}      // w_z
    };
    /*  W : 센서 신뢰도 (diag)   R : 전압 페널티 (diag)  */
    constexpr float W[6] = {1, 1, 1, 0.3f, 0.3f, 0.3f};
    constexpr float R[4] = {0.01f, 0.01f, 0.01f, 0.01f};

    constexpr uint16_t PWM_RES = 8191; // ESP32 LEDC 13-bit
}

/* ── 생성자/소멸자는 그대로 ───────────────────────────────────────── */
Motor::Motor() : voltage{0, 0, 0, 0}, motor_num(0) {}
Motor::~Motor() {}

/* ── 핵심 함수: v·ω  →  모터 4개 전압 적용 ───────────────────────── */
void Motor::setVoltage(float v_x, float v_y, float v_z,
                       float w_x, float w_y, float w_z)
{
    /* 1) 상태 벡터 x */
    const float x[6] = {v_x, v_y, v_z, w_x, w_y, w_z};

    /* 2) H = SᵀWS + R  ,  f = SᵀW x  산출 (4×4 대칭) */
    float H[4][4] = {0};
    float f[4] = {0};

    for (int i = 0; i < 4; ++i)
    {
        for (int j = i; j < 4; ++j)
        {
            float hij = (i == j ? R[i] : 0.f);
            for (int k = 0; k < 6; ++k)
                hij += S[k][i] * W[k] * S[k][j];
            H[i][j] = H[j][i] = hij;
        }
        float sum = 0.f;
        for (int k = 0; k < 6; ++k)
            sum += S[k][i] * W[k] * x[k];
        f[i] = sum;
    }

    /* 3) ΔV* = −H⁻¹ f   (4×4 SPD ⇒ Cholesky) */
    float L[4][4] = {0};
    for (int i = 0; i < 4; ++i)
    {
        for (int j = 0; j <= i; ++j)
        {
            float s = H[i][j];
            for (int k = 0; k < j; ++k)
                s -= L[i][k] * L[j][k];
            L[i][j] = (i == j) ? std::sqrt(s) : s / L[j][j];
        }
    }
    float y[4]; // L y = f
    for (int i = 0; i < 4; ++i)
    {
        float s = f[i];
        for (int k = 0; k < i; ++k)
            s -= L[i][k] * y[k];
        y[i] = s / L[i][i];
    }
    float dV[4]; // Lᵀ dV = y
    for (int i = 3; i >= 0; --i)
    {
        float s = y[i];
        for (int k = i + 1; k < 4; ++k)
            s -= L[k][i] * dV[k];
        dV[i] = -s / L[i][i]; // 부호 포함
    }

    /* 4) 증분·전압·PWM 포화 후 하드웨어 적용 */
    for (int m = 0; m < 4; ++m)
    {
        dV[m] = std::clamp(dV[m], -dVmax, dVmax);
        voltage[m] = std::clamp(voltage[m] + dV[m], 0.f, vMax);

        float duty = voltage[m] / vMax; // 0‥1
        uint16_t cnt = static_cast<uint16_t>(duty * PWM_RES);
        ledcWrite(m, cnt); // LEDC 채널 m → GPIO 매핑은 setup()에서
    }
}

void Motor::start() { /* 필요 시 ESC 암·시작 코드 */ }
void Motor::stop() { /* 모터 컷오프 코드 */ }
