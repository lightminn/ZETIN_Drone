//==================================================================================
// ESP32, MPU-6500 및 칼만 필터 예제 코드
//==================================================================================
// 설명: MPU-6500으로부터 자이로스코프 데이터를 읽어와 각 축에 1차원 칼만 필터를
//      적용하여 노이즈를 줄인 결과를 시리얼 모니터에 출력합니다.
//==================================================================================

#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// MPU6050 객체 생성
Adafruit_MPU6050 mpu;

// --- 1D 칼만 필터 클래스 ---
// 이 클래스는 단일 변수에 대한 칼만 필터를 구현합니다.
class SimpleKalmanFilter
{
private:
    float _err_measure;      // R: 측정 오차 공분산 (센서 노이즈)
    float _err_estimate;     // P: 추정 오차 공분산
    float _q;                // Q: 프로세스 노이즈 공분산 (시간에 따른 변화량)
    float _current_estimate; // 현재 추정값
    float _kalman_gain;      // 칼만 게인

public:
    SimpleKalmanFilter(float err_measure, float err_estimate, float q)
    {
        _err_measure = err_measure;
        _err_estimate = err_estimate;
        _q = q;
        _current_estimate = 0; // 초기 추정값은 0으로 시작
    }

    // 새로운 측정값을 입력받아 필터링된 값을 반환하는 함수
    float updateEstimate(float measurement)
    {
        // 1. 칼만 게인 계산
        _kalman_gain = _err_estimate / (_err_estimate + _err_measure);

        // 2. 현재 추정값 계산
        _current_estimate = _current_estimate + _kalman_gain * (measurement - _current_estimate);

        // 3. 오차 공분산 업데이트
        _err_estimate = (1.0 - _kalman_gain) * _err_estimate + _q;

        return _current_estimate;
    }
};

// --- 칼만 필터 객체 생성 ---
// 각 자이로스코프 축에 대한 칼만 필터 인스턴스를 만듭니다.
// 파라미터: (측정 오차, 추정 오차, 프로세스 노이즈)
// 이 값들은 실험을 통해 조정해야 최적의 성능을 낼 수 있습니다.
SimpleKalmanFilter kalmanGyroX(2, 2, 0.01);
SimpleKalmanFilter kalmanGyroY(2, 2, 0.01);
SimpleKalmanFilter kalmanGyroZ(2, 2, 0.01);

//==================================================================================
//  SETUP 함수
//==================================================================================
void setup(void)
{
    Serial.begin(115200);
    while (!Serial)
        ; // 시리얼 포트가 열릴 때까지 대기

    // I2C 통신 및 MPU-6500 초기화
    if (!mpu.begin())
    {
        Serial.println("MPU6500을 찾지 못했습니다. 연결을 확인하세요.");
        while (1)
        {
            delay(10);
        }
    }
    Serial.println("MPU6500이 성공적으로 초기화되었습니다.");

    // 센서 범위 설정
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);

    delay(100);

    // 출력 헤더
    Serial.println("RawGyroX, FilteredGyroX, RawGyroY, FilteredGyroY, RawGyroZ, FilteredGyroZ");
}

//==================================================================================
//  LOOP 함수
//==================================================================================
void loop()
{
    // 센서 이벤트(가속도, 자이로, 온도)를 담을 변수 생성
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    // --- 자이로 데이터 필터링 ---
    // 1. 원본(Raw) 자이로 데이터 추출
    float rawGyroX = g.gyro.x;
    float rawGyroY = g.gyro.y;
    float rawGyroZ = g.gyro.z;

    // 2. 칼만 필터의 update 함수를 호출하여 필터링된 값 계산
    float filteredGyroX = kalmanGyroX.updateEstimate(rawGyroX);
    float filteredGyroY = kalmanGyroY.updateEstimate(rawGyroY);
    float filteredGyroZ = kalmanGyroZ.updateEstimate(rawGyroZ);

    // --- 결과 출력 ---
    // 시리얼 모니터 또는 시리얼 플로터로 확인하기 좋도록 CSV 형식으로 출력
    Serial.print(rawGyroX);
    Serial.print(",");
    Serial.print(filteredGyroX);
    Serial.print(",");

    Serial.print(rawGyroY);
    Serial.print(",");
    Serial.print(filteredGyroY);
    Serial.print(",");

    Serial.print(rawGyroZ);
    Serial.print(",");
    Serial.println(filteredGyroZ);

    delay(10); // 출력 속도 조절
}