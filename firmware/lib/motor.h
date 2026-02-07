#ifndef MOTOR_H
#define MOTOR_H

class Motor
{
public:
    Motor();  // 생성자
    ~Motor(); // 소멸자

    /*  v_x, v_y, v_z,  w_x, w_y, w_z  → 모터 전압 4개 계산·적용  */
    void setVoltage(float v_x, float v_y, float v_z,
                    float w_x, float w_y, float w_z);

    void start(); // 모터 시작
    void stop();  // 모터 정지

    float voltage[4]; // 각 모터 현재 전압

private:
    int motor_num;    // (예: 1 ~ 4)
};

#endif
