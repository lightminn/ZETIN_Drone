#ifndef MOTOR_H
#define MOTOR_H

class Motor {
public:
    Motor();                  // 생성자
    ~Motor();                 // 소멸자

    void setSpeed();          // 속도 설정
    void start();             // 모터 시작
    void stop();              // 모터 정지

private:
    float voltage;         // 현재 전압
    int motor_num;         // 모터 번호 (예: 1, 2, 3 등)
};

#endif
