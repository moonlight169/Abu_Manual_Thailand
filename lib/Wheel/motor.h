#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>

class Motor{
public:
    Motor(int pinA, int pinB);
    void setspeed(int speed);
    int getSpeed();
    void run(int speed);
    void run();
    void smoothRun(int targetSpeed);
    void update();
    ~Motor();

private:
    // ระยะ PWM ที่ยอมให้ _speed ขยับเข้าใกล้ target ต่อการเรียก smoothRun()/update() หนึ่งครั้ง
    // (เรียกที่ COMMAND_RATE 100Hz ใน 2_slave_wheel -> 20 ต่อ 10ms คือไล่ครบ 255 ใน ~130ms)
    static const int SMOOTH_STEP = 20;

    int _pinA;
    int _pinB;
    int _speed = 0;
    int _maxRPM = 0;
    int _targetSpeed = 0;
    unsigned long _lastStepMillis = 0;

};


#endif