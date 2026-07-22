#include <Arduino.h>
#include "motor.h"

Motor::Motor(int pinA, int pinB){
    this->_pinA = pinA;
    this->_pinB = pinB;

    analogWriteResolution(8);
    analogWriteFrequency(12000);
    
    pinMode(pinA, OUTPUT);
    pinMode(pinB, OUTPUT);

    this->_speed = 0;
    this->_targetSpeed = 0;
    this->_lastStepMillis = 0;
}

Motor::~Motor(){
}

void Motor::setspeed(int speed){
    this->_speed = constrain(speed, -255, 255);
}

int Motor::getSpeed(){
    return this->_speed;
}

void Motor::run(int speed){
    this->setspeed(speed);
    this->run();
}

void Motor::run(){
    if (this->_speed > 0){
        analogWrite(this->_pinA, abs(this->_speed));
        analogWrite(this->_pinB, 0);
    } else if (this->_speed < 0){
        analogWrite(this->_pinA, 0);
        analogWrite(this->_pinB, abs(this->_speed)); 
    } else {
        analogWrite(this->_pinB, 0);
        analogWrite(this->_pinA, 0);
    }
}

void Motor::smoothRun(int targetSpeed){
    // ไล่ _speed เข้าหา targetSpeed ทีละ SMOOTH_STEP ต่อครั้งที่เรียก (ไม่ใช่กระโดดตรงเป็น targetSpeed ทันที)
    if (targetSpeed > this->_speed){
        this->_speed = min(this->_speed + SMOOTH_STEP, targetSpeed);
    } else if (targetSpeed < this->_speed){
        this->_speed = max(this->_speed - SMOOTH_STEP, targetSpeed);
    }
    this->run();
}

void Motor::update(){
    // Gradually move current speed toward target speed
    if (this->_targetSpeed > this->_speed){
        this->_speed = min(this->_speed + SMOOTH_STEP, this->_targetSpeed);
    } else if (this->_targetSpeed < this->_speed){
        this->_speed = max(this->_speed - SMOOTH_STEP, this->_targetSpeed);
    }
    this->run();
}