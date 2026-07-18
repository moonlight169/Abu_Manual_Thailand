#ifndef PS5_INPUT_H
#define PS5_INPUT_H

#include <Arduino.h>
#include <ps5Controller.h>

class PS5Input{
public:
    struct cmd_vel {
        float valX = 0.000;
        float valY = 0.000;
        float valW = 0.000;
    };

    void begin();
    void update();
    cmd_vel getVelocity();
    bool isConnected();
    void printLog(HardwareSerial &serialPort);

private:
    cmd_vel velocity;
    unsigned long prev_control_time = 0;
};

#endif