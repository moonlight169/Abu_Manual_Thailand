#ifndef PS5_INPUT_H
#define PS5_INPUT_H

#include <Arduino.h>
#include <ps5Controller.h>

class PS5Input{
public:
    PS5Input(const char* mac);

    void begin();

    enum PS5Button {
        Cross, Circle, Triangle, Square,
        L1, R1, Up, Down, Left, Right
    };

    struct cmd_vel {
        float valX = 0.000;
        float valY = 0.000;
        float valW = 0.000;
    };

    void update();
    bool isPressed(PS5Button btn);
    bool wasPressed(PS5Button btn);
    cmd_vel getVelocity();
    bool isConnected();
    void printLog(HardwareSerial &serialPort);

private:
    const char* _mac;
    cmd_vel velocity;

    uint16_t currentButtons = 0;
    uint16_t prevButtons = 0;

    uint16_t readRawButtons();
};

#endif