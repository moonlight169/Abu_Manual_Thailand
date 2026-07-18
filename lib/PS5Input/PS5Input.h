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
        L1, R1, L2, R2, Share, Options,
        Touchpad, Up, Down, Right, Left
    };

    void update();
    bool isPressed(PS5Button btn);
    bool wasPressed(PS5Button btn);
    bool isConnected();
    void printLog(HardwareSerial &serialPort);

private:
    const char* _mac;

    uint16_t currentButtons = 0;
    uint16_t prevButtons = 0;

    uint16_t readRawButtons();
};

#endif