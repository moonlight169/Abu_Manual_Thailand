#ifndef RELAY_H
#define RELAY_H

#include <Arduino.h>

class Relay{
public:
    Relay(int pin);
    void toggle();
    void write(int status);

private:
    bool _state;
    int _pin;
};

#endif