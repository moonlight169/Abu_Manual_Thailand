#include "relay.h"

Relay::Relay(int pin) : _pin(pin), _state(false) {
    pinMode(_pin, OUTPUT);
}

void Relay::write(int status){
    _state = status ? true : false;
    digitalWrite(_pin, _state ? HIGH : LOW);
}

void Relay::toggle(){
    _state = !_state;
    digitalWrite(_pin, _state ? HIGH : LOW);
}