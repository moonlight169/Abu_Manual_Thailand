#include "relay.h"

Relay::Relay(int pin) : _pin(pin), _state(false) {
    pinMode(_pin, OUTPUT);
}

void Relay::write(int status){
    digitalWrite(_pin, status ? HIGH : LOW);
}

void Relay::toggle(){
    _state = !_state;
    digitalWrite(_pin, _state ? HIGH : LOW);
}