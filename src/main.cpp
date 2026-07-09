#include <Arduino.h>
#include <ps5Controller.h>

#include "config.h"
#include "PS5Input.h"

PS5Input joyInput;

void setup(){
    Serial.begin(115200);
    joyInput.begin();
}

void loop() {
    joyInput.update();
    if (joyInput.isConnected()) {
        joyInput.printLog(Serial);
    }
}
