#include <Arduino.h>
#include <ps5Controller.h>
#include "config.h"

void setup(){
    Serial.begin(115200);
    ps5.begin(MAC_PS5); 
}

void loop() {
    if (ps5.isConnected()) {
        Serial.println("PS5 Controller Connected!");
    }
    delay(1000);
}
