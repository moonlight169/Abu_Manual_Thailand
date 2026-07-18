#include <Arduino.h>
#include <ps5Controller.h>
#include <protocol.h>
#include <relay.h>

#include "config_esp32.h"
#include "PS5Input.h"

PS5Input joyInput(MAC_PS5_WHITE);

Relay relay1(Relay1);
Relay relay2(Relay2);
Relay relay3(Relay3);
Relay relay4(Relay4);
Relay relay5(Relay5);
Relay relay6(Relay6);

HardwareSerial WheelSerial(1);

unsigned long prev_wheel_send_time = 0;

void digitalControl(){
    if (joyInput.wasPressed(PS5Input::Cross)) {
        relay1.toggle();
    } 
    
    if (joyInput.isPressed(PS5Input::Square)) {
        relay2.write(0);
    } else {
        relay2.write(1);
    }
}

void setup(){
    Serial.begin(115200);
    joyInput.begin();

    WheelSerial.begin(WHEEL_UART_BAUD, SERIAL_8N1, WHEEL_UART_RX, WHEEL_UART_TX);
}

void loop() {
    unsigned long now = millis();
    if ((now - prev_wheel_send_time) >= (1000 / COMMAND_RATE)) {
        prev_wheel_send_time = now;

        joyInput.update();
        PS5Input::cmd_vel velocity = joyInput.getVelocity();
        sendWheelCommand(WheelSerial, velocity.valX, velocity.valY, velocity.valW);

        digitalControl();
    }
}
