#include <Arduino.h>
#include <ps5Controller.h>
#include <protocol.h>

#include "config_esp32.h"
#include "PS5Input.h"

PS5Input joyInput;

HardwareSerial WheelSerial(1);

unsigned long prev_wheel_send_time = 0;

void setup(){
    Serial.begin(115200);
    joyInput.begin();

    WheelSerial.begin(WHEEL_UART_BAUD, SERIAL_8N1, WHEEL_UART_RX, WHEEL_UART_TX);
}

void loop() {
    joyInput.update();
    unsigned long now = millis();
    if ((now - prev_wheel_send_time) >= (1000 / COMMAND_RATE)) {
        prev_wheel_send_time = now;

        PS5Input::cmd_vel velocity = joyInput.getVelocity();
        sendWheelCommand(WheelSerial, velocity.valX, velocity.valY, velocity.valW);
    }
}
