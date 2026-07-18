#include "PS5Input.h"
#include "config_esp32.h"

void PS5Input::begin(){
    ps5.begin(MAC_PS5_WHITE);
    Serial.println("PS5 Connected");
}

void PS5Input::update(){
    unsigned long now = millis();

    if ((now - prev_control_time) >= (1000 / COMMAND_RATE)) {
        prev_control_time = now;

        if (ps5.isConnected()){
            float x = 0.000;
            float y = 0.000;
            float w = 0.000;

            if (ps5.Up())    x += 1.000;
            if (ps5.Down())  x -= 1.000;
            if (ps5.Right()) y += 1.000;
            if (ps5.Left())  y -= 1.000;

            if (ps5.R1()) w += 1.000;
            if (ps5.L1()) w -= 1.000;

            velocity.valX = x;
            velocity.valY = y;
            velocity.valW = w;
        } else {
            velocity.valX = 0.000;
            velocity.valY = 0.000;
            velocity.valW = 0.000;
        }
    }
}

PS5Input::cmd_vel PS5Input::getVelocity(){
    return velocity;
}

bool PS5Input::isConnected(){
    return ps5.isConnected();
}

void PS5Input::printLog(HardwareSerial &serialPort) {
    serialPort.print("X: "); serialPort.print(velocity.valX, 3);
    serialPort.print(" | Y: "); serialPort.print(velocity.valY, 3);
    serialPort.print(" | W: "); serialPort.println(velocity.valW, 3);
}