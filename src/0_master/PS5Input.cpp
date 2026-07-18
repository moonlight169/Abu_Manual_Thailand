#include "PS5Input.h"
#include "config_esp32.h"

void PS5Input::begin(){
    ps5.begin(MAC_PS5_WHITE);
    Serial.println("PS5 Connected");
}

int PS5Input::applyDeadzone(int value){
    if (abs(value) < deadzoneThreshold) {
        return 0;
    }
    return value;
}

void PS5Input::update(){
    unsigned long now = millis();

    if ((now - prev_control_time) >= (1000 / COMMAND_RATE)) {
        prev_control_time = now;

        if (ps5.isConnected()){
            int rawX = ps5.LStickY();
            int rawY = ps5.RStickX();
            int rawW = ps5.LStickX();

            int filteredX = applyDeadzone(rawX);
            int filteredY = applyDeadzone(rawY);
            int filteredW = applyDeadzone(rawW);

            velocity.valX = (float)filteredX / 128.0;
            velocity.valY = (float)filteredY / 128.0;
            velocity.valW = (float)filteredW / 128.0;
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