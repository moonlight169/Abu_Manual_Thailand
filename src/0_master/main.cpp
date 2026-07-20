#include <Arduino.h>
#include <ps5Controller.h>
#include <protocol.h>
#include <relay.h>
#include <PS5Input.h>

#include "config_esp32.h"

PS5Input joyInput(MAC_PS5_WHITE);

struct CmdVel {
    float valX = 0.000;
    float valY = 0.000;
    float valW = 0.000;
};

CmdVel velocity;

Relay relay1(Relay1);
Relay relay2(Relay2);
Relay relay3(Relay3);
Relay relay4(Relay4);
Relay relay5(Relay5);
Relay relay6(Relay6);

HardwareSerial WheelSerial(1);
HardwareSerial ArmSerial(2);

unsigned long prev_wheel_send_time = 0;

void updateControl(){
    if (ps5.isConnected()){
        float x = 0.000;
        float y = 0.000;
        float w = 0.000;

        if (ps5.Up()    || ps5.UpRight()   || ps5.UpLeft())    x += 1.500;
        if (ps5.Down()  || ps5.DownRight() || ps5.DownLeft())  x -= 1.500;
        if (ps5.Right() || ps5.UpRight()   || ps5.DownRight()) y -= 1.500;
        if (ps5.Left()  || ps5.UpLeft()    || ps5.DownLeft())  y += 1.500;

        if (ps5.R1()) w -= 2.000;
        if (ps5.L1()) w += 2.000;

        velocity.valX = x;
        velocity.valY = y;
        velocity.valW = w;
    } else {
        velocity.valX = 0.000;
        velocity.valY = 0.000;
        velocity.valW = 0.000;
    }
}

void digitalControl(){
    if (joyInput.wasPressed(PS5Input::Cross)) {
        relay1.toggle();
    } 
    
    if (joyInput.isPressed(PS5Input::Square)) {
        relay2.write(0);
    } else {
        relay2.write(1);
    }

    if (joyInput.wasPressed(PS5Input::Triangle)) {
        relay3.toggle();
    } 

    if (joyInput.wasPressed(PS5Input::Circle)) {
        relay4.toggle();
    } 
    
    if (joyInput.wasPressed(PS5Input::Share)) {
        relay5.toggle();
    }

    if (joyInput.wasPressed(PS5Input::Options)) {
        relay6.toggle();
    } 
}

void setup(){
    Serial.begin(115200);
    joyInput.begin();
    relay1.write(1);
    relay2.write(1);
    relay3.write(1);
    relay4.write(1);
    relay5.write(1);
    relay6.write(1);

    WheelSerial.begin(WHEEL_UART_BAUD, SERIAL_8N1, WHEEL_UART_RX, WHEEL_UART_TX);
    ArmSerial.begin(ARM_UART_BAUD, SERIAL_8N1, ARM_UART_RX, ARM_UART_TX);
}

void loop() {
    unsigned long now = millis();
    if ((now - prev_wheel_send_time) >= (1000 / COMMAND_RATE)) {
        prev_wheel_send_time = now;

        joyInput.update();

        updateControl();
        sendWheelCommand(WheelSerial, velocity.valX, velocity.valY, velocity.valW);

        digitalControl();
    }
}
