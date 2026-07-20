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

const int ARM_Normal = 255;
const int ARM_Slow = 120;
const int ARM_SuperSlow = 80;

const int BOX_Normal = 255;
const int BOX_Slow = 120;
const int BOX_SuperSlow = 80;

const int LIFT_Normal = 255;
const int LIFT_Slow = 120;
const int LIFT_SuperSlow = 80;

int16_t lift_pwm = 0;
int16_t arm_pwm = 0;
int16_t box_pwm = 0;

//----------------------------------------
const float wheel_Walk_Normal = 1.500;
const float wheel_Walk_Slow = 0.600;
const float wheel_Walk_SuperSlow = 0.300;

const float wheel_Slide_Normal = 1.500;
const float wheel_Slide_Slow = 0.600;
const float wheel_Slide_SuperSlow = 0.300;

const float wheel_Turn_Normal = 2.000;
const float wheel_Turn_Slow = 1.000;
const float wheel_Turn_SuperSlow = 0.500;

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
        float walk_speed = wheel_Walk_Normal;
        float slide_speed = wheel_Slide_Normal;
        float turn_speed = wheel_Turn_Normal;

        if (ps5.L2()) {
            walk_speed = wheel_Walk_SuperSlow;
            slide_speed = wheel_Slide_SuperSlow;
            turn_speed = wheel_Turn_SuperSlow;
        } else if (ps5.R2()) {
            walk_speed = wheel_Walk_Slow;
            slide_speed = wheel_Slide_Slow;
            turn_speed = wheel_Turn_Slow;
        }

        float x = 0.000;
        float y = 0.000;
        float w = 0.000;

        if (ps5.Up()    || ps5.UpRight()   || ps5.UpLeft())    x += walk_speed;
        if (ps5.Down()  || ps5.DownRight() || ps5.DownLeft())  x -= walk_speed;
        if (ps5.Right() || ps5.UpRight()   || ps5.DownRight()) y -= slide_speed;
        if (ps5.Left()  || ps5.UpLeft()    || ps5.DownLeft())  y += slide_speed;

        if (ps5.R1()) w -= turn_speed;
        if (ps5.L1()) w += turn_speed;

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

void armControl(){
    if (ps5.isConnected()){
        int arm_speed = ARM_Normal;
        int box_speed = BOX_Normal;

        if (ps5.L2()) {
            arm_speed = ARM_SuperSlow;
            box_speed = BOX_SuperSlow;
        } else if (ps5.R2()) {
            arm_speed = ARM_Slow;
            box_speed = BOX_Slow;
        }

        arm_pwm = map(ps5.LStickY(), -128, 127, -arm_speed, arm_speed);
        box_pwm = map(ps5.RStickY(), -128, 127, -box_speed, box_speed);
    } else {
        arm_pwm = 0;
        box_pwm = 0;
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

        armControl();
        digitalControl();
    }
}
