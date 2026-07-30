#include <Arduino.h>
#include <ps5Controller.h>
#include <protocol.h>
#include <relay.h>
#include <PS5Input.h>

#include "config_esp32.h"

PS5Input joyInput(PS5_MAC_IN_USE);

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

bool touchpadMode = false;

const int32_t ARM_POS_TRIANGLE = 90000;

const int16_t ARM_POS_TRIANGLE_PWM = 0;

const uint8_t ARM_POS_TX_REPEAT = 3;

const int STICK_DEADZONE = 10;

//---------------------------------------l-
const float wheel_Walk_Normal = 8.00;
const float wheel_Walk_Slow = 0.700;
const float wheel_Walk_SuperSlow = 0.300;

const float wheel_Slide_Normal = 6.00;
const float wheel_Slide_Slow = 0.700;
const float wheel_Slide_SuperSlow = 0.300;

const float wheel_Turn_Normal = 9.800;
const float wheel_Turn_Slow = 2.000;
const float wheel_Turn_SuperSlow = 0.800;

Relay relay1(Relay1);
Relay relay2(Relay2);
Relay relay3(Relay3);
Relay relay4(Relay4);
Relay relay5(Relay5);
Relay relay6(Relay6);

HardwareSerial WheelSerial(1);
HardwareSerial ArmSerial(2);

unsigned long prev_wheel_send_time = 0;
unsigned long prev_debug_print_time = 0;
const unsigned long DEBUG_PRINT_RATE = 10;

unsigned long g_arm_tx_count = 0;
unsigned long g_arm_pos_tx_count = 0;

unsigned long prev_status_time = 0;
unsigned long prev_status_print_time = 0;
bool status_led_on = false;
bool prev_ps5_connected = false;

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
    if (joyInput.isPressed(PS5Input::Touchpad)) {
        touchpadMode = true;
    } else {
        touchpadMode = false;
    }

    if (joyInput.wasPressed(PS5Input::Cross)) {
        relay3.toggle();
    }
    
    if (joyInput.wasPressed(PS5Input::Square)) {
        relay2.toggle();
    }

    if (joyInput.wasPressed(PS5Input::Triangle)) {
        if (touchpadMode) {
            // กดทีเดียวจบ: slave จะวิ่งไปตำแหน่งเองต่อ ปล่อย touchpad ได้เลยไม่ต้องกดค้าง
            for (uint8_t i = 0; i < ARM_POS_TX_REPEAT; i++) {
                sendArmPosCommand(ArmSerial, ARM_POS_TRIANGLE, ARM_POS_TRIANGLE_PWM);
            }

            g_arm_pos_tx_count++;
        } else {
            relay5.toggle();
        }
    }

    if (joyInput.wasPressed(PS5Input::Circle)) {
        relay4.toggle();
    } 
    
    if (joyInput.wasPressed(PS5Input::Share)) {
        relay1.toggle();
    }

    if (joyInput.wasPressed(PS5Input::Options)) {
        relay6.toggle();
    }
}

void armControl(){
    if (ps5.isConnected()){
        int arm_speed = ARM_Normal;
        int box_speed = BOX_Normal;
        int lift_speed = LIFT_Normal;

        if (ps5.L2()) {
            arm_speed = ARM_SuperSlow;
            box_speed = BOX_SuperSlow;
            lift_speed = LIFT_SuperSlow;
        } else if (ps5.R2()) {
            arm_speed = ARM_Slow;
            box_speed = BOX_Slow;
            lift_speed = LIFT_Slow;
        }

        int ly = ps5.LStickY();
        int ry = ps5.RStickY();

        if (abs(ly) < STICK_DEADZONE) ly = 0;
        if (abs(ry) < STICK_DEADZONE) ry = 0;

        if (touchpadMode) {
            lift_pwm = map(ry, -128, 127, -lift_speed, lift_speed);
            arm_pwm = 0;
            box_pwm = 0;
        } else {
            arm_pwm = map(ly, -128, 127, arm_speed, -arm_speed);
            box_pwm = map(ry, -128, 127, -box_speed, box_speed);
            lift_pwm = 0;
        }
    } else {
        arm_pwm = 0;
        box_pwm = 0;
        lift_pwm = 0;
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

    while (ArmSerial.available() > 0) {
        ArmSerial.read();
    }

    if ((now - prev_debug_print_time) >= (1000 / DEBUG_PRINT_RATE)) {
#if MASTER_DEBUG_ARM
        Serial.print("ARMTX,");

        Serial.print("tx=");
        Serial.print(g_arm_tx_count);

        Serial.print(",ps5=");
        Serial.print(ps5.isConnected() ? 1 : 0);

        Serial.print(",arm=");
        Serial.print(arm_pwm);

        Serial.print(",box=");
        Serial.print(box_pwm);

        Serial.print(",lift=");
        Serial.print(lift_pwm);

        Serial.print(",pad=");
        Serial.print(touchpadMode ? 1 : 0);

        // จำนวนครั้งที่ส่งคำสั่งวิ่งไปตำแหน่ง (Touchpad + สามเหลี่ยม)
        Serial.print(",posTx=");
        Serial.println(g_arm_pos_tx_count);
#endif

        prev_debug_print_time = now;
    }

    if ((now - prev_wheel_send_time) >= (1000 / COMMAND_RATE)) {
        prev_wheel_send_time = now;

        joyInput.update();

        updateControl();
        sendWheelCommand(WheelSerial, velocity.valX, velocity.valY, velocity.valW);

        armControl();
        sendArmCommand(ArmSerial, arm_pwm);
        sendBoxCommand(ArmSerial, box_pwm);
        sendLiftCommand(ArmSerial, lift_pwm);
        g_arm_tx_count++;

        digitalControl();
    }
}