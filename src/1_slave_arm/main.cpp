#include <Arduino.h>
#include <protocol.h>
#include <motor.h>

#include "config_arm.h"

Motor motorArm(MotorArm_A, MotorArm_B);
Motor motorBox(MotorBox_A, MotorBox_B);
Motor motorLift(MotorLift_A, MotorLift_B);

MotorReceiver motorReceiver;

int16_t g_arm_pwm = 0;
int16_t g_box_pwm = 0;
int16_t g_lift_pwm = 0;

unsigned long g_prev_arm_command_time = 0;
unsigned long g_prev_box_command_time = 0;
unsigned long g_prev_lift_command_time = 0;
unsigned long prev_control_time = 0;
unsigned long prev_debug_time = 0;

int16_t g_arm_pwm_out = 0;
int16_t g_box_pwm_out = 0;
int16_t g_lift_pwm_out = 0;

bool g_arm_timed_out = false;
bool g_box_timed_out = false;
bool g_lift_timed_out = false;

unsigned long g_rx_byte_count = 0;
uint8_t g_last_rx_byte = 0;

unsigned long g_arm_frame_count = 0;
unsigned long g_box_frame_count = 0;
unsigned long g_lift_frame_count = 0;

#define MOTOR_CMD_TIMEOUT_MS 300 // ms
#define CONTROL_RATE 100        // Hz

void setup(){
    Serial.begin(115200);
    Serial1.begin(115200);

    pinMode(LimitSW_Arm_Front, INPUT_PULLUP);
    pinMode(LimitSW_Arm_Back, INPUT_PULLUP);
    pinMode(LimitSW_Box_Front, INPUT_PULLUP);
    pinMode(LimitSW_Box_Back, INPUT_PULLUP);
    pinMode(LimitSW_Lift_Up, INPUT_PULLUP);
    pinMode(LimitSW_Lift_Down, INPUT_PULLUP);

#if ARM_DEBUG
    Serial.println("Arm Slave Start...");
#endif
}

void loop(){
    while (Serial1.available() > 0){
        uint8_t incomingByte = Serial1.read();

        g_rx_byte_count++;
        g_last_rx_byte = incomingByte;

        motorReceiverFeed(motorReceiver, incomingByte);
    }

    if (motorReceiver.hasNewArmCommand){
        g_arm_pwm = motorReceiver.lastArmCommand.pwm;
        g_prev_arm_command_time = millis();
        g_arm_frame_count++;
        motorReceiver.hasNewArmCommand = false;
    }

    if (motorReceiver.hasNewBoxCommand){
        g_box_pwm = motorReceiver.lastBoxCommand.pwm;
        g_prev_box_command_time = millis();
        g_box_frame_count++;
        motorReceiver.hasNewBoxCommand = false;
    }

    if (motorReceiver.hasNewLiftCommand){
        g_lift_pwm = motorReceiver.lastLiftCommand.pwm;
        g_prev_lift_command_time = millis();
        g_lift_frame_count++;
        motorReceiver.hasNewLiftCommand = false;
    }

    unsigned long now = millis();
    if ((now - prev_control_time) >= (1000 / CONTROL_RATE)){
        prev_control_time = now;

        bool armTimedOut = (now - g_prev_arm_command_time) > MOTOR_CMD_TIMEOUT_MS;
        bool boxTimedOut = (now - g_prev_box_command_time) > MOTOR_CMD_TIMEOUT_MS;
        bool liftTimedOut = (now - g_prev_lift_command_time) > MOTOR_CMD_TIMEOUT_MS;

        g_arm_timed_out = armTimedOut;
        g_box_timed_out = boxTimedOut;
        g_lift_timed_out = liftTimedOut;

        int16_t safeArmPwm = armTimedOut ? 0 : g_arm_pwm;
        int16_t safeBoxPwm = boxTimedOut ? 0 : g_box_pwm;
        int16_t safeLiftPwm = liftTimedOut ? 0 : g_lift_pwm;

        if (digitalRead(LimitSW_Arm_Front) == LOW && safeArmPwm > 0) {
            safeArmPwm = 0;
        }
        if (digitalRead(LimitSW_Box_Front) == LOW && safeBoxPwm > 0) {
            safeBoxPwm = 0;
        }
        // if (digitalRead(LimitSW_Lift_Up) == LOW && safeLiftPwm > 0) {
        //     safeLiftPwm = 0;
        // }

        if (digitalRead(LimitSW_Arm_Back) == LOW && safeArmPwm < 0) {
            safeArmPwm = 0;
        }
        if (digitalRead(LimitSW_Box_Back) == LOW && safeBoxPwm < 0) {
            safeBoxPwm = 0;
        }
        // if (digitalRead(LimitSW_Lift_Down) == LOW && safeLiftPwm < 0) {
        //     safeLiftPwm = 0;
        // }

        g_arm_pwm_out = safeArmPwm;
        g_box_pwm_out = safeBoxPwm;
        g_lift_pwm_out = safeLiftPwm;

        motorArm.run(safeArmPwm);
        motorBox.run(safeBoxPwm);
        motorLift.run(safeLiftPwm);
    }

#if ARM_DEBUG
    if ((now - prev_debug_time) >= (1000 / ARM_DEBUG_RATE)){
        Serial.print("RX,");

        Serial.print("bytes=");
        Serial.print(g_rx_byte_count);

        Serial.print(",last=0x");
        Serial.print(g_last_rx_byte, HEX);

        Serial.print(",frameArm=");
        Serial.print(g_arm_frame_count);

        Serial.print(",frameBox=");
        Serial.print(g_box_frame_count);

        Serial.print(",frameLift=");
        Serial.println(g_lift_frame_count);

        // ค่า PWM ดิบที่รับมาจาก Master
        Serial.print("CMD,");

        Serial.print("arm=");
        Serial.print(g_arm_pwm);

        Serial.print(",box=");
        Serial.print(g_box_pwm);

        Serial.print(",lift=");
        Serial.println(g_lift_pwm);

        // ค่า PWM ที่สั่งออกมอเตอร์จริง ถ้าไม่ตรงกับ CMD แปลว่าโดน timeout หรือลิมิตสวิตช์ตัด
        Serial.print("OUT,");

        Serial.print("arm=");
        Serial.print(g_arm_pwm_out);

        Serial.print(",box=");
        Serial.print(g_box_pwm_out);

        Serial.print(",lift=");
        Serial.println(g_lift_pwm_out);

        // 1 = ขาดการติดต่อกับ Master เกิน MOTOR_CMD_TIMEOUT_MS แล้วสั่งหยุดเอง
        Serial.print("TIMEOUT,");

        Serial.print("arm=");
        Serial.print(g_arm_timed_out ? 1 : 0);

        Serial.print(",box=");
        Serial.print(g_box_timed_out ? 1 : 0);

        Serial.print(",lift=");
        Serial.println(g_lift_timed_out ? 1 : 0);

        // ลิมิตสวิตช์เป็น INPUT_PULLUP ปกติอ่านได้ HIGH ตอนกดจะเป็น LOW
        // ตรงนี้พิมพ์เป็น 1 = กดอยู่ (ชนแล้ว) เพื่อให้อ่านง่าย
        Serial.print("LIMIT,");

        Serial.print("armF=");
        Serial.print(digitalRead(LimitSW_Arm_Front) == LOW ? 1 : 0);

        Serial.print(",armB=");
        Serial.print(digitalRead(LimitSW_Arm_Back) == LOW ? 1 : 0);

        Serial.print(",boxF=");
        Serial.print(digitalRead(LimitSW_Box_Front) == LOW ? 1 : 0);

        Serial.print(",boxB=");
        Serial.print(digitalRead(LimitSW_Box_Back) == LOW ? 1 : 0);

        Serial.print(",liftU=");
        Serial.print(digitalRead(LimitSW_Lift_Up) == LOW ? 1 : 0);

        Serial.print(",liftD=");
        Serial.println(digitalRead(LimitSW_Lift_Down) == LOW ? 1 : 0);

        prev_debug_time = now;
    }
#endif
}