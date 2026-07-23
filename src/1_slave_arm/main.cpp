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

#define MOTOR_CMD_TIMEOUT_MS 300 // ms
#define CONTROL_RATE 100        // Hz

void setup(){
    Serial1.begin(115200);

    pinMode(LimitSW_Arm_Front, INPUT_PULLUP);
    pinMode(LimitSW_Arm_Back, INPUT_PULLUP);
    pinMode(LimitSW_Box_Front, INPUT_PULLUP);
    pinMode(LimitSW_Box_Back, INPUT_PULLUP);
    pinMode(LimitSW_Lift_Up, INPUT_PULLUP);
    pinMode(LimitSW_Lift_Down, INPUT_PULLUP);
}

void loop(){
    while (Serial1.available() > 0){
        uint8_t incomingByte = Serial1.read();
        motorReceiverFeed(motorReceiver, incomingByte);
    }

    if (motorReceiver.hasNewArmCommand){
        g_arm_pwm = motorReceiver.lastArmCommand.pwm;
        g_prev_arm_command_time = millis();
        motorReceiver.hasNewArmCommand = false;
    }

    if (motorReceiver.hasNewBoxCommand){
        g_box_pwm = motorReceiver.lastBoxCommand.pwm;
        g_prev_box_command_time = millis();
        motorReceiver.hasNewBoxCommand = false;
    }

    if (motorReceiver.hasNewLiftCommand){
        g_lift_pwm = motorReceiver.lastLiftCommand.pwm;
        g_prev_lift_command_time = millis();
        motorReceiver.hasNewLiftCommand = false;
    }

    unsigned long now = millis();
    if ((now - prev_control_time) >= (1000 / CONTROL_RATE)){
        prev_control_time = now;

        bool armTimedOut = (now - g_prev_arm_command_time) > MOTOR_CMD_TIMEOUT_MS;
        bool boxTimedOut = (now - g_prev_box_command_time) > MOTOR_CMD_TIMEOUT_MS;
        bool liftTimedOut = (now - g_prev_lift_command_time) > MOTOR_CMD_TIMEOUT_MS;

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

        motorArm.run(safeArmPwm);
        motorBox.run(safeBoxPwm);
        motorLift.run(safeLiftPwm);
    }
}