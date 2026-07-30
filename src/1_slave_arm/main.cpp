#include <Arduino.h>
#include <protocol.h>
#include <motor.h>
#include <encoder.h>

#include "config_arm.h"

Motor motorArm(MotorArm_A, MotorArm_B);
Motor motorBox(MotorBox_A, MotorBox_B);
Motor motorLift(MotorLift_A, MotorLift_B);

Encoder encArm(EncArm_A, EncArm_B, 0.00);

void isrArm_A() { encArm.handleA(); }
void isrArm_B() { encArm.handleB(); }

MotorReceiver motorReceiver;

int16_t g_arm_pwm = 0;
int16_t g_box_pwm = 0;
int16_t g_lift_pwm = 0;

unsigned long g_prev_arm_command_time = 0;
unsigned long g_prev_box_command_time = 0;
unsigned long g_prev_lift_command_time = 0;
unsigned long prev_control_time = 0;
unsigned long prev_debug_time = 0;
unsigned long prev_enc_debug_time = 0;

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
unsigned long g_arm_pos_frame_count = 0;

#define MOTOR_CMD_TIMEOUT_MS 300 // ms
#define CONTROL_RATE 100        // Hz

#define ARM_PULSE_STOP_NONE 0    // ยังไม่เคยสั่ง
#define ARM_PULSE_STOP_DONE 1    // ถึงเป้าหมายแล้ว
#define ARM_PULSE_STOP_TIMEOUT 2 // วิ่งนานเกิน ARM_PULSE_TIMEOUT_MS
#define ARM_PULSE_STOP_STALL 3   // ระยะไม่ลด = วิ่งผิดทาง/แขนติด/encoder ไม่มา
#define ARM_PULSE_STOP_LIMIT 4   // ชนลิมิตสวิตช์ฝั่งที่กำลังวิ่งไป
#define ARM_PULSE_STOP_MANUAL 5  // คนขับโยกสติ๊กแทรก
#define ARM_PULSE_STOP_LINK 6    // ขาดการติดต่อกับ master

bool g_arm_pulse_active = false;
long g_arm_pulse_target = 0;
int16_t g_arm_pulse_pwm = ARM_PULSE_PWM;
unsigned long g_arm_pulse_start_time = 0;

unsigned long g_arm_pulse_check_time = 0;
unsigned long g_arm_pulse_escape_time = 0;
long g_arm_pulse_last_abs_error = 0;
uint8_t g_arm_pulse_stop_reason = ARM_PULSE_STOP_NONE;
bool g_arm_pulse_flip_done = false;

bool g_arm_homed = false;

int8_t g_arm_enc_dir = ARM_ENC_DIR;
int8_t g_arm_pwm_dir = ARM_PULSE_DIR;
bool g_arm_dir_learned = false;
long g_arm_learn_prev_count = 0;

long arm_getPos(){
    return (long)g_arm_enc_dir * encArm.getCount();
}

void armLearnDirection(int16_t appliedPwm, bool zeroPressed){
    return; 
}

void arm_pulseStop(uint8_t reason){
    g_arm_pulse_active = false;
    g_arm_pulse_stop_reason = reason;
}

void arm_pulseTo(long targetCount, int16_t pwm){
    g_arm_pulse_target = targetCount;
    g_arm_pulse_pwm = constrain((int)abs(pwm), 0, 255);
    g_arm_pulse_active = true;
    g_arm_pulse_start_time = millis();

    g_arm_pulse_check_time = g_arm_pulse_start_time;
    g_arm_pulse_escape_time = g_arm_pulse_start_time;
    g_arm_pulse_last_abs_error = labs(targetCount - arm_getPos());
    g_arm_pulse_stop_reason = ARM_PULSE_STOP_NONE;
    g_arm_pulse_flip_done = false;
}

void arm_pulseTo(long targetCount){
    arm_pulseTo(targetCount, ARM_PULSE_PWM);
}

void arm_pulseCancel(){
    arm_pulseStop(ARM_PULSE_STOP_MANUAL);
}

bool arm_pulseBusy(){
    return g_arm_pulse_active;
}

bool armPulseTryFlip(unsigned long now, long absError){
    return false;
}

int16_t armPulseCompute(unsigned long now, bool zeroPressed, bool farPressed){
    if (!g_arm_pulse_active){
        return 0;
    }

    if ((now - g_arm_pulse_start_time) > ARM_PULSE_TIMEOUT_MS){
        arm_pulseStop(ARM_PULSE_STOP_TIMEOUT);
        return 0;
    }

    long error = g_arm_pulse_target - arm_getPos();
    long absError = labs(error);

    if (absError <= ARM_PULSE_TOLERANCE){
        arm_pulseStop(ARM_PULSE_STOP_DONE);
        return 0;
    }

    if (zeroPressed && error < 0){
        arm_pulseStop(ARM_PULSE_STOP_LIMIT);
        return 0;
    }
    if (farPressed && error > 0){
        arm_pulseStop(ARM_PULSE_STOP_LIMIT);
        return 0;
    }

    if (zeroPressed){
        if ((now - g_arm_pulse_escape_time) > ARM_PULSE_ESCAPE_MS){
            if (!armPulseTryFlip(now, absError)){
                arm_pulseStop(ARM_PULSE_STOP_STALL);
                return 0;
            }
        }

        g_arm_pulse_check_time = now;
        g_arm_pulse_last_abs_error = absError;
    } else {
        g_arm_pulse_escape_time = now;

        if ((now - g_arm_pulse_check_time) >= ARM_PULSE_STALL_MS){
            if ((g_arm_pulse_last_abs_error - absError) < ARM_PULSE_MIN_PROGRESS){
                if (!armPulseTryFlip(now, absError)){
                    arm_pulseStop(ARM_PULSE_STOP_STALL);
                    return 0;
                }
            } else {
                g_arm_pulse_check_time = now;
                g_arm_pulse_last_abs_error = absError;
            }
        }
    }

    long magnitude = g_arm_pulse_pwm;
    if (absError < ARM_PULSE_BRAKE_ZONE){
        magnitude = (long)g_arm_pulse_pwm * absError / ARM_PULSE_BRAKE_ZONE;
        if (magnitude < ARM_PULSE_MIN_PWM){
            magnitude = ARM_PULSE_MIN_PWM;
        }
    }

    long signedPwm = (error > 0) ? magnitude : -magnitude;

    return (int16_t)(g_arm_pwm_dir * signedPwm);
}

void setup(){
    Serial.begin(115200);
    Serial1.begin(115200);

    pinMode(LimitSW_Arm_Front, INPUT_PULLUP);
    pinMode(LimitSW_Arm_Back, INPUT_PULLUP);
    pinMode(LimitSW_Box_Front, INPUT_PULLUP);
    pinMode(LimitSW_Box_Back, INPUT_PULLUP);
    pinMode(LimitSW_Lift_Up, INPUT_PULLUP);
    pinMode(LimitSW_Lift_Down, INPUT_PULLUP);

    pinMode(EncArm_A, INPUT_PULLUP);
    pinMode(EncArm_B, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(EncArm_A), isrArm_A, CHANGE);
    attachInterrupt(digitalPinToInterrupt(EncArm_B), isrArm_B, CHANGE);

    encArm.reset();

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

    if (motorReceiver.hasNewArmPosCommand){
        int16_t requestedPwm = motorReceiver.lastArmPosCommand.pwm;

        if (requestedPwm == 0){
            arm_pulseTo(motorReceiver.lastArmPosCommand.target);
        } else {
            arm_pulseTo(motorReceiver.lastArmPosCommand.target, requestedPwm);
        }

        g_arm_pos_frame_count++;
        motorReceiver.hasNewArmPosCommand = false;
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

        bool armFrontPressed = (digitalRead(LimitSW_Arm_Front) == LOW);
        bool armBackPressed = (digitalRead(LimitSW_Arm_Back) == LOW);

#if ARM_ZERO_AT_BACK_LIMIT
        bool armZeroPressed = armBackPressed;
        bool armFarPressed = armFrontPressed;
#else
        bool armZeroPressed = armFrontPressed;
        bool armFarPressed = armBackPressed;
#endif

        if (armZeroPressed){
            encArm.reset();
            g_arm_homed = true;
        }

        if (armTimedOut && g_arm_pulse_active){
            arm_pulseStop(ARM_PULSE_STOP_LINK);
        }

        bool armManualActive = (abs(safeArmPwm) > ARM_MANUAL_OVERRIDE_PWM);

        bool atZeroSide = armZeroPressed || (g_arm_homed && arm_getPos() <= 0);

        if (armManualActive){
            if (g_arm_pulse_active){
                arm_pulseStop(ARM_PULSE_STOP_MANUAL);
            }

            // ตัดลิมิตด้วยทิศที่รู้จริง (g_arm_pwm_dir = ทิศ PWM ที่วิ่งออกจากจุด 0)
            bool pwmTowardFar = ((safeArmPwm > 0) == (g_arm_pwm_dir > 0));

            if (atZeroSide && !pwmTowardFar) {
                safeArmPwm = 0;
            }
            if (armFarPressed && pwmTowardFar) {
                safeArmPwm = 0;
            }
        } else {
            safeArmPwm = armPulseCompute(now, atZeroSide, armFarPressed);
        }

        if (digitalRead(LimitSW_Box_Front) == LOW && safeBoxPwm > 0) {
            safeBoxPwm = 0;
        }
        // if (digitalRead(LimitSW_Lift_Up) == LOW && safeLiftPwm > 0) {
        //     safeLiftPwm = 0;
        // }

        if (digitalRead(LimitSW_Box_Back) == LOW && safeBoxPwm < 0) {
            safeBoxPwm = 0;
        }
        // if (digitalRead(LimitSW_Lift_Down) == LOW && safeLiftPwm < 0) {
        //     safeLiftPwm = 0;
        // }

        armLearnDirection(g_arm_pwm_out, armZeroPressed);

        g_arm_pwm_out = safeArmPwm;
        g_box_pwm_out = safeBoxPwm;
        g_lift_pwm_out = safeLiftPwm;

        motorArm.run(safeArmPwm);
        motorBox.run(safeBoxPwm);
        motorLift.run(safeLiftPwm);
    }

#if ARM_ENC_DEBUG
    if ((now - prev_enc_debug_time) >= (1000 / ARM_ENC_DEBUG_RATE)){
        Serial.print("ENC,raw=");
        Serial.print(encArm.getCount());

        Serial.print(",pos=");
        Serial.print(arm_getPos());

        Serial.print(",homed=");
        Serial.print(g_arm_homed ? 1 : 0);

        Serial.print(",swB=");
        Serial.print(digitalRead(LimitSW_Arm_Back) == LOW ? 1 : 0);

        Serial.print(",swF=");
        Serial.print(digitalRead(LimitSW_Arm_Front) == LOW ? 1 : 0);

        Serial.print(",pulse=");
        Serial.print(g_arm_pulse_active ? 1 : 0);

        Serial.print(",target=");
        Serial.print(g_arm_pulse_target);

        Serial.print(",out=");
        Serial.print(g_arm_pwm_out);

        Serial.print(",encd=");
        Serial.print(g_arm_enc_dir);

        Serial.print(",pwmd=");
        Serial.print(g_arm_pwm_dir);

        Serial.print(",lrn=");
        Serial.print(g_arm_dir_learned ? 1 : 0);

        Serial.print(",stop=");
        Serial.println(g_arm_pulse_stop_reason);

        prev_enc_debug_time = now;
    }
#endif

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
        Serial.print(g_lift_frame_count);

        Serial.print(",framePos=");
        Serial.println(g_arm_pos_frame_count);

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