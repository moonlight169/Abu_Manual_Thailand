#include <Arduino.h>
#include <protocol.h>
#include <wheel.h>
#include <Kinematics.h>
#include <PID.h>

#include "config_wheels.h"

Wheel wheelFL(MotorFL_A, MotorFL_B, EncFL_A, EncFL_B, PULSE_PER_REV, "FL");
Wheel wheelFR(MotorFR_A, MotorFR_B, EncFR_A, EncFR_B, PULSE_PER_REV, "FR");
Wheel wheelRL(MotorRL_A, MotorRL_B, EncRL_A, EncRL_B, PULSE_PER_REV, "RL");
Wheel wheelRR(MotorRR_A, MotorRR_B, EncRR_A, EncRR_B, PULSE_PER_REV, "RR");

WheelReceiver wheelReceiver;

Kinematics kinematics(Kinematics::MECANUM, MAX_RPM, WHEEL_RADIUS, FR_WHEELS_DISTANCE, LR_WHEELS_DISTANCE);

PID MotorFL_Pid(PWM_MIN, PWM_MAX, FL_K_P, FL_K_I, FL_K_D);
PID MotorFR_Pid(PWM_MIN, PWM_MAX, FR_K_P, FR_K_I, FR_K_D);
PID MotorRL_Pid(PWM_MIN, PWM_MAX, RL_K_P, RL_K_I, RL_K_D);
PID MotorRR_Pid(PWM_MIN, PWM_MAX, RR_K_P, RR_K_I, RR_K_D);

PID Heading_Pid(-HEADING_MAX_RPM, HEADING_MAX_RPM, HEAD_K_P, HEAD_K_I, HEAD_K_D);

float g_req_linear_vel_x = 0;
float g_req_linear_vel_y = 0;
float g_req_angular_vel_z = 0;
float g_gyro_yaw_deg = 0;

bool g_gyro_valid = false;
bool g_has_received_command = false;

bool  g_heading_active = false;
float g_heading_target_deg = 0;
float g_heading_error_deg = 0;
float g_heading_correction = 0;
float g_prev_gyro_yaw_deg = 0;

float current_rpm1 = 0, current_rpm2 = 0, current_rpm3 = 0, current_rpm4 = 0;
Kinematics::rpm g_req_rpm;

// PWM ที่ PID สั่งออกมาในรอบล่าสุด (ก่อนถูก smoothRun ไล่ทีละ step)
// เก็บไว้เทียบกับ wheel.getPWM() ที่เป็นค่าจริงที่ออกขามอเตอร์
int g_pwm_cmd1 = 0, g_pwm_cmd2 = 0, g_pwm_cmd3 = 0, g_pwm_cmd4 = 0;

unsigned long prev_control_time = 0;
unsigned long g_prev_command_time = 0;
unsigned long prev_debug_time = 0;

#define COMMAND_RATE 100         // Hz
#define WHEEL_CMD_TIMEOUT_MS 300 // ms
#define DEBUG_RATE 5             // Hz

void isrFL_A() { wheelFL.handleA(); }
void isrFL_B() { wheelFL.handleB(); }

void isrFR_A() { wheelFR.handleA(); }
void isrFR_B() { wheelFR.handleB(); }

void isrRL_A() { wheelRL.handleA(); }
void isrRL_B() { wheelRL.handleB(); }

void isrRR_A() { wheelRR.handleA(); }
void isrRR_B() { wheelRR.handleB(); }

void moveBase();
void stopBase();
float normalizeAngleDeg(float angle_deg);
void resetHeadingAssist();
void scaleRpmWithinLimit(float rpm[4]);

void setup(){
    Serial.begin(115200);
    Serial1.begin(115200);

    Serial.println("Robot Start...");

    MotorFL_Pid.limitIntegral(-WHEEL_I_LIMIT, WHEEL_I_LIMIT);
    MotorFR_Pid.limitIntegral(-WHEEL_I_LIMIT, WHEEL_I_LIMIT);
    MotorRL_Pid.limitIntegral(-WHEEL_I_LIMIT, WHEEL_I_LIMIT);
    MotorRR_Pid.limitIntegral(-WHEEL_I_LIMIT, WHEEL_I_LIMIT);

    Heading_Pid.limitIntegral(-HEADING_I_LIMIT, HEADING_I_LIMIT);
    resetHeadingAssist();

    wheelFL.run(0);
    wheelFR.run(0);
    wheelRL.run(0);
    wheelRR.run(0);

    attachInterrupt(digitalPinToInterrupt(EncFL_A), isrFL_A, CHANGE);
    attachInterrupt(digitalPinToInterrupt(EncFL_B), isrFL_B, CHANGE);

    attachInterrupt(digitalPinToInterrupt(EncFR_A), isrFR_A, CHANGE);
    attachInterrupt(digitalPinToInterrupt(EncFR_B), isrFR_B, CHANGE);

    attachInterrupt(digitalPinToInterrupt(EncRL_A), isrRL_A, CHANGE);
    attachInterrupt(digitalPinToInterrupt(EncRL_B), isrRL_B, CHANGE);

    attachInterrupt(digitalPinToInterrupt(EncRR_A), isrRR_A, CHANGE);
    attachInterrupt(digitalPinToInterrupt(EncRR_B), isrRR_B, CHANGE);
}

void loop() {
    while (Serial1.available() > 0){
        uint8_t incomingByte = Serial1.read();
        wheelReceiverFeed(wheelReceiver, incomingByte);
    }

    if (wheelReceiver.hasNewCommand){
        g_req_linear_vel_x = wheelReceiver.lastCommand.vx;
        g_req_linear_vel_y = wheelReceiver.lastCommand.vy;
        g_req_angular_vel_z = wheelReceiver.lastCommand.omega;
        g_gyro_yaw_deg = wheelReceiver.lastCommand.yaw;
        g_gyro_valid = (wheelReceiver.lastCommand.flags & WHEEL_FLAG_GYRO_VALID) != 0;
        g_prev_command_time = millis();
        g_has_received_command = true;
        wheelReceiver.hasNewCommand = false;
    }

    unsigned long now = millis();

    bool commandTimedOut = (!g_has_received_command) ||
                           ((now - g_prev_command_time) > WHEEL_CMD_TIMEOUT_MS);

    if (commandTimedOut){
        g_req_linear_vel_x = 0;
        g_req_linear_vel_y = 0;
        g_req_angular_vel_z = 0;

        g_gyro_valid = false;
    }

    if ((now - prev_control_time) >= (1000 / COMMAND_RATE)){
        current_rpm1 = wheelFL.getRPM();
        current_rpm2 = wheelFR.getRPM();
        current_rpm3 = wheelRL.getRPM();
        current_rpm4 = wheelRR.getRPM();

        if (commandTimedOut){
            resetHeadingAssist();
            stopBase();
        } else {
            moveBase();
        }
        prev_control_time = now;
    }

#if WHEEL_DEBUG
    if ((now - prev_debug_time) >= (1000 / DEBUG_RATE)){
        // คำสั่งความเร็วที่รับมาจาก Master
        // Serial.print("CMD,");

        // Serial.print("vx=");
        // Serial.print(g_req_linear_vel_x, 2);

        // Serial.print(",vy=");
        // Serial.print(g_req_linear_vel_y, 2);

        // Serial.print(",omega=");
        // Serial.print(g_req_angular_vel_z, 2);

        // Serial.print(",timeout=");
        // Serial.println(commandTimedOut ? 1 : 0);

        // // มุม Yaw ที่ส่งมาพร้อมคำสั่ง
        // Serial.print("GYRO,");

        // Serial.print("yaw=");
        // Serial.print(g_gyro_yaw_deg, 2);

        // Serial.print(",valid=");
        // Serial.println(g_gyro_valid ? 1 : 0);

        // // สถานะ heading assist (ล็อกหัวหุ่นตอนวิ่งตรง)
        // Serial.print("HEAD,");

        // Serial.print("active=");
        // Serial.print(g_heading_active ? 1 : 0);

        // Serial.print(",target=");
        // Serial.print(g_heading_target_deg, 2);

        // Serial.print(",error=");
        // Serial.print(g_heading_error_deg, 2);

        // Serial.print(",corr=");
        // Serial.println(g_heading_correction, 2);

        // Serial.print("RPM REQ,");

        // Serial.print("FL=");
        // Serial.print(g_req_rpm.motor1);
        // Serial.print("/");
        // Serial.print(current_rpm1, 1);

        // Serial.print(",FR=");
        // Serial.print(g_req_rpm.motor2);
        // Serial.print("/");
        // Serial.print(current_rpm2, 1);

        // Serial.print(",RL=");
        // Serial.print(g_req_rpm.motor3);
        // Serial.print("/");
        // Serial.print(current_rpm3, 1);

        // Serial.print(",RR=");
        // Serial.print(g_req_rpm.motor4);
        // Serial.print("/");
        // Serial.println(current_rpm4, 1);

        // Serial.print("COUNT,");

        // Serial.print("FL=");
        // Serial.print(wheelFL.getCount());

        // Serial.print(",FR=");
        // Serial.print(wheelFR.getCount());

        // Serial.print(",RL=");
        // Serial.print(wheelRL.getCount());

        // Serial.print(",RR=");
        // Serial.println(wheelRR.getCount());

        prev_debug_time = now;
    }
#endif

}

float normalizeAngleDeg(float angle_deg)
{
    angle_deg = fmodf(angle_deg + 180.0f, 360.0f);
    if (angle_deg < 0.0f)
        angle_deg += 360.0f;
    return angle_deg - 180.0f;
}

void resetHeadingAssist()
{
    g_heading_active = false;
    g_heading_target_deg = g_gyro_yaw_deg;
    g_heading_error_deg = 0;
    g_heading_correction = 0;

    g_prev_gyro_yaw_deg = g_gyro_yaw_deg;

    Heading_Pid.reset();
}

float updateHeadingAssist()
{
#if HEADING_ASSIST_ENABLED == 0
    return 0.0f;
#else
    if (!g_gyro_valid){
        resetHeadingAssist();
        return 0.0f;
    }

    float yaw_step = fabsf(normalizeAngleDeg(g_gyro_yaw_deg - g_prev_gyro_yaw_deg));
    g_prev_gyro_yaw_deg = g_gyro_yaw_deg;

    if (yaw_step > HEADING_JUMP_LIMIT_DEG){
        resetHeadingAssist();
        return 0.0f;
    }

    bool commandingTurn  = fabsf(g_req_angular_vel_z) > HEADING_OMEGA_DEADZONE;
    bool commandingDrive = (fabsf(g_req_linear_vel_x) > HEADING_MIN_CMD) ||
                           (fabsf(g_req_linear_vel_y) > HEADING_MIN_CMD);

    if (commandingTurn || !commandingDrive){
        resetHeadingAssist();
        return 0.0f;
    }

    if (!g_heading_active){
        g_heading_active = true;
        g_heading_target_deg = g_gyro_yaw_deg;
        Heading_Pid.reset();
    }

    g_heading_error_deg = normalizeAngleDeg(g_heading_target_deg - g_gyro_yaw_deg);

    if (fabsf(g_heading_error_deg) < HEADING_DEADBAND_DEG){
        g_heading_correction = 0.0f;
        return 0.0f;
    }

    g_heading_correction = HEADING_ASSIST_DIR * (float)Heading_Pid.compute(g_heading_error_deg, 0.0f);
    return g_heading_correction;
#endif
}

void scaleRpmWithinLimit(float rpm[4])
{
    float maxAbsRpm = 0.0f;

    for (int i = 0; i < 4; i++){
        float absRpm = fabsf(rpm[i]);
        if (absRpm > maxAbsRpm){
            maxAbsRpm = absRpm;
        }
    }

    if (maxAbsRpm > (float)MAX_RPM){
        float scale = (float)MAX_RPM / maxAbsRpm;

        for (int i = 0; i < 4; i++){
            rpm[i] *= scale;
        }
    }
}

void moveBase()
{
    g_req_rpm = kinematics.getRPM(g_req_linear_vel_x, g_req_linear_vel_y, g_req_angular_vel_z);

    float trim = updateHeadingAssist();

    float target_rpm[4];
    target_rpm[0] = (float)g_req_rpm.motor1 - trim; // FL
    target_rpm[1] = (float)g_req_rpm.motor2 + trim; // FR
    target_rpm[2] = (float)g_req_rpm.motor3 - trim; // RL
    target_rpm[3] = (float)g_req_rpm.motor4 + trim; // RR

    scaleRpmWithinLimit(target_rpm);

    // แยกผลลัพธ์ PID ออกมาเก็บไว้ก่อนสั่งมอเตอร์ เพื่อให้ debug เห็นค่า PWM ที่สั่งจริง
    g_pwm_cmd1 = MotorFL_Pid.compute(target_rpm[0], current_rpm1);
    g_pwm_cmd2 = MotorFR_Pid.compute(target_rpm[1], current_rpm2);
    g_pwm_cmd3 = MotorRL_Pid.compute(target_rpm[2], current_rpm3);
    g_pwm_cmd4 = MotorRR_Pid.compute(target_rpm[3], current_rpm4);

    wheelFL.smoothRun(g_pwm_cmd1);
    wheelFR.smoothRun(g_pwm_cmd2);
    wheelRL.smoothRun(g_pwm_cmd3);
    wheelRR.smoothRun(g_pwm_cmd4);
}

void stopBase()
{
    g_req_rpm.motor1 = 0;
    g_req_rpm.motor2 = 0;
    g_req_rpm.motor3 = 0;
    g_req_rpm.motor4 = 0;

    g_pwm_cmd1 = 0;
    g_pwm_cmd2 = 0;
    g_pwm_cmd3 = 0;
    g_pwm_cmd4 = 0;

    MotorFL_Pid.reset();
    MotorFR_Pid.reset();
    MotorRL_Pid.reset();
    MotorRR_Pid.reset();
    Heading_Pid.reset();

    wheelFL.smoothRun(0);
    wheelFR.smoothRun(0);
    wheelRL.smoothRun(0);
    wheelRR.smoothRun(0);
}