#include <Arduino.h>
#include <ps5Controller.h>
#include <protocol.h>
#include <relay.h>
#include <PS5Input.h>
#include <Adafruit_BNO08x.h>

#include "config_esp32.h"

Adafruit_BNO08x _bno08x(BNO08X_RESET);
sh2_SensorValue_t _sensorValue;
static bool _imuReady = false;

// interval หน่วยไมโครวินาที: 2500us = 400Hz
static const uint32_t IMU_REPORT_INTERVAL_US = 2500;

static float _imuYaw = 0;
static float _imuRoll = 0;
static float _imuPitch = 0;

float g_targetYawDeg = 0.000;

float wrapAngle180(float angleDeg) {
    while (angleDeg > 180.000)  angleDeg -= 360.000;
    while (angleDeg < -180.000) angleDeg += 360.000;
    return angleDeg;
}

static void quaternionToEuler(float qr, float qi, float qj, float qk,
                              float &yaw, float &pitch, float &roll){
    float sqr = qr * qr;
    float sqi = qi * qi;
    float sqj = qj * qj;
    float sqk = qk * qk;

    yaw = atan2f(2.0f * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr));
    pitch = asinf(-2.0f * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr));
    roll = atan2f(2.0f * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr));

    // แปลงเป็นองศา
    yaw   *= RAD_TO_DEG;
    pitch *= RAD_TO_DEG;
    roll  *= RAD_TO_DEG;

    // Normalize yaw ให้อยู่ในช่วง -180 ถึง 180 องศา (จุดตัดอยู่ด้านหลังหุ่นยนต์ ไม่ใช่ด้านหน้า)
    yaw = wrapAngle180(yaw);
}

static void updateImu(){
    if (!_imuReady) return;
    if (_bno08x.wasReset()) {
        _bno08x.enableReport(SH2_GYRO_INTEGRATED_RV, IMU_REPORT_INTERVAL_US);
    }
    if (_bno08x.getSensorEvent(&_sensorValue)) {
        if (_sensorValue.sensorId == SH2_GYRO_INTEGRATED_RV) {
            float qw = _sensorValue.un.gyroIntegratedRV.real;
            float qx = _sensorValue.un.gyroIntegratedRV.i;
            float qy = _sensorValue.un.gyroIntegratedRV.j;
            float qz = _sensorValue.un.gyroIntegratedRV.k;
            quaternionToEuler(qw, qx, qy, qz, _imuYaw, _imuRoll, _imuPitch);
        }
    }
}

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

bool g_liftMode = false;

const int STICK_DEADZONE = 10;

//----------------------------------------
const float wheel_Walk_Normal = 1.800;
const float wheel_Walk_Slow = 0.700;
const float wheel_Walk_SuperSlow = 0.300;

const float wheel_Slide_Normal = 1.800;
const float wheel_Slide_Slow = 0.700;
const float wheel_Slide_SuperSlow = 0.300;

const float wheel_Turn_Normal = 4.000;
const float wheel_Turn_Slow = 2.000;
const float wheel_Turn_SuperSlow = 0.800;

const float YAW_LOCK_KP = 6.0;

Relay relay1(Relay1);
Relay relay2(Relay2);
Relay relay3(Relay3);
Relay relay4(Relay4);
Relay relay5(Relay5);
Relay relay6(Relay6);

HardwareSerial WheelSerial(1);
HardwareSerial ArmSerial(2);

unsigned long prev_wheel_send_time = 0;
unsigned long prev_imu_print_time = 0;
const unsigned long IMU_PRINT_RATE = 10;

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

        bool manualTurn = (w != 0.000);

        if (manualTurn) {
            g_targetYawDeg = _imuYaw;
        } else if (x != 0.000 || y != 0.000) {
            float yawErrorDeg = wrapAngle180(g_targetYawDeg - _imuYaw);
            // omega ที่ส่งให้ wheel slave เป็นหน่วย rad/s (ตาม Kinematics::calculateRPM)
            // ต้องแปลง yawError จากองศาเป็นเรเดียนก่อนคูณ Kp มิฉะนั้น error องศาเดียว
            // จะกลายเป็นคำสั่ง omega ที่ใหญ่เกินจริง ทำให้ turn_speed ตัดอิ่มตัวแทบทุกครั้ง (bang-bang)
            float yawErrorRad = yawErrorDeg * DEG_TO_RAD;
            w = yawErrorRad * YAW_LOCK_KP;
            w = constrain(w, -turn_speed, turn_speed);
        } else {
            g_targetYawDeg = _imuYaw;
        }

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
        relay3.toggle();
    } 
    
    if (joyInput.wasPressed(PS5Input::Square)) {
        relay2.toggle();
    }

    if (joyInput.wasPressed(PS5Input::Triangle)) {
        relay5.toggle();
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

    if (joyInput.isPressed(PS5Input::Touchpad)) {
        g_liftMode = true;
    } else {
        g_liftMode = false;
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

        if (g_liftMode) {
            lift_pwm = map(ry, -128, 127, -lift_speed, lift_speed);
            arm_pwm = 0;
            box_pwm = 0;
        } else {
            arm_pwm = map(ry, -128, 127, arm_speed, -arm_speed);
            box_pwm = map(ly, -128, 127, -box_speed, box_speed);
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

    Wire.begin(SDA, SCL, 100000);
    relay1.write(1);
    relay2.write(1);
    relay3.write(1);
    relay4.write(1);
    relay5.write(1);
    relay6.write(1);

    WheelSerial.begin(WHEEL_UART_BAUD, SERIAL_8N1, WHEEL_UART_RX, WHEEL_UART_TX);
    ArmSerial.begin(ARM_UART_BAUD, SERIAL_8N1, ARM_UART_RX, ARM_UART_TX);

    if (!_bno08x.begin_I2C()) {
        Serial.println(F(">> BNO085 not found"));
        _imuReady = false;
    } else {
        Serial.println(F(">> BNO085 Found!"));
        // ใช้ Gyro Integrated RV interval 2500us = 400Hz (เร็วกว่า rotation vector เดิม, เหมาะกับ master loop 80Hz)
        if (!_bno08x.enableReport(SH2_GYRO_INTEGRATED_RV, IMU_REPORT_INTERVAL_US)) {
            Serial.println(F(">> Could not enable gyro integrated RV"));
        } else {
            Serial.println(F(">> Gyro Integrated RV enabled at 400Hz"));
        }
        _imuReady = true;
    }
}

void loop() {
    updateImu();

    unsigned long now = millis();

    if ((now - prev_imu_print_time) >= (1000 / IMU_PRINT_RATE)) {
        prev_imu_print_time = now;
        Serial.print("Yaw: ");
        Serial.println(_imuYaw);
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

        digitalControl();
    }
}