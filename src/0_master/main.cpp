#include <Arduino.h>
#include <ps5Controller.h>
#include <protocol.h>
#include <relay.h>
#include <PS5Input.h>
#include <Adafruit_BNO08x.h>

#include "config_esp32.h"

Adafruit_BNO08x bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;

constexpr uint32_t IMU_REPORT_RATE_HZ = 30;

float gyroRollDeg = 0.0f;
float gyroPitchDeg = 0.0f;
float gyroRawYawDeg = 0.0f;
float gyroYawDeg = 0.0f;
float gyroHeadingRad = 0.0f;

float gyroYawOffsetDeg = 0.0f;

bool gyroOnline = false;
uint32_t lastGyroMs = 0;
uint32_t lastPrintMs = 0;

float normalizeAngleDeg(float angle)
{
  while (angle > 180.0f)
    angle -= 360.0f;

  while (angle < -180.0f)
    angle += 360.0f;

  return angle;
}

bool enableGyroReport()
{
  return bno08x.enableReport(
      SH2_GAME_ROTATION_VECTOR,
      IMU_REPORT_RATE_HZ);
}

bool beginGyro()
{
  Wire.begin(21, 22);
  Wire.setClock(400000);

  Serial.println("Searching for BNO085...");

  bool found = bno08x.begin_I2C(0x4A, &Wire);

  if (!found)
  {
    Serial.println("BNO085 not found at 0x4A, trying 0x4B...");
    found = bno08x.begin_I2C(0x4B, &Wire);
  }

  if (!found)
  {
    Serial.println("ERROR: BNO085 NOT FOUND");
    Serial.println("Check wiring:");
    Serial.println("VIN -> 3.3V or 5V according to module");
    Serial.println("GND -> GND");
    Serial.println("SDA -> GPIO21");
    Serial.println("SCL -> GPIO22");

    gyroOnline = false;
    return false;
  }

  if (!enableGyroReport())
  {
    Serial.println("ERROR: Cannot enable Game Rotation Vector");
    gyroOnline = false;
    return false;
  }

  Serial.println("BNO085 READY");
  Serial.println("I2C SDA=21, SCL=22");
  Serial.println("Send R to reset Yaw to 0");
  Serial.println();

  return true;
}


void readGyro()
{
  if (bno08x.wasReset())
  {
    gyroOnline = false;
    Serial.println("WARNING: BNO085 was reset");

    if (!enableGyroReport())
    {
      Serial.println("ERROR: Cannot restart gyro report");
      return;
    }

    Serial.println("BNO085 report restarted");
  }

  while (bno08x.getSensorEvent(&sensorValue))
  {
    if (sensorValue.sensorId != SH2_GAME_ROTATION_VECTOR)
      continue;
    const float qr = sensorValue.un.gameRotationVector.real;
    const float qi = sensorValue.un.gameRotationVector.i;
    const float qj = sensorValue.un.gameRotationVector.j;
    const float qk = sensorValue.un.gameRotationVector.k;

    const float rollRad = atan2f(
        2.0f * (qr * qi + qj * qk),
        1.0f - 2.0f * (qi * qi + qj * qj));

    float sinPitch = 2.0f * (qr * qj - qk * qi);
    sinPitch = constrain(sinPitch, -1.0f, 1.0f);
    const float pitchRad = asinf(sinPitch);

    const float yawRad = atan2f(
        2.0f * (qr * qk + qi * qj),
        1.0f - 2.0f * (qj * qj + qk * qk));

    gyroRollDeg = rollRad * RAD_TO_DEG;
    gyroPitchDeg = pitchRad * RAD_TO_DEG;
    gyroRawYawDeg = yawRad * RAD_TO_DEG;

    gyroYawDeg = normalizeAngleDeg(
        gyroRawYawDeg - gyroYawOffsetDeg);

    gyroHeadingRad = gyroYawDeg * DEG_TO_RAD;

    gyroOnline = true;
    lastGyroMs = millis();
  }
}

void resetGyroYaw()
{
  gyroYawOffsetDeg = gyroRawYawDeg;
  gyroYawDeg = 0.0f;
  gyroHeadingRad = 0.0f;

  Serial.println("BNO085 YAW RESET TO 0 DEG");
}

void readSerialCommand()
{
  while (Serial.available())
  {
    char command = Serial.read();

    if (command == 'R' || command == 'r')
    {
      resetGyroYaw();
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
    Serial.print("start");

    Wire.begin(SDA, SCL, 100000);
    relay1.write(1);
    relay2.write(1);
    relay3.write(1);
    relay4.write(1);
    relay5.write(1);
    relay6.write(1);

    WheelSerial.begin(WHEEL_UART_BAUD, SERIAL_8N1, WHEEL_UART_RX, WHEEL_UART_TX);
    ArmSerial.begin(ARM_UART_BAUD, SERIAL_8N1, ARM_UART_RX, ARM_UART_TX);

    beginGyro();
}

void loop() {
    unsigned long now = millis();
    readGyro();
    readSerialCommand();
    
    // if ((now - prev_imu_print_time) >= (1000 / IMU_PRINT_RATE)) {
    //     Serial.print("GYRO,");

    //     if (gyroOnline)
    //         Serial.print("ONLINE");
    //     else
    //         Serial.print("OFFLINE");

    //     Serial.print(",Roll=");
    //     Serial.print(gyroRollDeg, 2);

    //     Serial.print(",Pitch=");
    //     Serial.print(gyroPitchDeg, 2);

    //     Serial.print(",Yaw=");
    //     Serial.print(gyroYawDeg, 2);

    //     Serial.print(",YawRad=");
    //     Serial.println(gyroHeadingRad, 4);

    //     prev_imu_print_time = now;
    // }

    if ((now - prev_wheel_send_time) >= (1000 / COMMAND_RATE)) {
        prev_wheel_send_time = now;

        joyInput.update();

        updateControl();
        sendWheelCommand(WheelSerial, velocity.valX, velocity.valY, velocity.valW, gyroYawDeg);

        armControl();
        sendArmCommand(ArmSerial, arm_pwm);
        sendBoxCommand(ArmSerial, box_pwm);
        sendLiftCommand(ArmSerial, lift_pwm);

        digitalControl();
    }
}