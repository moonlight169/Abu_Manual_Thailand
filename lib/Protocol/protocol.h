#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>
#include <Arduino.h>

#define PROTOCOL_START_BYTE 0xAA
#define WHEEL_CMD_LEN sizeof(WheelCommand)
#define MOTOR_CMD_LEN sizeof(MotorCommand)

#define CMD_ARM 0x01
#define CMD_BOX 0x02

struct WheelCommand{
    float vx;
    float vy;
    float omega;
};

struct WheelFrame{
    uint8_t start;
    uint8_t len;
    WheelCommand payload;
    uint8_t checksum;
};

enum ParserState{
    WAIT_START,
    READ_CMD_ID,
    READ_LEN,
    READ_PAYLOAD,
    READ_CHECKSUM
};

struct WheelReceiver{
    ParserState state = WAIT_START;
    uint8_t buffer[sizeof(WheelCommand)];
    uint8_t bufferIndex = 0;
    uint8_t expectedLen = 0;
    WheelCommand lastCommand;
    bool hasNewCommand = false;
    unsigned long lastReceivedTime = 0;
};

struct MotorCommand{
    int16_t pwm;
};

struct MotorFrame{
    uint8_t start;
    uint8_t cmdId;
    uint8_t len;
    MotorCommand payload;
    uint8_t checksum;
};

struct MotorReceiver{
    ParserState state = WAIT_START;
    uint8_t cmdId = 0;
    uint8_t buffer[sizeof(MotorCommand)];
    uint8_t bufferIndex = 0;
    uint8_t expectedLen = 0;
    MotorCommand lastArmCommand;
    MotorCommand lastBoxCommand;
    bool hasNewArmCommand = false;
    bool hasNewBoxCommand = false;
    unsigned long lastArmReceivedTime = 0;
    unsigned long lastBoxReceivedTime = 0;
};

uint8_t calculateChecksum(const uint8_t* data, uint8_t len);
void wheelReceiverFeed(WheelReceiver &receiver, uint8_t incomingByte);
void sendWheelCommand(HardwareSerial &port, float vx, float vy, float omega);

void motorReceiverFeed(MotorReceiver &receiver, uint8_t incomingByte);
void sendArmCommand(HardwareSerial &port, int16_t arm_pwm);
void sendBoxCommand(HardwareSerial &port, int16_t box_pwm);

#endif