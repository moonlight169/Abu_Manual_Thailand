#ifndef CONFIG_ESP32_H
#define CONFIG_ESP32_H

#define MAC_PS5_WHITE "D4:2F:4B:00:DB:AB"
#define MAC_PS5_RED "0c:27:56:1a:0c:6e"

// จอยที่ใช้อยู่ตอนนี้ สลับเป็น MAC_PS5_RED ถ้าเปลี่ยนไปใช้ตัวแดง
#define PS5_MAC_IN_USE MAC_PS5_WHITE

#define COMMAND_RATE 50

#define SDA 21
#define SCL 22

#define BNO08X_RESET -1

#define WHEEL_UART_RX 16
#define WHEEL_UART_TX 17
#define WHEEL_UART_BAUD 115200

#define ARM_UART_RX 19
#define ARM_UART_TX 18
#define ARM_UART_BAUD 115200

#define Relay1 13
#define Relay2 32
#define Relay3 33
#define Relay4 26
#define Relay5 27
#define Relay6 14

#define MASTER_DEBUG_GYRO 0
#define MASTER_DEBUG_ARM 0

#endif