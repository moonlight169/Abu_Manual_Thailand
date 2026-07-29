#ifndef CONFIG_WHEELS_H
#define CONFIG_WHEELS_H

// --- Motor Pins ---
#define MotorFL_A PB0
#define MotorFL_B PB1

#define MotorFR_A PA7
#define MotorFR_B PA6

#define MotorRL_A PA2
#define MotorRL_B PA3

#define MotorRR_A PA1
#define MotorRR_B PA0

// --- Encoder Pins ---
// ข้อดี: ขา PB ที่คุณเลือกใช้ในบอร์ดตระกูล STM32 ไม่มีปัญหา EXTI Conflict กันเลยครับ ถือว่าออกแบบฮาร์ดแวร์มาดีมาก
#define EncFL_A PB12
#define EncFL_B PB13

#define EncFR_A PB5
#define EncFR_B PB4

#define EncRL_A PB14
#define EncRL_B PB15

#define EncRR_A PB7
#define EncRR_B PB6

#define MAX_RPM 550

#define PULSE_PER_REV 844.4

#define LR_WHEELS_DISTANCE 0.420
#define FR_WHEELS_DISTANCE 0.430

// ⚠️ ค่านี้ถูกส่งเข้าพารามิเตอร์ชื่อ wheel_diameter ของ Kinematics (ไม่ใช่ radius)
// Kinematics จึงคิดเส้นรอบวงเป็น PI*0.0508 = 0.1596 m ซึ่งเป็น "ครึ่งเดียว" ของล้อ 4 นิ้วจริง (0.319 m)
// ผลคือ RPM ที่ขอสูงเกินจริง 2 เท่า แต่ค่าความเร็วในจอย (wheel_Walk_Normal ฯลฯ) ถูกจูนชดเชยไว้แล้ว
// >>> ห้ามแก้ตัวเลขนี้เดี่ยวๆ ถ้าจะแก้ให้ถูกหน่วย SI ต้องแก้ค่าความเร็วใน 0_master พร้อมกันแล้วจูนใหม่ทั้งชุด
#define WHEEL_RADIUS 0.0508

#define PWM_MIN -255
#define PWM_MAX 255

#define stepDelay 10

#define FL_K_P 1.0
#define FL_K_I 0.10 
#define FL_K_D 0.05

#define FR_K_P 1.0
#define FR_K_I 0.10
#define FR_K_D 0.05

#define RL_K_P 1.0
#define RL_K_I 0.10
#define RL_K_D 0.05

#define RR_K_P 1.0
#define RR_K_I 0.10
#define RR_K_D 0.05

#define WHEEL_I_LIMIT 1000.0f

#define WHEEL_DEBUG 1

#define HEADING_ASSIST_ENABLED 1

#define HEADING_ASSIST_DIR 1

#define HEAD_K_P 0.50
#define HEAD_K_I 0.00
#define HEAD_K_D 0.00

#define HEADING_I_LIMIT 300.0f    
#define HEADING_MAX_RPM 80.0f    
#define HEADING_DEADBAND_DEG 1.0f   
#define HEADING_MIN_CMD 0.05f        
#define HEADING_OMEGA_DEADZONE 0.05f 
#define HEADING_JUMP_LIMIT_DEG 45.0f 

#endif
