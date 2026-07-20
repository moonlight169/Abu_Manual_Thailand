# Project Overview
โปรเจกต์ซอร์สโค้ดสำหรับควบคุมหุ่นยนต์แมนนวล (Manual Robot - R1) ในการแข่งขัน ABU Robocon 
พัฒนาโดยทีม KTC_DINO_ROBOT วิทยาลัยเทคนิคกาฬสินธุ์ by.Ittichai Wachiraphiphatkun

# Hardware & Tech Stack
- Master: ESP32-WROOM-32 (38pin)
- Slave Controller: STM32 Blackpill (F411CE) x2 ตัว (Arm, Wheel)
- Development Environment: VS Code + PlatformIO (Multi-Environment Setup)
- Control Algorithm: PID Control, Mecanum Kinematics

# Project Structure
ระบบเป็น Distributed System เชื่อมต่อแบบ Star Topology ผ่านสาย UART (JST-XH 4 pin) แยกอิสระต่อบอร์ด (Point-to-Point)
โดย Slave ทั้ง 2 ตัวจะต่อตรงเข้าหา Master โดยตรง (Slave 1-2 -> Master) ไม่มี TOF sensor ในระบบนี้
PlatformIO ใช้ single project แยก 3 Environment ด้วย `build_src_filter` ใน platformio.ini (ไม่ได้แยกเป็นโฟลเดอร์ระดับบนสุดแบบ 3 โปรเจกต์อิสระ):
├── src/
│   ├── 0_master/       # ESP32-WROOM(38pin): ศูนย์กลางสั่งการ
│   │                   #   - UART 2 ช่อง คุยกับ Slave 2 ตัว (Arm, Wheel) โดยตรง
│   │                   #   - คุม Relay 6 ตัว (Relay 1-6) ต่อ GPIO ตรงจาก Master เลย (32,33,25,26,27,14)
│   │                   #   - อ่าน Limit Switch เพิ่มเติมต่อ GPIO ตรงจาก Master (36,39,34,35,13) ยังไม่ระบุหน้าที่แต่ละตัวใน diagram
│   │                   #   - I2C 1 ตัวต่อ IMU (BNO08x 9-DOF) ผ่านขา 21(SDA) 22(SCL)
│   ├── 1_slave_arm/    # STM32: รวมชุด Box + Arm (Up/Down) + Lift (Front/Back) ไว้บอร์ดเดียว
│   │                   #   - มอเตอร์ผ่าน bts7960: Box, Arm, Lift (+ อีก 1 ช่องสำรองยังไม่ระบุ)
│   │                   #   - Limit Switch: BoxFront/BoxBack, ArmFront/ArmBack, LiftUp/LiftDown (+ อีก 1 ชุดสำรองยังไม่ระบุ)
│   └── 2_slave_wheel/  # STM32: คุมล้อ Mecanum 4 ล้อ (ลูป PID ความถี่สูง) + อ่าน Encoder 4 ตัว
│                       #   - มอเตอร์ผ่าน bts7960: Front Left, Front Right, Rear Left, Rear Right
│                       #   - Encoder: Front Left, Front Right, Rear Left, Rear Right
├── include/            # Header เฉพาะ config รายบอร์ด + PS5Input.h (ไม่ได้รวม logic แบบใช้ร่วมกันหลาย Environment)
│   ├── config_esp32.h  #   pin/config เฉพาะ 0_master
│   ├── config_arm.h    #   pin/config เฉพาะ 1_slave_arm
│   ├── config_wheels.h #   pin/config เฉพาะ 2_slave_wheel
│   └── PS5Input.h
└── lib/                # โมดูลใช้ร่วมกันข้าม Environment ได้ (PlatformIO auto-detect เป็น library, include ด้วย <ชื่อ.h>)
    ├── Kinematics/      #   Mecanum kinematics (ใช้ใน 2_slave_wheel)
    ├── PID/             #   PID controller (ใช้ใน 2_slave_wheel)
    ├── Protocol/        #   UART frame protocol (WheelCommand/ServoCommand) ใช้ทั้ง Master และ Slave
    └── Wheel/           #   encoder.h/.cpp, motor.h/.cpp, wheel.h/.cpp (ย้ายมาจาก include/+src/2_slave_wheel/ เดิม)

# UART Pin Mapping
 1. 1_slave_arm    A9(TX1) A10(RX1) -> 0_master 19(Rx) 18(Tx)
 2. 2_slave_wheel  A9(TX1) A10(RX1) -> 0_master 16(Rx1) 17(Tx1)

หมายเหตุ: pin 19/18 และ 16/17 อ้างอิงจาก diagram ล่าสุด (Diagram_manual_robot.pdf) ซึ่งต่างจากที่เคยระบุไว้ก่อนหน้านี้ (0/1, 7/8, 28/29) กรุณาตรวจสอบกับบอร์ดจริงอีกครั้งก่อนแก้โค้ด

# Coding Rules (กฎเหล็ก)
1. **File Extension:** โฟลเดอร์ `include/` ใช้ `.h` เท่านั้น, โฟลเดอร์ `src/` ใช้ `.cpp` เท่านั้น ส่วน `lib/<Module>/` เก็บคู่ `.h`+`.cpp` ของโมดูลนั้นไว้ด้วยกันได้ (ตามแพทเทิร์น PlatformIO library)
2. **Readability:** เขียนโค้ดให้เข้าใจและไล่ง่ายมากกว่าเน้นโค้ดสั้น ห้ามยัดคำสั่งในบรรทัดเดียวจนอ่านยาก
3. **Legacy Code:** ห้ามลบหรือเปลี่ยนโค้ดเดิมเด็ดขาด หากยังไม่เข้าใจหน้าที่และผลกระทบของโค้ดนั้น
4. **No delay():** ห้ามใช้คำสั่ง `delay()` ในบอร์ดใดๆ ทั้งสิ้น ให้ใช้ `millis()` หรือโปรแกรมโครงสร้าง Timer/Non-blocking เท่านั้น เพื่อไม่ให้ลูปควบคุม (PID/Mecanum) สะดุด

# AI Workflow (ลำดับขั้นตอนทำงานของ Claude Code)
ก่อนทำการแก้ไขหรือแนะนำโค้ด ให้เดินตามลูป 6 ขั้นตอนนี้เสมอ:
1. **Analyze:** อ่านและวิเคราะห์ไฟล์ที่เกี่ยวข้องใน Environment นั้นๆ 
2. **Plan:** อธิบายแผนการปรับปรุงสั้นๆ เป็นข้อๆ ให้ผู้ใช้ฟังสรุปก่อน
3. **Impact Check:** ประเมินผลกระทบว่ากระทบต่อลูป PID, ไทม์มิ่ง หรือการสื่อสารระหว่างบอร์ดหรือไม่
4. เขียนให้เลย
5. **Summary:** สรุปสิ่งที่ได้แก้ไข และข้อควรระวังหลังจากทำเสร็จ
6. **Git Commit:** แนะนำข้อความสำหรับการ `git commit` ตามมาตรฐาน Conventional Commits (เช่น feat(wheel): ..., fix(sensor): ...)