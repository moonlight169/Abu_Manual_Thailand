# Abu_Manual_Thailand

โปรเจกต์ซอร์สโค้ดสำหรับควบคุมหุ่นยนต์บังคับมือ (Manual Robot - R1) 
ในการแข่งขัน ABU Robocon พัฒนาโดยทีม KTC_DINO_ROBOT วิทยาลัยเทคนิคกาฬสินธุ์
by.Ittichai Wachiraphiphatkun

[![PlatformIO](https://img.shields.io/badge/PlatformIO-orange?style=flat&logo=platformio)](https://platformio.org/) [![C++](https://img.shields.io/badge/C++-00599C?style=flat&logo=c%2B%2B)](https://isocpp.org/)
[![ESP32](https://img.shields.io/badge/ESP-WROVER-black?style=flat&logo=espressif)](https://www.espressif.com/) [![STM32](https://img.shields.io/badge/STM32-F411CE-blue?style=flat&logo=stmicroelectronics)](https://www.st.com/)



## System Architecture
โปรเจกต์นี้ใช้การประมวลผลแบบกระจายศูนย์ (Distributed Control) แบบ Master-Slave เพื่อลดภาระการประมวลผลและทำให้หุ่นยนต์ตอบสนองต่อผู้ใช้งาน (Latency) ได้รวดเร็วที่สุด

## 🛠️Hardware & Tech Stack
-   **Master Controller:** ESP32-WROVER-KIT
-   **Slave Controller:** STM32 Blackpill (F411CE) x2 (Arm, Wheel)
-   **Wireless Control:** PS5 Controller (via Bluetooth)
-   **Control Algorithm:** PID Control, Mecanum Kinematics
-   **Development Environment:** VS Code + PlatformIO (Multi-Environment Setup)

## 📂 Project Structure
PlatformIO ใช้ single project แยก 3 Environment ด้วย `build_src_filter` ใน `platformio.ini`
```text
src/
 ├── 0_master/       # ESP32-WROVER: ศูนย์กลางสั่งการ (Bluetooth PS5, UART, Relay, Limit Switch, IMU)
 ├── 1_slave_arm/    # STM32: ควบคุม Box + Arm (Up/Down) + Lift (Front/Back)
 └── 2_slave_wheel/  # STM32: ควบคุมล้อ Mecanum 4 ล้อ (PID loop ความถี่สูง) + อ่าน Encoder

lib/
 ├── Kinematics/     # Mecanum kinematics library
 └── PID/            # PID control library

include/             # Header ไฟล์ .h รวมทุก Environment (config_esp32.h, config_stm32.h, ...)
```