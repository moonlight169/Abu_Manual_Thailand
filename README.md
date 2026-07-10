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
-   **Slave Controller:** STM32 Blackpill (F411CE)
-   **Wireless Control:** PS5 Controller (via Bluetooth)
-   **Development Environment:** VS Code + PlatformIO (Multi-Environment Setup)

## 📂 Project Structure
```text
src/
 ├── esp32/ # โค้ดสำหรับ ESP32 (จัดการ Bluetooth PS5 & ส่งข้อมูล)
 └── stm32/ # โค้ดสำหรับ STM32 (รับข้อมูล & ขับมอเตอร์ล้อ Mecanum)
```