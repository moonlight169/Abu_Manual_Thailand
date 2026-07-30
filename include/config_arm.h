#ifndef CONFIG_ARM_H
#define CONFIG_ARM_H

#define MotorBox_A PB0
#define MotorBox_B PB1

#define MotorArm_A PA6
#define MotorArm_B PA7

//0-150000
#define EncArm_A PB7
#define EncArm_B PB6

#define MotorLift_A PA2
#define MotorLift_B PA3

#define LimitSW_Box_Front PB13
#define LimitSW_Box_Back PB12

#define LimitSW_Arm_Front PB15
#define LimitSW_Arm_Back PB14

#define LimitSW_Lift_Up PB4
#define LimitSW_Lift_Down PB5

// ---- arm_pulseTo(): วิ่งแขนไปยังตำแหน่ง encoder ที่กำหนดแบบ non-blocking ----
// จุด 0 ของแขน = ตอนแขนชนลิมิตท้าย (LimitSW_Arm_Back) โค้ดจะ reset encoder ให้ตรงนั้น
// คนขับเป็นคนพาแขนไปชนเพื่อ set 0 เอง แล้วค่อยกด touchpad+สามเหลี่ยมสั่งวิ่งไปตำแหน่ง
// ตำแหน่งเป้าหมาย (เช่น 100000) จึงนับ "ออกจากลิมิตท้าย" เป็นบวกเสมอ

// ฝั่งที่คนขับดันจอยไปสุดแล้วถือเป็นจุด 0 ใช้ลิมิตสวิตช์ตัวไหน
//   1 = LimitSW_Arm_Back (ค่าเริ่มต้น)
//   0 = LimitSW_Arm_Front
// เช็คได้จาก swB= / swF= ในบรรทัด ENC ว่าตอนดันไปสุดตัวไหนขึ้นเป็น 1
#define ARM_ZERO_AT_BACK_LIMIT 0

// ---- ทิศทาง: เป็นแค่ "ค่าเดาเริ่มต้น" ไม่ต้องคาลิเบรตเอง ----
// พอแขนออกจากลิมิตท้ายจริงเมื่อไหร่ โค้ดจะจำทิศที่ถูกต้องเองแล้วใช้ค่านั้นแทน
// (ดู lrn=1 ในบรรทัด ENC ตอนเปิด ARM_ENC_DEBUG)

// encoder นับทางไหน ตอนแขนวิ่งออกจากลิมิตท้าย: 1 = count เพิ่ม, -1 = count ลด
#define ARM_ENC_DIR 1

// PWM ทางไหนทำให้แขนวิ่งออกจากลิมิตท้าย: 1 = PWM บวก, -1 = PWM ลบ
// ตั้ง -1 ไว้ตามอาการที่เจอตอนทดสอบ (PWM บวกทำให้แขนวิ่งลงไปหาลิมิตท้าย)
// ถ้าเดาผิด โค้ดจะกลับทิศให้เองภายในไม่กี่ร้อย ms ตอนสั่งวิ่งครั้งแรก
#define ARM_PULSE_DIR -1

// ต้องออกห่างจากจุด 0 เกินเท่านี้ (count) และขยับเกินเท่านี้ต่อรอบ จึงเชื่อว่าเป็นการขยับจริง
// แล้วเอาไปสรุปทิศทาง (กันสัญญาณรบกวน/backlash หลอก)
#define ARM_DIR_LEARN_COUNTS 300
#define ARM_DIR_LEARN_MIN_DELTA 20

// ตอนเริ่มวิ่งทั้งที่ยังคาลิมิตท้าย ให้เวลาถอยออกจากสวิตช์เท่านี้
// ถ้าเกินแล้วสวิตช์ยังไม่หลุด = ดันผิดทาง โค้ดจะลองกลับทิศให้เอง 1 ครั้ง
#define ARM_PULSE_ESCAPE_MS 700

// PWM ปกติที่ใช้ไล่เข้าหาเป้าหมาย (0-255)
#define ARM_PULSE_PWM 180

// จำนวน count ที่ถือว่า "ถึงเป้าหมายแล้ว" ให้หยุด (กันแกว่งไปกลับรอบเป้า)
#define ARM_PULSE_TOLERANCE 200

// เข้าใกล้เป้าหมายเหลือไม่ถึงเท่านี้ (count) จะเริ่มลด PWM ลงตามระยะที่เหลือ
#define ARM_PULSE_BRAKE_ZONE 3000

// PWM ต่ำสุดที่มอเตอร์ยังเอาชนะแรงเสียดทานได้ ตอนอยู่ใน brake zone จะไม่ลดต่ำกว่านี้
#define ARM_PULSE_MIN_PWM 80

// ถ้าวิ่งเกินเวลานี้แล้วยังไม่ถึงเป้า ให้ยกเลิกเอง (กันแขนติด/encoder หลุดแล้วดันค้าง)
#define ARM_PULSE_TIMEOUT_MS 5000

// ทุกๆ ช่วงเวลานี้ ระยะที่เหลือต้องลดลงอย่างน้อย ARM_PULSE_MIN_PROGRESS count
// ถ้าไม่ลด = วิ่งผิดทาง/แขนติด/encoder ไม่มา ให้ตัดกำลังทิ้งทันที
#define ARM_PULSE_STALL_MS 400
#define ARM_PULSE_MIN_PROGRESS 50

// สติ๊กแขนต้องเกินค่านี้จึงถือว่าคนขับแทรกเข้ามาแล้วยกเลิกคำสั่งวิ่งไปตำแหน่ง
// (ตอนสติ๊กอยู่กลาง master คำนวณ map() ออกมาได้ -1 ไม่ใช่ 0 พอดี ถ้าเทียบ != 0 pulse จะถูกยกเลิกทิ้งทันที)
#define ARM_MANUAL_OVERRIDE_PWM 10

#define ARM_DEBUG 0

#define ARM_DEBUG_RATE 5 // Hz

// เปิดไว้ก่อนสำหรับคาลิเบรต ARM_ENC_DIR / ARM_PULSE_DIR (ปิดกลับเป็น 0 ได้เมื่อเซ็ตค่าตรงแล้ว)
#define ARM_ENC_DEBUG 1

#define ARM_ENC_DEBUG_RATE 5 // Hz

#endif