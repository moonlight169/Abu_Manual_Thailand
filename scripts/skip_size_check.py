Import("env")

# ข้ามขั้นตอน "Checking size" (เรียก arm-none-eabi-size.exe) เพราะเครื่องนี้มี
# Windows Application Control (WDAC) block ตัว exe นั้นอยู่ (ไม่เกี่ยวกับ compiler/upload)
# ผลคือ build/upload ปกติทุกอย่าง แค่ไม่มีรายงานขนาด flash/RAM ท้าย log เท่านั้น
env.Replace(SIZETOOL="", SIZECHECKCMD="")
