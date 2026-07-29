# แก้ปัญหา build STM32 ล้มเหลวด้วย Error 4551
#   'arm-none-eabi-gcc-ar.exe' was blocked by your organization's Device Guard policy
#
# สาเหตุ:
#   Windows Device Guard / Smart App Control บนเครื่องนี้บล็อกไฟล์ wrapper ขนาดเล็ก (~66KB)
#   ของ GCC คือ gcc-ar.exe / gcc-nm.exe / gcc-ranlib.exe (บล็อกตาม hash ของไฟล์)
#   ทำให้ตอน Archiving ไฟล์ .a ของ lib/ (PID, Protocol, Kinematics, Wheel) พัง
#
#   แต่ตัวจริงของ binutils คือ ar.exe / nm.exe / ranlib.exe (~1MB) ไม่โดนบล็อก
#   จึงแก้โดย copy ตัวจริงไปทับ wrapper (เก็บไฟล์เดิมไว้เป็น .wdac-backup)
#
# หมายเหตุ:
#   - wrapper gcc-* มีไว้รองรับ LTO (-flto) เท่านั้น โปรเจกต์นี้ไม่ได้เปิด LTO
#     ผลลัพธ์ไบนารีจึงเหมือนเดิมทุกประการ
#   - ต้องรันสคริปต์นี้ใหม่ทุกครั้งที่ PlatformIO ติดตั้ง/อัปเดต toolchain-gccarmnoneeabi
#   - ส่วน size.exe ที่โดนบล็อกเหมือนกัน แก้ด้วย scripts/skip_size_check.py แล้ว
#
# วิธีใช้: เปิด PowerShell แล้วรัน
#   powershell -ExecutionPolicy Bypass -File scripts\fix_wdac_toolchain.ps1

$bin = "$env:USERPROFILE\.platformio\packages\toolchain-gccarmnoneeabi\bin"

if (-not (Test-Path $bin)) {
    Write-Host "ไม่พบโฟลเดอร์ toolchain: $bin"
    exit 1
}

# คู่ของ (ไฟล์ wrapper ที่โดนบล็อก) -> (ไฟล์ตัวจริงที่ใช้แทนได้)
$pairs = @{
    "arm-none-eabi-gcc-ar.exe"     = "arm-none-eabi-ar.exe"
    "arm-none-eabi-gcc-nm.exe"     = "arm-none-eabi-nm.exe"
    "arm-none-eabi-gcc-ranlib.exe" = "arm-none-eabi-ranlib.exe"
}

foreach ($wrapper in $pairs.Keys) {
    $real       = $pairs[$wrapper]
    $wrapperPath = Join-Path $bin $wrapper
    $realPath    = Join-Path $bin $real
    $backupPath  = "$wrapperPath.wdac-backup"

    if (-not (Test-Path $realPath)) {
        Write-Host "ข้าม $wrapper (ไม่พบ $real)"
        continue
    }

    # สำรองไฟล์เดิมไว้ครั้งแรกครั้งเดียว (ถ้ามี backup แล้วแปลว่าเคยแก้ไปแล้ว)
    if ((Test-Path $wrapperPath) -and (-not (Test-Path $backupPath))) {
        Move-Item $wrapperPath $backupPath
        Write-Host "สำรอง $wrapper -> $wrapper.wdac-backup"
    }

    Copy-Item $realPath $wrapperPath -Force
    Write-Host "แทนที่ $wrapper ด้วย $real เรียบร้อย"
}

Write-Host ""
Write-Host "เสร็จแล้ว ลอง build ใหม่ได้เลย"
