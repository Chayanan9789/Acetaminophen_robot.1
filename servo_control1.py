# servo_control.py
# ควบคุม Servo Motor ด้วย pyfirmata โดยส่งองศา 0–180 ผ่านขา PWM

from pyfirmata2 import Arduino        
import time                           

# ─────────────────── 1) เชื่อมต่อบอร์ด ───────────────────
board = Arduino('COM19')
print("เริ่มการทำงาน")
# ─────────────────── 2) ตั้งขาเป็นโหมด Servo ─────────────
# get_pin('d:9:s')
#  • 'd'  = digital pin
#  • '9'  = หมายเลขขา D9 (รองรับ PWM / Servo)
#  • 's'  = servo mode  (รับค่ามุม 0–180°)
servo_pin = board.get_pin('d:12:s')

print("📍 เริ่มควบคุมเซอร์โวมอเตอร์ที่ D9 (Ctrl-C เพื่อหยุด)")

try:
    while True:
        # ── หมุนจาก 0 → 180 องศา ทีละ 30° ──
        for angle in range(0, 181, 5):        # range(start, stop, step)
            print("➡ หมุนไปที่", angle, "°")
            servo_pin.write(angle)             # write(angle) → ส่งค่ามุม (0-180°)
            time.sleep(9)                      # หน่วง 1 s ให้หมุนถึงตำแหน่ง

        # ── หมุนกลับจาก 180 → 0 องศา ──
        for angle in range(180, -1, -5):
            print("⬅ หมุนกลับที่", angle, "°")
            servo_pin.write(angle)             # ส่งค่ามุม
            time.sleep(0.09)                      # หน่วงให้เซอร์โวหมุนเสร็จ

except KeyboardInterrupt:
    print("\n ผู้ใช้สั่งหยุดการทำงาน")

finally:
    board.exit()  # ปิดพอร์ตอย่างปลอดภัยเมื่อจบโปรแกรม