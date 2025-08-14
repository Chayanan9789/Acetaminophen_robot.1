import pyfirmata2
import time

# ระบุพอร์ตของ Arduino
# ให้เปลี่ยน "COMX" เป็นชื่อพอร์ตจริงที่พบใน Device Manager หรือ Terminal
PORT = "COM19"

# สร้างอ็อบเจกต์ board เพื่อเชื่อมต่อกับ Arduino
board = pyfirmata2.Arduino(PORT)

# กำหนดขา PWM ของเซอร์โว (ขา 9) ให้เป็นโหมดเซอร์โว (s)
servo_pin = board.get_pin('d:3:s')

print("Servo is now ready.")
print("Press Ctrl+C to exit.")

try:
    while True:
        # หมุนไปข้างหน้าด้วยความเร็วสูงสุด (ค่า 180)
        print("Moving forward...")
        servo_pin.write(100)
        time.sleep(2)  # รอ 2 วินาที

        # หยุดหมุน (ค่า 90)
        print("Stopping...")
        servo_pin.write(92)
        time.sleep(3)  # รอ 2 วินาที

        # หมุนถอยหลังด้วยความเร็วสูงสุด (ค่า 0)
        print("Moving backward...")
        servo_pin.write(88)
        time.sleep(2)  # รอ 2 วินาที

        # หยุดหมุน (ค่า 90)
        print("Stopping...")
        servo_pin.write(92)
        time.sleep(3)  # รอ 2 วินาที

except KeyboardInterrupt:
    print("\nExiting program.")
finally:
    # ปิดการเชื่อมต่อกับบอร์ดอย่างถูกต้อง
    board.exit()