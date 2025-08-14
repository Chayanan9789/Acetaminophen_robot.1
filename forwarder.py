import cv2

# ใช้เลข Index ของกล้อง Oker D08Z (ส่วนใหญ่จะเป็น 0 หรือ 1)
CAMERA_INDEX = 0 

cap = cv2.VideoCapture(CAMERA_INDEX)

if not cap.isOpened():
    print(f"ไม่สามารถเปิดกล้องที่ Index {CAMERA_INDEX} ได้")
else:
    print("เปิดกล้องสำเร็จ! กำลังทดสอบการปรับค่า Brightness...")
    
    # 1. อ่านค่าความสว่างเริ่มต้น (ถ้ากล้องไม่รองรับอาจได้ค่า -1 หรือ 0)
    try:
        initial_brightness = cap.get(cv2.CAP_PROP_BRIGHTNESS)
        print(f"ค่า Brightness เริ่มต้น: {initial_brightness}")

        # 2. ลองตั้งค่าใหม่ (ลองใช้ค่าที่ต่างจากเดิม)
        new_brightness_value = 100.0 
        print(f"กำลังลองตั้งค่า Brightness เป็น: {new_brightness_value}")
        cap.set(cv2.CAP_PROP_BRIGHTNESS, new_brightness_value)

        # 3. อ่านค่าที่ตั้งใหม่อีกครั้งเพื่อตรวจสอบ
        # ให้หน่วงเวลาเล็กน้อยเพื่อให้กล้องมีเวลาปรับค่า
        cv2.waitKey(500) 
        final_brightness = cap.get(cv2.CAP_PROP_BRIGHTNESS)
        print(f"ค่า Brightness หลังจากตั้งค่า: {final_brightness}")

        # 4. เปรียบเทียบและสรุปผล
        if final_brightness == new_brightness_value:
            print("\n✅ ยอดเยี่ยม! กล้องของคุณรองรับการปรับ Brightness ผ่านโค้ด (วิธีที่ 1)")
        else:
            print("\n❌ น่าเสียดาย... กล้องของคุณไม่รองรับการปรับ Brightness ผ่านโค้ด หรือค่าไม่เปลี่ยนแปลง")
            print("แนะนำให้ใช้วิธีที่ 2 (การปรับแก้รูปภาพด้วยซอฟต์แวร์) แทนครับ")
            
    except Exception as e:
        print(f"เกิดข้อผิดพลาดระหว่างทดสอบ: {e}")

    # ปิดกล้องหลังทดสอบเสร็จ
    cap.release()
    print("ปิดกล้องเรียบร้อย")