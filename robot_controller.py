import cv2
import time
import zmq
from ultralytics import YOLO
import pyfirmata2

# --- การตั้งค่าที่ต้องปรับแก้ ---
# ปรับแก้ port ให้ตรงกับ Arduino ของคุณ
ARDUINO_PORT = "COM19"

# ปรับแก้ Pin ของ Servo ทั้ง 6 ตัวให้ถูกต้อง
PIN_SERVO_CAMERA = 9 # Servo หมุนกล้อง
PIN_SERVO_BASE = 10  # Servo หมุนฐานแขนกล
PIN_SERVO_LIFT = 11  # Servo ยกแขนขึ้น-ลง
PIN_SERVO_REACH = 12 # Servo ยืด/หด แขน (ปรับระดับ)
PIN_SERVO_GRIPPER = 13# Servo ตัวคีบ

# --- ค่ามุมการสแกนของกล้อง ---
CAMERA_SCAN_START_ANGLE = 0
CAMERA_SCAN_END_ANGLE = 180
CAMERA_SCAN_STEP = 5

# --- ค่ามุมของแขนกล (ปรับจูนให้เหมาะกับแขนกลของคุณ) ---
HOME_LIFT_ANGLE = 20
HOME_REACH_ANGLE = 75
HOME_GRIPPER_ANGLE = 15
PICKUP_LIFT_ANGLE = 83
PICKUP_REACH_ANGLE = 0
GRIPPER_OPEN_ANGLE = 110 # ปรับปรุงค่าตามที่ผู้ใช้ระบุ
GRIPPER_CLOSE_ANGLE = 0
DROP_LIFT_ANGLE = 40 # ปรับปรุงค่าตามที่ผู้ใช้ระบุ
DROP_REACH_ANGLE = 35 # ปรับปรุงค่าตามที่ผู้ใช้ระบุ

# --- การตั้งค่าอื่นๆ ---
WEBCAM_INDEX = 0
YOLO_MODEL_PATH = 'best1.pt' # Path ไปยังโมเดล .pt ที่เทรนมา

# ตั้งค่าความมั่นใจขั้นต่ำ (เช่น 0.80 คือ 80%)
# หุ่นยนต์จะทำงานต่อเมื่อค่าความมั่นใจสูงกว่าค่านี้
CONFIDENCE_THRESHOLD = 0.80

# --- ค่าสำหรับปรับแก้การติดตั้ง ---
# ปรับค่านี้เพื่อชดเชยการติดตั้งที่ทำให้มุมกล้องและแขนกลไม่ตรงกัน
# หากแขนกลหมุนไปทางซ้ายของวัตถุ ให้เพิ่มค่านี้
# หากแขนกลหมุนไปทางขวาของวัตถุ ให้ลดค่านี้
BASE_TO_CAMERA_OFFSET_DEGREES_PICKUP = -5
BASE_TO_CAMERA_OFFSET_DEGREES1_DROP = -2

class RobotController:
    def __init__(self):
        self.running = True
        self.base_drop_angle = None

        # ===== [ปรับปรุง] สร้างตัวแปรเก็บมุมปัจจุบันของ Servo ทุกตัว =====
        self.current_base_angle = 0    # ปรับปรุงค่าตามที่ผู้ใช้ระบุ
        self.current_camera_angle = 0  # ปรับปรุงค่าตามที่ผู้ใช้ระบุ
        self.current_lift_angle = HOME_LIFT_ANGLE
        self.current_reach_angle = HOME_REACH_ANGLE
        self.current_gripper_angle = HOME_GRIPPER_ANGLE

        print("CONTROLLER: กำลังโหลด YOLOv8 model...")
        self.model = YOLO(YOLO_MODEL_PATH)

        print("CONTROLLER: กำลังเชื่อมต่อกับ Arduino...")
        try:
            self.board = pyfirmata2.Arduino(ARDUINO_PORT)
            it = pyfirmata2.util.Iterator(self.board)
            it.start()
            print("CONTROLLER: เชื่อมต่อ Arduino สำเร็จ")
        except Exception as e:
            print(f"CONTROLLER: ไม่สามารถเชื่อมต่อ Arduino ที่ {ARDUINO_PORT} ได้: {e}")
            self.running = False
            return

        self.servo_camera = self.board.get_pin(f'd:{PIN_SERVO_CAMERA}:s')
        self.servo_base = self.board.get_pin(f'd:{PIN_SERVO_BASE}:s')
        self.servo_lift = self.board.get_pin(f'd:{PIN_SERVO_LIFT}:s')
        self.servo_reach = self.board.get_pin(f'd:{PIN_SERVO_REACH}:s')
        self.servo_gripper = self.board.get_pin(f'd:{PIN_SERVO_GRIPPER}:s')
        
        # ===== [ปรับปรุง] ตั้งค่ามุมเริ่มต้นของ Servo ทุกตัวเมื่อเริ่มโปรแกรม =====
        print("CONTROLLER: กำลังตั้งค่ามุมเริ่มต้นของ Servo...")
        self.servo_base.write(self.current_base_angle)
        self.servo_camera.write(self.current_camera_angle)
        self.servo_lift.write(self.current_lift_angle)
        self.servo_reach.write(self.current_reach_angle)
        self.servo_gripper.write(self.current_gripper_angle)
        time.sleep(1)

        context = zmq.Context()
        self.gui_socket = context.socket(zmq.PUB)
        self.gui_socket.bind("tcp://*:5555")

        print("CONTROLLER: กำลังเปิด Webcam...")
        self.cap = cv2.VideoCapture(WEBCAM_INDEX)
        if not self.cap.isOpened():
            print("CONTROLLER: ไม่สามารถเปิด Webcam ได้")
            self.running = False
            return

        print("CONTROLLER: ระบบพร้อมทำงาน")

    def send_status_to_gui(self, status_text):
        if not self.running: return
        self.gui_socket.send_multipart([b"STATUS", status_text.encode('utf-8')])
        print(f"STATUS: {status_text}")

    def send_frame_to_gui(self, frame):
        if not self.running: return
        _, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
        self.gui_socket.send_multipart([b"VIDEO", buffer.tobytes()])

    # ===== [ใหม่] ระบบควบคุมการเคลื่อนที่แบบนุ่มนวลสำหรับ Servo แต่ละตัว =====
    def move_servo_smoothly(self, servo_obj, target_angle, current_angle, step=2, speed_delay=0.008):
        """ฟังก์ชันกลางสำหรับเคลื่อน Servo อย่างนุ่มนวล"""
        target_angle = int(max(0, min(180, target_angle)))
        if target_angle == current_angle: return target_angle
        
        direction = 1 if target_angle > current_angle else -1
        
        for angle in range(current_angle + (step * direction), target_angle + direction, step * direction):
            if not self.running: return current_angle
            servo_obj.write(angle)
            time.sleep(speed_delay)
        
        servo_obj.write(target_angle)
        time.sleep(0.1)
        return target_angle

    # ===== [ปรับปรุง] แยกฟังก์ชัน Smooth ของ Base Servo =====
    def move_base_smoothly_main(self, angle):
        """การเคลื่อนที่สำหรับช่วงหยิบ-วางปกติ"""
        self.send_status_to_gui(f"กำลังหมุนฐาน (Main) ไปที่ {angle}°")
        self.current_base_angle = self.move_servo_smoothly(self.servo_base, angle, self.current_base_angle, step=5, speed_delay=0.01)

    def move_base_smoothly_final(self, angle):
        """การเคลื่อนที่สำหรับช่วงสุดท้าย (อาจจะต้องการให้ช้าลงหรือเร็วขึ้น)"""
        self.send_status_to_gui(f"กำลังหมุนฐาน (Final) ไปที่ {angle}°")
        # คุณสามารถปรับ step และ speed_delay ที่นี่ให้แตกต่างจาก main ได้
        self.current_base_angle = self.move_servo_smoothly(self.servo_base, angle, self.current_base_angle, step=5, speed_delay=0.05)

    def move_lift_smoothly(self, angle):
        self.current_lift_angle = self.move_servo_smoothly(self.servo_lift, angle, self.current_lift_angle)

    def move_reach_smoothly(self, angle):
        self.current_reach_angle = self.move_servo_smoothly(self.servo_reach, angle, self.current_reach_angle)

    def move_gripper_smoothly(self, angle):
        self.current_gripper_angle = self.move_servo_smoothly(self.servo_gripper, angle, self.current_gripper_angle, speed_delay=0.01)
        
    def move_camera_smoothly(self, angle):
        self.current_camera_angle = self.move_servo_smoothly(self.servo_camera, angle, self.current_camera_angle, speed_delay=0.01)

    def go_to_home_position(self):
        self.send_status_to_gui("กำลังกลับสู่ตำแหน่งเริ่มต้น (Home)")
        self.move_gripper_smoothly(HOME_GRIPPER_ANGLE)
        self.move_lift_smoothly(HOME_LIFT_ANGLE)
        self.move_reach_smoothly(HOME_REACH_ANGLE)
        time.sleep(0.5)

    def pickup_sequence(self):
        self.send_status_to_gui("เริ่มลำดับการจับวัตถุ")
        self.move_gripper_smoothly(GRIPPER_OPEN_ANGLE)
        self.move_reach_smoothly(PICKUP_REACH_ANGLE)
        self.move_lift_smoothly(PICKUP_LIFT_ANGLE)
        self.move_gripper_smoothly(GRIPPER_CLOSE_ANGLE)
        self.move_lift_smoothly(HOME_LIFT_ANGLE)
        self.move_reach_smoothly(HOME_REACH_ANGLE)
        time.sleep(0.5)
        self.send_status_to_gui("จับวัตถุสำเร็จ")

    def pickup_sequence2(self):
        self.send_status_to_gui("เริ่มลำดับการจับวัตถุ (ทั้งหมด)")
        self.move_gripper_smoothly(GRIPPER_OPEN_ANGLE)
        self.move_reach_smoothly(PICKUP_REACH_ANGLE)
        self.move_lift_smoothly(PICKUP_LIFT_ANGLE)
        self.move_gripper_smoothly(GRIPPER_CLOSE_ANGLE)
        time.sleep(0.5)
        self.send_status_to_gui("จับวัตถุทั้งหมดสำเร็จ")

    def drop_sequence(self):
        self.send_status_to_gui("เริ่มลำดับการวางวัตถุ")
        self.move_lift_smoothly(DROP_LIFT_ANGLE)
        self.move_reach_smoothly(DROP_REACH_ANGLE)
        self.move_gripper_smoothly(GRIPPER_OPEN_ANGLE)
        self.move_lift_smoothly(HOME_LIFT_ANGLE)
        self.move_reach_smoothly(HOME_REACH_ANGLE)
        self.move_base_smoothly_main(90) # ใช้ฟังก์ชัน main สำหรับกลับไปที่ 90 องศา
        self.send_status_to_gui("วางวัตถุสำเร็จ")
        
    def drop_sequence2(self):
        self.send_status_to_gui("เริ่มลำดับการวางวัตถุ (ทั้งหมด)")
        self.move_reach_smoothly(HOME_REACH_ANGLE)
        self.move_gripper_smoothly(GRIPPER_OPEN_ANGLE)
        self.move_lift_smoothly(HOME_LIFT_ANGLE)
        self.move_reach_smoothly(HOME_REACH_ANGLE)
        self.send_status_to_gui("วางวัตถุทั้งหมดสำเร็จ")

    def search_for_object(self, target_label):
        if target_label == "Base" and self.base_drop_angle is not None:
            status_msg = f"ใช้ตำแหน่งที่จำไว้สำหรับ Base ที่มุมกล้อง {self.base_drop_angle}°"
            self.send_status_to_gui(status_msg)
            time.sleep(1)
            return True, self.base_drop_angle

        self.send_status_to_gui(f"กำลังค้นหา: {target_label}...")
        
        if target_label == "Base":
            self.send_status_to_gui("เริ่มสแกนหา Base จาก 180 องศา")
            self.servo_camera.write(CAMERA_SCAN_END_ANGLE)
            time.sleep(0.5)
            scan_angles = list(range(CAMERA_SCAN_END_ANGLE, CAMERA_SCAN_START_ANGLE, -CAMERA_SCAN_STEP))
        else:
            scan_angles = list(range(CAMERA_SCAN_START_ANGLE, CAMERA_SCAN_END_ANGLE, CAMERA_SCAN_STEP)) + \
                          list(range(CAMERA_SCAN_END_ANGLE, CAMERA_SCAN_START_ANGLE, -CAMERA_SCAN_STEP))

        for angle in scan_angles:
            if not self.running: return False, -1
            self.servo_camera.write(angle)
            self.current_camera_angle = angle
            time.sleep(0.5)
            ret, frame = self.cap.read()
            if not ret: continue
            results = self.model(frame, verbose=False)
            annotated_frame = results[0].plot()

            for box in results[0].boxes:
                detected_label = self.model.names[int(box.cls)]
                confidence = float(box.conf[0])
                if detected_label == target_label and confidence >= CONFIDENCE_THRESHOLD:
                    x1, y1, x2, y2 = box.xyxy[0]
                    center_x = (x1 + x2) / 2
                    frame_width = frame.shape[1]
                    if frame_width * 0.2 < center_x < frame_width * 0.8:
                        status_msg = f"เจอ {target_label} (มั่นใจ {confidence:.0%}) ที่มุมกล้อง {angle}°!"
                        self.send_status_to_gui(status_msg)
                        if target_label == "Base":
                            self.base_drop_angle = angle
                            self.send_status_to_gui(f"จดจำตำแหน่ง Base ที่มุม: {angle}°")
                        cv2.putText(annotated_frame, f"TARGET: {target_label} @ {angle} deg", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                        self.send_frame_to_gui(annotated_frame)
                        return True, angle

            cv2.putText(annotated_frame, f"Scanning... Angle: {angle}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 0), 2)
            self.send_frame_to_gui(annotated_frame)

        self.send_status_to_gui(f"ไม่พบ {target_label} ในการสแกนรอบนี้")
        return False, -1

    def search_for_all_objects(self, target_labels):
        found_objects = {label: None for label in target_labels}
        remaining_targets = set(target_labels)
        
        status_msg = f"กำลังค้นหาวัตถุทั้งหมด: {', '.join(remaining_targets)}"
        self.send_status_to_gui(status_msg)

        scan_angles = list(range(CAMERA_SCAN_START_ANGLE, CAMERA_SCAN_END_ANGLE, CAMERA_SCAN_STEP)) + \
                      list(range(CAMERA_SCAN_END_ANGLE, CAMERA_SCAN_START_ANGLE, -CAMERA_SCAN_STEP))

        while self.running and remaining_targets:
            self.send_status_to_gui(f"เริ่มการสแกนรอบใหม่... ยังเหลือ: {', '.join(remaining_targets)}")
            for angle in scan_angles:
                if not self.running or not remaining_targets: break
                
                self.servo_camera.write(angle)
                self.current_camera_angle = angle
                time.sleep(0.05)
                ret, frame = self.cap.read()
                if not ret: continue

                results = self.model(frame, verbose=False)
                annotated_frame = results[0].plot()

                for box in results[0].boxes:
                    detected_label = self.model.names[int(box.cls)]
                    confidence = float(box.conf[0])

                    if detected_label in remaining_targets and confidence >= CONFIDENCE_THRESHOLD:
                        x1, y1, x2, y2 = box.xyxy[0]
                        center_x = (x1 + x2) / 2
                        frame_width = frame.shape[1]

                        if frame_width * 0.4 < center_x < frame_width * 0.6:
                            status_msg = f"พบ {detected_label} ที่มุม {angle}°! เหลือ {len(remaining_targets)-1} ชิ้นที่ต้องหา"
                            self.send_status_to_gui(status_msg)
                            found_objects[detected_label] = angle
                            remaining_targets.remove(detected_label)
                            cv2.putText(annotated_frame, f"FOUND: {detected_label} @ {angle} deg", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

                cv2.putText(annotated_frame, f"Scanning... Angle: {angle}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 0), 2)
                self.send_frame_to_gui(annotated_frame)

        if not self.running: return None
        self.send_status_to_gui("ค้นหาวัตถุทั้งหมดเจอแล้ว!")
        return found_objects

    def run_main_sequence(self):
        if not self.running:
            self.send_status_to_gui("เกิดข้อผิดพลาดในการเริ่มต้นระบบ")
            return
            
        self.go_to_home_position()
        object_sequence = ["Red", "Green", "Blue"]

        self.send_status_to_gui("กำลังเตรียมกล้อง... แสดงภาพตัวอย่าง")
        start_time = time.time()
        while time.time() - start_time < 0.5:
            if not self.running: return
            ret, frame = self.cap.read()
            if ret:
                cv2.putText(frame, "Camera Preview...", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
                self.send_frame_to_gui(frame)
            time.sleep(0.03)
        
        all_object_locations = self.search_for_all_objects(object_sequence)

        if all_object_locations is None:
            self.send_status_to_gui("หยุดการทำงานเนื่องจากไม่สามารถหาวัตถุทั้งหมดได้")
            return

        self.send_status_to_gui("จัดตำแหน่งกล้องใหม่เพื่อค้นหา Base")
        self.move_camera_smoothly(90)
        
        # 2. วนลูปเพื่อหยิบ-วาง ทีละชิ้นตามลำดับ (ใช้ move_base_smoothly_main)
        for obj_to_find in object_sequence:
            if not self.running: break

            target_angle = all_object_locations[obj_to_find]
            self.send_status_to_gui(f"เริ่มกระบวนการสำหรับ {obj_to_find} ที่มุมกล้อง {target_angle}°")

            base_angle = 180 - target_angle - BASE_TO_CAMERA_OFFSET_DEGREES_PICKUP
            self.move_base_smoothly_main(base_angle) # <--- ใช้ฟังก์ชัน Main
            self.pickup_sequence()

            found, drop_base_angle = self.search_for_object("Base")
            if not found:
                self.send_status_to_gui("ไม่สามารถหาตำแหน่งวาง (Base) ได้, หยุดการทำงาน")
                break
            
            self.move_base_smoothly_main(180 - drop_base_angle - BASE_TO_CAMERA_OFFSET_DEGREES1_DROP) # <--- ใช้ฟังก์ชัน Main
            self.drop_sequence()

        if not self.running: return

        # 3. ขั้นตอนสุดท้าย (ใช้ move_base_smoothly_final)
        self.send_status_to_gui("จัดเรียงโดนัทครบแล้ว, เริ่มขั้นตอนสุดท้าย")
        
        found, final_base_angle = self.search_for_object("Base")
        if not found:
             self.send_status_to_gui("ไม่พบ Base ในขั้นตอนสุดท้าย, หยุดการทำงาน")
             return

        self.move_base_smoothly_final(180 - final_base_angle - BASE_TO_CAMERA_OFFSET_DEGREES1_DROP) # <--- ใช้ฟังก์ชัน Final
        self.pickup_sequence2()
        self.move_base_smoothly_final(100) # <--- ใช้ฟังก์ชัน Final (ปรับปรุงค่าตามที่ผู้ใช้ระบุ)
        self.drop_sequence2()
        self.send_status_to_gui("ภารกิจเสร็จสิ้น!")


    def cleanup(self):
        print("CONTROLLER: กำลังปิดระบบ...")
        self.running = False
        time.sleep(0.5)
        self.cap.release()
        self.gui_socket.close()
        if hasattr(self, 'board'):
            print("CONTROLLER: กำลังนำแขนกลกลับสู่ตำแหน่งเริ่มต้น...")
            self.go_to_home_position()
            self.move_camera_smoothly(90)
            self.move_base_smoothly_main(0) # กลับไปที่มุม 0 องศา
            self.board.exit()
        print("CONTROLLER: ระบบปิดเรียบร้อย")

def run_robot():
    """ฟังก์ชันนี้จะถูกเรียกโดย Process ของ GUI"""
    controller = RobotController()
    try:
        controller.run_main_sequence()
    except KeyboardInterrupt:
        print("CONTROLLER: ได้รับคำสั่งหยุด (KeyboardInterrupt)")
    finally:
        controller.cleanup()

# หากต้องการทดสอบไฟล์นี้โดยตรง สามารถ un-comment บรรทัดข้างล่างได้
# if __name__ == '__main__':
#     run_robot()
