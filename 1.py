# robot_controller.py
import cv2
import time
import zmq
from ultralytics import YOLO
import pyfirmata2

# --- การตั้งค่าที่ต้องปรับแก้ ---
# ปรับแก้ port ให้ตรงกับ Arduino ของคุณ
ARDUINO_PORT = "COM19"

# ปรับแก้ Pin ของ Servo ทั้ง 6 ตัวให้ถูกต้อง
PIN_SERVO_CAMERA = 9   # Servo หมุนกล้อง
PIN_SERVO_BASE = 10    # Servo หมุนฐานแขนกล
PIN_SERVO_LIFT = 11    # Servo ยกแขนขึ้น-ลง
PIN_SERVO_REACH = 12   # Servo ยืด/หด แขน (ปรับระดับ)
PIN_SERVO_GRIPPER = 13 # Servo ตัวคีบ

# --- ค่ามุมการสแกนของกล้อง ---
CAMERA_SCAN_START_ANGLE = 0
CAMERA_SCAN_END_ANGLE = 180
CAMERA_SCAN_STEP = 5

# --- ค่ามุมของแขนกล (ปรับจูนให้เหมาะกับแขนกลของคุณ) ---
HOME_LIFT_ANGLE = 20
HOME_REACH_ANGLE = 75
HOME_GRIPPER_ANGLE = 15
PICKUP_LIFT_ANGLE = 80
PICKUP_REACH_ANGLE = 10
GRIPPER_OPEN_ANGLE = 100
GRIPPER_CLOSE_ANGLE = 0
DROP_LIFT_ANGLE = 45
DROP_REACH_ANGLE = 51

# --- การตั้งค่าอื่นๆ ---
WEBCAM_INDEX = 0
YOLO_MODEL_PATH = 'best1.pt'  # Path ไปยังโมเดล .pt ที่เทรนมา

# ตั้งค่าความมั่นใจขั้นต่ำ (เช่น 0.80 คือ 80%)
# หุ่นยนต์จะทำงานต่อเมื่อค่าความมั่นใจสูงกว่าค่านี้
CONFIDENCE_THRESHOLD = 0.80

# --- ค่าสำหรับปรับแก้การติดตั้ง ---
# ปรับค่านี้เพื่อชดเชยการติดตั้งที่ทำให้มุมกล้องและแขนกลไม่ตรงกัน
# หากแขนกลหมุนไปทางซ้ายของวัตถุ ให้เพิ่มค่านี้
# หากแขนกลหมุนไปทางขวาของวัตถุ ให้ลดค่านี้
BASE_TO_CAMERA_OFFSET_DEGREES_PICKUP = 0
BASE_TO_CAMERA_OFFSET_DEGREES1_DROP = 0

class RobotController:
    def __init__(self):
        self.running = True
        self.base_drop_angle = None # ตัวแปรสำหรับจดจำมุมของ Base

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

        context = zmq.Context()
        self.gui_socket = context.socket(zmq.PUB)
        self.gui_socket.bind("tcp://*:555")

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

    def go_to_home_position(self):
        self.send_status_to_gui("กำลังกลับสู่ตำแหน่งเริ่มต้น (Home)")
        self.servo_gripper.write(HOME_GRIPPER_ANGLE)
        time.sleep(0.5)
        self.servo_lift.write(HOME_LIFT_ANGLE)
        self.servo_reach.write(HOME_REACH_ANGLE)
        time.sleep(0.5)

    def pickup_sequence(self):
        self.send_status_to_gui("เริ่มลำดับการจับวัตถุ")
        self.servo_gripper.write(GRIPPER_OPEN_ANGLE)
        time.sleep(0.7)
        self.servo_reach.write(PICKUP_REACH_ANGLE)
        time.sleep(0.5)
        self.servo_lift.write(PICKUP_LIFT_ANGLE)
        time.sleep(0.5)
        self.servo_gripper.write(GRIPPER_CLOSE_ANGLE)
        time.sleep(0.7)
        self.servo_lift.write(HOME_LIFT_ANGLE)
        self.servo_reach.write(HOME_REACH_ANGLE)
        time.sleep(1)
        self.send_status_to_gui("จับวัตถุสำเร็จ")

    def pickup_sequence2(self):
        self.send_status_to_gui("เริ่มลำดับการจับวัตถุ (ทั้งหมด)")
        self.servo_gripper.write(GRIPPER_OPEN_ANGLE)
        time.sleep(0.7)
        self.servo_reach.write(PICKUP_REACH_ANGLE)
        time.sleep(0.5)
        self.servo_lift.write(PICKUP_LIFT_ANGLE)
        time.sleep(0.5)
        self.servo_gripper.write(GRIPPER_CLOSE_ANGLE)
        time.sleep(1)
        self.send_status_to_gui("จับวัตถุทั้งหมดสำเร็จ")

    def drop_sequence(self):
        self.send_status_to_gui("เริ่มลำดับการวางวัตถุ")
        self.servo_lift.write(DROP_LIFT_ANGLE)
        self.servo_reach.write(DROP_REACH_ANGLE)
        time.sleep(0.7)
        self.servo_gripper.write(GRIPPER_OPEN_ANGLE)
        time.sleep(0.7)
        self.servo_lift.write(HOME_LIFT_ANGLE)
        self.servo_reach.write(HOME_REACH_ANGLE)
        time.sleep(0.5)
        self.move_base(90)
        self.send_status_to_gui("วางวัตถุสำเร็จ")
        
    def drop_sequence2(self):
        self.send_status_to_gui("เริ่มลำดับการวางวัตถุ (ทั้งหมด)")
        self.servo_reach.write(HOME_REACH_ANGLE)
        time.sleep(0.7)
        self.servo_gripper.write(GRIPPER_OPEN_ANGLE)
        time.sleep(0.7)
        self.servo_lift.write(HOME_LIFT_ANGLE)
        self.servo_reach.write(HOME_REACH_ANGLE)
        time.sleep(0.5)
        self.send_status_to_gui("วางวัตถุทั้งหมดสำเร็จ")

    def move_base(self, angle):
        self.send_status_to_gui(f"กำลังหมุนฐานไปที่ {angle}°")
        self.servo_base.write(angle)
        time.sleep(1.5)

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
                    if frame_width * 0.3 < center_x < frame_width * 0.7:
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
                if not self.running: return None
                
                if not remaining_targets:
                    break

                self.servo_camera.write(angle)
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

                        if frame_width * 0.3 < center_x < frame_width * 0.7:
                            status_msg = f"พบ {detected_label} ที่มุม {angle}°! เหลือ {len(remaining_targets)-1} ชิ้นที่ต้องหา"
                            self.send_status_to_gui(status_msg)
                            
                            found_objects[detected_label] = angle
                            remaining_targets.remove(detected_label)
                            
                            cv2.putText(annotated_frame, f"FOUND: {detected_label} @ {angle} deg", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

                cv2.putText(annotated_frame, f"Scanning... Angle: {angle}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 0), 2)
                self.send_frame_to_gui(annotated_frame)

        if not self.running:
            return None
            
        self.send_status_to_gui("ค้นหาวัตถุทั้งหมดเจอแล้ว!")
        return found_objects

    # ===== [ปรับปรุงใหม่] ลำดับการทำงานหลัก =====
    def run_main_sequence(self):
        if not self.running:
            self.send_status_to_gui("เกิดข้อผิดพลาดในการเริ่มต้นระบบ")
            return
            
        self.go_to_home_position()
        object_sequence = ["Red", "Green", "Blue"]

        # ===== ลบส่วนแสดงภาพตัวอย่าง 1 วินาทีออกไปแล้ว =====
        
        # 1. ค้นหาตำแหน่งของวัตถุ (Red, Green, Blue) ทั้งหมดก่อน
        all_object_locations = self.search_for_all_objects(object_sequence)

        if all_object_locations is None:
            self.send_status_to_gui("หยุดการทำงานเนื่องจากไม่สามารถหาวัตถุทั้งหมดได้")
            return

        # --- เพิ่มโค้ดตามคำขอ: จัดตำแหน่งกล้องใหม่ ---
        self.send_status_to_gui("จัดตำแหน่งกล้องใหม่เพื่อค้นหา Base")
        self.servo_camera.write(90)
        time.sleep(1) # รอให้กล้องหมุนไปที่ 90 องศา
        # -----------------------------------------

        # 2. วนลูปเพื่อหยิบ-วาง ทีละชิ้นตามลำดับ
        for obj_to_find in object_sequence:
            if not self.running: break

            target_angle = all_object_locations[obj_to_find]
            self.send_status_to_gui(f"เริ่มกระบวนการสำหรับ {obj_to_find} ที่มุมกล้อง {target_angle}°")

            # หมุนฐานไปหาวัตถุและหยิบ
            base_angle = 180 - target_angle - BASE_TO_CAMERA_OFFSET_DEGREES_PICKUP
            self.move_base(base_angle)
            self.pickup_sequence()

            # ค้นหา Base เพื่อวาง (จะค้นหาแค่ครั้งแรก ครั้งต่อไปจะใช้ค่าที่จำไว้)
            found, drop_base_angle = self.search_for_object("Base")
            if not found:
                self.send_status_to_gui("ไม่สามารถหาตำแหน่งวาง (Base) ได้, หยุดการทำงาน")
                break
            
            # หมุนฐานไปที่ตำแหน่งวาง และวางวัตถุ
            self.move_base(180 - drop_base_angle - BASE_TO_CAMERA_OFFSET_DEGREES1_DROP)
            self.drop_sequence()

        if not self.running: return

        # 3. ขั้นตอนสุดท้าย: กลับไปหยิบโดนัททั้งหมดที่วางซ้อนกันแล้ว
        self.send_status_to_gui("จัดเรียงโดนัทครบแล้ว, เริ่มขั้นตอนสุดท้าย")
        
        # ค้นหา Base อีกครั้งเพื่อหยิบทั้งหมด (จะใช้ค่าที่จำไว้)
        found, final_base_angle = self.search_for_object("Base")
        if not found:   
            self.send_status_to_gui("ไม่สามารถหาตำแหน่งสุดท้าย (Base) ได้, หยุดการทำงาน")
            return

        self.move_base(180 - final_base_angle - BASE_TO_CAMERA_OFFSET_DEGREES1_DROP)
        self.pickup_sequence2()
        self.move_base(96) # หมุนไปที่ตำแหน่งสุดท้าย
        self.drop_sequence2()
        self.send_status_to_gui("ภารกิจเสร็จสิ้น!")


    def cleanup(self):
        print("CONTROLLER: กำลังปิดระบบ...")
        self.running = False
        time.sleep(0.5)
        if hasattr(self, 'cap') and self.cap.isOpened():
             self.cap.release()
        if hasattr(self, 'gui_socket'):
             self.gui_socket.close()
        if hasattr(self, 'board'):
            self.go_to_home_position() # กลับสู่ท่าเริ่มต้น
            self.servo_camera.write(90)
            self.servo_base.write(90)
            self.board.exit()
        print("CONTROLLER: ระบบปิดเรียบร้อย")

# ===== [ปรับปรุงใหม่] ฟังก์ชันหลักสำหรับรันใน Process แยก =====
def run_robot():
    """
    ฟังก์ชันนี้จะถูกเรียกโดย Process ของ GUI
    จะทำการเปิดกล้องและส่งภาพไปเรื่อยๆ และรอรับคำสั่ง 'START'
    """
    controller = RobotController()
    if not controller.running:
        print("CONTROLLER: ไม่สามารถเริ่มต้น Controller ได้")
        controller.cleanup()
        return

    # ตั้งค่า Socket สำหรับรับคำสั่งจาก GUI
    context = zmq.Context()
    command_socket = context.socket(zmq.SUB)
    command_socket.connect("tcp://localhost:5556")
    command_socket.setsockopt(zmq.SUBSCRIBE, b"COMMAND")
    
    sequence_started = False

    try:
        # Loop หลัก: ส่งภาพวิดีโอและรอรับคำสั่ง
        while controller.running:
            # 1. รับและส่งภาพจากกล้อง
            ret, frame = controller.cap.read()
            if ret:
                # เพิ่มข้อความบอกสถานะ "Ready" บนวิดีโอถ้ายังไม่เริ่ม
                if not sequence_started:
                     cv2.putText(frame, "Ready - Press START", (10, 30), 
                                 cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                controller.send_frame_to_gui(frame)
            
            # 2. ตรวจสอบว่ามีคำสั่ง 'START' เข้ามาหรือไม่
            if not sequence_started:
                try:
                    topic, msg = command_socket.recv_multipart(flags=zmq.NOBLOCK)
                    if topic == b"COMMAND" and msg == b"START":
                        print("CONTROLLER: ได้รับคำสั่ง START!")
                        sequence_started = True
                        controller.run_main_sequence()
                        # เมื่อทำงานเสร็จ ให้ส่งสถานะว่าเสร็จแล้ว และหยุดการทำงานของ Loop
                        controller.send_status_to_gui("ภารกิจเสร็จสิ้น! กด STOP เพื่อปิด")
                        break # ออกจาก while loop หลังทำงานเสร็จ
                except zmq.Again:
                    pass # ยังไม่มีคำสั่งเข้ามา
            
            time.sleep(0.03) # หน่วงเวลาเล็กน้อย

    except KeyboardInterrupt:
        print("CONTROLLER: ได้รับคำสั่งหยุด (KeyboardInterrupt)")
    finally:
        command_socket.close()
        context.term()
        controller.cleanup()