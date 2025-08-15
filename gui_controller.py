import sys
import time
import zmq
import cv2
import numpy as np
import pyfirmata2
from ultralytics import YOLO
from PyQt6.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QLabel
from PyQt6.QtCore import QTimer, Qt, QThread, pyqtSignal
from PyQt6.QtGui import QPixmap, QImage
from multiprocessing import freeze_support

# ==============================================================================
# คลาส RobotController
# ==============================================================================

class RobotController:
    # --- การตั้งค่าที่ต้องปรับแก้ ---
    ARDUINO_PORT = "COM19"
    PIN_SERVO_CAMERA = 9
    PIN_SERVO_BASE = 10
    PIN_SERVO_LIFT = 11
    PIN_SERVO_REACH = 12
    PIN_SERVO_GRIPPER = 13

    # --- ตำแหน่งวางวัตถุ (Base) ---
    BASE_DROP_TARGET_ANGLE = 180

    # --- ค่ามุมการสแกนของกล้อง ---
    CAMERA_SCAN_START_ANGLE = 0
    CAMERA_SCAN_END_ANGLE = 180
    CAMERA_SCAN_STEP = 5

    # --- ค่ามุมของแขนกล ---
    HOME_LIFT_ANGLE = 20
    HOME_REACH_ANGLE = 75
    HOME_GRIPPER_ANGLE = 15
    PICKUP_LIFT_ANGLE = 90
    PICKUP_REACH_ANGLE = 0
    
    GRIPPER_OPEN_ANGLE_NORMAL = 85
    GRIPPER_OPEN_ANGLE_WIDE = 120

    GRIPPER_CLOSE_ANGLE = 0
    DROP_LIFT_ANGLE = 40
    DROP_REACH_ANGLE = 67

    # --- การตั้งค่าอื่นๆ ---
    WEBCAM_INDEX = 0
    YOLO_MODEL_PATH = 'best6.pt'
    CONFIDENCE_THRESHOLD = 0.20

    # --- ค่าสำหรับปรับแก้การติดตั้ง ---
    BASE_TO_CAMERA_OFFSET_DEGREES_PICKUP = -6
    BASE_TO_CAMERA_OFFSET_DEGREES1_DROP = -9

    def __init__(self):
        self.running = True
        self.cap = None

        self.current_base_angle = 0
        self.current_camera_angle = 0
        self.current_lift_angle = self.HOME_LIFT_ANGLE
        self.current_reach_angle = self.HOME_REACH_ANGLE
        self.current_gripper_angle = self.HOME_GRIPPER_ANGLE

        context = zmq.Context()
        self.gui_socket = context.socket(zmq.PUB)
        self.gui_socket.bind("tcp://*:5555")

        self.send_status_to_gui("กำลังโหลด YOLOv8 model...")
        self.model = YOLO(self.YOLO_MODEL_PATH)
        self.send_status_to_gui("โหลด YOLOv8 model สำเร็จ")

        self.send_status_to_gui("กำลังเชื่อมต่อกับ Arduino...")
        try:
            self.board = pyfirmata2.Arduino(self.ARDUINO_PORT)
            it = pyfirmata2.util.Iterator(self.board)
            it.start()
            self.send_status_to_gui("เชื่อมต่อ Arduino สำเร็จ")
        except Exception as e:
            self.send_status_to_gui(f"ไม่สามารถเชื่อมต่อ Arduino ได้: {e}")
            self.running = False
            return

        self.servo_camera = self.board.get_pin(f'd:{self.PIN_SERVO_CAMERA}:s')
        self.servo_base = self.board.get_pin(f'd:{self.PIN_SERVO_BASE}:s')
        self.servo_lift = self.board.get_pin(f'd:{self.PIN_SERVO_LIFT}:s')
        self.servo_reach = self.board.get_pin(f'd:{self.PIN_SERVO_REACH}:s')
        self.servo_gripper = self.board.get_pin(f'd:{self.PIN_SERVO_GRIPPER}:s')

        self.send_status_to_gui("กำลังตั้งค่ามุมเริ่มต้นของ Servo...")
        self.servo_camera.write(self.current_camera_angle)
        self.servo_base.write(self.current_base_angle)
        self.servo_lift.write(self.current_lift_angle)
        self.servo_reach.write(self.current_reach_angle)
        self.servo_gripper.write(self.current_gripper_angle)
        time.sleep(1)

        self.send_status_to_gui("ระบบพร้อมทำงาน กด START เพื่อเริ่มภารกิจ")

    def send_status_to_gui(self, status_text):
        if not self.running: return
        self.gui_socket.send_multipart([b"STATUS", status_text.encode('utf-8')])
        print(f"STATUS: {status_text}")

    def send_frame_to_gui(self, frame):
        if not self.running: return
        _, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
        self.gui_socket.send_multipart([b"VIDEO", buffer.tobytes()])

    def move_servo_smoothly(self, servo_obj, target_angle, current_angle, step=2, speed_delay=0.015):
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

    def decrease_brightness(self, frame, value=200):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        h, s, v = cv2.split(hsv)
        v = np.where(v > value, v - value, 0).astype('uint8')
        final_hsv = cv2.merge((h, s, v))
        darker_frame = cv2.cvtColor(final_hsv, cv2.COLOR_HSV2BGR)
        return darker_frame

    def move_base_smoothly_main(self, angle):
        self.send_status_to_gui(f"กำลังหมุนฐาน (Main) ไปที่ {angle}°")
        self.current_base_angle = self.move_servo_smoothly(self.servo_base, angle, self.current_base_angle, step=5, speed_delay=0.03)
    def move_base_smoothly_final(self, angle):
        self.send_status_to_gui(f"กำลังหมุนฐาน (Final) ไปที่ {angle}°")
        self.current_base_angle = self.move_servo_smoothly(self.servo_base, angle, self.current_base_angle, step=5, speed_delay=0.05)
    def move_lift_smoothly(self, angle):
        self.current_lift_angle = self.move_servo_smoothly(self.servo_lift, angle, self.current_lift_angle)
    def move_reach_smoothly(self, angle):
        self.current_reach_angle = self.move_servo_smoothly(self.servo_reach, angle, self.current_reach_angle)
    def move_gripper_smoothly(self, angle):
        self.current_gripper_angle = self.move_servo_smoothly(self.servo_gripper, angle, self.current_gripper_angle, speed_delay=0.009)
    def move_camera_smoothly(self, angle):
        self.current_camera_angle = self.move_servo_smoothly(self.servo_camera, angle, self.current_camera_angle, speed_delay=0.09)

    def go_to_home_position(self):
        self.send_status_to_gui("กำลังกลับสู่ตำแหน่งเริ่มต้น (Home)")
        self.move_gripper_smoothly(self.HOME_GRIPPER_ANGLE)
        self.move_lift_smoothly(self.HOME_LIFT_ANGLE)
        self.move_reach_smoothly(self.HOME_REACH_ANGLE)
        time.sleep(0.4)

    def pickup_sequence_dynamic(self, open_angle):
        self.send_status_to_gui(f"เริ่มลำดับการจับวัตถุ (เปิด Gripper ที่ {open_angle}°)")
        self.move_gripper_smoothly(open_angle)
        self.move_reach_smoothly(3)
        time.sleep(0.5)
        self.move_lift_smoothly(self.PICKUP_LIFT_ANGLE)
        time.sleep(0.2)
        self.move_reach_smoothly(45)
        self.move_gripper_smoothly(self.GRIPPER_CLOSE_ANGLE)
        self.move_lift_smoothly(self.HOME_LIFT_ANGLE)
        self.move_reach_smoothly(self.HOME_REACH_ANGLE)
        time.sleep(0.5)
        self.send_status_to_gui("จับวัตถุสำเร็จ")

    def pickup_sequence2(self):
        self.send_status_to_gui("เริ่มลำดับการจับวัตถุ (ทั้งหมด)")
        self.move_gripper_smoothly(self.GRIPPER_OPEN_ANGLE_WIDE) 
        self.move_reach_smoothly(30)
        self.move_lift_smoothly(self.PICKUP_LIFT_ANGLE)
        self.move_gripper_smoothly(self.GRIPPER_CLOSE_ANGLE)
        time.sleep(0.8)
        self.send_status_to_gui("จับวัตถุทั้งหมดสำเร็จ")
        
    def drop_sequence(self):
        self.send_status_to_gui("เริ่มลำดับการวางวัตถุ")
        self.move_lift_smoothly(self.DROP_LIFT_ANGLE)
        self.move_reach_smoothly(self.DROP_REACH_ANGLE)
        self.move_gripper_smoothly(self.GRIPPER_OPEN_ANGLE_WIDE)
        self.move_lift_smoothly(self.HOME_LIFT_ANGLE)
        self.move_reach_smoothly(self.HOME_REACH_ANGLE)
        self.move_base_smoothly_main(90)
        self.send_status_to_gui("วางวัตถุสำเร็จ")
        
    def drop_sequence2(self):
        self.send_status_to_gui("เริ่มลำดับการวางวัตถุ (ทั้งหมด)")
        self.move_gripper_smoothly(self.GRIPPER_OPEN_ANGLE_WIDE)
        self.move_reach_smoothly(self.PICKUP_REACH_ANGLE)
        time.sleep(0.2)
        self.send_status_to_gui("วางวัตถุทั้งหมดสำเร็จ")
        
    def drop_sequence3(self):
        self.send_status_to_gui("เริ่มการดัน")
        self.move_lift_smoothly(80)
        self.move_gripper_smoothly(self.GRIPPER_CLOSE_ANGLE)
        self.move_reach_smoothly(45)   #ลบบรรทัดนี้เพื่อชนะ
        time.sleep(0.2)
        self.move_lift_smoothly(self.HOME_LIFT_ANGLE)
        self.move_reach_smoothly(self.HOME_REACH_ANGLE)
        time.sleep(0.3)
        self.send_status_to_gui("ดันวัตถุทั้งหมดสำเร็จ")

    def search_for_object(self, target_label):
        if target_label == "Base":
            status_msg = f"ใช้ตำแหน่ง Base ที่กำหนดไว้: {self.BASE_DROP_TARGET_ANGLE}°"
            self.send_status_to_gui(status_msg)
            self.move_camera_smoothly(self.BASE_DROP_TARGET_ANGLE)
            time.sleep(0.5)
            return True, self.BASE_DROP_TARGET_ANGLE

        self.send_status_to_gui(f"กำลังค้นหา: {target_label}...")
        scan_angles = list(range(self.CAMERA_SCAN_START_ANGLE, self.CAMERA_SCAN_END_ANGLE, self.CAMERA_SCAN_STEP)) + \
                      list(range(self.CAMERA_SCAN_END_ANGLE, self.CAMERA_SCAN_START_ANGLE, -self.CAMERA_SCAN_STEP))
        
        for angle in scan_angles:
            if not self.running: return False, -1
            self.servo_camera.write(angle)
            self.current_camera_angle = angle
            time.sleep(0.5)
            ret, frame = self.cap.read()
            if not ret: continue

            frame = self.decrease_brightness(frame, value=60)

            results = self.model(frame, verbose=False)
            annotated_frame = results[0].plot()
            for box in results[0].boxes:
                detected_label = self.model.names[int(box.cls)]
                confidence = float(box.conf[0])
                if detected_label == target_label and confidence >= self.CONFIDENCE_THRESHOLD:
                    x1, y1, x2, y2 = box.xyxy[0]
                    center_x = (x1 + x2) / 2
                    frame_width = frame.shape[1]
                    if frame_width * 0.3 < center_x < frame_width * 0.7:
                        status_msg = f"เจอ {target_label} (มั่นใจ {confidence:.0%}) ที่มุมกล้อง {angle}°!"
                        self.send_status_to_gui(status_msg)
                        cv2.putText(annotated_frame, f"TARGET: {target_label} @ {angle} deg", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                        self.send_frame_to_gui(annotated_frame)
                        return True, angle
            cv2.putText(annotated_frame, f"Scanning... Angle: {angle}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 0), 2)
            self.send_frame_to_gui(annotated_frame)
            
        self.send_status_to_gui(f"ไม่พบ {target_label} ในการสแกนรอบนี้")
        return False, -1

    # <<< [การเปลี่ยนแปลง 1/2] >>> เพิ่มการค้นหาซ้ำ 2 ครั้ง และดำเนินการต่อหากไม่พบ
    def search_for_all_objects(self, target_labels):
        MAX_SCANS = 2
        found_objects = {label: None for label in target_labels}
        remaining_targets = set(target_labels)
        
        for scan_attempt in range(MAX_SCANS):
            # ถ้าหาเจอหมดแล้ว หรือถ้าผู้ใช้สั่งหยุด ก็ออกจากลูป
            if not self.running or not remaining_targets:
                break

            status_msg = f"เริ่มการสแกนหาวัตถุ (รอบที่ {scan_attempt + 1}/{MAX_SCANS})... ยังเหลือ: {', '.join(remaining_targets)}"
            self.send_status_to_gui(status_msg)

            scan_angles = list(range(self.CAMERA_SCAN_START_ANGLE, self.CAMERA_SCAN_END_ANGLE, self.CAMERA_SCAN_STEP)) + \
                          list(range(self.CAMERA_SCAN_END_ANGLE, self.CAMERA_SCAN_START_ANGLE, -self.CAMERA_SCAN_STEP))
            
            for angle in scan_angles:
                if not self.running or not remaining_targets: break
                self.servo_camera.write(angle)
                self.current_camera_angle = angle
                time.sleep(0.09)
                ret, frame = self.cap.read()
                if not ret: continue

                frame = self.decrease_brightness(frame, value=100)

                results = self.model(frame, verbose=False)
                annotated_frame = results[0].plot()
                for box in results[0].boxes:
                    detected_label = self.model.names[int(box.cls)]
                    confidence = float(box.conf[0])
                    if detected_label in remaining_targets and confidence >= self.CONFIDENCE_THRESHOLD:
                        x1, y1, x2, y2 = box.xyxy[0]
                        center_x = (x1 + x2) / 2
                        frame_width = frame.shape[1]
                        if frame_width * 0.2 < center_x < frame_width * 0.8:
                            status_msg = f"พบ {detected_label} ที่มุม {angle}°! เหลือ {len(remaining_targets)-1} ชิ้นที่ต้องหา"
                            self.send_status_to_gui(status_msg)
                            found_objects[detected_label] = angle
                            remaining_targets.remove(detected_label)
                            cv2.putText(annotated_frame, f"FOUND: {detected_label} @ {angle} deg", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv2.putText(annotated_frame, f"Scanning... Angle: {angle}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 0), 2)
                self.send_frame_to_gui(annotated_frame)
            
            # หน่วงเวลาก่อนเริ่มสแกนรอบใหม่ (ถ้ายังหาไม่ครบ)
            if self.running and remaining_targets and scan_attempt < MAX_SCANS - 1:
                time.sleep(1)

        if not self.running: return None

        if not remaining_targets:
            self.send_status_to_gui("ค้นหาวัตถุทั้งหมดเจอแล้ว!")
        else:
            self.send_status_to_gui(f"ค้นหาครบ {MAX_SCANS} รอบแล้ว ไม่พบ: {', '.join(remaining_targets)}. จะดำเนินการต่อเฉพาะวัตถุที่พบ")
            
        return found_objects

    def run_main_sequence(self):
        try:
            if not self.running:
                self.send_status_to_gui("เกิดข้อผิดพลาดในการเริ่มต้นระบบ")
                return

            self.send_status_to_gui("กำลังเปิด Webcam...")
            self.cap = cv2.VideoCapture(self.WEBCAM_INDEX)
            if not self.cap or not self.cap.isOpened():
                self.send_status_to_gui("ไม่สามารถเปิด Webcam ได้, หยุดการทำงาน")
                return
            self.send_status_to_gui("Webcam เปิดสำเร็จ")
            
            time.sleep(0.5)

            self.go_to_home_position()
            object_sequence = ["Red", "Green", "Blue"]
            
            all_object_locations = self.search_for_all_objects(object_sequence)

            if all_object_locations is None:
                self.send_status_to_gui("หยุดการทำงานระหว่างการค้นหา")
                return

            self.send_status_to_gui("จัดตำแหน่งกล้องใหม่เพื่อเตรียมทำงาน")
            self.move_camera_smoothly(90)
            
            for obj_to_find in object_sequence:
                if not self.running: break
                
                target_angle = all_object_locations.get(obj_to_find)

                # <<< [การเปลี่ยนแปลง 2/2] >>> เพิ่มการตรวจสอบว่าหาวัตถุเจอหรือไม่
                if target_angle is None:
                    self.send_status_to_gui(f"ไม่พบวัตถุ '{obj_to_find}', ข้ามขั้นตอนนี้")
                    continue # ข้ามไปทำงานกับวัตถุชิ้นถัดไป

                self.send_status_to_gui(f"เริ่มกระบวนการสำหรับ {obj_to_find} ที่มุมกล้อง {target_angle}°")
                
                base_angle = 180 - target_angle - self.BASE_TO_CAMERA_OFFSET_DEGREES_PICKUP
                self.move_base_smoothly_main(base_angle)
                
                if base_angle <= 90:
                    self.pickup_sequence_dynamic(self.GRIPPER_OPEN_ANGLE_NORMAL)
                else:
                    self.pickup_sequence_dynamic(self.GRIPPER_OPEN_ANGLE_WIDE)
                
                found, drop_base_angle = self.search_for_object("Base")
                
                if not found:
                    self.send_status_to_gui("ไม่สามารถหาตำแหน่งวาง (Base) ได้, หยุดการทำงาน")
                    break
                    
                self.move_base_smoothly_main(180 - drop_base_angle - self.BASE_TO_CAMERA_OFFSET_DEGREES1_DROP)
                self.drop_sequence()

            if not self.running: return

            self.send_status_to_gui("จัดเรียงโดนัทครบแล้ว, เริ่มขั้นตอนสุดท้าย")
            
            found, final_base_angle = self.search_for_object("Base")

            if not found:
                    self.send_status_to_gui("ไม่พบ Base ในขั้นตอนสุดท้าย, หยุดการทำงาน")
                    return

            self.move_base_smoothly_final(180 - final_base_angle - self.BASE_TO_CAMERA_OFFSET_DEGREES1_DROP)
            self.pickup_sequence2()
            self.move_base_smoothly_final(100)
            self.drop_sequence2()
            self.drop_sequence3()
            self.send_status_to_gui("ภารกิจเสร็จสิ้น!")

        finally:
            if self.cap and self.cap.isOpened():
                self.cap.release()
                self.cap = None
                print("CONTROLLER: Webcam released after sequence.")


# ==============================================================================
# คลาส RobotWorker (QThread)
# ==============================================================================

class RobotWorker(QThread):
    def __init__(self):
        super().__init__()
        self.controller = None
        self._is_running = True
        self._execute_sequence = False

    def run(self):
        self.controller = RobotController()
        if not self.controller.running:
            print("WORKER: การเริ่มต้น Controller ล้มเหลว, Thread จะหยุดทำงาน")
            return

        while self._is_running:
            if self._execute_sequence:
                self.controller.run_main_sequence()
                self._execute_sequence = False 
                self.controller.send_status_to_gui("ภารกิจเสร็จสิ้น! ระบบพร้อมสำหรับงานต่อไป")
            else:
                self.msleep(100) 

    def start_mission(self):
        if self.controller and self.controller.running:
            self._execute_sequence = True

    def stop_worker(self):
        self._is_running = False
        if self.controller:
            self.controller.running = False


# ==============================================================================
# คลาส MainWindow (GUI หลัก)
# ==============================================================================

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("ระบบควบคุมแขนกล (Gemini Robotics Expert)")
        self.setGeometry(100, 100, 800, 600)

        self.video_label = QLabel("กด START MISSION เพื่อเริ่มการทำงานและเปิดกล้อง")
        self.video_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.video_label.setFixedSize(640, 480)
        self.video_label.setStyleSheet("background-color: black; color: white; font-size: 18px;")

        self.status_label = QLabel("สถานะ: กำลังเริ่มต้น...")
        self.status_label.setStyleSheet("font-size: 16px; font-weight: bold; color: #333;")

        self.start_button = QPushButton("START MISSION")
        self.start_button.setStyleSheet("background-color: #4CAF50; color: white; font-size: 18px; padding: 10px;")

        central_widget = QWidget()
        main_layout = QVBoxLayout(central_widget)
        button_layout = QHBoxLayout()
        button_layout.addWidget(self.start_button)
        main_layout.addWidget(self.video_label, alignment=Qt.AlignmentFlag.AlignCenter)
        main_layout.addWidget(self.status_label, alignment=Qt.AlignmentFlag.AlignCenter)
        main_layout.addLayout(button_layout)
        self.setCentralWidget(central_widget)

        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.SUB)
        self.socket.connect("tcp://localhost:5555")
        self.socket.setsockopt(zmq.SUBSCRIBE, b"VIDEO")
        self.socket.setsockopt(zmq.SUBSCRIBE, b"STATUS")

        self.zmq_timer = QTimer(self)
        self.zmq_timer.timeout.connect(self.update_from_zmq)
        self.zmq_timer.start(33)

        self.robot_worker = RobotWorker()
        self.start_button.clicked.connect(self.start_mission_clicked)
        self.robot_worker.start()

    def start_mission_clicked(self):
        self.status_label.setText("สถานะ: ได้รับคำสั่ง! กำลังเริ่มภารกิจ...")
        self.video_label.setText("กำลังเปิดกล้อง...")
        self.video_label.setStyleSheet("background-color: black; color: white; font-size: 18px;")
        
        self.start_button.setEnabled(False)
        self.start_button.setText("MISSION IN PROGRESS...")
        self.start_button.setStyleSheet("background-color: #E0A800; color: white; font-size: 18px; padding: 10px;")
        self.robot_worker.start_mission()

    def update_from_zmq(self):
        try:
            topic, data = self.socket.recv_multipart(flags=zmq.NOBLOCK)

            if topic == b"VIDEO":
                np_arr = np.frombuffer(data, np.uint8)
                frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                if frame is None: return
                rgb_image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                h, w, ch = rgb_image.shape
                bytes_per_line = ch * w
                qt_image = QImage(rgb_image.data, w, h, bytes_per_line, QImage.Format.Format_RGB888)
                self.video_label.setPixmap(QPixmap.fromImage(qt_image).scaled(640, 480, Qt.AspectRatioMode.KeepAspectRatio))

            elif topic == b"STATUS":
                status_text = data.decode('utf-8')
                self.status_label.setText(f"สถานะ: {status_text}")
                if "ภารกิจเสร็จสิ้น" in status_text or "ระบบพร้อม" in status_text:
                    self.start_button.setEnabled(True)
                    self.start_button.setText("START MISSION")
                    self.start_button.setStyleSheet("background-color: #4CAF50; color: white; font-size: 18px; padding: 10px;")
                    if "ภารกิจเสร็จสิ้น" in status_text:
                        self.video_label.setText("ภารกิจเสร็จสิ้น\nกด START MISSION อีกครั้งเพื่อเริ่มใหม่")
                        self.video_label.setStyleSheet("background-color: black; color: white; font-size: 18px;")

        except zmq.Again:
            pass

    def closeEvent(self, event):
        print("GUI: ได้รับคำสั่งปิดหน้าต่าง, กำลังหยุด Worker Thread...")
        self.status_label.setText("สถานะ: กำลังปิดระบบ...")
        self.robot_worker.stop_worker()
        self.robot_worker.wait(5000)
        
        self.zmq_timer.stop()
        self.socket.close()
        self.context.term()
        print("GUI: ปิดระบบเรียบร้อย")
        event.accept()

# ==============================================================================
# จุดเริ่มต้นของโปรแกรม
# ==============================================================================

if __name__ == "__main__":
    freeze_support()
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())