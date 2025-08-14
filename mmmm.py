import tkinter as tk
from tkinter import ttk, messagebox
import cv2
from PIL import Image, ImageTk
import threading
import time
import numpy as np
from pyfirmata2 import Arduino, SERVO
import serial.tools.list_ports
from ultralytics import YOLO
import queue
import logging

# ตั้งค่า logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class RoboticArmController:
    def __init__(self):
        # สถานะระบบ
        self.is_running = False
        self.current_step = "รอการเริ่มต้น"
        self.sequence_step = 0
        
        # Arduino connection
        self.arduino = None
        self.servo_pins = {
            'camera': 9,    # Servo หมุนกล้อง
            'base': 3,      # Servo หมุนฐาน (360°)
            'lift': 10,      # Servo ยกแขน
            'arm': 11,       # Servo ปรับระดับแขน
            'gripper': 12   # Servo คีบ
        }
        self.servos = {}
        
        # กล้อง
        self.cap = None
        self.current_frame = None
        
        # YOLO model
        try:
            self.model = YOLO('best1.pt')  # ใช้ pretrained model ก่อน
            logger.info("โหลด YOLO model สำเร็จ")
        except:
            self.model = None
            logger.warning("ไม่สามารถโหลด YOLO model ได้")
        
        # ตำแหน่งของ servo
        self.servo_positions = {
            'camera': 90,     # ตำแหน่งกล้อง (0-180°)
            'base': 90,       # ตำแหน่งฐาน (0-180°)
            'lift': 90,       # ตำแหน่งแขนยก (0-180°)
            'arm': 0,         # ตำแหน่งแขน (0-180°)
            'gripper': 50     # ตำแหน่งคีบ (0=ปิด, 50=เปิด)
        }
        
        # การแปลงตำแหน่งกล้องเป็นมุม servo
        self.camera_scan_direction = 1  # 1 = ไปขวา, -1 = ไปซ้าย
        self.camera_angle = 90  # มุมปัจจุบันของกล้อง
        
        # ลำดับการทำงาน
        self.work_sequence = ['Red', 'Green', 'Blue']
        self.current_target = 0
        self.target_found = False
        self.base_position_angle = 90  # ตำแหน่งที่วาง Base
        
        # คิวสำหรับการสื่อสาร
        self.status_queue = queue.Queue()
        
    def connect_arduino(self, port):
        """เชื่อมต่อ Arduino"""
        try:
            self.arduino = Arduino(com19)
            time.sleep(2)  # รอให้ Arduino พร้อม
            
            # ตั้งค่า servo pins
            for name, pin in self.servo_pins.items():
                self.arduino.digital[pin].mode = SERVO
                self.servos[name] = self.arduino.digital[pin]
            
            # ตั้งค่าตำแหน่งเริ่มต้น
            self.move_to_initial_position()
            logger.info(f"เชื่อมต่อ Arduino ที่ {port} สำเร็จ")
            return True
            
        except Exception as e:
            logger.error(f"ไม่สามารถเชื่อมต่อ Arduino: {e}")
            return False
    
    def get_available_ports(self):
        """หาพอร์ต Arduino ที่ใช้ได้"""
        ports = []
        for port in serial.tools.list_ports.comports():
            if 'Arduino' in port.description or 'CH340' in port.description or 'USB' in port.description:
                ports.append(port.device)
        return ports
    
    def connect_camera(self, camera_id=0):
        """เชื่อมต่อกล้อง"""
        try:
            self.cap = cv2.VideoCapture(camera_id)
            if self.cap.isOpened():
                # ตั้งค่าความละเอียด
                self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                logger.info(f"เชื่อมต่อกล้อง {camera_id} สำเร็จ")
                return True
            return False
        except Exception as e:
            logger.error(f"ไม่สามารถเชื่อมต่อกล้อง: {e}")
            return False
    
    def move_servo(self, servo_name, angle):
        """ขับ servo ไปยังมุมที่กำหนด"""
        if self.arduino and servo_name in self.servos:
            # จำกัดมุมให้อยู่ในช่วง 0-180
            angle = max(0, min(180, angle))
            
            # สำหรับ servo 360° (base) ใช้วิธีการควบคุมพิเศษ
            if servo_name == 'base':
                # แปลงมุม 0-180 เป็นสัญญาณควบคุม servo 360°
                # 90 = หยุด, <90 = หมุนทวนเข็ม, >90 = หมุนตามเข็ม
                if angle != self.servo_positions[servo_name]:
                    if angle > self.servo_positions[servo_name]:
                        # หมุนตามเข็มนาฬิกา
                        self.servos[servo_name].write(120)
                        time.sleep(0.1)
                    else:
                        # หมุนทวนเข็มนาฬิกา
                        self.servos[servo_name].write(60)
                        time.sleep(0.1)
                    # หยุดการหมุน
                    self.servos[servo_name].write(90)
            else:
                self.servos[servo_name].write(angle)
            
            self.servo_positions[servo_name] = angle
            time.sleep(0.5)  # รอให้ servo เคลื่อนที่เสร็จ
    
    def move_to_initial_position(self):
        """เคลื่อนไปยังตำแหน่งเริ่มต้น"""
        self.move_servo('camera', 90)
        self.move_servo('base', 90)
        self.move_servo('lift', 90)
        self.move_servo('arm', 0)
        self.move_servo('gripper', 50)  # เปิดคีบ
        self.camera_angle = 90
    
    def scan_for_objects(self):
        """สแกนหาวัตถุด้วยการหมุนกล้อง"""
        if not self.cap:
            return None, None
            
        # อ่านภาพจากกล้อง
        ret, frame = self.cap.read()
        if not ret:
            return None, None
            
        self.current_frame = frame.copy()
        
        # ตรวจจับวัตถุด้วย YOLO (จำลอง)
        detected_objects = self.detect_objects(frame)
        
        # หาวัตถุเป้าหมาย
        target_name = self.work_sequence[self.current_target] if self.current_target < len(self.work_sequence) else 'Base'
        target_object = None
        
        for obj in detected_objects:
            if obj['class'] == target_name:
                target_object = obj
                break
        
        if target_object:
            # คำนวณมุมที่ต้องหมุนฐานแขนกล
            object_x = target_object['center_x']
            # เนื่องจากกล้องหมุนได้ เราต้องคำนวณตำแหน่งจริงของวัตถุ
            # ตำแหน่งจริง = มุมกล้อง + ตำแหน่งในภาพที่ปรับแล้ว
            
            # แปลงตำแหน่ง x ในภาพ (0-640) เป็นมุมออฟเซ็ต (-45 ถึง +45 องศา)
            frame_center = 320  # กลางภาพ
            offset_angle = (object_x - frame_center) / frame_center * 45  # ±45 องศา
            
            # คำนวณมุมจริงของวัตถุ
            real_angle = self.camera_angle + offset_angle
            
            # จำกัดมุมให้อยู่ในช่วง 0-180
            real_angle = max(0, min(180, real_angle))
            
            return target_object, real_angle
        
        # ถ้าไม่เจอวัตถุ ให้หมุนกล้องต่อ
        self.rotate_camera()
        return None, None
    
    def rotate_camera(self):
        """หมุนกล้องสแกนหาวัตถุ"""
        # หมุนกล้องไป 5 องศา
        step = 5
        self.camera_angle += step * self.camera_scan_direction
        
        # ถ้าถึงขอบ ให้เปลี่ยนทิศทาง
        if self.camera_angle >= 180:
            self.camera_angle = 180
            self.camera_scan_direction = -1
        elif self.camera_angle <= 0:
            self.camera_angle = 0
            self.camera_scan_direction = 1
        
        self.move_servo('camera', self.camera_angle)
    
    def detect_objects(self, frame):
        """ตรวจจับวัตถุในภาพ (จำลอง)"""
        # จำลองการตรวจจับวัตถุ
        # ในการใช้งานจริง ให้ใช้ YOLO model ที่เทรนแล้ว
        detected = []
        
        # สร้างข้อมูลจำลอง
        if np.random.random() > 0.7:  # 30% โอกาสเจอวัตถุ
            colors = ['Red', 'Green', 'Blue', 'Base']
            obj_class = np.random.choice(colors)
            
            center_x = np.random.randint(100, 540)
            center_y = np.random.randint(100, 380)
            
            detected.append({
                'class': obj_class,
                'center_x': center_x,
                'center_y': center_y,
                'confidence': 0.9
            })
        
        return detected
    
    def pick_object(self, target_angle):
        """จับวัตถุที่ตำแหน่งที่กำหนด"""
        self.current_step = f"กำลังจับ {self.work_sequence[self.current_target]}"
        
        # 1. หมุนฐานไปยังตำแหน่งเป้าหมาย
        self.move_servo('base', target_angle)
        
        # 2. เปิดปากคีบ 50°
        self.move_servo('gripper', 50)
        
        # 3. กดแขนลง 30°
        current_lift = self.servo_positions['lift']
        self.move_servo('lift', current_lift - 30)
        
        # 4. ปรับระดับแขน 30°
        self.move_servo('arm', 30)
        
        # 5. ปิดปากคีบ 0°
        self.move_servo('gripper', 0)
        time.sleep(1)  # รอให้คีบแน่น
        
        # 6. ยกแขนขึ้น 90°
        self.move_servo('lift', 90)
        
        # 7. ปรับระดับแขนเป็น 0°
        self.move_servo('arm', 0)
        
        self.current_step = f"จับ {self.work_sequence[self.current_target]} สำเร็จ"
    
    def place_in_base(self, base_angle):
        """วางวัตถุใน Base"""
        self.current_step = f"กำลังวาง {self.work_sequence[self.current_target]} ใน Base"
        
        # 1. หมุนฐานไปยัง Base
        self.move_servo('base', base_angle)
        
        # 2. ลดแขนลงเพื่อวาง
        self.move_servo('lift', 60)
        self.move_servo('arm', 30)
        
        # 3. เปิดคีบปล่อยวัตถุ
        self.move_servo('gripper', 50)
        time.sleep(1)
        
        # 4. ยกแขนขึ้น
        self.move_servo('lift', 90)
        self.move_servo('arm', 0)
        
        self.current_step = f"วาง {self.work_sequence[self.current_target]} ใน Base สำเร็จ"
        
        # เปลี่ยนไปหาวัตถุถัดไป
        self.current_target += 1
    
    def move_base_to_final_position(self):
        """ยก Base ไปวางในตำแหน่ง 90°"""
        self.current_step = "กำลังยก Base ไปตำแหน่งสุดท้าย"
        
        # หา Base ก่อน
        base_found = False
        attempts = 0
        
        while not base_found and attempts < 20:  # พยายาม 20 ครั้ง
            base_obj, base_angle = self.scan_for_objects()
            if base_obj and base_obj['class'] == 'Base':
                base_found = True
                
                # จับ Base
                self.move_servo('base', base_angle)
                self.move_servo('gripper', 50)
                self.move_servo('lift', 60)
                self.move_servo('arm', 30)
                self.move_servo('gripper', 0)
                time.sleep(1)
                self.move_servo('lift', 90)
                self.move_servo('arm', 0)
                
                # วางที่ตำแหน่ง 90°
                self.move_servo('base', 90)
                self.move_servo('lift', 60)
                self.move_servo('arm', 30)
                self.move_servo('gripper', 50)
                time.sleep(1)
                self.move_servo('lift', 90)
                self.move_servo('arm', 0)
                
                self.current_step = "วาง Base ในตำแหน่งสุดท้ายสำเร็จ"
                break
            
            attempts += 1
            time.sleep(0.5)
        
        if not base_found:
            self.current_step = "ไม่พบ Base"
    
    def main_loop(self):
        """ลูปหลักของระบบ"""
        try:
            while self.is_running:
                if self.current_target < len(self.work_sequence):
                    # ขั้นตอน 1-7: จับวัตถุตามลำดับ
                    target_obj, target_angle = self.scan_for_objects()
                    
                    if target_obj and target_obj['class'] == self.work_sequence[self.current_target]:
                        # เจอวัตถุเป้าหมาย
                        self.pick_object(target_angle)
                        
                        # หา Base เพื่อวางวัตถุ
                        base_found = False
                        attempts = 0
                        
                        while not base_found and attempts < 10:
                            base_obj, base_angle = self.scan_for_objects()
                            if base_obj and base_obj['class'] == 'Base':
                                self.place_in_base(base_angle)
                                base_found = True
                            attempts += 1
                            time.sleep(0.5)
                        
                        if not base_found:
                            self.current_step = "ไม่พบ Base สำหรับวางวัตถุ"
                            time.sleep(2)
                    
                elif self.current_target >= len(self.work_sequence):
                    # ขั้นตอน 8: ยก Base ไปตำแหน่งสุดท้าย
                    self.move_base_to_final_position()
                    
                    # รีเซ็ตเพื่อเริ่มรอบใหม่
                    self.current_target = 0
                    self.current_step = "เสร็จสิ้นรอบ - เริ่มรอบใหม่"
                    time.sleep(2)
                
                time.sleep(0.1)  # หน่วงเวลาเล็กน้อย
                
        except Exception as e:
            logger.error(f"เกิดข้อผิดพลาดในลูปหลัก: {e}")
            self.current_step = f"ข้อผิดพลาด: {e}"
        finally:
            self.is_running = False
    
    def start_system(self):
        """เริ่มระบบ"""
        if not self.arduino:
            return False, "ไม่ได้เชื่อมต่อ Arduino"
        
        if not self.cap:
            return False, "ไม่ได้เชื่อมต่อกล้อง"
        
        self.is_running = True
        self.current_target = 0
        self.current_step = "เริ่มระบบ"
        
        # เริ่ม thread สำหรับลูปหลัก
        self.main_thread = threading.Thread(target=self.main_loop)
        self.main_thread.daemon = True
        self.main_thread.start()
        
        return True, "เริ่มระบบสำเร็จ"
    
    def stop_system(self):
        """หยุดระบบ"""
        self.is_running = False
        self.current_step = "หยุดระบบ"
        
        # กลับไปตำแหน่งเริ่มต้น
        if self.arduino:
            self.move_to_initial_position()
    
    def get_camera_frame(self):
        """ดึงภาพล่าสุดจากกล้อง"""
        if self.cap:
            ret, frame = self.cap.read()
            if ret:
                self.current_frame = frame
                return frame
        return self.current_frame
    
    def cleanup(self):
        """ทำความสะอาดทรัพยากร"""
        self.stop_system()
        
        if self.cap:
            self.cap.release()
        
        if self.arduino:
            try:
                self.arduino.exit()
            except:
                pass

class RoboticArmGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("ระบบแขนกลจับวัตถุอัตโนมัติ")
        self.root.geometry("1200x800")
        
        # สร้าง controller
        self.controller = RoboticArmController()
        
        # สร้าง GUI
        self.setup_gui()
        
        # เริ่ม video loop
        self.video_running = True
        self.update_video()
        
        # เริ่ม status loop
        self.update_status()
        
        # จัดการเมื่อปิดหน้าต่าง
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
    
    def setup_gui(self):
        # Main frame
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # Configure grid weights
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
        main_frame.columnconfigure(1, weight=1)
        main_frame.rowconfigure(1, weight=1)
        
        # Control Panel
        control_frame = ttk.LabelFrame(main_frame, text="ควบคุมระบบ", padding="10")
        control_frame.grid(row=0, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=(0, 10))
        
        # Arduino connection
        ttk.Label(control_frame, text="พอร์ต Arduino:").grid(row=0, column=0, sticky=tk.W)
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(control_frame, textvariable=self.port_var, width=15)
        self.port_combo.grid(row=0, column=1, padx=(5, 10))
        
        ttk.Button(control_frame, text="ค้นหา", command=self.refresh_ports).grid(row=0, column=2, padx=(0, 10))
        ttk.Button(control_frame, text="เชื่อมต่อ", command=self.connect_arduino).grid(row=0, column=3, padx=(0, 20))
        
        # Camera connection
        ttk.Label(control_frame, text="กล้อง:").grid(row=0, column=4, sticky=tk.W)
        self.camera_var = tk.StringVar(value="0")
        ttk.Entry(control_frame, textvariable=self.camera_var, width=5).grid(row=0, column=5, padx=(5, 10))
        ttk.Button(control_frame, text="เชื่อมต่อกล้อง", command=self.connect_camera).grid(row=0, column=6)
        
        # Control buttons
        button_frame = ttk.Frame(control_frame)
        button_frame.grid(row=1, column=0, columnspan=7, pady=(10, 0))
        
        self.start_btn = ttk.Button(button_frame, text="เริ่ม", command=self.start_system, style="Green.TButton")
        self.start_btn.pack(side=tk.LEFT, padx=(0, 10))
        
        self.stop_btn = ttk.Button(button_frame, text="หยุด", command=self.stop_system, style="Red.TButton")
        self.stop_btn.pack(side=tk.LEFT, padx=(0, 10))
        
        self.reset_btn = ttk.Button(button_frame, text="รีเซ็ต", command=self.reset_system)
        self.reset_btn.pack(side=tk.LEFT)
        
        # Status Panel
        status_frame = ttk.LabelFrame(main_frame, text="สถานะระบบ", padding="10")
        status_frame.grid(row=1, column=0, sticky=(tk.W, tk.E, tk.N, tk.S), padx=(0, 10))
        status_frame.columnconfigure(0, weight=1)
        
        # Connection status
        self.arduino_status = ttk.Label(status_frame, text="Arduino: ไม่เชื่อมต่อ", foreground="red")
        self.arduino_status.grid(row=0, column=0, sticky=tk.W, pady=(0, 5))
        
        self.camera_status = ttk.Label(status_frame, text="กล้อง: ไม่เชื่อมต่อ", foreground="red")
        self.camera_status.grid(row=1, column=0, sticky=tk.W, pady=(0, 10))
        
        # Current operation
        ttk.Label(status_frame, text="ขั้นตอนปัจจุบัน:", font=("TkDefaultFont", 10, "bold")).grid(row=2, column=0, sticky=tk.W)
        self.status_label = ttk.Label(status_frame, text="รอการเริ่มต้น", font=("TkDefaultFont", 9))
        self.status_label.grid(row=3, column=0, sticky=tk.W, pady=(0, 10))
        
        # Target sequence
        ttk.Label(status_frame, text="ลำดับการทำงาน:", font=("TkDefaultFont", 10, "bold")).grid(row=4, column=0, sticky=tk.W)
        self.sequence_label = ttk.Label(status_frame, text="Red → Base → Green → Base → Blue → Base", font=("TkDefaultFont", 9))
        self.sequence_label.grid(row=5, column=0, sticky=tk.W, pady=(0, 10))
        
        # Current target
        ttk.Label(status_frame, text="เป้าหมายปัจจุบัน:", font=("TkDefaultFont", 10, "bold")).grid(row=6, column=0, sticky=tk.W)
        self.target_label = ttk.Label(status_frame, text="ไม่มี", font=("TkDefaultFont", 9))
        self.target_label.grid(row=7, column=0, sticky=tk.W, pady=(0, 10))
        
        # Servo positions
        ttk.Label(status_frame, text="ตำแหน่ง Servo:", font=("TkDefaultFont", 10, "bold")).grid(row=8, column=0, sticky=tk.W)
        self.servo_frame = ttk.Frame(status_frame)
        self.servo_frame.grid(row=9, column=0, sticky=(tk.W, tk.E), pady=(0, 10))
        
        self.servo_labels = {}
        servo_names = ['กล้อง', 'ฐาน', 'แขนยก', 'แขน', 'คีบ']
        servo_keys = ['camera', 'base', 'lift', 'arm', 'gripper']
        
        for i, (name, key) in enumerate(zip(servo_names, servo_keys)):
            ttk.Label(self.servo_frame, text=f"{name}:").grid(row=i, column=0, sticky=tk.W)
            self.servo_labels[key] = ttk.Label(self.servo_frame, text="90°")
            self.servo_labels[key].grid(row=i, column=1, sticky=tk.W, padx=(10, 0))
        
        # Video Panel
        video_frame = ttk.LabelFrame(main_frame, text="ภาพจากกل้อง", padding="10")
        video_frame.grid(row=1, column=1, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        self.video_label = ttk.Label(video_frame)
        self.video_label.pack()
        
        # Initialize
        self.refresh_ports()
        
        # Style configuration
        style = ttk.Style()
        style.configure("Green.TButton", foreground="green")
        style.configure("Red.TButton", foreground="red")
    
    def refresh_ports(self):
        """ค้นหาพอร์ต Arduino"""
        ports = self.controller.get_available_ports()
        self.port_combo['values'] = ports
        if ports:
            self.port_combo.set(ports[0])
    
    def connect_arduino(self):
        """เชื่อมต่อ Arduino"""
        port = self.port_var.get()
        if not port:
            messagebox.showerror("ข้อผิดพลาด", "กรุณาเลือกพอร์ต Arduino")
            return
        
        if self.controller.connect_arduino(port):
            self.arduino_status.config(text=f"Arduino: เชื่อมต่อ ({port})", foreground="green")
            messagebox.showinfo("สำเร็จ", "เชื่อมต่อ Arduino สำเร็จ")
        else:
            messagebox.showerror("ข้อผิดพลาด", "ไม่สามารถเชื่อมต่อ Arduino ได้")
    
    def connect_camera(self):
        """เชื่อมต่อกล้อง"""
        try:
            camera_id = int(self.camera_var.get())
            if self.controller.connect_camera(camera_id):
                self.camera_status.config(text=f"กล้อง: เชื่อมต่อ (ID: {camera_id})", foreground="green")
                messagebox.showinfo("สำเร็จ", "เชื่อมต่อกล้องสำเร็จ")
            else:
                messagebox.showerror("ข้อผิดพลาด", "ไม่สามารถเชื่อมต่อกล้องได้")
        except ValueError:
            messagebox.showerror("ข้อผิดพลาด", "ID กล้องต้องเป็นตัวเลข")
    
    def start_system(self):
        """เริ่มระบบ"""
        success, message = self.controller.start_system()
        if success:
            self.start_btn.config(state="disabled")
            self.stop_btn.config(state="normal")
            messagebox.showinfo("สำเร็จ", message)
        else:
            messagebox.showerror("ข้อผิดพลาด", message)
    
    def stop_system(self):
        """หยุดระบบ"""
        self.controller.stop_system()
        self.start_btn.config(state="normal")
        self.stop_btn.config(state="disabled")
        messagebox.showinfo("สำเร็จ", "หยุดระบบแล้ว")
    
    def reset_system(self):
        """รีเซ็ตระบบ"""
        if self.controller.is_running:
            self.stop_system()
        
        if self.controller.arduino:
            self.controller.move_to_initial_position()
            messagebox.showinfo("สำเร็จ", "รีเซ็ตระบบแล้ว")
        else:
            messagebox.showwarning("คำเตือน", "ไม่ได้เชื่อมต่อ Arduino")
    
    def update_video(self):
        """อัปเดตวิดีโอ"""
        if self.video_running:
            frame = self.controller.get_camera_frame()
            if frame is not None:
                # ปรับขนาดภาพ
                frame = cv2.resize(frame, (480, 360))
                
                # วาดข้อมูลบนภาพ
                self.draw_overlay(frame)
                
                # แปลงเป็น RGB
                frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                
                # แปลงเป็น PIL Image
                image = Image.fromarray(frame_rgb)
                photo = ImageTk.PhotoImage(image)
                
                # อัปเดต label
                self.video_label.configure(image=photo)
                self.video_label.image = photo
            
            # เรียกซ้ำ
            self.root.after(33, self.update_video)  # ~30 FPS
    
    def draw_overlay(self, frame):
        """วาดข้อมูลเพิ่มเติมบนภาพ"""
        height, width = frame.shape[:2]
        
        # วาดเส้นกลางภาพ
        cv2.line(frame, (width//2, 0), (width//2, height), (0, 255, 0), 1)
        cv2.line(frame, (0, height//2), (width, height//2), (0, 255, 0), 1)
        
        # แสดงมุมกล้องปัจจุบัน
        camera_text = f"Camera Angle: {self.controller.camera_angle:.1f}°"
        cv2.putText(frame, camera_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        # แสดงเป้าหมายปัจจุบัน
        if self.controller.current_target < len(self.controller.work_sequence):
            target = self.controller.work_sequence[self.controller.current_target]
            target_text = f"Target: {target}"
            cv2.putText(frame, target_text, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        
        # จำลองการแสดงผลการตรวจจับ (ในการใช้งานจริงจะใช้ YOLO)
        if np.random.random() > 0.8:  # 20% โอกาสแสดงกรอบ
            x, y, w, h = 100, 100, 80, 60
            cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 0, 255), 2)
            cv2.putText(frame, "Object", (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
    
    def update_status(self):
        """อัปเดตสถานะ"""
        # อัปเดตขั้นตอนปัจจุบัน
        self.status_label.config(text=self.controller.current_step)
        
        # อัปเดตเป้าหมายปัจจุบัน
        if self.controller.current_target < len(self.controller.work_sequence):
            target = self.controller.work_sequence[self.controller.current_target]
            self.target_label.config(text=target)
        else:
            self.target_label.config(text="Base (Final)")
        
        # อัปเดตตำแหน่ง Servo
        for key, label in self.servo_labels.items():
            angle = self.controller.servo_positions[key]
            label.config(text=f"{angle:.0f}°")
        
        # เรียกซ้ำ
        self.root.after(500, self.update_status)  # อัปเดตทุก 0.5 วินาที
    
    def on_closing(self):
        """จัดการเมื่อปิดหน้าต่าง"""
        self.video_running = False
        self.controller.cleanup()
        self.root.destroy()

# Arduino Code สำหรับอัปโหลดไปยัง Arduino
ARDUINO_CODE = '''
// Arduino Code สำหรับระบบแขนกลจับวัตถุ
// อัปโหลดโค้ดนี้ไปยัง Arduino ก่อนใช้งาน Python

#include <Servo.h>

// สร้างออบเจ็กต์ servo
Servo cameraServo;    // Pin 3
Servo baseServo;      // Pin 5 (360°)
Servo liftServo;      // Pin 6
Servo armServo;       // Pin 9
Servo gripperServo;   // Pin 10

// ตัวแปรเก็บตำแหน่งปัจจุบัน
int cameraPos = 90;
int basePos = 90;
int liftPos = 90;
int armPos = 0;
int gripperPos = 50;

void setup() {
  // เริ่มต้น Serial communication
  Serial.begin(57600);
  
  // กำหนดขา servo
  cameraServo.attach(3);
  baseServo.attach(5);
  liftServo.attach(6);
  armServo.attach(9);
  gripperServo.attach(10);
  
  // ตั้งค่าตำแหน่งเริ่มต้น
  moveToInitialPosition();
  
  Serial.println("Arduino Ready");
}

void loop() {
  // Firmata จะจัดการ loop นี้
  // ไม่ต้องใส่โค้ดเพิ่มเติม
}

void moveToInitialPosition() {
  cameraServo.write(90);
  baseServo.write(90);  // สำหรับ servo 360° นี่คือตำแหน่งหยุด
  liftServo.write(90);
  armServo.write(0);
  gripperServo.write(50);
  
  delay(1000);  // รอให้ servo เคลื่อนที่เสร็จ
}
'''

def save_arduino_code():
    """บันทึกโค้ด Arduino"""
    try:
        with open("robotic_arm_arduino.ino", "w", encoding="utf-8") as f:
            f.write(ARDUINO_CODE)
        print("บันทึกไฟล์ robotic_arm_arduino.ino สำเร็จ")
        print("กรุณาอัปโหลดไฟล์นี้ไปยัง Arduino ก่อนใช้งาน")
    except Exception as e:
        print(f"ไม่สามารถบันทึกไฟล์ได้: {e}")

def main():
    # บันทึกโค้ด Arduino
    save_arduino_code()
    
    # สร้าง GUI
    root = tk.Tk()
    app = RoboticArmGUI(root)
    
    try:
        root.mainloop()
    except KeyboardInterrupt:
        print("ปิดโปรแกรม")
        app.controller.cleanup()

if __name__ == "__main__":
    # แสดงข้อมูลการใช้งาน
    print("=" * 60)
    print("ระบบแขนกลจับวัตถุอัตโนมัติ")
    print("=" * 60)
    print("การเตรียมใช้งาน:")
    print("1. ติดตั้ง libraries: pip install opencv-python pillow pyfirmata ultralytics numpy")
    print("2. อัปโหลดไฟล์ robotic_arm_arduino.ino ไปยัง Arduino")
    print("3. เชื่อมต่อ servo ตามขา:")
    print("   - Pin 3: Servo หมุนกล้อง")
    print("   - Pin 5: Servo หมุนฐาน (360°)")
    print("   - Pin 6: Servo ยกแขน")
    print("   - Pin 9: Servo ปรับระดับแขน")
    print("   - Pin 10: Servo คีบ")
    print("4. เชื่อมต่อ webcam")
    print("5. เทรน YOLO model สำหรับตรวจจับ Red, Green, Blue, Base")
    print("=" * 60)
    print()
    
    main()