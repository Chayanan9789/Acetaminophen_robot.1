import zmq
import time
from pyfirmata2 import Arduino

# === เชื่อมต่อกับ Arduino โดยระบุ COM Port ===
# เปลี่ยน COM3 เป็นพอร์ตที่ Arduino เชื่อมต่อ (Windows)
# หรือ /dev/ttyUSB0 (Linux/macOS)
board = Arduino('COM19')  # สำหรับ Windows
# board = Arduino('/dev/ttyUSB0')  # สำหรับ Linux/macOS

# === ตั้งค่า Servo Pin ===
servo_base = board.get_pin('d:3:s')     # MG90s 360° (ฐานหมุน)
servo_lift = board.get_pin('d:11:s')     # ยกแขน
servo_elbow = board.get_pin('d:12:s')    # ระดับความสูงแขน
servo_gripper = board.get_pin('d:13:s')  # คีบ
servo_cam = board.get_pin('d:9:s')     # กล้อง 180°

# === คลาสบริหารการหมุนฐาน 360° ===
class ServoNavigator360:
    def __init__(self, servo, speed_dps=180):
        self.servo = servo
        self.speed_dps = speed_dps
        self.current_angle = 90  # สมมุติเริ่มตรงกลาง

    def rotate_to(self, target_angle):
        angle_diff = (target_angle - self.current_angle + 360) % 360
        if angle_diff < 3 or angle_diff > 357:
            self.servo.write(90)  # หยุด
            return

        if angle_diff <= 180:
            pwm = 120
            duration = angle_diff / self.speed_dps
            self.servo.write(pwm)
        else:
            pwm = 60
            duration = (360 - angle_diff) / self.speed_dps
            self.servo.write(pwm)

        time.sleep(duration)
        self.servo.write(90)
        self.current_angle = target_angle % 360

# === คลาสคำนวณมุมของวัตถุในโลกจริงจากภาพกล้อง ===
class CameraAngleHelper:
    def __init__(self, cam_servo_angle=90, cam_fov_deg=60, frame_width=640):
        self.cam_servo_angle = cam_servo_angle
        self.cam_fov_deg = cam_fov_deg
        self.frame_width = frame_width

    def update_camera_angle(self, angle):
        self.cam_servo_angle = angle

    def get_world_angle(self, object_x):
        center_x = self.frame_width / 2
        ratio = (object_x - center_x) / center_x
        offset = ratio * (self.cam_fov_deg / 2)
        return (self.cam_servo_angle + offset) % 360

# === Setup ZMQ เพื่อรับข้อมูลจาก Node 1 ===
context = zmq.Context()
sub = context.socket(zmq.SUB)
sub.connect("tcp://localhost:5555")
sub.setsockopt_string(zmq.SUBSCRIBE, "")

# === ตัวช่วยควบคุมฐานและกล้อง ===
base_ctrl = ServoNavigator360(servo_base)
cam_calc = CameraAngleHelper()

# === ฟังก์ชันควบคุมแขนกล ===
def pick_and_place():
    print("  [ACTION] Lowering...")
    servo_lift.write(120)
    servo_elbow.write(100)
    time.sleep(1)

    print("  [ACTION] Gripping...")
    servo_gripper.write(30)
    time.sleep(1)

    print("  [ACTION] Lifting...")
    servo_lift.write(60)
    time.sleep(1)

    print("  [ACTION] Locating base...")
    servo_cam.write(0)
    cam_calc.update_camera_angle(0)
    time.sleep(1)

    base_ctrl.rotate_to(90)  # วางโดนัทที่ตำแหน่ง 90°
    time.sleep(1)

    print("  [ACTION] Releasing...")
    servo_gripper.write(90)
    time.sleep(1)

    servo_cam.write(90)
    cam_calc.update_camera_angle(90)

# === คำสั่งควบคุมหลังจากรับข้อมูลจาก Node 1 ===
color_order = ["Red", "Green", "Blue"]
color_seen = []

print("[READY] Waiting for object positions from Node 1...")

while True:
    data = sub.recv_json()
    label = data["label"]
    object_x = data["x"]

    if label not in color_order or label in color_seen:
        continue

    print(f"\n[RECEIVED] {label} at x={object_x}")

    world_angle = cam_calc.get_world_angle(object_x)
    print(f"  [CALC] Target world angle = {world_angle:.2f}°")

    base_ctrl.rotate_to(world_angle)
    pick_and_place()

    color_seen.append(label)
    print(f"✅ Completed handling {label}")

    if len(color_seen) == len(color_order):
        print("\n[FINISH] All donuts processed!")
        break
