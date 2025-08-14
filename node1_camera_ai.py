import cv2
import zmq
import time
from ultralytics import YOLO
from pyfirmata2 import Arduino
import threading

# === CONFIG ===
CAM_PORT = 0
YOLO_MODEL_PATH = "best1.pt"
SERIAL_PORT = Arduino.AUTODETECT
CAM_SERVO_PIN = 'd:9:s'

# === Setup YOLOv8 model ===
model = YOLO(YOLO_MODEL_PATH)

# === Setup Camera ===
cap = cv2.VideoCapture(CAM_PORT)
frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))

# === Setup Arduino for controlling camera servo ===
board = Arduino(SERIAL_PORT)
servo_cam = board.get_pin(CAM_SERVO_PIN)
servo_cam.write(90)
cam_angle = 90

# === Setup ZMQ PUB socket ===
context = zmq.Context()
pub = context.socket(zmq.PUB)
pub.bind("tcp://*:5555")

# === Track processing state ===
color_sequence = ["Red", "Green", "Blue"]
processed_colors = []

# === Sweep camera (0-180°) continuously ===
def camera_sweep_loop():
    global cam_angle
    while len(processed_colors) < len(color_sequence):
        for angle in range(60, 121, 5):
            servo_cam.write(angle)
            cam_angle = angle
            time.sleep(0.05)
        for angle in range(120, 59, -5):
            servo_cam.write(angle)
            cam_angle = angle
            time.sleep(0.05)

# === Start sweep in background thread ===
threading.Thread(target=camera_sweep_loop, daemon=True).start()

# === Main detection loop ===
print("[NODE1] Camera detection started.")

while True:
    ret, frame = cap.read()
    if not ret:
        continue

    results = model.predict(source=frame, verbose=False)
    for r in results:
        for box in r.boxes.data.tolist():
            x1, y1, x2, y2, conf, cls = box
            label = r.names[int(cls)]
            if label in color_sequence and label not in processed_colors:
                cx = int((x1 + x2) / 2)
                print(f"[DETECTED] {label} at x={cx}, cam_angle={cam_angle}")
                pub.send_json({"label": label, "x": cx})
                processed_colors.append(label)
                time.sleep(3)

    cv2.imshow("Camera View", frame)
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break
