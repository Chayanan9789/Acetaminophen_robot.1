import cv2
import zmq
from ultralytics import YOLO
import numpy as np
import time

# ==== CONFIG ====
MODEL_PATH = 'best1.pt'   # path ไปยังไฟล์ model เทรนด้วย roboflow
CAMERA_PORT = 0
FRAME_WIDTH, FRAME_HEIGHT = 640, 480
CAMERA_FOV = 62
SERVO_CENTER = 90        # home pos (degree)
PUBLISH_IP = "tcp://*:5555"
DETECTION_CLASSES = {0:"Red", 1:"Green", 2:"Blue", 3:"Base"}

# === ZeroMQ pub ===
ctx = zmq.Context()
pub = ctx.socket(zmq.PUB)
pub.bind(PUBLISH_IP)

# === Model ===
model = YOLO(MODEL_PATH)
cap = cv2.VideoCapture(CAMERA_PORT)
cap.set(3, FRAME_WIDTH)
cap.set(4, FRAME_HEIGHT)

def calc_servo_angle(x):
    offset = x - (FRAME_WIDTH/2)
    angle_offset = (offset/(FRAME_WIDTH/2)) * (CAMERA_FOV/2)
    angle = int(SERVO_CENTER + angle_offset)
    angle = max(0, min(angle, 180))
    return angle

print("CAMERA NODE started.")
while True:
    ret, frame = cap.read()
    if not ret:
        print('Camera error')
        continue
    # -- detection --
    results = model.predict(frame, verbose=False)
    send_obj = []
    for r in results:
        for box in r.boxes:
            cls = int(box.cls)
            b = box.xyxy[0].cpu().numpy()
            x = int((b[0]+b[2])//2)
            y = int((b[1]+b[3])//2)
            angle = calc_servo_angle(x)
            send_obj.append({"class":cls, "x":x, "y":y, "angle":angle})
            # annotation
            cv2.circle(frame, (x, y), 6, (0, 255, 0), 2)
            cv2.putText(frame, f"{DETECTION_CLASSES[cls]}:{angle}deg", (int(b[0]), int(b[1]-8)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,255), 2)
    # -- pub --
    if send_obj:
        pub.send_pyobj(send_obj)
    # -- show realtime --
    cv2.imshow("Webcam", frame)
    if cv2.waitKey(1) == 27: break
    time.sleep(0.1)
cap.release()
cv2.destroyAllWindows()
