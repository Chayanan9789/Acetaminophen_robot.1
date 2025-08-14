# custom_model_detection.py
# ใช้โมเดล custom (best.pt) พร้อมกำหนดความมั่นใจขั้นต่ำที่จะแสดงผล

import cv2
from ultralytics import YOLO

# โหลดโมเดลที่ฝึกเอง (เช่นจาก Roboflow, PyTorch ฯลฯ)
model = YOLO("best1.pt")

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

# ✅ ค่าความมั่นใจ (confidence) ต่ำสุดที่ยอมให้แสดงผล (0.0–1.0)
CONFIDENCE_THRESHOLD = 0.5

while True:
    ret, frame = cap.read()
    if not ret:
        print("❌ ไม่สามารถอ่านภาพจากกล้องได้")
        break

    results = model(frame)[0]

    for box in results.boxes:
        conf = float(box.conf)  # ค่าความมั่นใจ (confidence score)

        # กรองเฉพาะกล่องที่ความมั่นใจสูงพอ
        if conf < CONFIDENCE_THRESHOLD:
            continue

        x1, y1, x2, y2 = map(int, box.xyxy[0])
        cls_id = int(box.cls[0])
        label = model.names[cls_id]
        color = (0, 255, 0)  # เขียว

        # วาดกรอบกล่อง
        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)

        # สร้างข้อความรวมชื่อ + confidence เช่น "cat 0.87"
        text = f"{label} {conf:.2f}"

        # วาด label บนภาพ
        cv2.putText(frame, text, (x1, y1 - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

    cv2.imshow("Custom YOLOv8 Detection", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
