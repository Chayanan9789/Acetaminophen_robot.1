# pub_json.py
# Publisher ที่ส่งข้อความ JSON ออกไปพร้อมกับ topic

import zmq
import time
import json

context = zmq.Context()
publisher = context.socket(zmq.PUB)
publisher.bind("tcp://*:5560")  # ใช้พอร์ตใหม่เพื่อแยกจากโปรแกรมอื่น

time.sleep(1)  # ให้เวลาสำหรับ subscriber เชื่อมต่อ

while True:
    topic = "jsondata"
    data = {
        "temperature": 25.3,
        "humidity": 60,
        "status": "normal",
        "timestamp": time.time()
    }

    json_message = json.dumps(data)  # แปลง dict เป็น string json
    full_message = f"{topic} {json_message}"

    print("📤 กำลังส่ง JSON:", full_message)
    publisher.send_string(full_message)
    time.sleep(2)
