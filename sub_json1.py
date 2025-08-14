# sub_json.py
# Subscriber ที่รับข้อความ JSON จากหัวข้อ jsondata แล้วแปลงกลับมาเป็น dict

import zmq
import json

context = zmq.Context()
subscriber = context.socket(zmq.SUB)
subscriber.connect("tcp://127.0.0.1:5560")
subscriber.setsockopt_string(zmq.SUBSCRIBE, "jsondata")  # ติดตามเฉพาะหัวข้อ jsondata

while True:
    message = subscriber.recv_string()
    
    # แยก topic และ payload json
    topic, json_payload = message.split(' ', 1)

    # แปลงกลับเป็น dictionary
    try:
        data = json.loads(json_payload)
        print("📥 ได้รับ JSON:", data)
        print(f"   - อุณหภูมิ: {data['temperature']} °C")
        print(f"   - ความชื้น: {data['humidity']} %")
        print(f"   - สถานะ: {data['status']}")
    except json.JSONDecodeError as e:
        print("❌ ผิดพลาดในการแปลง JSON:", e)
