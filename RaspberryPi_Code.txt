import cv2
from ultralytics import YOLO
import serial
import numpy as np
import time

try:
    ser = serial.Serial('/dev/ttyAMA0', 115200, timeout=1)
    print("UART Portu basariyla acildi: /dev/ttyAMA0")
except serial.SerialException as e:

    print(f"Hata: UART portu acilamadi. Hata: {e}")
    ser = None

camera = cv2.VideoCapture(0)
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
camera.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
camera.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

CENTER_X = FRAME_WIDTH // 2
CENTER_Y = FRAME_HEIGHT // 2

MODEL_PATH = "/home/emre/myenv/yolov8n.pt" 
model = YOLO(MODEL_PATH)
model.conf = 0.45 
TARGET_CLASS_ID = 39 

ALPHA = 0.05          
filtered_dx = 0.0     
filtered_dy = 0.0  

while True:
    success, frame = camera.read()
    if not success:
        print("Kamera okuma hatasi")
        time.sleep(0.1)
        continue

    results = model.predict(frame, classes=[TARGET_CLASS_ID], conf=model.conf, verbose=False)
    
    raw_dx, raw_dy = None, None 

    best_target_found = False

    if results and len(results) > 0:
        result = results[0] 
        boxes = result.boxes
        
        if boxes.xyxy.numel() > 0:
            
            best_score_index = boxes.conf.argmax().item()
            
            box = boxes.xyxy.cpu().numpy()[best_score_index]
            
            x1, y1, x2, y2 = map(int, box)
            cx = (x1 + x2) // 2
            cy = (y1 + y2) // 2
            
            raw_dx = cx - CENTER_X
            raw_dy = CENTER_Y - cy
            
            best_target_found = True
            
   
    if raw_dx is not None:
        filtered_dx = (ALPHA * raw_dx) + ((1 - ALPHA) * filtered_dx)
        filtered_dy = (ALPHA * raw_dy) + ((1 - ALPHA) * filtered_dy)
    else:
        pass 
        
    final_dx = int(filtered_dx)
    final_dy = int(filtered_dy)
   
    if ser is not None:
        data_str = f"DX:{final_dx},DY:{final_dy}\n"
        try:
            ser.write(data_str.encode('utf-8'))
            print(f"SENT: DX:{final_dx}, DY:{final_dy} | Last Raw DX/DY: {raw_dx}/{raw_dy}")
        except Exception as e:
            print(f"UART Gonderme Hatasi: {e}")
    else:
        print(f"Simüle SENT: DX:{final_dx}, DY:{final_dy} | Last Raw DX/DY: {raw_dx}/{raw_dy}")


