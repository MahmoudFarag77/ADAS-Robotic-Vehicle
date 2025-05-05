# =============================
# Include required packages
# =============================
import cv2
from picamera2 import Picamera2
import pandas as pd
from ultralytics import YOLO
import cvzone
import numpy as np
import smbus2
import time

# =============================
# I2C settings
# =============================
I2C_ADDRESS = 0x08  # ESP32 I2C address
bus = smbus2.SMBus(1)  # Use I2C bus 1

# =============================
# Camera settings
# =============================
picam2 = Picamera2()
picam2.preview_configuration.main.size = (640, 480)
picam2.preview_configuration.main.format = "RGB888"
picam2.preview_configuration.align()
picam2.configure("preview")
picam2.start()

# =============================
# Load YOLO model
# =============================
model = YOLO('best.pt')

# =============================
# Load class labels
# =============================
my_file = open("coco1.txt", "r")
data = my_file.read()
class_list = data.split("\n")

# =============================
# I2C signal sending function
# =============================
def send_i2c_signal(signal):
    try:
        bus.write_byte(I2C_ADDRESS, signal)
        print(f"Sent signal: {signal}")
    except Exception as e:
        print(f"Failed to send I2C signal: {e}")

# =============================
# Main Loop
# =============================
count = 0
while True:
    im = picam2.capture_array()

    # Skip Frames for better performance
    count += 1
    if count % 3 != 0:
        continue

    im = cv2.flip(im, -1)
    results = model.predict(im)
    a = results[0].boxes.data
    px = pd.DataFrame(a).astype("float")

    for index, row in px.iterrows():
        x1 = int(row[0])
        y1 = int(row[1])
        x2 = int(row[2])
        y2 = int(row[3])
        d = int(row[5])
        c = class_list[d]

        cv2.rectangle(im, (x1, y1), (x2, y2), (0, 0, 255), 2)
        cvzone.putTextRect(im, f'{c}', (x1, y1), 1, 1)

        # Send signals to ESP32 according to detected object
        if c == 'Speed_60':
            send_i2c_signal(1)
        elif c == 'Speed_120':
            send_i2c_signal(2)
        elif c == 'STOP':
            send_i2c_signal(3)
        else:
            send_i2c_signal(0)

    cv2.imshow("Camera", im)
    if cv2.waitKey(1) == ord('q'):
        break

cv2.destroyAllWindows()