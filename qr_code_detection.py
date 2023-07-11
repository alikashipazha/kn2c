import cv2
from pyzbar.pyzbar import decode
import time
import numpy as np

# Initialize the webcam
cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    flipped = cv2.flip(frame, 1)
#   cv2.imshow('webcam', frame)
#    time.sleep(1)
    decoded_objects = decode(frame)

    # Draw a rectangle around the QR code
    for obj in decoded_objects:
        cv2.rectangle(frame, obj.rect, (0, 255, 0), 2)

        # Print the decoded data
        data = obj.data.decode('utf-8')
        cv2.putText(frame, data, (obj.rect.left, obj.rect.top), cv2.FONT_HERSHEY_SIMPLEX, 0.9,
                    (0, 255, 0), 2)

    # Show the frame
    cv2.imshow('frame', frame)

    if cv2.waitKey(1) == 13:
        break

cap.release()
