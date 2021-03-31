import cv2
import time
import numpy as np
import datetime


cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)

frame_id = 0

while True:
    _, frame = cap.read()
    # frame = cv2.flip(frame, -1)
    frame_id = frame_id + 1

    time_now = datetime.datetime.now()
    cv2.imwrite('D:\\frames_warsztat\\filming\\frame{0}_{1}.jpg'.format(frame_id, time_now.strftime("%d_%H_%M_%S")) , frame)
    print(frame_id)

    cv2.imshow("Image", frame)
    key = cv2.waitKey(1)

    time.sleep(0.3)
    if key == 27:
        break
cap.release()
cv2.destroyAllWindows()
