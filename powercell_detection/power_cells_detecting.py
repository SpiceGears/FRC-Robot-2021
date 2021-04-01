import cv2
import time
import numpy as np
import datetime
import logging
import json
import sys

from networktables import NetworkTables

def nothing(something):
    pass

net = cv2.dnn.readNet("D:/Programowanie/Python/power_cells/power_cells_model/model.weights",
 "D:/Programowanie/Python/power_cells/power_cells_model/model.cfg")

classes = ['power_cell']

# classes_file = "power_cells_model/names.txt"
# with open(classes_file, 'r') as f:
#    classes = [line.strip() for line in f.readlines()]

layer_names= net.getLayerNames()
outputlayers = [layer_names[i[0] - 1] for i in net.getUnconnectedOutLayers()]

logging.basicConfig(level=logging.DEBUG)

ip = "10.58.83.2"
NetworkTables.initialize(server=ip)
network_tables = NetworkTables.getTable("DetectedObjects")

img_width = 800
img_height = 600

cv2.namedWindow("Image")
switch = '0 : OFF\n1 : ON'
cv2.createTrackbar('y_pose', 'Image', 0, img_height, nothing)
cv2.setTrackbarPos('y_pose', 'Image', 340)

cv2.createTrackbar('x_right_pose', 'Image', 100, img_width, nothing)
cv2.setTrackbarPos('x_right_pose', 'Image', 150)

cv2.createTrackbar('x_left_pose', 'Image', 0, img_width, nothing)
cv2.setTrackbarPos('x_left_pose', 'Image', img_width-150)

cam = 1
if cam == 0:
    cap = cv2.VideoCapture(0)
else:
    cap = cv2.VideoCapture("http://10.58.83.2:1181/stream.mjpg")
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 0)
    cap.set(cv2.CAP_PROP_FPS, 2)

font = cv2.FONT_HERSHEY_PLAIN

frame_id = 0


while True:
    try:
        starting_time = time.time()

        for x in range(4):
            cap.grab()

        _, frame = cap.read()
        # frame = cv2.flip(frame, -1)
        time_now = datetime.datetime.now()

        # cv2.imwrite('D:\\frames_warsztat\\frame{0}_{1}.jpg'.format(frame_id, time_now.strftime("%d_%H_%M_%S")) , frame)
        print(frame_id)

        height, width, channels = frame.shape
        blob = cv2.dnn.blobFromImage(frame, 0.00392, (250, 250), (0, 0, 0), True, crop=False)

        net.setInput(blob)
        outs = net.forward(outputlayers)

        detected_objects = []
        class_ids = []
        confidences = []
        boxes = []
        for out in outs:
            for detection in out:
                scores = detection[5:]
                class_id = np.argmax(scores)
                confidence = scores[class_id]
                if confidence > 0.30:
                    center_x = int(detection[0] * width)
                    center_y = int(detection[1] * height)
                    w = int(detection[2] * width)
                    h = int(detection[3] * height)
                    x = int(center_x - w/2)
                    y = int(center_y - h/2)

                    detected_object = {
                        "confidence":float(confidence),
                        "center_x": float(center_x),
                        "center_y": float(center_y),
                        "width": float(w),
                        "height": float(h),
                        "x": float(x),
                        "y": float(y)
                    }

                    detected_objects.append(detected_object)

                    boxes.append([x, y, w, h])
                    confidences.append(float(confidence))
                    class_ids.append(class_id)

        indexes = cv2.dnn.NMSBoxes(boxes, confidences, 0.4, 0,6)
        network_tables.putString("data", json.dumps(detected_objects))

        # Draw rectangles
        for i in range(len(boxes)):
            if i in indexes:
                x, y, w, h = boxes[i]
                label = str(classes[class_ids[i]])
                confidence = confidences[i]
                color = (0, 0, 255)
                cv2.rectangle(frame, (x, y), (x+w, y+h), color, 2)
                cv2.putText(frame, label + " " + str(round(confidence, 2)), (x, y + 30), font, 1, (0, 0, 0), 2)
                # cv2.putText(frame,"x: " + str(x), (x, y + 45), font, 1, (0, 0, 0), 2)
                # cv2.putText(frame,"y: " + str(y), (x, y + 60), font, 1, (0, 0, 0), 2)
                print(label + " " + str(round(confidence, 2)))

        # Calculate FPS 
        elapsed_time = time.time() - starting_time
        fps = 1 / elapsed_time
        print("Time per frame: " + str(elapsed_time) + " FPS: " + str(fps))
        cv2.putText(frame, "FPS: " + str(round(fps, 2)), (10, 20), font, 1, (0, 0, 0), 2)

        # Check if ball under the y line 
        line_y = cv2.getTrackbarPos('y_pose', 'Image')
        cv2.line(frame, (0, line_y), (width, line_y), (0, 0, 255), 1)
        is_ball_under_line = False
        for power_cell in detected_objects:
            if power_cell["center_y"] >= line_y:
                print("na dole")
                is_ball_under_line = True

        # check right
        line_x_right = cv2.getTrackbarPos('x_right_pose', 'Image')
        cv2.line(frame, (line_x_right, 0), (line_x_right, height), (255, 255, 255), 1)
        is_ball_right = False
        for power_cell in detected_objects:
            if power_cell["center_x"] <= line_x_right and power_cell["center_y"] < line_y:
                print("na prawo")
                is_ball_right = True

        # check left
        line_x_left = cv2.getTrackbarPos('x_left_pose', 'Image')
        cv2.line(frame, (line_x_left, 0), (line_x_left, height), (255, 255, 255), 1)
        is_ball_left = False
        for power_cell in detected_objects:
            if power_cell["center_x"] >= line_x_left and power_cell["center_y"] < line_y:
                print("na lewo")
                is_ball_left = True

        network_tables.putBoolean("is_ball_left", is_ball_left)
        network_tables.putBoolean("is_ball_right", is_ball_right)
        network_tables.putBoolean("is_ball_under_line", is_ball_under_line)
        
        cv2.imshow("Image", frame)

        frame_id = frame_id + 1        
    except:
        print("no robot connection")
        time.sleep(2)
        
    key = cv2.waitKey(1)
    if key == 27:
            break
cap.release()
cv2.destroyAllWindows()