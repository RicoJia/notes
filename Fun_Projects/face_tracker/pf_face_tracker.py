import numpy as np
import cv2
from face_tracker import face_tracker_pf

# setup initial location of window by using two corner points
corner_points = []
SET_ROI = False
PINK = (179, 102, 255)

def draw_point(frame, coords): 
    cv2.circle(frame, coords, radius=2, color=PINK, thickness=-1)

def draw_box(frame, corners):
    cv2.rectangle(frame, corners[0], corners[1], PINK, 1)

def mouse_drawing(event, x_temp, y_temp, flags, params):
    if event == cv2.EVENT_LBUTTONDOWN:
        global corner_points
        if not SET_ROI: 
            corner_points.append((x_temp, y_temp))

cv2.namedWindow("face_tracker")
cv2.setMouseCallback("face_tracker", mouse_drawing)
cap = cv2.VideoCapture(2)

while True:
    rval, frame = cap.read()
    if len(corner_points) < 2:  #for initialization
        for pt in corner_points: 
            draw_point(frame, pt)
    else: 
        #do_stuff
        draw_box(frame, corner_points)
        SET_ROI = True

    cv2.imshow("face_tracker", frame)
    key = cv2.waitKey(20)
    if key == 27: # exit on ESC
        break

