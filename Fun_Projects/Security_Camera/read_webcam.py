import cv2
# https://stackoverflow.com/questions/604749/how-do-i-access-my-webcam-in-python
# check a webcam vlc v4l2:///dev/video2

cv2.namedWindow("preview")
vc = cv2.VideoCapture(2)

if vc.isOpened(): # try to get the first frame
    rval, frame = vc.read()
else:
    rval = False

while rval:
    cv2.imshow("preview", frame)
    rval, frame = vc.read()
    key = cv2.waitKey(20)
    if key == 27: # exit on ESC
        break
cv2.destroyWindow("preview")

