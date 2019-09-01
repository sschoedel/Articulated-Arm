import cv2 as cv
import numpy as np

cv.namedWindow("Video")
vc = cv.VideoCapture(0)

if vc.isOpened():
    rval, image = vc.read()
else:
    rval = False

while rval:
    rval, image = vc.read()

    flip(image, image, +1)

    cv.imshow("Video", image)

    key = cv.waitKey(10)
    if key == 27:
        break
cv.destroyWindow("Video")
