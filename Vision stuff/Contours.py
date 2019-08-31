import cv2 as cv
import numpy as np

cv.namedWindow("Video")
cv.namedWindow("threshold")

vc = cv.VideoCapture(0)

if vc.isOpened():
    rval, image = vc.read()
else:
    rval = False

while rval:
    rval, image = vc.read()
    gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
    ret, thresh = cv.threshold(gray, 120, 255, 0)
    contours, hierarchy = cv.findContours(thresh, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

    cv.drawContours(image, contours, -1, (100, 255, 100), 2)
    cv.imshow("Video", image)
    cv.imshow("threshold", thresh)

    key = cv.waitKey(20)
    if key == 27:
        break

cv.destroyWindow("Video")
