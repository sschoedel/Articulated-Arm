import numpy as np
import cv2 as cv
#import matplotlib.pyplot as pyplot
#from matplotlib.widgets import Slider


cv.namedWindow("Hull")
cv.namedWindow("Thresh")

vc = cv.VideoCapture(0)

if vc.isOpened():
    rval, image = vc.read()
else:
    rval = False

kernel = np.ones((5, 5), np.uint8)

while rval:
    #slider = Slider(threshValue, 'Threshold', 0.1, 255, valinit=a0)
    contourColor = (255, 0, 0)
    rval, image = vc.read()
    gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
    ret, thresh = cv.threshold(gray, 130, 255, cv.THRESH_BINARY)

    thresh = cv.dilate(thresh, kernel, iterations = 2)
    thresh = cv.erode(thresh, kernel, iterations = 2)

    contours, hierarchy = cv.findContours(thresh, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    hull = []
    for i in range(len(contours)):
        hull.append(cv.convexHull(contours[i], False))
    for i in range(len(contours)):
        cv.drawContours(image, hull, i, contourColor, 1, 8)

    cv.imshow("Hull", image)
    cv.imshow("Thresh", thresh)

    key = cv.waitKey(20)
    if key == 27:
        break

cv.destroyWindow("Video")
