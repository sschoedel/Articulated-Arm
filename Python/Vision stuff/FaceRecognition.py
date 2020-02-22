import numpy as np
import cv2 as cv

fontType = cv.FONT_HERSHEY_COMPLEX

cv.namedWindow("Face")

vc = cv.VideoCapture(0)

if vc.isOpened():
    rval, img = vc.read()
else:
    rval = False

while rval == True:
    rval, img = vc.read()
    face_cascade = cv.CascadeClassifier('C:\\Users\\sesch\\Downloads\\opencv-master\\data\\haarcascades\\haarcascade_frontalface_default.xml')
    eye_cascade = cv.CascadeClassifier('C:\\Users\\sesch\\Downloads\\opencv-master\\data\\haarcascades\\haarcascade_eye.xml')
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

    faces = face_cascade.detectMultiScale(gray, 1.3, 5)
    for (x,y,w,h) in faces:
        cv.rectangle(img,(x,y),(x+w,y+h),(255,0,0),2)
        cv.putText(img, 'Face', (x, y), fontType, .01*w, (255, 255, 255), 1)
    print(faces)
    cv.imshow('Face',img)

    key = cv.waitKey(20)
    if key == 27:
        break

cv.destroyAllWindows()
