import numpy as np
import cv2 as cv
import serial
import time
import threading
import serial.tools.list_ports

fontType = cv.FONT_HERSHEY_TRIPLEX
newPos = False
newx = 0
newy = 0

cv.namedWindow('Hand')
vc = cv.VideoCapture(0)

arduinoData = serial.Serial('COM4', 250000)
time.sleep(1)
myports = [tuple(p) for p in list(serial.tools.list_ports.comports())]
arduino_port = [port for port in myports if 'COM4' in port][0]

def check_serial_connect(correct_port, interval = 0.1):
    arduinoData.flushOutput()
    arduinoData.flushInput()
    myports = [tuple(p) for p in list(serial.tools.list_ports.comports())]
    if correct_port not in myports:
        print("Arduino not connected")
        return False
    else:
        return True

if vc.isOpened():
    rval, img = vc.read()
else:
    rval = False

while rval == True:


    rval, img = vc.read()
    fist_cascade = cv.CascadeClassifier('C:\\Users\\sesch\\Downloads\\opencv-master\\haarcascade\\aGest.xml')
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

    fists = fist_cascade.detectMultiScale(gray, 1.3, 5)
    if len(fists) > 0:
        if newPos == False:
            for (x,y,w,h) in fists:
                cv.rectangle(img,(x,y),(x+w,y+h),(255,0,0),2)
                cv.putText(img, 'Center', (int(x+w/x), int(y+h/2)), fontType, 1, (255, 255, 255), 2)
                cv.circle(img, (int(x+w/2), int(y+h/2)), 1, (255, 0, 255), 20)
            x = fists[0][0]+fists[0][2]/2
            y = fists[0][1]+fists[0][3]/2
            newPos = True
            newx = x
            newy = y
        else:
            for (x,y,w,h) in fists:
                cv.rectangle(img,(x,y),(x+w,y+h),(255,0,0),2)
                cv.putText(img, 'Center', (int(x+w/x), int(y+h/2)), fontType, 1, (255, 255, 255), 2)
                cv.circle(img, (int(x+w/2), int(y+h/2)), 1, (255, 0, 255), 20)
                cv.line(img, (int(x+w/2), int(y+h/2)), (int(newx), int(newy)), (0, 255, 255), 3)
            x = fists[0][0]+fists[0][2]/2
            y = fists[0][1]+fists[0][3]/2

            x_axis = newx - x
            y_axis = newy - y
            if check_serial_connect(arduino_port):
                if x_axis > 30:
                    arduinoData.write(b'1')
                if x_axis < 30:
                    arduinoData.write(b'0')
                print("sent x: {} and y: {}".format(x_axis, y_axis))

    if len(fists) == 0:
        newPos = False
        if check_serial_connect(arduino_port):
            arduinoData.write(b'0')
            print("wrote 0")

    cv.imshow('Hand',img)

    key = cv.waitKey(20)
    if key == 27:
        break

cv.destroyAllWindows()
