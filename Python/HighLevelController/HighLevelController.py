import serial
import time

ard = serial.Serial('COM7', 9600)
time.sleep(2)

while 1:
    inp = input()
    ard.write(str.encode(inp))
    time.sleep(.1)
    line = ard.readline()
    print('return message:')
    print(line)