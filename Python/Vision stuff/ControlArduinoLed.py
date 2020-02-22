import serial
import time

arduinoData = serial.Serial('com4', 9600)

def led_on():
    arduinoData.write(b'1')

def led_off():
    arduinoData.write(b'0')

time.sleep(1)

while True:
    led_on()
    print("on")
    led_off()
    print("off")
