import keyboard  # using module keyboard
import pynput.keyboard as pykey
import time
chars = ['i','j','k','f','h','n','b','y','r','t',' ', 'i']
letters = []
while True:  # making a loop
    for i in chars:
        try:
            if keyboard.is_pressed(i):
                print(i)
                letters.append(i)
                time.sleep(.1)
        except:
            break
    if keyboard.is_pressed(i) == '.':
        print('.')
        break
print(letters)