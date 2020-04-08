from time import sleep
import serial
import serial.tools.list_ports as port_list

try:
	ports = list(port_list.comports())
	port = ports[0]
	with serial.Serial('COM9', 115200, timeout=0) as ser:
		sleep(0.5)
		print("start")
		while True:
			send = input("Enter a value to send: ")
			ser.write(send.encode())
			sleep(.01) # Allow time for arduino to echo
			byteSize = ser.in_waiting
			readBytes = ser.read(byteSize).decode().split()
			vals = [chr(int(rawVal)) for rawVal in readBytes]
			print(vals)

except KeyboardInterrupt as kbi_error:
	print(kbi_error)
	ser.close()
	print("{} closed".format(port))

while True:
	# Check input from 
	# Get image from camera
	# Process image
	# Update end position OR calculate new thetas with image and sensor data
	# Calculate thetas for new position if new position updated and thetas not updated
	# Send new thetas to teensy/ard
