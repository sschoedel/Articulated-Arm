from time import sleep
import serial
import serial.tools.list_ports as port_list

try:
	ports = list(port_list.comports())
	port = ports[0]
	with serial.Serial('COM9', 57600, timeout=0) as ser:
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