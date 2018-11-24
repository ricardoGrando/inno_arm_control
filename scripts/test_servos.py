import serial
import time

serialComm = serial.Serial()
serialComm.port = '/dev/ttyUSB0'
serialComm.baudrate = 9600

try:
	serialComm.open()
except:
	print("error")
	exit(0)

while True:	
	i = 5
	
	a = raw_input()
	cont = str(a)
		
	msg = "#"+str(i)+"P" + str(cont) + "T100\r\n"
	print(msg)	
		
	serialComm.write(msg)

	time.sleep(0.1)
