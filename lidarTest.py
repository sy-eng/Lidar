import serial
import time
import RPi.GPIO as GPIO

ser = serial.Serial('/dev/ttyS0', 115200, timeout = 3)
tmpString = ""
aaFlag = False

#Init
GPIO.setmode(GPIO.BCM)
GPIO.setup(18, GPIO.OUT)
GPIO.output(18, 1)

for i in range(60):
	loopFlag = True

#	if i > 30:
#		val = 0xA5
#		ser.write(val.to_bytes(1, 'big'))
#		val = 0x25
#		ser.write(val.to_bytes(1, 'big'))

	while loopFlag:
		b = ser.read()
		#print(b)
		tmpInt = int.from_bytes(b, 'big')
		if tmpInt == 0xAA:
			aaFlag = True
			tmpString += b.hex() + " "
		elif (tmpInt == 0x55 and aaFlag):
			aaFlag = False
			tmpString += b.hex()
			print(tmpString)
			tmpString = ""
			loopFlag = False
		elif (len(b) == 0):
			print("!")
		else:
			aaFlag = False
			tmpString += b.hex() + " "

#Clean up
GPIO.output(18, 0)
GPIO.cleanup()
ser.close()
