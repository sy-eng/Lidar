import cv2
import serial
#import datetime
import RPi.GPIO as GPIO
import math

ANGLE_OFFSET = 0
DIST_RANGE = 2

def printData(data):
	tmpString = ""

	for d in data:
		tmpString += d.to_bytes(1, 'big').hex() + " "

	print(tmpString)

def parseData(data):
	retValue = []
	
	prevIndex = 0

	for i in range(1, len(data) - 1):
		if data[i] == 0xaa and data[i + 1] == 0x55:
			retValue.append(data[prevIndex:i])
			prevIndex = i

	retValue.append(data[prevIndex:])

	return retValue

def checkSum(data):
	check = [0, 0]

	for i in range(int(len(data)/2)):
		if i != 4:
			check[0] ^= data[2*i]
			check[1] ^= data[2*i + 1]

	return (check[0] == data[8] and check[1] == data[9])

def processData(data):
	distances = []
	
	dataNum = int(data[3])
	
	for i in range(dataNum):
		distances.append(int((data[2*(i + 5)] | data[2*(i + 5) + 1] << 8) >> 2))
		#distances.append(int(data[2*(i + 5)] | data[2*(i + 5) + 1] << 8)/4)
	
	angleCorrect = [math.atan(19.16 * (d - 90.15)/(90.15 * d)) if d != 0 else 0.0 for d in distances]
	
	startAngleByte = (data[4] | data[5] << 8) >> 1
	endAngleByte = (data[6] | data[7] << 8) >> 1

	startAngle = int(startAngleByte) * math.pi / 64 / 180
	endAngle = int(endAngleByte) * math.pi / 64 / 180

	if dataNum > 1:
		angleStep = (endAngle - startAngle) / (dataNum - 1)
	else:
		angleStep = 0

	angles = [startAngle + angleStep * i + angleCorrect[i] for i in range(len(angleCorrect))]

	return distances, angles

def drawPoints(img, distances, angles):
	global ANGLE_OFFSET
	global DIST_RANGE
	width, height = img.shape[:2]
	centerX = width / 2
	centerY = height / 2
	POINT_R = 2

	for (d, a) in zip(distances, angles):
		if d > 0:
			ang = a + ANGLE_OFFSET
			xPos = centerX * (1 + (math.cos(ang) * d) / (DIST_RANGE * 1000))
			yPos = centerY * (1 + (math.sin(ang) * d) / (DIST_RANGE * 1000))
			cv2.circle(img,(int(xPos), int(yPos)), POINT_R, (0, 255, 0), -1)

#print(cv2.__version__)

originalImg = cv2.imread("lena.png")
width, height = originalImg.shape[:2]

ser = serial.Serial('/dev/ttyS0', 115200, timeout = 0.001)

GPIO.setmode(GPIO.BCM)
GPIO.setup(18, GPIO.OUT)
GPIO.output(18, 1)

data = []

while True:

	img = originalImg.copy()

	cv2.line(img, (0, int(height/2)), (width, int(height/2)), (255,0,0), 1)
	cv2.line(img, (int(width/2), 0), (int(width/2), height), (255,0,0), 1)

	R = min([width, height])/(DIST_RANGE * 2)
	for i in range(1, DIST_RANGE + 1):
		cv2.circle(img,(int(width/2), int(height/2)), int(i*R), (255, 0, 0), 1)

	data += ser.read(100000)

	startIndex = 0
	nextIndex = 0

	for i in range(len(data) - 2):
		if data[i] == 0xaa:
			if data[i + 1] == 0x55 and data[i + 2] == 0x01:
				#Sometimes, a check sum 0xAA and 0x55
				if i > nextIndex + 10:
					startIndex = nextIndex
					nextIndex = i

	if startIndex > 0:
		pData = parseData(data[startIndex : nextIndex])
		distances = []
		angles = []

		for pd in pData:
			if len(pd) > 10:
				if checkSum(pd):
					dist, ang = processData(pd)
					distances += dist
					angles += ang

		drawPoints(img, distances, angles)

		data = data[nextIndex:]
	elif nextIndex > 0:
		data = data[nextIndex:]
 
	cv2.imshow("Lidar", img)
	#print("Reload : " + str(datetime.datetime.now()))


	k = cv2.waitKey(1)
	if k >= 0:
		break
	else:
		loopFlag = True

GPIO.output(18, 0)
GPIO.cleanup()
ser.close()
cv2.destroyAllWindows()
