import cv2
from picamera2 import Picamera2
from time import sleep

picam2 = Picamera2()
picam2.start()

print("please wait")
sleep(2)
while True:
	print("in")
	frame = picam2.capture_array()

	gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
	
	cv2.imshow("frame", gray_frame)
	
	if cv2.waitKey(1) & 0xFF == ord('q'):
		break
		
cv2.destroyAllWindows()