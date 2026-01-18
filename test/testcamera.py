import cv2
import picamera
camera = picamera.PiCamera()
one = cv2.VideoCapture(camera)

ret = one.read()
print(ret)