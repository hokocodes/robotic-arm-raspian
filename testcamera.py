import cv2

one = cv2.VideoCapture(1)
two = cv2.VideoCapture(2)
zero = cv2.VideoCapture(0)
neg1 = cv2.VideoCapture(-1)

print("one:", one.isOpened())
print("two:", two.isOpened())
print("zero:", zero.isOpened())
print("neg1:", neg1.isOpened())

if one.isOpened():
    ret1, frame1 = one.read()
    print("ret1:", ret1)
if two.isOpened():
    ret2, frame2 = two.read()
    print("ret2:", ret2)
if zero.isOpened():
    ret0, frame0 = zero.read()
    print("ret0:", ret0)
if neg1.isOpened():
    ret_neg1, frame_neg1 = neg1.read()
    print("ret_neg1:", ret_neg1)