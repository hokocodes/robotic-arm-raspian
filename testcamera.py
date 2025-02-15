import cv2

# indices from ls /dev/video*
one = cv2.VideoCapture(0)
two = cv2.VideoCapture(12)
three = cv2.VideoCapture(15)
four = cv2.VideoCapture(19)
five = cv2.VideoCapture(22)
six = cv2.VideoCapture(10)
seven = cv2.VideoCapture(13)
eight = cv2.VideoCapture(16)
nine = cv2.VideoCapture(20)
ten = cv2.VideoCapture(23)
eleven = cv2.VideoCapture(11)
twelve = cv2.VideoCapture(14)
thirteen = cv2.VideoCapture(18)
fourteen = cv2.VideoCapture(21)
fifteen = cv2.VideoCapture(31)

print("one:", one.isOpened())
print("two:", two.isOpened())
print("three:", three.isOpened())
print("four:", four.isOpened())
print("five:", five.isOpened())
print("six:", six.isOpened())
print("seven:", seven.isOpened())
print("eight:", eight.isOpened())
print("nine:", nine.isOpened())
print("ten:", ten.isOpened())
print("eleven:", eleven.isOpened())
print("twelve:", twelve.isOpened())
print("thirteen:", thirteen.isOpened())
print("fourteen:", fourteen.isOpened())
print("fifteen:", fifteen.isOpened())

if one.isOpened():
    ret1, frame1 = one.read()
    print("ret1:", ret1)
if two.isOpened():
    ret2, frame2 = two.read()
    print("ret2:", ret2)
if three.isOpened():
    ret3, frame3 = three.read()
    print("ret3:", ret3)
if four.isOpened():
    ret4, frame4 = four.read()
    print("ret4:", ret4)
if five.isOpened():
    ret5, frame5 = five.read()
    print("ret5:", ret5)
if six.isOpened():
    ret6, frame6 = six.read()
    print("ret6:", ret6)
if seven.isOpened():
    ret7, frame7 = seven.read()
    print("ret7:", ret7)
if eight.isOpened():
    ret8, frame8 = eight.read()
    print("ret8:", ret8)
if nine.isOpened():
    ret9, frame9 = nine.read()
    print("ret9:", ret9)
if ten.isOpened():
    ret10, frame10 = ten.read()
    print("ret10:", ret10)
if eleven.isOpened():
    ret11, frame11 = eleven.read()
    print("ret11:", ret11)
if twelve.isOpened():
    ret12, frame12 = twelve.read()
    print("ret12:", ret12)
if thirteen.isOpened():
    ret13, frame13 = thirteen.read()
    print("ret13:", ret13)
if fourteen.isOpened():
    ret14, frame14 = fourteen.read()
    print("ret14:", ret14)
if fifteen.isOpened():
    ret15, frame15 = fifteen.read()
    print("ret15:", ret15)
