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
cameras = [one, two, three, four, five, six, seven, eight, nine, ten, eleven, twelve, thirteen, fourteen, fifteen]

for i, cam in enumerate(cameras, start=1):
    if cam.isOpened():
        ret, frame = cam.read()
        print(f"ret{i}:", ret)
    else:
        print(f"Failed to open camera {i}")
        cam.release()

