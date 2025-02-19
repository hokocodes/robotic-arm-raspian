import cv2
for device_number in range(32):
    # Open the camera (0 for the default camera)
    cap = cv2.VideoCapture(device_number, cv2.CAP_V4L2)

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    # Check if the camera is opened successfully
    if not cap.isOpened():
        print(f"Error: Could not open camera /dev/video{device_number}.")
        continue

    print(f"Opened camera /dev/video{device_number}.")
    
    while True:
        # Capture frame-by-frame
        ret, frame = cap.read()
        
        if not ret:
            print("Error: Failed to grab frame.")
            break
        
        # Display the resulting frame
        cv2.imshow(f'Camera Feed /dev/video{device_number}', frame)

        # Exit on pressing 'q'
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release the camera and close all OpenCV windows
    cap.release()
    cv2.destroyAllWindows()

