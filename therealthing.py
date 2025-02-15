import cv2
import numpy as np
import ikpy
from ikpy.chain import Chain
from ikpy.link import OriginLink, URDFLink
import time
import threading
import tensorflow_hub as hub
import tensorflow as tf
from adafruit_servokit import ServoKit
import logging

# Set up logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

# Load pre-trained object detection model from TensorFlow Hub
logging.info("Loading object detection model...")
try:
    detector = hub.load("https://www.kaggle.com/models/tensorflow/faster-rcnn-inception-resnet-v2/TensorFlow2/640x640/1")
except Exception as e:
    logging.error("Failed to load object detection model: %s", e)
    exit(1)

# Object classes (You can update this list based on your model's expected output classes)
LABELS = {1: "car key", 2: "bottle", 3: "cube"}  # Example labels

# Initialize the robotic arm model for inverse kinematics
logging.info("Initializing robotic arm model...")
arm_chain = Chain(name='robot_arm', links=[
    OriginLink(),
    URDFLink(name="joint_1", origin_translation=[0, 0, 0.1], origin_orientation=[0, 0, 1]),
    URDFLink(name="joint_2", origin_translation=[0, 0, 0.2], origin_orientation=[0, 1, 0]),
    URDFLink(name="joint_3", origin_translation=[0, 0, 0.2], origin_orientation=[0, 1, 0]),
    URDFLink(name="joint_4", origin_translation=[0, 0, 0.15], origin_orientation=[0, 1, 0]),
    URDFLink(name="joint_5", origin_translation=[0, 0, 0.1], origin_orientation=[0, 1, 0]),
    URDFLink(name="joint_6", origin_translation=[0, 0, 0.05], origin_orientation=[0, 1, 0]),
])

# Initialize ServoKit for 16-channel PWM board
logging.info("Initializing ServoKit...")
kit = ServoKit(channels=16)

# Function to move servos
def move_arm(joint_angles):
    logging.info("Moving arm to joint angles: %s", joint_angles)
    working_servos = [1, 2, 13, 14, 15, 16]
    for i, angle in zip(working_servos, joint_angles[1:1+len(working_servos)]):
        servo_angle = np.degrees(angle)  # Convert radians to degrees
        servo_angle = max(0, min(180, servo_angle))  # Clamp between 0-180 degrees
        kit.servo[i].angle = servo_angle
        time.sleep(0.2)

# Preprocess image for TensorFlow model
def preprocess_image(image):
    logging.info("Preprocessing image...")
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    image = np.expand_dims(image, axis=0).astype(np.float32)
    image = tf.image.resize(image, (640, 640))  # Resize image to 640x640 as expected by the model
    return image

# Object detection function using TensorFlow Hub model
def detect_objects(frame):
    logging.info("Running object detection...")
    image_tensor = preprocess_image(frame)

    # Run detection
    detector_output = detector(image_tensor)
    
    # Extract the detection classes and confidence scores
    class_ids = detector_output["detection_classes"][0].numpy().astype(int)
    confidence = detector_output["detection_scores"][0].numpy()

    # Check if any objects are detected with a confidence above a threshold
    detected_object = None
    if confidence.any() > 0.6:  # Threshold for confidence
        max_confidence_idx = np.argmax(confidence)
        detected_object = class_ids[max_confidence_idx]
        logging.info("Detected object: %s with confidence: %f", LABELS.get(detected_object, "unknown"), confidence[max_confidence_idx])
        return LABELS.get(detected_object, "unknown"), confidence[max_confidence_idx]
    
    logging.info("No objects detected with sufficient confidence.")
    return None, 0

# Capture camera feed
logging.info("Opening camera feed...")
cap = cv2.VideoCapture(0)

def process_frame():
    logging.info("Starting frame processing...")
    while True:
        ret, frame = cap.read()
        if not ret:
            logging.warning("Failed to read frame from camera.")
            continue
        
        obj, conf = detect_objects(frame)
        if obj:
            logging.info("Detected: %s with confidence %f", obj, conf)
            target_position = [0.2, 0.1, 0.3]  # Placeholder target (X, Y, Z)
            joint_angles = arm_chain.inverse_kinematics(target_position)
            move_arm(joint_angles)
        
        cv2.imshow("Camera", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            logging.info("Exiting frame processing loop.")
            break

# Run camera processing in a separate thread to improve performance
logging.info("Starting camera processing thread...")
thread = threading.Thread(target=process_frame)
thread.start()

thread.join()
cap.release()
cv2.destroyAllWindows()
logging.info("Camera feed closed and resources released.")

