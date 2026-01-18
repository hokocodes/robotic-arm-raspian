#!/usr/bin/env python3
"""
Web-based camera viewer with object detection for Raspberry Pi
Uses the same TensorFlow object detection model as therealthing.py
Access from any browser on your network at http://<raspberry-pi-ip>:5000
"""

# Compatibility shim for Python 3.13 (imp module was removed)
import sys
import importlib
import importlib.machinery
import importlib.util
import types

# Check if imp module needs to be created (Python 3.13 removed it)
if 'imp' not in sys.modules or not hasattr(sys.modules['imp'], 'find_module'):
    class ImpCompat:
        """Compatibility shim for removed imp module"""
        PY_SOURCE = 1
        PY_COMPILED = 2
        C_EXTENSION = 3
        PKG_DIRECTORY = 5
        C_BUILTIN = 6
        PY_FROZEN = 7
        
        @staticmethod
        def find_module(name, path=None):
            """Find a module - compatibility shim for removed imp module"""
            try:
                # Try to find the module using importlib
                spec = importlib.util.find_spec(name)
                if spec is None:
                    raise ImportError(f"No module named {name}")
                
                # Return tuple matching old imp.find_module format: (file, pathname, description)
                # description is (suffix, mode, type)
                file_obj = None
                pathname = spec.origin or ''
                suffix = ''
                mode = ''
                type_code = 0
                
                if spec.origin:
                    if spec.origin.endswith('.py'):
                        suffix = '.py'
                        mode = 'r'
                        type_code = ImpCompat.PY_SOURCE
                    elif spec.origin.endswith('.pyc'):
                        suffix = '.pyc'
                        mode = 'rb'
                        type_code = ImpCompat.PY_COMPILED
                    elif spec.origin.endswith(('.so', '.pyd')):
                        suffix = '.so'
                        mode = 'rb'
                        type_code = ImpCompat.C_EXTENSION
                
                return (file_obj, pathname, (suffix, mode, type_code))
            except (ImportError, AttributeError, ValueError):
                raise ImportError(f"No module named {name}")
        
        @staticmethod
        def load_source(name, pathname, file=None):
            loader = importlib.machinery.SourceFileLoader(name, pathname)
            spec = importlib.util.spec_from_loader(loader.name, loader)
            module = importlib.util.module_from_spec(spec)
            loader.exec_module(module)
            return module
    
    sys.modules['imp'] = ImpCompat()

from flask import Flask, Response, render_template_string
import cv2
from picamera2 import Picamera2
import time
import numpy as np
import tensorflow as tf
import tensorflow_hub as hub
import logging

# Set up logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

app = Flask(__name__)

# Load pre-trained object detection model from TensorFlow Hub
logging.info("Loading object detection model...")
try:
    detector = hub.load("https://www.kaggle.com/models/tensorflow/faster-rcnn-inception-resnet-v2/TensorFlow2/640x640/1")
    logging.info("Object detection model loaded successfully!")
except Exception as e:
    logging.error("Failed to load object detection model: %s", e)
    detector = None

# Object classes (same as therealthing.py)
LABELS = {1: "car key", 2: "bottle", 3: "cube"}  # Example labels

# Initialize camera
logging.info("Initializing Raspberry Pi camera...")
picam2 = Picamera2()
camera_config = picam2.create_preview_configuration(
    main={"size": (640, 480), "format": "RGB888"}
)
picam2.configure(camera_config)
picam2.start()
time.sleep(2)  # Let camera warm up
logging.info("Camera initialized successfully!")

# Preprocess image for TensorFlow model (same as therealthing.py)
def preprocess_image(image):
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    image = np.expand_dims(image, axis=0).astype(np.float32)
    image = tf.image.resize(image, (640, 640))  # Resize image to 640x640 as expected by the model
    return image

# Object detection function using TensorFlow Hub model (same as therealthing.py)
def detect_objects(frame):
    if detector is None:
        return [], None, 0
        
    image_tensor = preprocess_image(frame)

    # Run detection
    detector_output = detector(image_tensor)
    
    # Extract detection results
    boxes = detector_output["detection_boxes"][0].numpy()
    class_ids = detector_output["detection_classes"][0].numpy().astype(int)
    confidence = detector_output["detection_scores"][0].numpy()

    # Get frame dimensions for scaling bounding boxes
    frame_height, frame_width = frame.shape[:2]
    
    # Find detections above confidence threshold
    detections = []
    for i in range(len(confidence)):
        if confidence[i] > 0.5:  # Threshold for confidence
            box = boxes[i]
            # Convert normalized coordinates [y_min, x_min, y_max, x_max] to pixel coordinates
            y_min = int(box[0] * frame_height)
            x_min = int(box[1] * frame_width)
            y_max = int(box[2] * frame_height)
            x_max = int(box[3] * frame_width)
            
            class_id = class_ids[i]
            conf = confidence[i]
            label = LABELS.get(class_id, f"class_{class_id}")
            
            detections.append({
                'box': (x_min, y_min, x_max, y_max),
                'class': label,
                'confidence': conf,
                'class_id': class_id
            })
    
    if detections:
        # Return the highest confidence detection
        best_detection = max(detections, key=lambda x: x['confidence'])
        logging.info("Detected object: %s with confidence: %.2f", best_detection['class'], best_detection['confidence'])
        return detections, best_detection['class'], best_detection['confidence']
    
    return [], None, 0

def generate_frames():
    """Generator function to continuously capture, detect, and encode frames"""
    while True:
        # Capture frame from camera
        frame = picam2.capture_array()
        
        # Convert RGB to BGR for OpenCV
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        
        # Run object detection
        detections, obj, conf = detect_objects(frame)
        
        # Draw bounding boxes on frame (same as therealthing.py)
        for detection in detections:
            x_min, y_min, x_max, y_max = detection['box']
            label = detection['class']
            confidence = detection['confidence']
            
            # Draw bounding box
            cv2.rectangle(frame, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)
            
            # Draw label with confidence
            label_text = f"{label}: {confidence:.2f}"
            label_size, _ = cv2.getTextSize(label_text, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
            label_y = max(y_min, label_size[1] + 10)
            
            # Draw label background
            cv2.rectangle(frame, (x_min, y_min - label_size[1] - 10), 
                         (x_min + label_size[0], y_min), (0, 255, 0), -1)
            # Draw label text
            cv2.putText(frame, label_text, (x_min, y_min - 5),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)
        
        # Encode frame as JPEG
        ret, buffer = cv2.imencode('.jpg', frame)
        if not ret:
            continue
            
        frame_bytes = buffer.tobytes()
        
        # Yield frame in multipart format
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')

@app.route('/')
def index():
    """Video streaming home page"""
    html = """
    <!DOCTYPE html>
    <html>
    <head>
        <title>Raspberry Pi - Object Detection</title>
        <meta name="viewport" content="width=device-width, initial-scale=1.0">
        <style>
            body {
                margin: 0;
                padding: 20px;
                background-color: #1e1e1e;
                font-family: Arial, sans-serif;
                color: #ffffff;
                text-align: center;
            }
            h1 {
                margin-bottom: 10px;
            }
            .subtitle {
                color: #4CAF50;
                margin-bottom: 20px;
                font-size: 14px;
            }
            .container {
                max-width: 900px;
                margin: 0 auto;
            }
            .video-wrapper {
                position: relative;
                display: inline-block;
                max-width: 100%;
            }
            img {
                width: 100%;
                max-width: 640px;
                border: 3px solid #4CAF50;
                border-radius: 8px;
                box-shadow: 0 4px 12px rgba(76, 175, 80, 0.4);
                display: block;
            }
            .status-indicator {
                position: absolute;
                top: 15px;
                right: 15px;
                width: 12px;
                height: 12px;
                background-color: #4CAF50;
                border-radius: 50%;
                animation: pulse 2s infinite;
                box-shadow: 0 0 10px #4CAF50;
            }
            @keyframes pulse {
                0%, 100% { opacity: 1; }
                50% { opacity: 0.5; }
            }
            .info {
                margin-top: 20px;
                padding: 20px;
                background-color: #2d2d2d;
                border-radius: 8px;
                color: #aaaaaa;
                text-align: left;
            }
            .info h3 {
                color: #4CAF50;
                margin-top: 0;
                text-align: center;
            }
            .info-grid {
                display: grid;
                grid-template-columns: repeat(auto-fit, minmax(200px, 1fr));
                gap: 15px;
                margin-top: 15px;
            }
            .info-item {
                padding: 10px;
                background-color: #1e1e1e;
                border-radius: 5px;
                border-left: 3px solid #4CAF50;
            }
            .info-label {
                font-size: 12px;
                color: #888;
                text-transform: uppercase;
                margin-bottom: 5px;
            }
            .info-value {
                font-size: 16px;
                color: #fff;
                font-weight: bold;
            }
            .legend {
                margin-top: 10px;
                padding: 10px;
                background-color: #1e1e1e;
                border-radius: 5px;
            }
            .legend-item {
                display: inline-block;
                margin: 5px 15px;
                font-size: 14px;
            }
            .color-box {
                display: inline-block;
                width: 20px;
                height: 20px;
                margin-right: 8px;
                vertical-align: middle;
                border: 2px solid #000;
                border-radius: 3px;
            }
        </style>
    </head>
    <body>
        <div class="container">
            <h1>🤖 Robotic Arm - Object Detection</h1>
            <p class="subtitle">Real-time TensorFlow Object Detection</p>
            
            <div class="video-wrapper">
                <img src="{{ url_for('video_feed') }}" alt="Object Detection Feed">
                <div class="status-indicator" title="Live"></div>
            </div>
            
            <div class="info">
                <h3>📊 Detection Info</h3>
                <div class="info-grid">
                    <div class="info-item">
                        <div class="info-label">Model</div>
                        <div class="info-value">Faster R-CNN</div>
                    </div>
                    <div class="info-item">
                        <div class="info-label">Resolution</div>
                        <div class="info-value">640x480</div>
                    </div>
                    <div class="info-item">
                        <div class="info-label">Confidence Threshold</div>
                        <div class="info-value">50%</div>
                    </div>
                    <div class="info-item">
                        <div class="info-label">Input Size</div>
                        <div class="info-value">640x640</div>
                    </div>
                </div>
                
                <div class="legend">
                    <strong>Detectable Objects:</strong><br>
                    <div class="legend-item">
                        <span class="color-box" style="background-color: #4CAF50;"></span>
                        Car Key
                    </div>
                    <div class="legend-item">
                        <span class="color-box" style="background-color: #4CAF50;"></span>
                        Bottle
                    </div>
                    <div class="legend-item">
                        <span class="color-box" style="background-color: #4CAF50;"></span>
                        Cube
                    </div>
                </div>
            </div>
        </div>
    </body>
    </html>
    """
    return render_template_string(html)

@app.route('/video_feed')
def video_feed():
    """Video streaming route"""
    return Response(generate_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    print("\n" + "="*70)
    print("  🤖 Robotic Arm - Object Detection Web Viewer")
    print("="*70)
    print("\n📹 Camera: Initialized")
    print("🧠 Model: Faster R-CNN Inception ResNet V2")
    print("🎯 Detection Threshold: 50%")
    print("📦 Detectable Objects: car key, bottle, cube")
    print("\n" + "-"*70)
    print("\n🌐 Access the object detection feed at:")
    print("  - Local: http://localhost:5000")
    print("  - Network: http://<raspberry-pi-ip>:5000")
    print("\n💡 Tip: Find your IP with: hostname -I")
    print("\n⌨️  Press Ctrl+C to stop the server")
    print("="*70 + "\n")
    
    try:
        app.run(host='0.0.0.0', port=5000, debug=False, threaded=True)
    except KeyboardInterrupt:
        print("\n\n🛑 Shutting down...")
    finally:
        picam2.stop()
        print("📹 Camera stopped. Goodbye!\n")
