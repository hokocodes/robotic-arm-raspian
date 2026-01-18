#!/usr/bin/env python3
"""
Robotic Arm Vision Controller
------------------------------
Cloud-based object detection with automatic arm positioning
Fast startup, web-based monitoring, and servo control

Features:
- Cloud object detection (instant startup, no heavy models)
- Real-time camera feed accessible via web browser
- Inverse kinematics for arm positioning
- Automatic object tracking and reaching
"""

import cv2
import numpy as np
from ikpy.chain import Chain
from ikpy.link import OriginLink, URDFLink
import time
import threading
from adafruit_servokit import ServoKit
import logging
from picamera2 import Picamera2
from flask import Flask, Response, render_template_string
import requests
import base64
import os
from dotenv import load_dotenv

# Load environment variables from .env file
load_dotenv()

# =============================================================================
# CONFIGURATION - All loaded from environment variables
# =============================================================================

# Cloud Detection Settings
DETECTION_PROVIDER = os.getenv("DETECTION_PROVIDER", "roboflow")

# Roboflow (80 objects, 1000/month free)
ROBOFLOW_API_KEY = os.getenv("ROBOFLOW_API_KEY", "")
ROBOFLOW_MODEL = os.getenv("ROBOFLOW_MODEL", "coco/3")

# Hugging Face Inference API (FREE - no key needed for public models, or get free token)
# Popular models: facebook/detr-resnet-50, facebook/detr-resnet-101, XintongHan/rt-detr-coco-1x
HUGGINGFACE_API_TOKEN = os.getenv("HUGGINGFACE_API_TOKEN", "")  # Optional - free token available
HUGGINGFACE_MODEL = os.getenv("HUGGINGFACE_MODEL", "facebook/detr-resnet-50")  # 80 COCO objects

# Google Cloud Vision (10,000+ objects, 1000/month free)
GOOGLE_CLOUD_CREDENTIALS = os.getenv("GOOGLE_APPLICATION_CREDENTIALS", "")

# Azure Computer Vision (10,000+ objects, 5000/month free)
AZURE_ENDPOINT = os.getenv("AZURE_ENDPOINT", "")
AZURE_KEY = os.getenv("AZURE_KEY", "")

# Detection settings
CONFIDENCE_THRESHOLD = float(os.getenv("CONFIDENCE_THRESHOLD", "0.5"))
DETECT_EVERY_N_FRAMES = int(os.getenv("DETECT_EVERY_N_FRAMES", "30"))

# Servo Configuration
WORKING_SERVOS = [int(x.strip()) for x in os.getenv("WORKING_SERVOS", "1,2,13,14,15,16").split(",")]

# Camera Configuration
CAMERA_WIDTH = int(os.getenv("CAMERA_WIDTH", "640"))
CAMERA_HEIGHT = int(os.getenv("CAMERA_HEIGHT", "480"))

# Web Server
WEB_SERVER_PORT = int(os.getenv("WEB_SERVER_PORT", "5000"))

# Target Objects (objects the arm will react to)
TARGET_OBJECTS = [x.strip() for x in os.getenv("TARGET_OBJECTS", "bottle,cup,cell phone,book,remote").split(",")]

# =============================================================================

# Set up logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

# Flask app for web interface
app = Flask(__name__)

# Global variables
current_detections = []
frame_count = 0
latest_frame = None
arm_busy = False

# Initialize the robotic arm model for inverse kinematics
logging.info("🤖 Initializing robotic arm model...")
arm_chain = Chain(name='robot_arm', links=[
    OriginLink(),
    URDFLink(name="joint_1", origin_translation=[0, 0, 0.1], origin_orientation=[0, 0, 1], rotation=[0, 0, 0]),
    URDFLink(name="joint_2", origin_translation=[0, 0, 0.2], origin_orientation=[0, 1, 0], rotation=[0, 0, 0]),
    URDFLink(name="joint_3", origin_translation=[0, 0, 0.2], origin_orientation=[0, 1, 0], rotation=[0, 0, 0]),
    URDFLink(name="joint_4", origin_translation=[0, 0, 0.15], origin_orientation=[0, 1, 0], rotation=[0, 0, 0]),
    URDFLink(name="joint_5", origin_translation=[0, 0, 0.1], origin_orientation=[0, 1, 0], rotation=[0, 0, 0]),
    URDFLink(name="joint_6", origin_translation=[0, 0, 0.05], origin_orientation=[0, 1, 0], rotation=[0, 0, 0]),
])

# Initialize ServoKit for 16-channel PWM board
logging.info("🔧 Initializing ServoKit...")
try:
    kit = ServoKit(channels=16)
    logging.info("✅ ServoKit initialized")
except Exception as e:
    logging.error(f"❌ Failed to initialize ServoKit: {e}")
    kit = None

# Initialize Raspberry Pi Camera
logging.info("📹 Initializing camera...")
try:
    picam2 = Picamera2()
    camera_config = picam2.create_preview_configuration(
        main={"size": (CAMERA_WIDTH, CAMERA_HEIGHT), "format": "RGB888"}
    )
    picam2.configure(camera_config)
    picam2.start()
    time.sleep(2)  # Camera warm-up
    logging.info("✅ Camera initialized")
except Exception as e:
    logging.error(f"❌ Failed to initialize camera: {e}")
    exit(1)

# =============================================================================
# SERVO CONTROL
# =============================================================================

def move_arm(joint_angles):
    """Move the robotic arm to specified joint angles"""
    global arm_busy
    
    if not kit:
        logging.warning("⚠️ ServoKit not initialized, skipping arm movement")
        return
    
    arm_busy = True
    logging.info(f"🦾 Moving arm to joint angles: {joint_angles}")
    
    try:
        for i, angle in zip(WORKING_SERVOS, joint_angles[1:1+len(WORKING_SERVOS)]):
            servo_angle = np.degrees(angle)  # Convert radians to degrees
            servo_angle = max(0, min(180, servo_angle))  # Clamp between 0-180
            kit.servo[i].angle = servo_angle
            time.sleep(0.2)
    except Exception as e:
        logging.error(f"❌ Error moving arm: {e}")
    finally:
        arm_busy = False

def calculate_arm_position(detection):
    """Calculate 3D target position from detected object location"""
    x_min, y_min, x_max, y_max = detection['box']
    
    # Calculate center of detected object
    center_x = (x_min + x_max) / 2
    center_y = (y_min + y_max) / 2
    
    # Convert pixel coordinates to arm workspace coordinates
    # This is a simplified mapping - adjust based on your calibration
    normalized_x = (center_x / CAMERA_WIDTH) - 0.5  # -0.5 to 0.5
    normalized_y = 0.5 - (center_y / CAMERA_HEIGHT)  # -0.5 to 0.5
    
    # Map to arm workspace (adjust these values for your arm)
    target_x = normalized_x * 0.4  # Scale to arm reach
    target_y = normalized_y * 0.4
    target_z = 0.3  # Fixed height for now
    
    return [target_x, target_y, target_z]

# =============================================================================
# OBJECT DETECTION
# =============================================================================

def detect_with_roboflow(frame):
    """Run object detection using Roboflow API (80 objects)"""
    if not ROBOFLOW_API_KEY:
        return []
        
    try:
        _, buffer = cv2.imencode('.jpg', frame)
        img_base64 = base64.b64encode(buffer).decode('utf-8')
        
        url = f"https://detect.roboflow.com/{ROBOFLOW_MODEL}?api_key={ROBOFLOW_API_KEY}&confidence={int(CONFIDENCE_THRESHOLD*100)}"
        response = requests.post(
            url, 
            data=img_base64, 
            headers={'Content-Type': 'application/x-www-form-urlencoded'}, 
            timeout=5
        )
        
        if response.status_code == 200:
            predictions = response.json().get('predictions', [])
            detections = []
            
            for pred in predictions:
                x = int(pred['x'] - pred['width']/2)
                y = int(pred['y'] - pred['height']/2)
                w = int(pred['width'])
                h = int(pred['height'])
                
                detections.append({
                    'box': (x, y, x+w, y+h),
                    'class': pred['class'],
                    'confidence': pred['confidence']
                })
            
            return detections
        else:
            logging.warning(f"⚠️ Roboflow API returned status {response.status_code}")
            return []
            
    except Exception as e:
        logging.error(f"❌ Roboflow error: {e}")
        return []

def detect_with_google_vision(frame):
    """Run object detection using Google Cloud Vision API (10,000+ objects)"""
    if not GOOGLE_CLOUD_CREDENTIALS:
        return []
        
    try:
        from google.cloud import vision
        
        # Set credentials (already set in environment from .env)
        # Initialize client
        client = vision.ImageAnnotatorClient()
        
        # Encode image
        _, buffer = cv2.imencode('.jpg', frame)
        image = vision.Image(content=buffer.tobytes())
        
        # Detect objects with localization
        response = client.object_localization(image=image)
        objects = response.localized_object_annotations
        
        detections = []
        frame_height, frame_width = frame.shape[:2]
        
        for obj in objects:
            if obj.score >= CONFIDENCE_THRESHOLD:
                # Get bounding box from normalized vertices
                vertices = obj.bounding_poly.normalized_vertices
                x_coords = [v.x for v in vertices]
                y_coords = [v.y for v in vertices]
                
                x_min = int(min(x_coords) * frame_width)
                y_min = int(min(y_coords) * frame_height)
                x_max = int(max(x_coords) * frame_width)
                y_max = int(max(y_coords) * frame_height)
                
                detections.append({
                    'box': (x_min, y_min, x_max, y_max),
                    'class': obj.name,
                    'confidence': obj.score
                })
        
        return detections
        
    except Exception as e:
        logging.error(f"❌ Google Vision error: {e}")
        return []

def detect_with_azure(frame):
    """Run object detection using Azure Computer Vision API (10,000+ objects)"""
    if not AZURE_KEY or not AZURE_ENDPOINT:
        return []
        
    try:
        _, buffer = cv2.imencode('.jpg', frame)
        
        # Azure API endpoint
        url = f"{AZURE_ENDPOINT}/vision/v3.2/detect"
        
        headers = {
            'Ocp-Apim-Subscription-Key': AZURE_KEY,
            'Content-Type': 'application/octet-stream'
        }
        
        params = {
            'visualFeatures': 'Objects'
        }
        
        response = requests.post(
            url,
            headers=headers,
            params=params,
            data=buffer.tobytes(),
            timeout=5
        )
        
        if response.status_code == 200:
            result = response.json()
            detections = []
            
            for obj in result.get('objects', []):
                if obj['confidence'] >= CONFIDENCE_THRESHOLD:
                    rect = obj['rectangle']
                    detections.append({
                        'box': (rect['x'], rect['y'], 
                               rect['x'] + rect['w'], rect['y'] + rect['h']),
                        'class': obj['object'],
                        'confidence': obj['confidence']
                    })
            
            return detections
        else:
            logging.warning(f"⚠️ Azure API returned status {response.status_code}")
            return []
            
    except Exception as e:
        logging.error(f"❌ Azure error: {e}")
        return []

def detect_with_huggingface(frame):
    """Run object detection using Hugging Face Inference API (FREE - many models available)"""
    try:
        # Encode frame
        _, buffer = cv2.imencode('.jpg', frame)
        
        # Hugging Face Inference API endpoint
        url = f"https://api-inference.huggingface.co/models/{HUGGINGFACE_MODEL}"
        
        headers = {}
        
        # Add token if provided (optional - public models work without token)
        if HUGGINGFACE_API_TOKEN:
            headers['Authorization'] = f'Bearer {HUGGINGFACE_API_TOKEN}'
        
        # Send request (Hugging Face expects raw image bytes)
        response = requests.post(
            url,
            headers=headers,
            data=buffer.tobytes(),
            timeout=10
        )
        
        if response.status_code == 200:
            predictions = response.json()
            detections = []
            frame_height, frame_width = frame.shape[:2]
            
            # Hugging Face models return different formats, handle common ones
            if isinstance(predictions, list):
                for pred in predictions:
                    # Handle DETR format: {"label": "...", "score": 0.9, "box": {"xmin": 10, "ymin": 20, "xmax": 100, "ymax": 200}}
                    if 'box' in pred and 'label' in pred:
                        box = pred['box']
                        score = pred.get('score', pred.get('confidence', 0))
                        
                        if score >= CONFIDENCE_THRESHOLD:
                            x_min = int(box.get('xmin', box.get('x1', 0)))
                            y_min = int(box.get('ymin', box.get('y1', 0)))
                            x_max = int(box.get('xmax', box.get('x2', 0)))
                            y_max = int(box.get('ymax', box.get('y2', 0)))
                            
                            # Handle normalized coordinates
                            if x_max <= 1.0:
                                x_min = int(x_min * frame_width)
                                y_min = int(y_min * frame_height)
                                x_max = int(x_max * frame_width)
                                y_max = int(y_max * frame_height)
                            
                            detections.append({
                                'box': (x_min, y_min, x_max, y_max),
                                'class': pred['label'],
                                'confidence': score
                            })
                    # Handle alternative format with coordinates list
                    elif 'label' in pred and 'score' in pred and len(pred.get('box', [])) == 4:
                        coords = pred['box']
                        if pred['score'] >= CONFIDENCE_THRESHOLD:
                            # DETR returns [x_center, y_center, width, height] normalized
                            if all(c <= 1.0 for c in coords):
                                x_center, y_center, w, h = coords
                                x_min = int((x_center - w/2) * frame_width)
                                y_min = int((y_center - h/2) * frame_height)
                                x_max = int((x_center + w/2) * frame_width)
                                y_max = int((y_center + h/2) * frame_height)
                            else:
                                x_min, y_min, x_max, y_max = map(int, coords)
                            
                            detections.append({
                                'box': (x_min, y_min, x_max, y_max),
                                'class': pred['label'],
                                'confidence': pred['score']
                            })
            
            return detections
        elif response.status_code == 503:
            # Model might be loading
            logging.warning("⚠️ Hugging Face model is loading, please wait...")
            return []
        else:
            logging.warning(f"⚠️ Hugging Face API returned status {response.status_code}: {response.text[:100]}")
            return []
            
    except Exception as e:
        logging.error(f"❌ Hugging Face error: {e}")
        return []

def detect_objects(frame):
    """Main detection function - routes to selected provider"""
    try:
        if DETECTION_PROVIDER == "roboflow":
            return detect_with_roboflow(frame)
        elif DETECTION_PROVIDER == "huggingface":
            return detect_with_huggingface(frame)
        elif DETECTION_PROVIDER == "google":
            return detect_with_google_vision(frame)
        elif DETECTION_PROVIDER == "azure":
            return detect_with_azure(frame)
        else:
            return []
    except Exception as e:
        logging.error(f"❌ Detection error: {e}")
        return []

# =============================================================================
# FRAME PROCESSING
# =============================================================================

def draw_detections(frame, detections):
    """Draw bounding boxes and labels on frame"""
    for det in detections:
        x_min, y_min, x_max, y_max = det['box']
        label = det['class']
        confidence = det['confidence']
        
        # Choose color based on whether it's a target object
        color = (0, 255, 0) if label in TARGET_OBJECTS else (255, 255, 0)
        
        # Draw bounding box
        cv2.rectangle(frame, (x_min, y_min), (x_max, y_max), color, 2)
        
        # Draw label
        label_text = f"{label}: {confidence:.2f}"
        label_size, _ = cv2.getTextSize(label_text, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
        
        cv2.rectangle(frame, (x_min, y_min - label_size[1] - 10), 
                     (x_min + label_size[0], y_min), color, -1)
        cv2.putText(frame, label_text, (x_min, y_min - 5),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)
    
    return frame

def process_detections(detections):
    """Process detections and control arm"""
    global arm_busy
    
    if not detections or arm_busy:
        return
    
    # Find target objects
    target_detections = [d for d in detections if d['class'] in TARGET_OBJECTS]
    
    if target_detections:
        # Get highest confidence target
        best_target = max(target_detections, key=lambda x: x['confidence'])
        logging.info(f"🎯 Target found: {best_target['class']} ({best_target['confidence']:.2f})")
        
        # Calculate target position
        target_position = calculate_arm_position(best_target)
        logging.info(f"📍 Target position: {target_position}")
        
        # Calculate inverse kinematics
        joint_angles = arm_chain.inverse_kinematics(target_position)
        
        # Move arm in separate thread to avoid blocking
        threading.Thread(target=move_arm, args=(joint_angles,), daemon=True).start()

def camera_loop():
    """Main camera processing loop"""
    global current_detections, frame_count, latest_frame
    
    logging.info("🎬 Starting camera processing...")
    
    while True:
        try:
            # Capture frame
            frame = picam2.capture_array()
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            
            frame_count += 1
            
            # Run detection periodically
            if frame_count % DETECT_EVERY_N_FRAMES == 0:
                current_detections = detect_objects(frame)
                if current_detections:
                    logging.info(f"🔍 Detected {len(current_detections)} object(s)")
                    process_detections(current_detections)
            
            # Draw detections
            frame = draw_detections(frame, current_detections)
            
            # Add status overlay
            status = f"Objects: {len(current_detections)} | Arm: {'BUSY' if arm_busy else 'READY'}"
            cv2.putText(frame, status, (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            # Store latest frame for web streaming
            latest_frame = frame.copy()
            
        except Exception as e:
            logging.error(f"❌ Camera loop error: {e}")
            break

# =============================================================================
# WEB INTERFACE
# =============================================================================

def generate_frames():
    """Generate frames for web streaming"""
    global latest_frame
    
    while True:
        if latest_frame is not None:
            try:
                ret, buffer = cv2.imencode('.jpg', latest_frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
                if ret:
                    yield (b'--frame\r\n'
                           b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')
            except Exception as e:
                logging.error(f"❌ Frame encoding error: {e}")
        
        time.sleep(0.033)  # ~30 FPS

@app.route('/')
def index():
    # Check API status based on provider
    if DETECTION_PROVIDER == "roboflow":
        api_status = "✅ Configured" if ROBOFLOW_API_KEY else "⚠️ No API Key"
        provider_name = "Roboflow (80 objects)"
    elif DETECTION_PROVIDER == "huggingface":
        api_status = "✅ Ready (Free - No key needed)"  # Works without token for public models
        provider_name = f"Hugging Face ({HUGGINGFACE_MODEL})"
    elif DETECTION_PROVIDER == "google":
        api_status = "✅ Configured" if GOOGLE_CLOUD_CREDENTIALS else "⚠️ No Credentials"
        provider_name = "Google Vision (10,000+ objects)"
    elif DETECTION_PROVIDER == "azure":
        api_status = "✅ Configured" if (AZURE_KEY and AZURE_ENDPOINT) else "⚠️ No API Key"
        provider_name = "Azure Vision (10,000+ objects)"
    else:
        api_status = "⚠️ Not Configured"
        provider_name = "None"
    
    html = f"""
    <!DOCTYPE html>
    <html>
    <head>
        <title>Robotic Arm Vision</title>
        <meta name="viewport" content="width=device-width, initial-scale=1.0">
        <style>
            body {{
                margin: 0;
                padding: 20px;
                background: linear-gradient(135deg, #1e1e1e 0%, #2d2d2d 100%);
                font-family: 'Segoe UI', Arial, sans-serif;
                color: #ffffff;
                text-align: center;
            }}
            h1 {{ 
                margin-bottom: 10px;
                font-size: 2.5em;
            }}
            .subtitle {{ 
                color: #4CAF50;
                margin-bottom: 30px;
                font-size: 1.1em;
            }}
            .container {{ 
                max-width: 1000px;
                margin: 0 auto;
            }}
            img {{
                width: 100%;
                max-width: 640px;
                border: 4px solid #4CAF50;
                border-radius: 12px;
                box-shadow: 0 8px 24px rgba(76, 175, 80, 0.4);
            }}
            .info {{
                margin-top: 30px;
                padding: 25px;
                background-color: rgba(45, 45, 45, 0.8);
                border-radius: 12px;
                text-align: left;
                border: 1px solid #4CAF50;
            }}
            .info h3 {{
                color: #4CAF50;
                margin-top: 0;
                text-align: center;
            }}
            .grid {{
                display: grid;
                grid-template-columns: repeat(auto-fit, minmax(220px, 1fr));
                gap: 15px;
                margin-top: 20px;
            }}
            .card {{
                padding: 15px;
                background-color: #1e1e1e;
                border-radius: 8px;
                border-left: 4px solid #4CAF50;
            }}
            .label {{ 
                font-size: 12px;
                color: #888;
                text-transform: uppercase;
                margin-bottom: 8px;
            }}
            .value {{ 
                font-size: 18px;
                font-weight: bold;
                color: #fff;
            }}
            .status-badge {{
                display: inline-block;
                padding: 6px 16px;
                background-color: {('#1e6b1e' if api_status == '✅ Configured' else '#8b6b00')};
                border-radius: 20px;
                font-size: 14px;
                font-weight: bold;
            }}
            .warning {{
                margin-top: 15px;
                padding: 15px;
                background-color: rgba(255, 152, 0, 0.1);
                border: 1px solid #ff9800;
                border-radius: 8px;
                color: #ff9800;
            }}
        </style>
    </head>
    <body>
        <div class="container">
            <h1>🤖 Robotic Arm Vision</h1>
            <p class="subtitle">Cloud-Powered Object Detection & Tracking</p>
            
            <img src="{{{{ url_for('video_feed') }}}}" alt="Camera Feed">
            
            <div class="info">
                <h3>📊 System Status</h3>
                <div class="grid">
                    <div class="card">
                        <div class="label">Detection Provider</div>
                        <div class="value" style="font-size: 14px;">{provider_name}</div>
                    </div>
                    <div class="card">
                        <div class="label">API Status</div>
                        <div class="value"><span class="status-badge">{api_status}</span></div>
                    </div>
                    <div class="card">
                        <div class="label">Detection Rate</div>
                        <div class="value">Every {DETECT_EVERY_N_FRAMES} frames</div>
                    </div>
                    <div class="card">
                        <div class="label">Confidence</div>
                        <div class="value">{int(CONFIDENCE_THRESHOLD*100)}%</div>
                    </div>
                    <div class="card">
                        <div class="label">Resolution</div>
                        <div class="value">{CAMERA_WIDTH}x{CAMERA_HEIGHT}</div>
                    </div>
                </div>
                
                <div class="grid" style="margin-top: 20px;">
                    <div class="card">
                        <div class="label">Target Objects</div>
                        <div class="value" style="font-size: 14px;">{', '.join(TARGET_OBJECTS)}</div>
                    </div>
                    <div class="card">
                        <div class="label">Active Servos</div>
                        <div class="value">{', '.join(map(str, WORKING_SERVOS))}</div>
                    </div>
                </div>
                
                {('<div class="warning">⚠️ <strong>Setup Required:</strong> Copy .env.example to .env and add your API credentials</div>' if api_status.startswith('⚠️') and DETECTION_PROVIDER != 'huggingface' else '')}
            </div>
        </div>
    </body>
    </html>
    """
    return render_template_string(html)

@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

# =============================================================================
# MAIN
# =============================================================================

if __name__ == '__main__':
    print("\n" + "="*70)
    print("  🤖 ROBOTIC ARM VISION CONTROLLER")
    print("="*70)
    print(f"\n📹 Camera: {CAMERA_WIDTH}x{CAMERA_HEIGHT}")
    print(f"☁️  Provider: {DETECTION_PROVIDER.upper()}")
    
    if DETECTION_PROVIDER == "roboflow":
        print(f"   - Objects: 80 (COCO dataset)")
        print(f"   - Free tier: 1,000/month")
    elif DETECTION_PROVIDER == "huggingface":
        print(f"   - Model: {HUGGINGFACE_MODEL}")
        print(f"   - Objects: 80 (COCO) - varies by model")
        print(f"   - FREE - No key needed for public models!")
        print(f"   - Unlimited free requests!")
    elif DETECTION_PROVIDER == "google":
        print(f"   - Objects: 10,000+")
        print(f"   - Free tier: 1,000/month")
    elif DETECTION_PROVIDER == "azure":
        print(f"   - Objects: 10,000+")
        print(f"   - Free tier: 5,000/month (MOST GENEROUS!)")
    
    print(f"🎯 Target Objects: {', '.join(TARGET_OBJECTS)}")
    print(f"🔧 Active Servos: {', '.join(map(str, WORKING_SERVOS))}")
    
    # Check configuration
    if DETECTION_PROVIDER == "roboflow" and not ROBOFLOW_API_KEY:
        print("\n⚠️  WARNING: No Roboflow API key configured!")
        print("   1. Copy .env.example to .env")
        print("   2. Get free key at: https://roboflow.com")
        print("   3. Add key to .env file")
    elif DETECTION_PROVIDER == "huggingface":
        print("\n✅ Hugging Face ready - No API key needed!")
        print(f"   Using model: {HUGGINGFACE_MODEL}")
        print("   💡 Get free token (optional) at: https://huggingface.co/settings/tokens")
        print("   💡 Change model in .env: HUGGINGFACE_MODEL=facebook/detr-resnet-101")
    elif DETECTION_PROVIDER == "azure" and (not AZURE_KEY or not AZURE_ENDPOINT):
        print("\n⚠️  WARNING: No Azure credentials configured!")
        print("   1. Copy .env.example to .env")
        print("   2. Get free key at: https://azure.microsoft.com/free/")
        print("   3. Add credentials to .env file")
    elif DETECTION_PROVIDER == "google" and not GOOGLE_CLOUD_CREDENTIALS:
        print("\n⚠️  WARNING: No Google credentials configured!")
        print("   1. Copy .env.example to .env")
        print("   2. Setup: https://cloud.google.com/vision/docs/setup")
        print("   3. Add path to credentials.json in .env file")
    else:
        print("\n✅ API configured and ready!")
    
    print("\n" + "-"*70)
    print("\n🌐 Web Interface: http://<raspberry-pi-ip>:5000")
    print("💡 Find IP: hostname -I")
    print("⌨️  Stop: Ctrl+C")
    print("\n" + "="*70 + "\n")
    
    # Start camera processing in background thread
    camera_thread = threading.Thread(target=camera_loop, daemon=True)
    camera_thread.start()
    
    # Start web server
    try:
        app.run(host='0.0.0.0', port=WEB_SERVER_PORT, debug=False, threaded=True)
    except KeyboardInterrupt:
        print("\n\n🛑 Shutting down...")
    finally:
        picam2.stop()
        logging.info("✅ System stopped. Goodbye!")
