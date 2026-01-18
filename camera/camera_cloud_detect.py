#!/usr/bin/env python3
"""
Web camera viewer with CLOUD-BASED object detection
Fast startup - detection happens in the cloud
Free tier: 1000 detections/month with Roboflow
"""

from flask import Flask, Response, render_template_string
import cv2
from picamera2 import Picamera2
import time
import requests
import base64
import numpy as np

app = Flask(__name__)

# =============================================================================
# CONFIGURATION - Choose your cloud provider
# =============================================================================

# Option 1: Roboflow (Recommended - Free tier, easy setup)
USE_ROBOFLOW = True
ROBOFLOW_API_KEY = "YOUR_API_KEY_HERE"  # Get free key at roboflow.com
ROBOFLOW_MODEL = "coco/3"  # COCO dataset model (80 common objects)

# Option 2: DeepAI (Alternative)
USE_DEEPAI = False
DEEPAI_API_KEY = "YOUR_DEEPAI_KEY"  # Get free key at deepai.org

# Detection settings
CONFIDENCE_THRESHOLD = 0.5
DETECT_EVERY_N_FRAMES = 30  # Only detect every 30 frames to save API calls

# =============================================================================

# Initialize camera
print("🎥 Initializing camera...")
picam2 = Picamera2()
camera_config = picam2.create_preview_configuration(
    main={"size": (640, 480), "format": "RGB888"}
)
picam2.configure(camera_config)
picam2.start()
time.sleep(2)
print("✅ Camera ready!")

# Global variables for detection results
current_detections = []
frame_count = 0
last_detection_time = 0

def detect_with_roboflow(frame):
    """Run object detection using Roboflow API"""
    try:
        # Encode frame as base64
        _, buffer = cv2.imencode('.jpg', frame)
        img_base64 = base64.b64encode(buffer).decode('utf-8')
        
        # Make API request
        url = f"https://detect.roboflow.com/{ROBOFLOW_MODEL}?api_key={ROBOFLOW_API_KEY}&confidence={CONFIDENCE_THRESHOLD*100}"
        response = requests.post(url, data=img_base64, headers={'Content-Type': 'application/x-www-form-urlencoded'}, timeout=5)
        
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
            print(f"❌ Roboflow API error: {response.status_code}")
            return []
            
    except Exception as e:
        print(f"❌ Detection error: {e}")
        return []

def detect_with_deepai(frame):
    """Run object detection using DeepAI API"""
    try:
        # Encode frame
        _, buffer = cv2.imencode('.jpg', frame)
        
        # Make API request
        response = requests.post(
            "https://api.deepai.org/api/object-detector",
            files={'image': buffer.tobytes()},
            headers={'api-key': DEEPAI_API_KEY},
            timeout=5
        )
        
        if response.status_code == 200:
            result = response.json()
            # Parse DeepAI response and convert to our format
            # (Implementation depends on DeepAI response format)
            return []
        else:
            return []
            
    except Exception as e:
        print(f"❌ Detection error: {e}")
        return []

def detect_objects(frame):
    """Main detection function - routes to selected provider"""
    if USE_ROBOFLOW and ROBOFLOW_API_KEY != "YOUR_API_KEY_HERE":
        return detect_with_roboflow(frame)
    elif USE_DEEPAI and DEEPAI_API_KEY != "YOUR_DEEPAI_KEY":
        return detect_with_deepai(frame)
    else:
        # No API key configured - return empty
        return []

def draw_detections(frame, detections):
    """Draw bounding boxes on frame"""
    for det in detections:
        x_min, y_min, x_max, y_max = det['box']
        label = det['class']
        confidence = det['confidence']
        
        # Draw bounding box
        cv2.rectangle(frame, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)
        
        # Draw label with confidence
        label_text = f"{label}: {confidence:.2f}"
        label_size, _ = cv2.getTextSize(label_text, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
        
        # Draw label background
        cv2.rectangle(frame, (x_min, y_min - label_size[1] - 10), 
                     (x_min + label_size[0], y_min), (0, 255, 0), -1)
        # Draw label text
        cv2.putText(frame, label_text, (x_min, y_min - 5),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)
    
    return frame

def generate_frames():
    """Generator function for video streaming"""
    global current_detections, frame_count, last_detection_time
    
    while True:
        try:
            # Capture frame
            frame = picam2.capture_array()
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            
            frame_count += 1
            
            # Run detection every N frames to save API calls
            if frame_count % DETECT_EVERY_N_FRAMES == 0:
                current_detections = detect_objects(frame)
                last_detection_time = time.time()
                if current_detections:
                    print(f"🎯 Detected {len(current_detections)} object(s)")
            
            # Draw existing detections
            frame = draw_detections(frame, current_detections)
            
            # Add status overlay
            status = "DETECTING..." if frame_count % DETECT_EVERY_N_FRAMES == 0 else f"Objects: {len(current_detections)}"
            cv2.putText(frame, status, (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            # Encode and yield
            ret, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
            if not ret:
                continue
            
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')
                   
        except Exception as e:
            print(f"❌ Frame error: {e}")
            break

@app.route('/')
def index():
    api_status = "✅ Configured" if (USE_ROBOFLOW and ROBOFLOW_API_KEY != "YOUR_API_KEY_HERE") else "⚠️ No API Key"
    
    html = f"""
    <!DOCTYPE html>
    <html>
    <head>
        <title>Cloud Object Detection</title>
        <meta name="viewport" content="width=device-width, initial-scale=1.0">
        <style>
            body {{
                margin: 0;
                padding: 20px;
                background-color: #1e1e1e;
                font-family: Arial, sans-serif;
                color: #ffffff;
                text-align: center;
            }}
            h1 {{ margin-bottom: 10px; }}
            .subtitle {{ color: #4CAF50; margin-bottom: 20px; }}
            .container {{ max-width: 900px; margin: 0 auto; }}
            img {{
                width: 100%;
                max-width: 640px;
                border: 3px solid #4CAF50;
                border-radius: 8px;
                box-shadow: 0 4px 12px rgba(76, 175, 80, 0.4);
            }}
            .info {{
                margin-top: 20px;
                padding: 20px;
                background-color: #2d2d2d;
                border-radius: 8px;
                text-align: left;
            }}
            .status {{ 
                display: inline-block;
                padding: 5px 15px;
                background-color: {('#1e6b1e' if api_status == '✅ Configured' else '#8b6b00')};
                border-radius: 20px;
                font-size: 14px;
            }}
        </style>
    </head>
    <body>
        <div class="container">
            <h1>☁️ Cloud Object Detection</h1>
            <p class="subtitle">Fast startup - Detection in the cloud</p>
            
            <img src="{{{{ url_for('video_feed') }}}}" alt="Camera Feed">
            
            <div class="info">
                <p><strong>API Status:</strong> <span class="status">{api_status}</span></p>
                <p><strong>Provider:</strong> {'Roboflow' if USE_ROBOFLOW else 'DeepAI'}</p>
                <p><strong>Detection Rate:</strong> Every {DETECT_EVERY_N_FRAMES} frames</p>
                <p><strong>Confidence:</strong> {CONFIDENCE_THRESHOLD*100}%</p>
                
                {('<p style="color: #ff9800;">⚠️ <strong>Setup Required:</strong> Add your API key in camera_cloud_detect.py</p>' if api_status != '✅ Configured' else '')}
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

if __name__ == '__main__':
    print("\n" + "="*70)
    print("  ☁️  Cloud Object Detection Camera")
    print("="*70)
    
    if USE_ROBOFLOW and ROBOFLOW_API_KEY == "YOUR_API_KEY_HERE":
        print("\n⚠️  WARNING: No API key configured!")
        print("   Get free key at: https://roboflow.com")
        print("   Edit ROBOFLOW_API_KEY in this file\n")
    else:
        print("\n✅ API configured and ready!")
    
    print("\n🌐 Access at: http://<raspberry-pi-ip>:5000")
    print("💡 Find IP: hostname -I")
    print("⌨️  Stop: Ctrl+C")
    print("="*70 + "\n")
    
    try:
        app.run(host='0.0.0.0', port=5000, debug=False, threaded=True)
    except KeyboardInterrupt:
        print("\n\n🛑 Stopping...")
    finally:
        picam2.stop()
        print("✅ Done!\n")
